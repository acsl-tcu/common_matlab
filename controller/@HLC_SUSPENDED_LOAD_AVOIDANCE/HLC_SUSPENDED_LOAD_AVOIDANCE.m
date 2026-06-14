% classdef HLC_SUSPENDED_LOAD_AVOIDANCE < handle
% % クアッドコプター用階層型線形化を使った入力算出
% properties
%     self
%     result
%     param
% end
% 
% methods
% 
%     function obj = HLC_SUSPENDED_LOAD_AVOIDANCE(self, param)
%         obj.self = self;
%         obj.param = param;
% 
%     end
% 
%     function result = do(obj, varargin)
%         Param = obj.param; % param (optional) : 構造体：ゲインF1-F4
%         model = obj.self.estimator.result; % 推定した状態
%         ref = obj.self.reference.result; % 目標値
% 
%         % 目標値を取得
%         if isprop(ref.state, 'xd')
%             xd = ref.state.xd; % 20次元の目標値に対応する用
%         else
%             xd = ref.state.get();
%         end
% 
%         pL = model.state.pL;
%         if isprop(model.state, "pT")
%             pT = model.state.pT;
%         else
%             delta = pL - model.state.p;
%             if norm(delta) > 1e-9
%                 pT = delta / norm(delta);
%             else
%                 pT = [0; 0; -1];
%             end
%         end
%         P = [obj.self.parameter.get(["mass", "jx", "jy", "jz", "gravity", "loadmass", "cableL"]), 0, 0];
% 
%         x = [model.state.getq('compact'); model.state.w; pL; model.state.vL; pT; model.state.wL]; % [q, w ,pL, vL, pT, wL]に並べ替え
%         % [model.state.p, x(8:10), xd(1:3), x(8:10) - xd(1:3)]
%         % yaw角の定義域の問題を回避,h4 = yaw - yawd(誤差)だがyawd = -(誤差)+yawの値を入れる．x,y,yawの仮想入力はVs_SuspendedLoadはクオータニオンで計算するため
%         % yawサブシステムの入力を設計するときにyaw角を打ち消して定義域修正した誤差を反映
%         yaw = wrapToPi(model.state.q(3)); % 機体yaw角[-pi,pi]にする特にyaw
%         yawd = xd(4); % 目標yaw角
%         yawUnit = [cos(yaw); sin(yaw); 0]; % yawの方向ベクトル
%         yawdUnit = [cos(yawd); sin(yawd); 0]; % yawdの方向ベクトル
%         deltaYaw = sign(cross(yawdUnit, yawUnit)) * acos(yawdUnit' * yawUnit); % 目標角度からみた機体角度との誤差
%         xd(4) = -deltaYaw(3) + yaw; % yaw打ち消しと誤差をyawの目標角に入れる．
%         %目標値の格納
%         xd = [xd; zeros(28 - size(xd, 1), 1)];
% 
%         % 階層型線形化による入力計算
%         % 仮想入力のゲイン
%         F1 = Param.F1; % z方向サブシステムのゲイン
%         F2 = Param.F2; % x方向サブシステムのゲイン
%         F3 = Param.F3; % y方向サブシステムのゲイン
%         F4 = Param.F4; % yaw方向サブシステムのゲイン
% 
%         % -----------------------------------------------------------------
%         % 名目（Nominal）仮想操作量の計算
%         % -----------------------------------------------------------------
%         vf_nominal = obj.Vfd_SuspendedLoadxyDst(Param.dt, x, xd', F1); % 実験で刻み時間が変わったときに対応
%         vs_nominal = obj.Vs_SuspendedLoadxyDst(x, xd', vf_nominal, P, F2, F3, F4); % 第二層x,y,yawサブシステムの仮想入力の計算
% 
%         v_nominal = [vf_nominal(1); vs_nominal(1); vs_nominal(2)]; % z,x,yの仮想入力の加速度の抜き出し
% 
%         % -----------------------------------------------------------------
%         % 障害物定義・保護球定義
%         % -----------------------------------------------------------------
%         obs_env = ENVIRONMENT_OBSTACLE(); % 障害物配列の一括取得
%         num_obstacles = length(obs_env);  % 障害物の数を自動カウント
% 
%         protection_config = [
%             % 位置割合 (λ_new),  その位置の球の半径 (rq)
%             0.00,               0.30;   % 1つ目：吊り荷本体（起点）
%             0.25,               0.25;   % 2つ目：ワイヤー下部
%             0.50,               0.25;   % 3つ目：ワイヤー中間
%             0.75,               0.25;   % 4つ目：ワイヤー上部
%             1.00,               0.40;   % 5つ目：ドローン本体（終点）
%         ];
% 
%         p_drone = model.state.p;     % ドローン本体の3次元現在位置 [x; y; z]
%         p_L     = pL(1:3);           % 牽引物（荷物）の3次元現在位置 [x; y; z]
%         v_L     = model.state.vL(1:3); % 牽引物（荷物）の3次元現在速度 [vx; vy; vz]
% 
%         % 荷物(起点)からドローン(終点)を見上げる「ワイヤー方向ベクトル」
%         n_vec_up = p_drone - p_L;
% 
%         % QP用の不等式制約（A_qp * v_safe <= b_qp）を蓄積する空の行列を用意
%         A_qp = [];
%         b_qp = [];
% 
%         num_protection_spheres = size(protection_config, 1); % 保護球の総数
% 
%         v_safe = v_nominal; % 安全入力に公称入力を
% 
%         %ここにいろいろなものを入れる
% 
%         % =================================================================
%         % 3. 安全化された仮想操作量を、各階層の変数へ再分配
%         % =================================================================
%         vf_safe = vf_nominal;
%         vs_safe = vs_nominal;
% 
%         vf_safe(1) = v_safe(1); % 安全化されたZ軸の仮想入力
%         vs_safe(1) = v_safe(2); % 安全化されたX軸の仮想入力
%         vs_safe(2) = v_safe(3); % 安全化されたY軸の仮想入力
% 
%         % =================================================================
%         % 4. 下流レイヤーへの非線形相殺・実物理入力への変換
%         % =================================================================
%         uf = obj.Uf_SuspendedLoadxyDst(x, xd', vf_safe, P); % 第一層の仮想入力の実入力(推力)への変換
%         % h234  = obj.H234_SuspendedLoadxyDst(x,xd',vf,P);  % ただの単位行列なのでなくてもいい
%         %tic
%         beta2 = obj.Beta2_SuspendedLoadxyDst(x, xd', vf_safe, P); % 第二層のbetaの逆行列
%         vs_alpha2 = obj.V2_alpha2_SuspendedLoadxyDst(x, xd', vf_safe, vs_safe', P); % 第二層のvs - alpha
%         us = beta2 \ vs_alpha2; % 第二層の実入力（roll,pitch,yawのトルク）への変換：bate^(-1)*(vs - alpha) %h234*invbeta2*a2;
%         %obj.result.aa=toc;
%         tmp = [uf(1); us]; % 実入力へ変換
%         obj.result.tmp = tmp; % 入力に制限を付けてない値を格納
% 
%         % 安全のため入力値に制限を付ける．推定した牽引物質量や紐の長さ，外乱などを表示．
%         %disp("time: "+ num2str(t,2)+" z position of drone:
%         %"+num2str(model.state.p(3),3)+" estimated load mass:
%         %"+num2str(P(6),4)+" dst:(x,y) "+num2str(P(end-1:end),4))
%         obj.result.input = [max(0, min(20, tmp(1))); max(-1, min(1, tmp(2))); max(-1, min(1, tmp(3))); max(-1, min(1, tmp(4)))]; %+[normrnd(0,0.01,1);normrnd(0,0.001,[3,1])]*1; %入力にノイズを付与可能
%         % obj.result.input = [0.0,0,0,(0.5236+0.04)*9.81]';%
%         obj.result.xd = xd;
%         obj.result.x = x;
%         result = obj.result;
% 
%     end
% 
%     function show(obj)
%         obj.result
%     end
% end
% 
% end


% classdef HLC_SUSPENDED_LOAD_AVOIDANCE < handle
% % クアッドコプター用階層型線形化を使った入力算出
% properties
%     self
%     result
%     param
% end
% 
% methods
% 
%     function obj = HLC_SUSPENDED_LOAD_AVOIDANCE(self, param)
%         obj.self = self;
%         obj.param = param;
% 
%     end
% 
%     function result = do(obj, t, varargin)
%         % 🚨【新規追記】：流れ込んできた t が TIME オブジェクトだった場合の数値抽出処理
%         if isobject(t) || isstruct(t)
%             if isprop(t, 't') || isfield(t, 't')
%                 t = t.t; % TIME.t から double の秒数を取り出す
%             elseif isprop(t, 'time') || isfield(t, 'time')
%                 t = t.time; % もしプロパティ名が time だった場合
%             else
%                 try
%                     t = double(t); % 強制型変換を試みる
%                 catch
%                     t = 0; % どうしてもダメなら0でフォールバック
%                 end
%             end
%         end
%         Param = obj.param; % param (optional) : 構造体：ゲインF1-F4
%         model = obj.self.estimator.result; % 推定した状態
%         ref = obj.self.reference.result; % 目標値
% 
%         % 目標値を取得
%         if isprop(ref.state, 'xd')
%             xd = ref.state.xd; % 20次元の目標値に対応する用
%         else
%             xd = ref.state.get();
%         end
% 
%         pL = model.state.pL;
%         if isprop(model.state, "pT")
%             pT = model.state.pT;
%         else
%             delta = pL - model.state.p;
%             if norm(delta) > 1e-9
%                 pT = delta / norm(delta);
%             else
%                 pT = [0; 0; -1];
%             end
%         end
%         P = [obj.self.parameter.get(["mass", "jx", "jy", "jz", "gravity", "loadmass", "cableL"]), 0, 0];
% 
%         x = [model.state.getq('compact'); model.state.w; pL; model.state.vL; pT; model.state.wL]; % [q, w ,pL, vL, pT, wL]に並べ替え
%         % [model.state.p, x(8:10), xd(1:3), x(8:10) - xd(1:3)]
%         % yaw角の定義域の問題を回避,h4 = yaw - yawd(誤差)だがyawd = -(誤差)+yawの値を入れる．x,y,yawの仮想入力はVs_SuspendedLoadはクオータニオンで計算するため
%         % yawサブシステムの入力を設計するときにyaw角を打ち消して定義域修正した誤差を反映
%         yaw = wrapToPi(model.state.q(3)); % 機体yaw角[-pi,pi]にする特にyaw
%         yawd = xd(4); % 目標yaw角
%         yawUnit = [cos(yaw); sin(yaw); 0]; % yawの方向ベクトル
%         yawdUnit = [cos(yawd); sin(yawd); 0]; % yawdの方向ベクトル
%         deltaYaw = sign(cross(yawdUnit, yawUnit)) * acos(yawdUnit' * yawUnit); % 目標角度からみた機体角度との誤差
%         xd(4) = -deltaYaw(3) + yaw; % yaw打ち消しと誤差をyawの目標角に入れる．
%         %目標値の格納
%         xd = [xd; zeros(28 - size(xd, 1), 1)];
% 
%         % 階層型線形化による入力計算
%         % 仮想入力のゲイン
%         F1 = Param.F1; % z方向サブシステムのゲイン
%         F2 = Param.F2; % x方向サブシステムのゲイン
%         F3 = Param.F3; % y方向サブシステムのゲイン
%         F4 = Param.F4; % yaw方向サブシステムのゲイン
% 
%         % -----------------------------------------------------------------
%         % 名目（Nominal）仮想操作量の計算 (既存部分)
%         % -----------------------------------------------------------------
%         vf_nominal = obj.Vfd_SuspendedLoadxyDst(Param.dt, x, xd', F1); 
%         vs_nominal = obj.Vs_SuspendedLoadxyDst(x, xd', vf_nominal, P, F2, F3, F4); 
%         v_nominal = [vf_nominal(1); vs_nominal(1); vs_nominal(2)]; % [v1_cmd; u2_cmd; u3_cmd]
% 
%         % -----------------------------------------------------------------
%         % 🔥【新規追記】完全同期型 HO-CBF-QP フィルター
%         % -----------------------------------------------------------------
%         obs_env = ENVIRONMENT_OBSTACLE(); 
%         num_obstacles = length(obs_env);  
% 
%         % 保護球の配置定義 [lambda値 (0:荷物 〜 1:機体), その球の半径]
%         protection_config = [
%             0.00,               0.30;   % 1: 牽引物本体
%             0.25,               0.25;   % 2: ワイヤー下部
%             0.50,               0.25;   % 3: ワイヤー中間
%             0.75,               0.25;   % 4: ワイヤー上部
%             1.00,               0.40;   % 5: ドローン本体
%         ];
%         num_spheres = size(protection_config, 1);
% 
%         % 5階層のHO-CBFゲイン (滑らかさを担保するため、最初は[1.5〜3.0]程度で同調するのが安全です)
%         K_gains = [2.0, 2.0, 2.0, 2.0, 2.0]; 
% 
%         % -----------------------------------------------------------------
%         % 🔥【デバッグ確認】入力変数と状態変数の生データチェック
%         % -----------------------------------------------------------------
%         fprintf('\n====== HLC DEBUG START (t = %.3f) ======\n', t);
%         fprintf('v_nominal (名目入力): [%.4f, %.4f, %.4f]\n', v_nominal(1), v_nominal(2), v_nominal(3));
%         fprintf('pL (荷物位置): [%.4f, %.4f, %.4f]\n', pL(1), pL(2), pL(3));
%         fprintf('pT (ワイヤー方向): [%.4f, %.4f, %.4f]\n', pT(1), pT(2), pT(3));
% 
%         A_qp = [];
%         b_qp = [];
% 
%         for i = 1:num_obstacles
%             ox_val = obs_env(i).p_obs(1);
%             oy_val = obs_env(i).p_obs(2);
%             oz_val = obs_env(i).p_obs(3);
%             ro_val = obs_env(i).r_obs; 
% 
%             for j = 1:num_spheres
%                 lambda_j = protection_config(j, 1);
%                 r_sph_j  = protection_config(j, 2);
% 
%                 cbf_base = [ox_val; oy_val; oz_val; ro_val; r_sph_j; lambda_j; ...
%                             K_gains(1); K_gains(2); K_gains(3); K_gains(4); K_gains(5)];
%                 cbfParam = [cbf_base; P(:)]; 
% 
%                 % 🌟 評価直前のパルス確認
%                 % fprintf('  -> 障害物 %d, 球 %d のCBFを評価中...\n', i, j);
% 
%                 % 関数の実行時間を計測
%                 tic;
%                 [A_s, b_s] = CBF_Constraints_Synced_Order5([], x, xd', cbfParam, t);
%                 eval_time = toc;
% 
%                 % 🌟【超重要】吐き出された数値のチェック
%                 if any(isnan(A_s)) || any(isinf(A_s)) || any(isnan(b_s)) || any(isinf(b_s))
%                     fprintf('  ⚠️ [警告] 障害物 %d, 球 %d で NaN または Inf が発生！\n', i, j);
%                     fprintf('     A_s: [%s], b_s: %.4f\n', num2str(A_s, ' %.2e'), b_s);
%                 elseif norm(A_s) < 1e-12
%                     fprintf('  ⚠️ [警告] 障害物 %d, 球 %d のゲイン A_s がほぼゼロ (%.2e) です。入力が効いていません。\n', i, j, norm(A_s));
%                 else
%                     fprintf('  ℹ️ 障害物 %d, 球 %d: 計算時間 = %.4f秒, A_sノルム = %.2e, b_s = %.2f\n', i, j, eval_time, norm(A_s), b_s);
%                 end
% 
%                 A_qp = [A_qp; -A_s];
%                 b_qp = [b_qp;  b_s];
%             end
%         end
% 
%         fprintf('--- quadprog 直前チェック ---\n');
%         fprintf('総制約数 (A_qpの行数): %d\n', size(A_qp, 1));
% 
%         % 🌟 quadprogの実行前パルス
%         disp('>> quadprog を起動します...');
% 
%         % 実機・実験用の超高速オプション設定
%         qp_options = optimoptions('quadprog', 'Display', 'final', 'Algorithm', 'active-set');
% 
%         tic;
%         try
%             [v_safe, ~, exitflag] = quadprog(H_quad, f_quad, A_qp, b_qp, [], [], [], [], [], qp_options);
%             qp_time = toc;
%             fprintf('>> quadprog 終了: 計算時間 = %.4f秒, exitflag = %d\n', qp_time, exitflag);
%             fprintf('v_safe (安全入力): [%.4f, %.4f, %.4f]\n', v_safe(1), v_safe(2), v_safe(3));
%             fprintf('入力変化量: [%.4f, %.4f, %.4f]\n', v_safe(1)-v_nominal(1), v_safe(2)-v_nominal(2), v_safe(3)-v_nominal(3));
%         catch ME
%             fprintf('❌ quadprog 内部でエラー発生: %s\n', ME.message);
%             v_safe = v_nominal; 
%         end
%         fprintf('====== HLC DEBUG END ======\n\n');
% 
%         % -----------------------------------------------------------------
%         % 安全化された仮想操作量を、各下流レイヤーの変数へ分配 (既存部分)
%         % -----------------------------------------------------------------
%         vf_safe = vf_nominal;
%         vs_safe = vs_nominal;
% 
%         vf_safe(1) = v_safe(1); % 完全連続性が保証された安全な v1_cmd
%         vs_safe(1) = v_safe(2); % 完全連続性が保証された安全な u2_cmd
%         vs_safe(2) = v_safe(3); % 完全連続性が保証された安全な u3_cmd
% 
%         % -----------------------------------------------------------------
%         % 下流レイヤーへの非線形相殺・実物理入力への変換 (既存部分)
%         % -----------------------------------------------------------------
%         uf = obj.Uf_SuspendedLoadxyDst(x, xd', vf_safe, P); 
%         beta2 = obj.Beta2_SuspendedLoadxyDst(x, xd', vf_safe, P); 
%         vs_alpha2 = obj.V2_alpha2_SuspendedLoadxyDst(x, xd', vf_safe, vs_safe', P); 
%         us = beta2 \ vs_alpha2; 
% 
%         tmp = [uf(1); us]; 
%         obj.result.tmp = tmp; 
%         obj.result.input = [max(0, min(20, tmp(1))); max(-1, min(1, tmp(2))); max(-1, min(1, tmp(3))); max(-1, min(1, tmp(4)))]; 
%         obj.result.xd = xd;
%         obj.result.x = x;
%         result = obj.result;
%     end
% 
%     function show(obj)
%         obj.result
%     end
% end
% 
% end

classdef HLC_SUSPENDED_LOAD_AVOIDANCE < handle
% クアッドコプター用階層型線形化を使った入力算出
properties
    self
    result
    param
end
methods
    function obj = HLC_SUSPENDED_LOAD_AVOIDANCE(self, param)
        obj.self = self;
        obj.param = param;
    end
    function result = do(obj, t, varargin)
        % 🚨【新規追記】：流れ込んできた t が TIME オブジェクトだった場合の数値抽出処理
        if isobject(t) || isstruct(t)
            if isprop(t, 't') || isfield(t, 't')
                t = t.t; % TIME.t から double の秒数を取り出す
            elseif isprop(t, 'time') || isfield(t, 'time')
                t = t.time; % もしプロパティ名が time だった場合
            else
                try
                    t = double(t); % 強制型変換を試みる
                catch
                    t = 0; % どうしてもダメなら0でフォールバック
                end
            end
        end
        Param = obj.param; % param (optional) : 構造体：ゲインF1-F4
        model = obj.self.estimator.result; % 推定した状態
        ref = obj.self.reference.result; % 目標値
        % 目標値を取得
        if isprop(ref.state, 'xd')
            xd = ref.state.xd; % 20次元の目標値に対応する用
        else
            xd = ref.state.get();
        end
        pL = model.state.pL;
        if isprop(model.state, "pT")
            pT = model.state.pT;
        else
            delta = pL - model.state.p;
            if norm(delta) > 1e-9
                pT = delta / norm(delta);
            else
                pT = [0; 0; -1];
            end
        end
        P = [obj.self.parameter.get(["mass", "jx", "jy", "jz", "gravity", "loadmass", "cableL"]), 0, 0];
        x = [model.state.getq('compact'); model.state.w; pL; model.state.vL; pT; model.state.wL]; % [q, w ,pL, vL, pT, wL]に並べ替え
        % [model.state.p, x(8:10), xd(1:3), x(8:10) - xd(1:3)]
        % yaw角の定義域の問題を回避,h4 = yaw - yawd(誤差)だがyawd = -(誤差)+yawの値を入れる．x,y,yawの仮想入力はVs_SuspendedLoadはクオータニオンで計算するため
        % yawサブシステムの入力を設計するときにyaw角を打ち消して定義域修正した誤差を反映
        yaw = wrapToPi(model.state.q(3)); % 機体yaw角[-pi,pi]にする特にyaw
        yawd = xd(4); % 目標yaw角
        yawUnit = [cos(yaw); sin(yaw); 0]; % yawの方向ベクトル
        yawdUnit = [cos(yawd); sin(yawd); 0]; % yawdの方向ベクトル
        deltaYaw = sign(cross(yawdUnit, yawUnit)) * acos(yawdUnit' * yawUnit); % 目標角度からみた機体角度との誤差
        xd(4) = -deltaYaw(3) + yaw; % yaw打ち消しと誤差をyawの目標角に入れる．
        %目標値の格納
        xd = [xd; zeros(28 - size(xd, 1), 1)];
        % 階層型線形化による入力計算
        % 仮想入力のゲイン
        F1 = Param.F1; % z方向サブシステムのゲイン
        F2 = Param.F2; % x方向サブシステムのゲイン
        F3 = Param.F3; % y方向サブシステムのゲイン
        F4 = Param.F4; % yaw方向サブシステムのゲイン
        % -----------------------------------------------------------------
        % 名目（Nominal）仮想操作量の計算 (既存部分)
        % -----------------------------------------------------------------
        vf_nominal = obj.Vfd_SuspendedLoadxyDst(Param.dt, x, xd', F1); 
        vs_nominal = obj.Vs_SuspendedLoadxyDst(x, xd', vf_nominal, P, F2, F3, F4); 
        v_nominal = [vf_nominal(1); vs_nominal(1); vs_nominal(2)]; % [v1_cmd; u2_cmd; u3_cmd]
        % -----------------------------------------------------------------
        % 🔥【新規追記】完全同期型 HO-CBF-QP フィルター
        % -----------------------------------------------------------------
        obs_env = ENVIRONMENT_OBSTACLE(); 
        num_obstacles = length(obs_env);  
        
        % 保護球の配置定義 [lambda値 (0:荷物 〜 1:機体), その球の半径]
        protection_config = [
            0.00,               0.30;   % 1: 牽引物本体
            0.25,               0.25;   % 2: ワイヤー下部
            0.50,               0.25;   % 3: ワイヤー中間
            0.75,               0.25;   % 4: ワイヤー上部
            1.00,               0.40;   % 5: ドローン本体
        ];
        num_spheres = size(protection_config, 1);
        
        % 5階層のHO-CBFゲイン (滑らかさを担保するため、最初は[1.5〜3.0]程度で同調するのが安全です)
        % K_gains = [2.0, 2.0, 2.0, 2.0, 2.0]; 
        K_gains = [0.2, 0.2, 0.2, 0.2, 0.2];
        
        % -----------------------------------------------------------------
        % 🔥【デバッグ確認】入力変数と状態変数の生データチェック
        % -----------------------------------------------------------------
        fprintf('\n====== HLC DEBUG START (t = %.3f) ======\n', t);
        fprintf('v_nominal (名目入力): [%.4f, %.4f, %.4f]\n', v_nominal(1), v_nominal(2), v_nominal(3));
        fprintf('pL (荷物位置): [%.4f, %.4f, %.4f]\n', pL(1), pL(2), pL(3));
        fprintf('pT (ワイヤー方向): [%.4f, %.4f, %.4f]\n', pT(1), pT(2), pT(3));
        A_qp = [];
        b_qp = [];
        
        for i = 1:num_obstacles
            ox_val = obs_env(i).p_obs(1);
            oy_val = obs_env(i).p_obs(2);
            oz_val = obs_env(i).p_obs(3);
            ro_val = obs_env(i).r_obs; 
            
            for j = 1:num_spheres
                lambda_j = protection_config(j, 1);
                r_sph_j  = protection_config(j, 2);
                
                cbf_base = [ox_val; oy_val; oz_val; ro_val; r_sph_j; lambda_j; ...
                            K_gains(1); K_gains(2); K_gains(3); K_gains(4); K_gains(5)];
                cbfParam = [cbf_base; P(:)]; 
                        
                % 関数の実行時間を計測
                tic;
                [A_s, b_s] = CBF_Constraints_Synced_Order5([], x, xd', cbfParam, t);
                % =========================================================
                % 🚨【幾何学ミッチマッチ確認用デバッグコードの修正】
                % =========================================================
                % 1. 生成スクリプトと100%同期した正しい球の位置を再現（+ から - へ変更）
                p_sphere_now = pL - lambda_j * P(7) * pT; % pl - lambda * cableL * pT
                
                % 2. ドローン側が認識している「障害物との距離の2乗」を計算
                dist_sq = (p_sphere_now(1) - ox_val)^2 + (p_sphere_now(2) - oy_val)^2 + (p_sphere_now(3) - oz_val)^2;
                dist_actual = sqrt(dist_sq); % 物理的な中心間距離 (m)
                
                % 3. 衝突限界半径の合計
                limit_radius = ro_val + r_sph_j;
                
                % 4. 本当の幾何学的マージン (これが0以下なら物理的にめり込んでいる)
                geometric_margin = dist_actual - limit_radius;
                
                % 5. ログに詳細をすべて焼き出す
                fprintf('  [[ 幾何チェック - 障害物%d / 球%d (lambda=%.2f) ]]\n', i, j, lambda_j);
                fprintf('    ・計算上の球の位置 : [%.3f, %.3f, %.3f]\n', p_sphere_now(1), p_sphere_now(2), p_sphere_now(3));
                fprintf('    ・計算上の障害物   : [%.3f, %.3f, %.3f]\n', ox_val, oy_val, oz_val);
                fprintf('    ・中心間距離 : %.3f m  (衝突限界: %.3f m)\n', dist_actual, limit_radius);
                fprintf('    ・本当の残り距離 : %.3f m\n', geometric_margin);
                % =========================================================
                eval_time = toc;
                
                % 🌟【超重要】吐き出された数値のチェック
                if any(isnan(A_s)) || any(isinf(A_s)) || any(isnan(b_s)) || any(isinf(b_s))
                    fprintf('  ⚠️ [警告] 障害物 %d, 球 %d で NaN または Inf が発生！\n', i, j);
                    fprintf('     A_s: [%s], b_s: %.4f\n', num2str(A_s, ' %.2e'), b_s);
                elseif norm(A_s) < 1e-12
                    fprintf('  ⚠️ [警告] 障害物 %d, 球 %d のゲイン A_s がほぼゼロ (%.2e) です。入力が効いていません。\n', i, j, norm(A_s));
                else
                    fprintf('  ℹ️ 障害物 %d, 球 %d: 計算時間 = %.4f秒, A_sノルム = %.2e, b_s = %.2f\n', i, j, eval_time, norm(A_s), b_s);
                end
                
                A_qp = [A_qp; -A_s];
                b_qp = [b_qp;  b_s];
            end
        end
        
        fprintf('--- quadprog 直前チェック ---\n');
        fprintf('総制約数 (A_qpの行数): %d\n', size(A_qp, 1));
        
        % 🚨【最重要追記】：これまで定義されていなかった二次形式用の重み行列をここで定義
        Q_matrix = diag([1.0, 1.0, 1.0]); % [v1, u2, u3] の変動に対するペナルティ重み
        H_quad   = Q_matrix;              % quadprog用のHessian
        f_quad   = -Q_matrix * v_nominal; % quadprog用の線形ベクトル
        
        % 🌟 quadprogの実行前パルス
        disp('>> quadprog を起動します...');
        
        % 実機・実験用の超高速オプション設定
        % 実機・実験用の超高速オプション設定
        qp_options = optimoptions('quadprog', 'Display', 'final', 'Algorithm', 'active-set');
        
        tic;
        try
            % 🚨【修正箇所】：第9引数の [] を v_nominal に書き換え、初期点として与える
            [v_safe, ~, exitflag] = quadprog(H_quad, f_quad, A_qp, b_qp, [], [], [], [], v_nominal, qp_options);
            qp_time = toc;
            fprintf('>> quadprog 終了: 計算時間 = %.4f秒, exitflag = %d\n', qp_time, exitflag);
            fprintf('v_safe (安全入力): [%.4f, %.4f, %.4f]\n', v_safe(1), v_safe(2), v_safe(3));
            fprintf('入力変化量: [%.4f, %.4f, %.4f]\n', v_safe(1)-v_nominal(1), v_safe(2)-v_nominal(2), v_safe(3)-v_nominal(3));
        catch ME
            fprintf('❌ quadprog 内部でエラー発生: %s\n', ME.message);
            v_safe = v_nominal; 
        end
        fprintf('====== HLC DEBUG END ======\n\n');
        
        % -----------------------------------------------------------------
        % 安全化された仮想操作量を、各下流レイヤーの変数へ分配 (既存部分)
        % -----------------------------------------------------------------
        vf_safe = vf_nominal;
        vs_safe = vs_nominal;
        
        vf_safe(1) = v_safe(1); % 完全連続性が保証された安全な v1_cmd
        vs_safe(1) = v_safe(2); % 完全連続性が保証された安全な u2_cmd
        vs_safe(2) = v_safe(3); % 完全連続性が保証された安全な u3_cmd
        
        % -----------------------------------------------------------------
        % 下流レイヤーへの非線形相殺・実物理入力への変換 (既存部分)
        % -----------------------------------------------------------------
        uf = obj.Uf_SuspendedLoadxyDst(x, xd', vf_safe, P); 
        beta2 = obj.Beta2_SuspendedLoadxyDst(x, xd', vf_safe, P); 
        vs_alpha2 = obj.V2_alpha2_SuspendedLoadxyDst(x, xd', vf_safe, vs_safe', P); 
        us = beta2 \ vs_alpha2; 
        
        tmp = [uf(1); us]; 
        obj.result.tmp = tmp; 
        obj.result.input = [max(0, min(20, tmp(1))); max(-1, min(1, tmp(2))); max(-1, min(1, tmp(3))); max(-1, min(1, tmp(4)))]; 
        obj.result.xd = xd;
        obj.result.x = x;
        result = obj.result;
    end
    function show(obj)
        obj.result
    end
end
end