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

    function result = do(obj, varargin)
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
        xd = [xd; zeros(28 - size(xd, 1), 1)]; % 足りない分は0で埋める．

        % =================================================================
        % 【新規追記】相対次数2のCBFによる目標速度・位置のサチュレーション（複数障害物一括管理版）
        % =================================================================
        % 1. 外部関数から障害物リストを取得
        obs_list = ENVIRONMENT_OBSTACLE(); 
        num_obs = length(obs_list);
        
        % CBFのゲイン (値が大きいほど障害物の手前で滑らかに減速を開始する)
        % 前回の4次ECBFで失敗したような発振を防ぐため、1.0〜2.0あたりのマイルドな値から調整してください
        alpha_cbf = 1.5; 
        
        % 2. 現在の荷物の位置 pL と 速度 vL (modelから取得)
        px = pL(1); py = pL(2); pz = pL(3);
        vx = model.state.vL(1); vy = model.state.vL(2); vz = model.state.vL(3);
        
        % 元々の目標値（xd）から計算される、目標移動速度（dxd1, dxd2）
        ref_vx = xd(5); 
        ref_vy = xd(6);
        
        % すべての障害物に対して順次安全チェックを実行
        for idx = 1:num_obs
            % 障害物のパラメータを抽出
            ox = obs_list(idx).p_obs(1);
            oy = obs_list(idx).p_obs(2);
            oz = obs_list(idx).p_obs(3); % 高度成分（今回はX-Y平面での回避を想定）
            R_safe = obs_list(idx).R_safe;
            
            % 3. 障害物との距離 h の計算 (位置だけで計算可能・ノイズフリー)
            % ※高度(Z)も考慮する場合は3次元距離に、平面のみなら(pz - oz)^2 の項を消してください
            h = (px - ox)^2 + (py - oy)^2 + (pz - oz)^2 - R_safe^2;
            
            % 障害物に向かうベクトル (機体から見た単位方向)
            dir_x = px - ox;
            dir_y = py - oy;
            dir_z = pz - oz;
            dist_to_obs = sqrt(dir_x^2 + dir_y^2 + dir_z^2);
            
            if dist_to_obs > 1e-6
                dir_x = dir_x / dist_to_obs;
                dir_y = dir_y / dist_to_obs;
                dir_z = dir_z / dist_to_obs;
                
                % 4. 障害物方向への接近速度の許容上限値 V_max をCBF式から逆算
                % CBF条件: h_dot + alpha_cbf * h >= 0 
                % => 2 * dist_to_obs * V_approach + alpha_cbf * h >= 0
                max_approach_speed = (alpha_cbf * h) / (2 * dist_to_obs);
                
                % 現在の目標速度の障害物方向への投影
                ref_v_approach = ref_vx * dir_x + ref_vy * dir_y;
                
                % 5. もし目標速度がCBFの制限を破って障害物に突っ込もうとしている場合
                if ref_v_approach < -max_approach_speed
                    % 障害物方向の目標速度成分を、安全な上限値（-max_approach_speed）にクランプ
                    v_ortho_x = ref_vx - ref_v_approach * dir_x;
                    v_ortho_y = ref_vy - ref_v_approach * dir_y;
                    
                    % 安全に修正された目標速度（X-Y平面）
                    ref_vx = v_ortho_x + (-max_approach_speed) * dir_x;
                    ref_vy = v_ortho_y + (-max_approach_speed) * dir_y;
                    
                    % 修正された速度を xd 構造体へ上書き格納
                    xd(5) = ref_vx; % dXd1 を修正
                    xd(6) = ref_vy; % dXd2 を修正
                    
                    % 速度制限に合わせて、目標位置（Xd1, Xd2）も現在位置から破綻しないように修正
                    xd(1) = px + xd(5) * Param.dt; % Xd1 を修正
                    xd(2) = py + xd(6) * Param.dt; % Xd2 を修正
                    
                    % 高階微分（加速度ターゲット等）の目標値も、急激な躍度変化を防ぐために安全側に落とす
                    xd(9)  = 0; % d2Xd1
                    xd(10) = 0; % d2Xd2
                    xd(13) = 0; % d3Xd1
                    xd(14) = 0; % d3Xd2
                end
            end
        end
        % =================================================================
        % 階層型線形化による入力計算
        % 仮想入力のゲイン
        F1 = Param.F1; % z方向サブシステムのゲイン
        F2 = Param.F2; % x方向サブシステムのゲイン
        F3 = Param.F3; % y方向サブシステムのゲイン
        F4 = Param.F4; % yaw方向サブシステムのゲイン

        vf = obj.Vfd_SuspendedLoadxyDst(Param.dt, x, xd', F1); % 実験で刻み時間が変わったときに対応
        vs = obj.Vs_SuspendedLoadxyDst(x, xd', vf, P, F2, F3, F4); % 第二層x,y,yawサブシステムの仮想入力の計算
        uf = obj.Uf_SuspendedLoadxyDst(x, xd', vf, P); % 第一層の仮想入力の実入力(推力)への変換
        % h234  = obj.H234_SuspendedLoadxyDst(x,xd',vf,P);  % ただの単位行列なのでなくてもいい
        %tic
        beta2 = obj.Beta2_SuspendedLoadxyDst(x, xd', vf, P); % 第二層のbetaの逆行列
        vs_alpha2 = obj.V2_alpha2_SuspendedLoadxyDst(x, xd', vf, vs', P); % 第二層のvs - alpha
        us = beta2 \ vs_alpha2; % 第二層の実入力（roll,pitch,yawのトルク）への変換：bate^(-1)*(vs - alpha) %h234*invbeta2*a2;
        %obj.result.aa=toc;
        tmp = [uf(1); us]; % 実入力へ変換
        obj.result.tmp = tmp; % 入力に制限を付けてない値を格納

        % 安全のため入力値に制限を付ける．推定した牽引物質量や紐の長さ，外乱などを表示．
        %disp("time: "+ num2str(t,2)+" z position of drone:
        %"+num2str(model.state.p(3),3)+" estimated load mass:
        %"+num2str(P(6),4)+" dst:(x,y) "+num2str(P(end-1:end),4))
        obj.result.input = [max(0, min(20, tmp(1))); max(-1, min(1, tmp(2))); max(-1, min(1, tmp(3))); max(-1, min(1, tmp(4)))]; %+[normrnd(0,0.01,1);normrnd(0,0.001,[3,1])]*1; %入力にノイズを付与可能
        % obj.result.input = [0.0,0,0,(0.5236+0.04)*9.81]';%
        obj.result.xd = xd;
        obj.result.x = x;
        result = obj.result;

    end

    function show(obj)
        obj.result
    end
end

end