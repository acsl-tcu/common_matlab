classdef HLC_SUSPENDED_LOAD_AVOIDANCE < handle
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
        Param = obj.param; 
        model = obj.self.estimator.result; 
        ref = obj.self.reference.result;   
        
        if isprop(ref.state, 'xd')
            xd = ref.state.xd; 
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
        x = [model.state.getq('compact'); model.state.w; pL; model.state.vL; pT; model.state.wL]; 
        
        yaw = wrapToPi(model.state.q(3)); 
        yawd = xd(4); 
        yawUnit = [cos(yaw); sin(yaw); 0]; 
        yawdUnit = [cos(yawd); sin(yawd); 0]; 
        deltaYaw = sign(cross(yawdUnit, yawUnit)) * acos(yawdUnit' * yawUnit); 
        xd(4) = -deltaYaw(3) + yaw; 
        xd = [xd; zeros(28 - size(xd, 1), 1)]; 
        
        F1 = Param.F1; F2 = Param.F2; F3 = Param.F3; F4 = Param.F4; 
        
        % -----------------------------------------------------------------
        % ★【完全復元】フェーズ①：名目上の仮想入力（目標加速度）の計算
        % -----------------------------------------------------------------
        vf_nominal = obj.Vfd_SuspendedLoadxyDst(Param.dt, x, xd', F1); 
        vs_nominal = obj.Vs_SuspendedLoadxyDst(x, xd', vf_nominal, P, F2, F3, F4); 
        
        % -----------------------------------------------------------------
        % フェーズ②：安全制約（CBF）パラメータの準備
        % ★【完全環境同期】手入力を廃止し、ENVIRONMENT_OBSTACLE.m からダイレクト取得
        % -----------------------------------------------------------------
        obs_env = ENVIRONMENT_OBSTACLE();
        ox = obs_env(1).p_obs(1);
        oy = obs_env(1).p_obs(2);
        oz = obs_env(1).p_obs(3);
        ro = obs_env(1).r_obs;
        
        r_drone = 0.3; r_load = 0.2;  
        
        % 回避壁の感度ゲイン（危険圏でグワッと力強く横スライドさせるための黄金比）
        c1 = 2.0; c2 = 0.5; 
        d1 = 2.0; d2 = 0.5;
        
        cbfParam_load  = [ox; oy; oz; ro; r_load;  c1; c2; P(:)];
        cbfParam_drone = [ox; oy; oz; ro; r_drone; d1; d2; P(:)];
        
        u_nominal = [vs_nominal(1); vs_nominal(2); vf_nominal(1)]; 
        u_nominal = u_nominal(:); 
        
        % 1. シミュレータ本来のFlight Changeフラグ（varargin{2}）の生状態を取得
        is_flight_now = false;
        if length(varargin) >= 2
            if strcmp(varargin{2}, 'f') || string(varargin{2}) == "f"
                is_flight_now = true;
            end
        end
        
        % =================================================================
        % ★【数理修正の核心】
        % インデックスのズレ（x(8:10)）を完全に排除し、
        % コントローラ内の生の 3次元荷物位置「pL」を使って正確な距離を評価します。
        % =================================================================
        current_h_load_eval = 0.5 * ((pL(1) - ox)^2 + (pL(2) - oy)^2 + (pL(3) - oz)^2 - (ro + r_load)^2);
        
        % 毎ステップの状況をシミュレーションを止めずにコマンドウィンドウへ出力
        fprintf('--- MONITOR -> t: %.3f | is_flight: %d | h_eval: %.2f | ', xd(1), is_flight_now, current_h_load_eval);
        
        % 判定：Flight phase であり、かつ本物の余裕度が危険圏内（10m以内）に突入したとき
        if is_flight_now && (current_h_load_eval <= 10.0)
            
            t_current = 0.0; 
            [A_load_raw,  b_load_raw]  = CBF_Constraints_Load(obj,  x, xd', u_nominal, cbfParam_load,  t_current);
            [A_drone_raw, b_drone_raw] = CBF_Constraints_Drone(obj, x, xd', u_nominal, cbfParam_drone, t_current);
            
            % 現在の結合状態（これが quadprog に引き渡される生データです）
            A_qp = -[A_load_raw(1:3); A_drone_raw(1:3)];
            b_qp = -[b_load_raw(1); b_drone_raw(1)];
            
            % =================================================================
            % 🚨【最重要：オレンジ球侵入の瞬間を捉える完全フリーズデバッグ】
            % =================================================================
            % 荷物の位置がオレンジの球の内部（h_eval < 0：つまり突っ切っている状態）に
            % 1マスでも侵入した瞬間に、その裏の数理をコンソールに引きずり出します。
            if current_h_load_eval < 0
                fprintf('\n=========== 🚨 CBF PENETRATION CRITICAL LOG 🚨 ===========\n');
                fprintf('タイムステップ t: %.3f\n', xd(1));
                fprintf('本物の荷物余裕度 h_eval: %f (マイナス＝オレンジ球の内部に侵入中)\n', current_h_load_eval);
                
                % 1. 関数から返ってきた生の A と b
                disp('--- [関数から返ってきた生の制約（荷物）] ---');
                fprintf('A_load_raw (1x3): [%s]\n', num2str(A_load_raw(1:3)));
                fprintf('b_load_raw (スカラー): %f\n', b_load_raw(1));
                
                % 2. 現在コントローラが要求している名目加速度
                disp('--- [名目コントローラが要求している入力 u_nominal] ---');
                fprintf('u_nominal (X, Y, Z): [%s]\n', num2str(u_nominal'));
                
                % 3. クアッドプログに実際に渡る A_qp と b_qp の状態での評価
                % 通常、安全であるならば A_qp * u_nominal <= b_qp が成り立たなければならず、
                % 侵入しているなら「A_qp * u_nominal - b_qp > 0 (ルール違反)」となっていなければおかしい。
                val_load_qp = A_qp(1,:) * u_nominal - b_qp(1);
                disp('--- [QPに課している不等式制約の評価 (A_qp * u - b_qp)] ---');
                fprintf('荷物制約の評価値: %f\n', val_load_qp);
                
                % 4. なぜ突き抜けるのか？符号の関係をチェック
                % 生の式 (A_raw * u - b_raw) の状態での評価
                val_load_raw = A_load_raw(1:3) * u_nominal - b_load_raw(1);
                fprintf('生の数式の評価値 (A_raw * u - b_raw): %f\n', val_load_raw);
                fprintf('===========================================================\n\n');
                
                error('CBF_PENETRATION_STOP: オレンジ球内部への侵入を検知。上の数理ログを回収してください。');
            end
            % =================================================================
            
            fprintf('STATUS: [CBF-QP ON (本物の回避起動！)]\n');
            
            H_mat = eye(3);
            f_vec = -u_nominal;
            options = optimoptions('quadprog', 'Display', 'off');
            
            [u_safe, ~, exitflag] = quadprog(H_mat, f_vec, A_qp, b_qp, [], [], [], [], [], options);
            u_safe = u_safe(:);
            
            current_h_load = current_h_load_eval;
            p_drone_curr = pL + P(7) * pT; 
            current_h_drone = 0.5 * ((p_drone_curr(1) - ox)^2 + (p_drone_curr(2) - oy)^2 + (p_drone_curr(3) - oz)^2 - (ro + r_drone)^2);
        else
            fprintf('STATUS: [CBF OFF (安全直進中)]\n');
            
            u_safe = u_nominal;
            exitflag = 1;
            current_h_load = 999.0; current_h_drone = 999.0;
        end
        % =================================================================
        
        % -----------------------------------------------------------------
        % ［クリーン版］リアルタイム・デバッグログ
        % -----------------------------------------------------------------
        fprintf('    └─ h_load: %.2f | Correction -> X: %.4f, Y: %.4f, Z: %.4f\n', ...
            current_h_load, u_safe(1)-u_nominal(1), u_safe(2)-u_nominal(2), u_safe(3)-u_nominal(3));
        
        % -----------------------------------------------------------------
        % フェーズ④：安全加速度の格納 ＆ 高次微分の同調再合成
        % -----------------------------------------------------------------
        vs_safe = vs_nominal;
        vf_safe = vf_nominal;
        
        if exitflag == 1
            vs_safe(1) = u_safe(1); 
            vs_safe(2) = u_safe(2); 
            vf_safe(1) = u_safe(3); 
            
            % 高度加速度の減速比率に合わせて、1次〜5次微分(2〜6番目)を一斉強制シャッフル
            ratio = vf_safe(1) / (vf_nominal(1) + 1e-6);
            vf_safe(2:6) = vf_nominal(2:6) * ratio; 
        else
            warning('CBF-QP: 安全解不完全のため名目値で続行します。');
        end
        
        % -----------------------------------------------------------------
        % フェーズ⑤：既存の非線形相殺・実物理入力への最終コンバート
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
    function result = show(obj)
        obj.result
    end
end
end