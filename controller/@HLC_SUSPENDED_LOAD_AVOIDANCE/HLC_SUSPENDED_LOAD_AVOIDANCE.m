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
        % フェーズ①：名目上の仮想入力（目標加速度）の計算
        % -----------------------------------------------------------------
        vf_nominal = obj.Vfd_SuspendedLoadxyDst(Param.dt, x, xd', F1); 
        vs_nominal = obj.Vs_SuspendedLoadxyDst(x, xd', vf_nominal, P, F2, F3, F4); 
        
        % -----------------------------------------------------------------
        % フェーズ②：安全制約（CBF）パラメータの準備
        % -----------------------------------------------------------------
        ox = 0.0; oy = 14.0; oz = 1.5;    
        ro = sqrt(0.5^2 + (3.0/2)^2); 
        r_drone = 0.3; r_load = 0.2;  
        c1 = 2.0; c2 = 2.0; 
        d1 = 2.0; d2 = 2.0; 
        
        cbfParam_load  = [ox; oy; oz; ro; r_load;  c1; c2; P(:)];
        cbfParam_drone = [ox; oy; oz; ro; r_drone; d1; d2; P(:)];
        
        % QP用に入力名目値を [3x1] の縦ベクトルとしてパッキング
        u_nominal = [vs_nominal(1); vs_nominal(2); vf_nominal(1)]; 
        u_nominal = u_nominal(:); 
        
        % -----------------------------------------------------------------
        % フェーズ③：CBF関数の呼び出しとQPの実行
        % -----------------------------------------------------------------
        t_current = 0.0; 
        % 自動生成された関数へ、3次元の u_nominal をダイレクトに引き渡す
        [A_load,  b_load]  = CBF_Constraints_Load(obj,  x, xd', u_nominal, cbfParam_load,  t_current);
        [A_drone, b_drone] = CBF_Constraints_Drone(obj, x, xd', u_nominal, cbfParam_drone, t_current);
        
        % 不等式 A*u <= b の形式へガッチャンコ (2x3 行列 と 2x1 ベクトル)
        A_qp = -[A_load; A_drone];
        b_qp = -[b_load; b_drone];
        
        H_mat = eye(3);
        f_vec = -u_nominal;
        options = optimoptions('quadprog', 'Display', 'off');
        [u_safe, ~, exitflag] = quadprog(H_mat, f_vec, A_qp, b_qp, [], [], [], [], [], options);
        u_safe = u_safe(:); 
        
        % -----------------------------------------------------------------
        % ［クリーン版］リアルタイム・デバッグログ
        % -----------------------------------------------------------------
        current_h_load = 0.5 * ((x(8) - ox)^2 + (x(9) - oy)^2 + (x(10) - oz)^2 - (ro + r_load)^2);
        p_drone_curr = [x(8) + P(7)*x(14); x(9) + P(7)*x(15); x(10) + P(7)*x(16)];
        current_h_drone = 0.5 * ((p_drone_curr(1) - ox)^2 + (p_drone_curr(2) - oy)^2 + (p_drone_curr(3) - oz)^2 - (ro + r_drone)^2);
        
        fprintf('\n--- CBF-QP Debug Log --- \n');
        fprintf('h_load (荷物余裕): %f, h_drone (ドローン余裕): %f\n', current_h_load, current_h_drone);
        diff_u = u_safe - u_nominal;
        fprintf('Correction -> X: %f, Y: %f, Z: %f (ExitFlag: %d)\n', diff_u(1), diff_u(2), diff_u(3), exitflag);
        
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