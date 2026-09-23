classdef HLC_SUSPENDED_LOAD_DATAACQUISITION < handle
% クアッドコプター用階層型線形化を使った入力算出
properties
    self
    result
    param
end

methods

    function obj = HLC_SUSPENDED_LOAD_DATAACQUISITION(self, param)
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
        L_cable = obj.self.parameter.cableL;
        P = [obj.self.parameter.get(["mass", "jx", "jy", "jz", "gravity", "loadmass", "cableL"]), 0, 0];
        x = [model.state.getq('compact'); model.state.w; pL; model.state.vL; pT; model.state.wL]; % [q, w ,pL, vL, pT, wL]に並べ替え
        
        yaw = wrapToPi(model.state.q(3)); % 機体yaw角[-pi,pi]にする
        yawd = xd(4); % 目標yaw角
        yawUnit = [cos(yaw); sin(yaw); 0];
        yawdUnit = [cos(yawd); sin(yawd); 0];
        deltaYaw = sign(cross(yawdUnit, yawUnit)) * acos(yawdUnit' * yawUnit);
        xd(4) = -deltaYaw(3) + yaw;
        
        % 目標値の格納
        xd = [xd; zeros(28 - size(xd, 1), 1)];
        tic
        
        % 階層型線形化による入力計算
        F1 = Param.F1;
        F2 = Param.F2;
        F3 = Param.F3;
        F4 = Param.F4;
        vf = obj.Vfd_SuspendedLoadxyDst(Param.dt, x, xd', F1);
        vs = obj.Vs_SuspendedLoadxyDst(x, xd', vf, P, F2, F3, F4);
        uf = obj.Uf_SuspendedLoadxyDst(x, xd', vf, P);
        beta2 = obj.Beta2_SuspendedLoadxyDst(x, xd', vf, P);
        vs_alpha2 = obj.V2_alpha2_SuspendedLoadxyDst(x, xd', vf, vs', P);
        us = beta2 \ vs_alpha2;
        
        tmp = [uf(1); us]; % 実入力へ変換
        obj.result.tmp = tmp;
        obj.result.controllertime = toc;
        
        % 入力制限
        obj.result.input = [max(0, min(20, tmp(1))); ...
                            max(-1, min(1, tmp(2))); ...
                            max(-1, min(1, tmp(3))); ...
                            max(-1, min(1, tmp(4)))];
        
        % =====================================================================
        % 追従誤差および索揺動幅のリアルタイム記録 (xd = 荷物目標位置)
        % =====================================================================
        p_uav_cur = model.state.p;
        pQ_ref    = xd(1:3) + [0; 0; L_cable];      % ドローン名目位置 (荷物の長さL上方)
        pL_nom    = p_uav_cur - [0; 0; L_cable];    % 荷物名目下垂位置 (ドローン直下 -Z 方向)

        obj.result.err_track_drone = norm(p_uav_cur - pQ_ref); % ドローン追従誤差
        obj.result.err_track_load  = norm(pL - xd(1:3));       % 荷物目標追従誤差
        obj.result.err_swing       = norm(pL - pL_nom);        % 索・荷物の揺動変位
        
        obj.result.xd = xd;
        obj.result.x = x;
        result = obj.result;
    end
    function show(obj)
        obj.result
    end
end
end