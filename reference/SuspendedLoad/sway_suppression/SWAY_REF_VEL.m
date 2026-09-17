classdef SWAY_REF_VEL < handle
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % SWAY_REF_VEL_CBF
    %  - 相対水平速度 vr_xy に基づく揺れ減衰（速度版）
    %  - CBF（角度制約）をゲートとして導入
    %  - 目標位置は変更しない
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    properties
        self
        param

        sway_on = 0
        sway_on_prev = 0
        on_time = -inf

        % LPF
        vrxy_f = [0;0]
    end

    methods
        function obj = SWAY_REF_VEL(self, param)
            obj.self  = self;
            obj.param = param;
        end

        function result = do(obj, varargin)
            % (time, cha, logger, env, agent_list, i)
            time = varargin{1};
            t = time.t;

            %----------------------------------------------------------
            % base reference
            %----------------------------------------------------------
            if isstruct(obj.self.reference) && isfield(obj.self.reference,'origin')
                base = obj.self.reference.origin;
            else
                base = obj.self.reference;
            end

            try
                xd = base.result.state.xd;
            catch
                result = base.result;
                return;
            end

            %----------------------------------------------------------
            % 推定状態
            %----------------------------------------------------------
            model = obj.self.estimator.result;
            p  = model.state.p;
            v  = model.state.v;
            pL = model.state.pL;
            vL = model.state.vL;

            rxy  = (pL - p);  rxy = rxy(1:2);
            vrxy = (vL - v);  vrxy = vrxy(1:2);

            %----------------------------------------------------------
            % LPF（実機重要）
            %----------------------------------------------------------
            dt = obj.param.dt;
            beta = dt / (obj.param.vr_lpf_tau + dt);
            obj.vrxy_f = (1-beta)*obj.vrxy_f + beta*vrxy;
            vr = obj.vrxy_f;
            vr_norm = norm(vr);

            %----------------------------------------------------------
            % CBF: 角度制約を水平距離で代理
            %----------------------------------------------------------
            L = obj.self.parameter.get("cableL");

            if isfield(obj.param,'theta_max_deg')
                rmax = L * sind(obj.param.theta_max_deg);
            else
                rmax = L * sin(obj.param.theta_max);
            end

            h = rmax^2 - (rxy.'*rxy);
            danger = (h < obj.param.h_gate);

            %----------------------------------------------------------
            % ON / OFF 判定（CBF込み）
            %----------------------------------------------------------
            if obj.sway_on == 0
                if (vr_norm > obj.param.vr_on) || danger
                    obj.sway_on = 1;
                    obj.on_time = t;
                end
            else
                hold_by_time = (t - obj.on_time) < obj.param.min_on_time;
                if ~hold_by_time && ~danger && vr_norm < obj.param.vr_off
                    obj.sway_on = 0;
                end
            end

            obj.sway_on_prev = obj.sway_on;

            %----------------------------------------------------------
            % 速度目標への減衰注入（CBF時は強化）
            %----------------------------------------------------------
            xd_cmd = xd;

            if obj.sway_on == 1 && numel(xd_cmd) >= 7
                kv_eff = obj.param.kv;
                if danger
                    kv_eff = kv_eff * obj.param.gain_boost;
                end

                v_damp = - kv_eff * vr;

                % 上限（実機必須）
                vmax = obj.param.v_damp_max;
                nv = norm(v_damp);
                if nv > vmax
                    v_damp = v_damp * (vmax / nv);
                end

                xd_cmd(5:6) = xd_cmd(5:6) + v_damp;
            end

            %----------------------------------------------------------
            % return
            %----------------------------------------------------------
            res = base.result;

            if isobject(res.state) && isprop(res.state,'xd')
                res.state.xd = xd_cmd;
            else
                st = res.state;
                st.xd = xd_cmd;
                res.state = st;
            end

            result = res;
        end
    end
end
