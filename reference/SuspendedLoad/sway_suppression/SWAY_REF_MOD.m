classdef SWAY_REF_MOD < handle
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % SWAY_REF_MOD
    %  - reference(origin)のxdを読み、揺れ時に水平成分を補正して返す
    %  - 重要: 戻り値 result.state を「xdプロパティを持つオブジェクト」に包む
    %          => HLC_SUSPENDED_LOAD の isprop(ref.state,'xd') を必ずtrueにする
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    properties
        self
        param

        sway_on = 0
        c = [0;0]

        sway_on_prev = 0
        last_print_time = -inf

        last_xd_origin
        last_xd_cmd
        last_c_xy
        last_S
        last_h
        last_danger
        last_sway_on
        on_time = -inf
    end

    methods
        function obj = SWAY_REF_MOD(self, param)
            obj.self  = self;
            obj.param = param;
        end

        function result = do(obj, varargin)
            % varargin: (time, cha, logger, env, agent_list, i, ...)

            % ---- minimal param check ----
            if ~isfield(obj.param,'dt');   error('SwayRefMod_Param: dt is missing'); end
            if ~isfield(obj.param,'S_on') || ~isfield(obj.param,'S_off')
                error('SwayRefMod_Param: S_on/S_off is missing');
            end
            if obj.param.S_off >= obj.param.S_on
                warning('SwayRefMod_Param: S_off >= S_on. Forcing S_off = 0.8*S_on');
                obj.param.S_off = 0.8*obj.param.S_on;
            end

            %--------------------------------------------------------------
            % 1) base(reference origin) の取得
            %--------------------------------------------------------------
            if isstruct(obj.self.reference) && isfield(obj.self.reference,'origin')
                base = obj.self.reference.origin;
            else
                base = obj.self.reference;
            end

            % originがまだ走っていない/xdが無い場合は何もしない
            try
                xd_origin = base.result.state.xd;
            catch
                result = base.result;
                return;
            end

            %--------------------------------------------------------------
            % 2) 推定状態から相対運動量を計算
            %--------------------------------------------------------------
            model = obj.self.estimator.result;
            p  = model.state.p;   v  = model.state.v;
            pL = model.state.pL;  vL = model.state.vL;

            r  = pL - p;
            vr = vL - v;
            rxy  = r(1:2);
            vrxy = vr(1:2);

            %--------------------------------------------------------------
            % 3) 揺れ指標 S
            %--------------------------------------------------------------
            S = norm(vrxy) + obj.param.sr * norm(rxy);

            %--------------------------------------------------------------
            % 4) CBFゲート（水平距離）
            %--------------------------------------------------------------
            L = obj.self.parameter.get("cableL");

            if isfield(obj.param,'theta_max_deg') && ~isempty(obj.param.theta_max_deg)
                rmax = L * sind(obj.param.theta_max_deg);
            else
                if ~isfield(obj.param,'theta_max') || isempty(obj.param.theta_max) || isnan(obj.param.theta_max)
                    obj.param.theta_max = deg2rad(15);
                end
                rmax = L * sin(obj.param.theta_max);
            end

            h = rmax^2 - (rxy.'*rxy);
            danger = (h < obj.param.h_gate);

            %--------------------------------------------------------------
            % 5) ヒステリシスで補正ON/OFF
            %--------------------------------------------------------------
            if obj.sway_on == 0
                if (S > obj.param.S_on) || (danger && obj.param.force_on_when_danger)
                    obj.sway_on = 1;
                end
            else
                if (S < obj.param.S_off) && ~(danger && obj.param.hold_on_when_danger)
                    obj.sway_on = 0;
                end
            end

            %--------------------------------------------------------------
            % 6) デバッグ表示
            %--------------------------------------------------------------
            t_now = varargin{1}.t;

            if obj.sway_on == 1 && obj.sway_on_prev == 0
                fprintf('[SWAY_REF] ON  t=%.2f  S=%.3f  |vxy|=%.3f  |rxy|=%.3f  h=%.3f\n', ...
                    t_now, S, norm(vrxy), norm(rxy), h);
            end
            if obj.sway_on == 0 && obj.sway_on_prev == 1
                fprintf('[SWAY_REF] OFF t=%.2f  S=%.3f  |vxy|=%.3f  |rxy|=%.3f  h=%.3f\n', ...
                    t_now, S, norm(vrxy), norm(rxy), h);
            end
            if obj.sway_on == 1 && (t_now - obj.last_print_time) >= obj.param.print_interval
                fprintf('[SWAY_REF] ACT t=%.2f  S=%.3f  h=%.3f  danger=%d\n', ...
                    t_now, S, h, danger);
                obj.last_print_time = t_now;
            end

            obj.sway_on_prev = obj.sway_on;

            %--------------------------------------------------------------
            % 7) alpha
            %--------------------------------------------------------------
            if danger
                alpha = obj.param.gain_boost;
            else
                alpha = 1.0;
            end

            %--------------------------------------------------------------
            % 8) 理想補正量 c* と一次遅れ更新
            %--------------------------------------------------------------
            if obj.sway_on == 1
                c_star = alpha*(obj.param.kv * obj.param.Tv * vrxy + obj.param.kr * rxy);
            else
                c_star = [0;0];
            end

            dt = obj.param.dt;
            if obj.sway_on == 1
                tau = obj.param.tau_on;

                % --- ON直後のソフトスタート ---
                if isfield(obj.param,'softstart_sec') && obj.param.softstart_sec > 0
                    if obj.sway_on_prev == 0
                        obj.on_time = t_now;   % ← propertiesに on_time を追加
                    end
                    if isfield(obj,'on_time') && (t_now - obj.on_time) < obj.param.softstart_sec
                        if isfield(obj.param,'tau_on_soft') && obj.param.tau_on_soft > tau
                            tau = obj.param.tau_on_soft;
                        end
                    end
                end

            else
                tau = obj.param.tau_off;
            end
beta = dt/(tau + dt);
obj.c = (1-beta)*obj.c + beta*c_star;

            % ---- 補正量の上限（追従不能な目標を作らない）----
            if isfield(obj.param,'c_max') && ~isempty(obj.param.c_max) && obj.param.c_max > 0
                cn = norm(obj.c);
                if cn > obj.param.c_max
                    obj.c = obj.c * (obj.param.c_max / cn);
                end
            end


            %--------------------------------------------------------------
            % 9) 目標へ適用（xd_cmd）
            %--------------------------------------------------------------
            xd_cmd = xd_origin;
            xd_cmd(1:2) = xd_cmd(1:2) - obj.c;

            if numel(xd_cmd) >= 7 && obj.param.apply_to_vref
                xd_cmd(5:6) = xd_cmd(5:6) - obj.param.kv_vref * obj.c;
            end

            %--------------------------------------------------------------
            % 10) 戻り値：result.state を「xdプロパティ付き」にする
            %     -> HLC の isprop(ref.state,'xd') を必ず通す
            %--------------------------------------------------------------
            res = base.result;

            % (A) res.state がオブジェクトで xd を持つなら、そのまま上書き
            if isobject(res.state) && isprop(res.state,'xd')
                res.state.xd = xd_cmd;

                % p,v も持っていれば整合
                if isprop(res.state,'p')
                    res.state.p = xd_cmd(1:3);
                end
                if isprop(res.state,'v') && numel(xd_cmd) >= 7
                    res.state.v = xd_cmd(5:7);
                end

            % (B) res.state が struct などなら、wrapperに包む
            else
                st = SwayStateWrap(res.state); % 元stateをコピーして保持
                st.xd = xd_cmd;
                st.p  = xd_cmd(1:3);

                if numel(xd_cmd) >= 7
                    st.v = xd_cmd(5:7);
                else
                    st.v = [];
                end

                res.state = st;
            end

            % 比較用（ログ）
            obj.last_xd_origin = xd_origin;
            obj.last_xd_cmd    = xd_cmd;
            obj.last_c_xy      = obj.c;
            obj.last_S         = S;
            obj.last_h         = h;
            obj.last_danger    = danger;
            obj.last_sway_on   = obj.sway_on;

            result = res;
        end
    end
end