classdef SWAY_REF_MOD < handle
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % SWAY_REF_MOD (real-flight robust)
    %  - vrxy LPF (phase/noise robustness)
    %  - S smoothing (movmean-like IIR)
    %  - min_on_time + softstart + c_max
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

        on_time = -inf     % ONになった時刻（min_on_time/softstart用）

        % --- filters (for real flight) ---
        vrxy_f = [0;0]     % LPF state for vrxy
        S_f = 0            % smoothed S for switching


        % ---- time series logs (for plotting) ----
        t_log = [];
        xd_origin_log = [];   % N×20
        xd_cmd_log    = [];   % N×20
        dxd_xy_log    = [];   % N×2
        c_xy_log      = [];   % N×2
        S_log         = [];   % N×1     (raw S)
        S_use_log     = [];   % N×1     (smoothed S used for switching)
        h_log         = [];   % N×1
        danger_log    = [];   % N×1
        sway_on_log   = [];   % N×1
        rxy_log = [];         % N×2
        vrxy_log = [];        % N×2 (raw)
        vrxy_f_log = [];      % N×2 (filtered)
        theta_log = [];       % N×1  [rad]
        Evr_log = NaN(0,1);   % N×1  cumulative ∫||vrxy||^2 dt
        violate_log = [];     % N×1
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

            % time
            t_now = varargin{1}.t;
            dt = obj.param.dt;

            %--------------------------------------------------------------
            % 1) base(reference origin) の取得
            %--------------------------------------------------------------
            if isstruct(obj.self.reference) && isfield(obj.self.reference,'origin')
                base = obj.self.reference.origin;
            else
                base = obj.self.reference;
            end

            try
                xd_origin = base.result.state.xd;
            catch
                result = base.result;
                return;
            end
            % base = obj.self.reference;           % ← origin ではなく reference 本体
            % xd_origin = base.result.state.xd;

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
            % 2.5) vrxy LPF（実機の位相/ノイズ対策）
            %  vrxy_f <- (1-a)*vrxy_f + a*vrxy
            %  a = dt/(tau + dt)
            %--------------------------------------------------------------
            if ~isfield(obj.param,'vr_lpf_tau') || isempty(obj.param.vr_lpf_tau)
                obj.param.vr_lpf_tau = 0.05; % default 50ms (light)
            end
            tau_v = max(0, obj.param.vr_lpf_tau);
            a_v = dt/(tau_v + dt);
            obj.vrxy_f = (1-a_v)*obj.vrxy_f + a_v*vrxy;
            vrxy_use = obj.vrxy_f;   % ←以降はこのvrxy_useを使う

            %------------------------------------------------------------------------------------------------------------------------
            % 4)CBFゲート（設定したケーブル長を使わない版）
            %   - theta is computed from relative position only:
            %       theta = atan2(||r_xy||, -r_z)
            %   - "barrier" h_theta = theta_max - theta  (>=0 is safe)
            %   - danger if h_theta < theta_gate  (theta_gate is a margin in [rad])
            %--------------------------------------------------------------

            % ---- theta_max [rad] ----
            if isfield(obj.param,'theta_max_deg') && ~isempty(obj.param.theta_max_deg)
                theta_max = deg2rad(obj.param.theta_max_deg);
            else
                if ~isfield(obj.param,'theta_max') || isempty(obj.param.theta_max) || isnan(obj.param.theta_max)
                    obj.param.theta_max = deg2rad(15);
                end
                theta_max = obj.param.theta_max;
            end

            % ---- theta from geometry (no L) ----
            rz = r(3);                                % r = pL - p (already computed)
            rxy_n = norm(rxy);
            theta = atan2(rxy_n, max(1e-6, -rz));     % [rad], assumes load is below drone (rz<0)

            % ---- barrier in angle domain ----
            h = theta_max - theta;                    % >=0 safe, <0 violated

            % ---- gate threshold (margin) ----
            % old code used p.h_gate in "m^2". Here we use [rad] margin.
            % If you already use p.h_gate elsewhere, keep it but interpret as "theta_gate [rad]".
            if ~isfield(obj.param,'theta_gate') || isempty(obj.param.theta_gate)
                % default: 2 deg margin (danger when within 2deg of theta_max)
                obj.param.theta_gate = deg2rad(2.0);
            end

            danger = (h < obj.param.theta_gate);

            %--------------------------------------------------------------
            % 3) 角度＋相対速度の指標（raw） + 平滑化（表示用/ノイズ対策）
            %   ※判定自体は theta/vr のしきい値で行う（下）
            %--------------------------------------------------------------
            vr_mag = norm(vrxy_use);

            % theta は既に計算している想定（あなたの 4) のところ）
            % theta = atan2(norm(rxy), max(1e-6, -rz));

            % 「S」はログ/デバッグ表示のためだけに作る（任意）
            % 正規化しておくと便利
            S_raw = max(theta / max(1e-6, obj.param.theta_on), ...
                vr_mag / max(1e-6, obj.param.vr_on));

            % 平滑（既存のIIRでOK）
            tau_S = max(0, obj.param.S_smooth_tau);
            a_S = dt/(tau_S + dt);
            if isempty(obj.S_f) || isnan(obj.S_f)
                obj.S_f = S_raw;
            else
                obj.S_f = (1-a_S)*obj.S_f + a_S*S_raw;
            end
            S_use = obj.S_f;

            %--------------------------------------------------------------
            % 5) ヒステリシス ON/OFF（theta と vr で判定）
            %    ON : theta>theta_on OR vr>vr_on OR (danger強制)
            %    OFF: theta<theta_off AND vr<vr_off （dangerなら保持も可）
            %--------------------------------------------------------------
            use_theta_vr = isfield(obj.param,'use_theta_vr_switch') && obj.param.use_theta_vr_switch;

            if use_theta_vr
                on_cond  = (theta > obj.param.theta_on)  || (vr_mag > obj.param.vr_on);
                off_cond = (theta < obj.param.theta_off) && (vr_mag < obj.param.vr_off);
            else
                % 従来S方式にフォールバック
                on_cond  = (S_use > obj.param.S_on);
                off_cond = (S_use < obj.param.S_off);
            end

            if obj.sway_on == 0
                if on_cond || (danger && obj.param.force_on_when_danger)
                    obj.sway_on = 1;
                    obj.on_time = t_now;
                end
            else
                hold_by_time = false;
                if isfield(obj.param,'min_on_time') && obj.param.min_on_time > 0
                    hold_by_time = (t_now - obj.on_time) < obj.param.min_on_time;
                end

                if ~hold_by_time
                    if off_cond && ~(danger && obj.param.hold_on_when_danger)
                        obj.sway_on = 0;
                    end
                end
            end%---------------------------------------------------------------------------------------------------------------------

            % %-----------------------------------------------------------------------------------------------------------------------------
            % % 3) 揺れ指標 S（raw） + 平滑化して判定に使用
            % %   S_raw = ||vrxy|| + sr*||rxy||
            % %   S_f   = IIRで平滑（movmean相当）
            % %--------------------------------------------------------------
            % S_raw = norm(vrxy_use) + obj.param.sr * norm(rxy);
            % 
            % if ~isfield(obj.param,'S_smooth_tau') || isempty(obj.param.S_smooth_tau)
            %     obj.param.S_smooth_tau = 0.15; % default 150ms
            % end
            % tau_S = max(0, obj.param.S_smooth_tau);
            % a_S = dt/(tau_S + dt);
            % 
            % if isempty(obj.S_f) || isnan(obj.S_f)
            %     obj.S_f = S_raw;
            % else
            %     obj.S_f = (1-a_S)*obj.S_f + a_S*S_raw;
            % end
            % S_use = obj.S_f;   % ←ON/OFF判定はこれを使う
            % 
            % %--------------------------------------------------------------
            % % 4) CBFゲート（水平距離）(ケーブルL使う版)
            % %--------------------------------------------------------------
            % % L = obj.self.parameter.get("cableL");
            % % 
            % % if isfield(obj.param,'theta_max_deg') && ~isempty(obj.param.theta_max_deg)
            % %     rmax = L * sind(obj.param.theta_max_deg);
            % %     theta_max = deg2rad(obj.param.theta_max_deg);
            % % else
            % %     if ~isfield(obj.param,'theta_max') || isempty(obj.param.theta_max) || isnan(obj.param.theta_max)
            % %         obj.param.theta_max = deg2rad(15);
            % %     end
            % %     rmax = L * sin(obj.param.theta_max);
            % %     theta_max = obj.param.theta_max;
            % % end
            % % 
            % % h = rmax^2 - (rxy.'*rxy);
            % % danger = (h < obj.param.h_gate);
            % 
            % %--------------------------------------------------------------
            % % 4)CBFゲート（設定したケーブル長を使わない版）
            % %   - theta is computed from relative position only:
            % %       theta = atan2(||r_xy||, -r_z)
            % %   - "barrier" h_theta = theta_max - theta  (>=0 is safe)
            % %   - danger if h_theta < theta_gate  (theta_gate is a margin in [rad])
            % %--------------------------------------------------------------
            % 
            % % ---- theta_max [rad] ----
            % if isfield(obj.param,'theta_max_deg') && ~isempty(obj.param.theta_max_deg)
            %     theta_max = deg2rad(obj.param.theta_max_deg);
            % else
            %     if ~isfield(obj.param,'theta_max') || isempty(obj.param.theta_max) || isnan(obj.param.theta_max)
            %         obj.param.theta_max = deg2rad(15);
            %     end
            %     theta_max = obj.param.theta_max;
            % end
            % 
            % % ---- theta from geometry (no L) ----
            % rz = r(3);                                % r = pL - p (already computed)
            % rxy_n = norm(rxy);
            % theta = atan2(rxy_n, max(1e-6, -rz));     % [rad], assumes load is below drone (rz<0)
            % 
            % % ---- barrier in angle domain ----
            % h = theta_max - theta;                    % >=0 safe, <0 violated
            % 
            % % ---- gate threshold (margin) ----
            % % old code used p.h_gate in "m^2". Here we use [rad] margin.
            % % If you already use p.h_gate elsewhere, keep it but interpret as "theta_gate [rad]".
            % if ~isfield(obj.param,'theta_gate') || isempty(obj.param.theta_gate)
            %     % default: 2 deg margin (danger when within 2deg of theta_max)
            %     obj.param.theta_gate = deg2rad(2.0);
            % end
            % 
            % danger = (h < obj.param.theta_gate);
            % 
            % 
            % %--------------------------------------------------------------
            % % 5) ヒステリシスで補正ON/OFF（min_on_time付き）
            % %   ※判定は S_use（平滑版）で行う
            % %--------------------------------------------------------------
            % if obj.sway_on == 0
            %     if (S_use > obj.param.S_on) || (danger && obj.param.force_on_when_danger)
            %         obj.sway_on = 1;
            %         obj.on_time = t_now;   % ON時刻保存
            %     end
            % else
            %     hold_by_time = false;
            %     if isfield(obj.param,'min_on_time') && obj.param.min_on_time > 0
            %         hold_by_time = (t_now - obj.on_time) < obj.param.min_on_time;
            %     end
            % 
            %     if ~hold_by_time
            %         if (S_use < obj.param.S_off) && ~(danger && obj.param.hold_on_when_danger)
            %             obj.sway_on = 0;
            %         end
            %     end
            % end%------------------------------------------------------------------------------------------------------------------------
            %--------------------------------------------------------------
            % 7) alpha（dangerで強化） ※softstart無しに戻す
            %--------------------------------------------------------------
            alpha = 1.0;
            if danger
                alpha = obj.param.gain_boost;
            end

            %--------------------------------------------------------------
            % 6) デバッグ表示（S_useも出す）
            %--------------------------------------------------------------
            if obj.sway_on == 1 && obj.sway_on_prev == 0
                fprintf('[SWAY] ON  t=%.2f  theta=%.2fdeg  |vr|=%.3f  Suse=%.2f  danger=%d\n', ...
                    t_now, rad2deg(theta), vr_mag, S_use, danger);
            end
            if obj.sway_on == 0 && obj.sway_on_prev == 1
                fprintf('[SWAY] OFF t=%.2f  theta=%.2fdeg  |vr|=%.3f  Suse=%.2f  danger=%d\n', ...
                    t_now, rad2deg(theta), vr_mag, S_use, danger);
            end
            if obj.sway_on == 1 && (t_now - obj.last_print_time) >= obj.param.print_interval
                fprintf('[SWAY] ACT t=%.2f  theta=%.2fdeg  |vr|=%.3f  alpha=%.2f  danger=%d\n', ...
                    t_now, rad2deg(theta), vr_mag, alpha, danger);
                obj.last_print_time = t_now;
            end


            %--------------------------------------------------------------
            % 7) alpha（dangerで強化）
            %--------------------------------------------------------------
            % if danger
            %     alpha = obj.param.gain_boost;
            % else
            %     alpha = 1.0;
            % end

            %--------------------------------------------------------------
            % 8) 理想補正量 c* と一次遅れ更新
            %   ※c*の速度項には vrxy_use（LPF後）を使う
            %--------------------------------------------------------------
            % if obj.sway_on == 1
            %     c_star = alpha*(obj.param.kv * obj.param.Tv * vrxy_use + obj.param.kr * rxy);
            % else
            %     c_star = [0;0];
            % end
            % 
            % % --- tau selection + softstart ---
            % if obj.sway_on == 1
            %     tau = obj.param.tau_on;
            % 
            %     % ソフトスタート：ON直後は tau を大きくしてゆっくり入れる
            %     if isfield(obj.param,'softstart_sec') && obj.param.softstart_sec > 0 ...
            %             && isfield(obj.param,'tau_on_soft') && obj.param.tau_on_soft > 0
            %         if (t_now - obj.on_time) < obj.param.softstart_sec
            %             tau = max(tau, obj.param.tau_on_soft);
            %         end
            %     end
            % else
            %     tau = obj.param.tau_off;
            % end
            % 
            % beta = dt/(tau + dt);
            % obj.c = (1-beta)*obj.c + beta*c_star;
            % 
            % % ---- 補正量の上限 ----
            % if isfield(obj.param,'c_max') && ~isempty(obj.param.c_max) && obj.param.c_max > 0
            %     cn = norm(obj.c);
            %     if cn > obj.param.c_max
            %         obj.c = obj.c * (obj.param.c_max / cn);
            %     end
            % end

            %--------------------------------------------------------------
            % 9) 目標へ適用（xd_cmd）
            %--------------------------------------------------------------
            % xd_cmd = xd_origin;
            % xd_cmd(1:2) = xd_cmd(1:2) + obj.c;
            %--------------------------------------------------------------
            % 9-a) 速度目標へ適用：v_ref' = v_ref - k_vref * vrxy_use
            %--------------------------------------------------------------
            
            %--------------------------------------------------------------
            % 9) 目標へ適用（xd_cmd）
            %--------------------------------------------------------------
            xd_cmd = xd_origin;

            %--------------------------------------------------------------
            % 9-a) 速度目標へ適用（速度補正のみ）
            %   v_ref' = v_ref + dv
            %   dv = alpha * kv_vref * vrxy_use   （符号は「+」が効いたので + を採用）
            %--------------------------------------------------------------
            if numel(xd_cmd) >= 7 && isfield(obj.param,'apply_to_vref') && obj.param.apply_to_vref

                if ~isfield(obj.param,'kv_vref') || isempty(obj.param.kv_vref)
                    obj.param.kv_vref = 0.6;   % 0.3〜1.2で調整
                end
                if ~isfield(obj.param,'dv_max') || isempty(obj.param.dv_max)
                    obj.param.dv_max = 0.3;    % 0.2〜0.6で調整
                end
                if ~isfield(obj.param,'vref_sign') || isempty(obj.param.vref_sign)
                    obj.param.vref_sign = +1;  % ★あなたの結果に合わせて + をデフォルト
                end

                if obj.sway_on == 1
                    dv = alpha * obj.param.kv_vref * vrxy_use;
                else
                    dv = [0;0];
                end

                % 上限
                if obj.param.dv_max > 0
                    ndv = norm(dv);
                    if ndv > obj.param.dv_max
                        dv = dv * (obj.param.dv_max / ndv);
                    end
                end

                xd_cmd(5:6) = xd_cmd(5:6) + obj.param.vref_sign * dv;
            end





            % --- during sway suppression, neutralize higher-order feedforward (recommended) ---
            if obj.sway_on == 1
                % accel, jerk, snap, 5th deriv (xy) を 0 に
                if numel(xd_cmd) >= 10
                    xd_cmd(9:10) = 0;      % d2Xd1, d2Xd2
                end
                if numel(xd_cmd) >= 14
                    xd_cmd(13:14) = 0;     % d3Xd1, d3Xd2
                end
                if numel(xd_cmd) >= 18
                    xd_cmd(17:18) = 0;     % d4Xd1, d4Xd2
                end
                if numel(xd_cmd) >= 22
                    xd_cmd(21:22) = 0;     % d5Xd1, d5Xd2
                end
            end

            %--------------------------------------------------------------
            % 9-b) 速度目標への適用（CBF-QP: 上限保証重視）（今回は使用しない）
            %   - v_ref を「外向きに h を減らす」方向へは出さない
            %   - 2D halfspace projection（quadprog不要）
            %--------------------------------------------------------------
            % if numel(xd_cmd) >= 7 && isfield(obj.param,'apply_to_vref') && obj.param.apply_to_vref
            % 
            %     % --- defaults ---
            %     if ~isfield(obj.param,'use_cbf_qp');   obj.param.use_cbf_qp = true; end
            %     if ~isfield(obj.param,'gamma_cbf');    obj.param.gamma_cbf = 3.0; end
            %     if ~isfield(obj.param,'eps_cbf');      obj.param.eps_cbf   = 1e-6; end
            % 
            %     if ~isfield(obj.param,'kv_vref');      obj.param.kv_vref   = 0.3; end  % まずは小さめ
            %     if ~isfield(obj.param,'vref_max');     obj.param.vref_max  = 2.0; end  % 必要なら
            %     if ~isfield(obj.param,'v_damp_max');   obj.param.v_damp_max = 0.8; end % 注入上限
            % 
            %     vref_xy = xd_cmd(5:6);                % current reference velocity
            %     % --- nominal damping (using c as a direction) ---
            %     % いまの実装に合わせて「c に比例した速度補正」を使う（形を崩さない）
            %     vnom = vref_xy - obj.param.kv_vref * obj.c;
            % 
            %     % clamp nominal vref magnitude
            %     if obj.param.vref_max > 0
            %         nv = norm(vnom);
            %         if nv > obj.param.vref_max
            %             vnom = vnom * (obj.param.vref_max / nv);
            %         end
            %     end
            % 
            %     vproj = vnom;
            % 
            %     if obj.param.use_cbf_qp
            %         % ここで使う rxy,vL は、上の推定値（model.state）から取る
            %         % すでに計算した rxy, vL があるのでそれを使う想定
            %         vL_xy = vL(1:2);
            % 
            %         % すでに計算した h を使う（上で定義済み）
            %         % constraint: a' v_ref >= b
            %         a = 2 * rxy;  % 2x1
            %         b = 2 * (rxy.' * vL_xy) - obj.param.gamma_cbf * h;
            % 
            %         if (a.'*a) > obj.param.eps_cbf
            %             if (a.' * vproj) < b
            %                 vproj = vproj + ((b - a.'*vproj) / (a.'*a)) * a;
            %             end
            %         end
            %     end
            % 
            %     % limit injected delta (avoid big step -> phase issues)
            %     dv = vproj - vref_xy;
            %     if obj.param.v_damp_max > 0
            %         ndv = norm(dv);
            %         if ndv > obj.param.v_damp_max
            %             dv = dv * (obj.param.v_damp_max / ndv);
            %         end
            %     end
            % 
            %     xd_cmd(5:6) = vref_xy + dv;
            % end


            %--------------------------------------------------------------
            % 10) 戻り値：result.state を「xdプロパティ付き」にする
            %--------------------------------------------------------------
            res = base.result;

            if isobject(res.state) && isprop(res.state,'xd')
                res.state.xd = xd_cmd;

                if isprop(res.state,'p')
                    res.state.p = xd_cmd(1:3);
                end
                if isprop(res.state,'v') && numel(xd_cmd) >= 7
                    res.state.v = xd_cmd(5:7);
                end
            else
                st = SwayStateWrap(res.state);
                st.xd = xd_cmd;
                st.p  = xd_cmd(1:3);
                if numel(xd_cmd) >= 7
                    st.v = xd_cmd(5:7);
                else
                    st.v = [];
                end
                res.state = st;
            end

            %--------------------------------------------------------------
            % 11) last values
            %--------------------------------------------------------------
            obj.last_xd_origin = xd_origin;
            obj.last_xd_cmd    = xd_cmd;
            obj.last_c_xy      = obj.c;
            obj.last_S         = S_use;
            obj.last_h         = h;
            obj.last_danger    = danger;
            obj.last_sway_on   = obj.sway_on;

            % sway_on_prev は最後に更新（ソフトスタート判定には使わない）
            obj.sway_on_prev = obj.sway_on;

            %--------------------------------------------------------------
            % (LOG) 時系列保存（プロット用）
            %--------------------------------------------------------------
            k = round(t_now/dt) + 1;

            % 20次元に揃える（不足があれば0埋め）
            xd0 = xd_origin(:);
            xdc = xd_cmd(:);
            if numel(xd0) < 20;  xd0(end+1:20,1) = 0; end
            if numel(xdc) < 20; xdc(end+1:20,1) = 0; end

            if size(obj.xd_origin_log,1) < k
                obj.t_log(k,1) = NaN;
                obj.xd_origin_log(k,20) = NaN;
                obj.xd_cmd_log(k,20)    = NaN;
                obj.dxd_xy_log(k,2)     = NaN;
                obj.c_xy_log(k,2)       = NaN;
                obj.S_log(k,1)          = NaN;
                obj.S_use_log(k,1)      = NaN;
                obj.h_log(k,1)          = NaN;
                obj.danger_log(k,1)     = NaN;
                obj.sway_on_log(k,1)    = NaN;
                obj.rxy_log(k,2)        = NaN;
                obj.vrxy_log(k,2)       = NaN;
                obj.vrxy_f_log(k,2)     = NaN;
                obj.theta_log(k,1)      = NaN;
                obj.Evr_log(k,1)        = NaN;
                obj.violate_log(k,1)    = NaN;
            end

            obj.t_log(k,1) = t_now;
            obj.xd_origin_log(k,:) = xd0(1:20).';
            obj.xd_cmd_log(k,:)    = xdc(1:20).';

            obj.dxd_xy_log(k,:) = (xdc(1:2) - xd0(1:2)).';
            obj.c_xy_log(k,:)   = obj.c(:).';
            obj.S_log(k,1)      = S_raw;
            obj.S_use_log(k,1)  = S_use;
            obj.h_log(k,1)      = h;
            obj.danger_log(k,1) = double(danger);
            obj.sway_on_log(k,1)= double(obj.sway_on);

            % theta, violation
            rxy_n = norm(rxy);
            % ratio = min(1.0, max(0.0, rxy_n / max(L,1e-9)));（ケーブルL使う版）
            % theta = asin(ratio);
            theta = atan2(rxy_n, max(1e-6, -r(3)));%（ケーブルL使わない版）


            obj.rxy_log(k,:)     = rxy(:).';
            obj.vrxy_log(k,:)    = vrxy(:).';
            obj.vrxy_f_log(k,:)  = vrxy_use(:).';
            obj.theta_log(k,1)   = theta;
            obj.violate_log(k,1) = double(theta > theta_max);

            % energy cumulative (safe)
            vr2 = (vrxy_use.'*vrxy_use);
            if k <= 1 || size(obj.Evr_log,1) < k-1 || isnan(obj.Evr_log(k-1))
                Evr = 0;
            else
                Evr = obj.Evr_log(k-1) + vr2 * dt;
            end
            obj.Evr_log(k,1) = Evr;

            result = res;
        end
    end
end