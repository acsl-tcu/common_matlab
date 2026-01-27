classdef SWAY_REF_MOD < handle
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % SWAY_REF_MOD (with 1st-order lag smoothing + origin保存)
    %
    % 既存reference(origin)が生成した目標xd(t)を基本として使用し、
    % 揺れが顕在化したときのみ、目標の水平成分を微修正する割り込みモジュール。
    %
    % 【追従用(適用)】 : xd_cmd を base.result.state.xd に上書きして返す
    % 【比較用(保存)】 : 元の目標 xd_origin を res.xd_origin に保持（後処理で比較可能）
    %
    % 目標修正：
    %   p_ref_new_xy = p_ref_xy - c
    %
    % 理想補正量（次元整合済）：
    %   c* = alpha*(kv*Tv*v_xy + kr*r_xy)
    %
    % 一次遅れ（離散）：
    %   c <- (1-beta)*c + beta*c*
    %
    % betaはON/OFFで切替可能：
    %   ON時：速く立ち上げ (tau_on 小)
    %   OFF時：ゆっくり戻す (tau_off 大)
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    properties
        self
        param

        sway_on = 0               % 0:補正OFF, 1:補正ON（ヒステリシス）
        c = [0;0]                 % 実際に適用する補正ベクトル（水平2次元）

        sway_on_prev = 0          % 1ステップ前の割り込み状態（遷移検出用）
        last_print_time = -inf    % 表示間引き用
        last_xd_origin            % 元の目標（上書き前）
        last_xd_cmd               % 適用後目標（上書き後と同じ）
        last_c_xy                 % 補正量
        last_S                    % 揺れ指標
        last_h                    % CBF用のh
        last_danger               % dangerフラグ
        last_sway_on              % ON/OFF
    end

    methods
        function obj = SWAY_REF_MOD(self, param)
            obj.self  = self;
            obj.param = param;
        end

        function result = do(obj, varargin)
            % varargin: (time, cha, logger, env, agent_list, i, base_opt, ...)

            % ---- minimal param check (軽量) ----
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
            %    ※originを保存したいので、基本は self.reference.origin を読む
            %--------------------------------------------------------------
            if isstruct(obj.self.reference) && isfield(obj.self.reference,'origin')
                base = obj.self.reference.origin;   % 既存TIME_VARYING_REFERENCE
            else
                base = obj.self.reference;          % 単体構成
            end

            % originがまだ走っていない/xdが無い場合は何もしない
            try
                xd_origin = base.result.state.xd;   % 「上書き前の元目標」
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

            r  = pL - p;          % 相対位置（load - drone）
            vr = vL - v;          % 相対速度（load - drone）
            rxy  = r(1:2);
            vrxy = vr(1:2);

            %--------------------------------------------------------------
            % 3) 揺れ指標 S（水平相対運動）
            %    S = ||v_xy|| + sr*||r_xy||
            %--------------------------------------------------------------
            S = norm(vrxy) + obj.param.sr * norm(rxy);

            %--------------------------------------------------------------
            % 4) CBFゲート：水平距離による安全集合 h>=0
            %    rmax = L*sin(theta_max),  h = rmax^2 - ||rxy||^2
            %    注意：theta_maxが「度」で入っているなら sind() を使うこと
            %--------------------------------------------------------------
            L = obj.self.parameter.get("cableL");

            % --- theta_max の単位対策（paramにtheta_max_degがあれば優先） ---
            if isfield(obj.param,'theta_max_deg') && ~isempty(obj.param.theta_max_deg)
                rmax = L * sind(obj.param.theta_max_deg);
            else
                % theta_maxをラジアンとして扱う（従来互換）
                if ~isfield(obj.param,'theta_max') || isempty(obj.param.theta_max) || isnan(obj.param.theta_max)
                    obj.param.theta_max = deg2rad(15); % デフォルト15deg
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
            % 6) デバッグ表示（必要なら）
            %--------------------------------------------------------------
            t_now = varargin{1}.t;

            % OFF->ON
            if obj.sway_on == 1 && obj.sway_on_prev == 0
                fprintf('[SWAY_REF] ON  t=%.2f  S=%.3f  |vxy|=%.3f  |rxy|=%.3f  h=%.3f\n', ...
                    t_now, S, norm(vrxy), norm(rxy), h);
            end

            % ON->OFF
            if obj.sway_on == 0 && obj.sway_on_prev == 1
                fprintf('[SWAY_REF] OFF t=%.2f  S=%.3f  |vxy|=%.3f  |rxy|=%.3f  h=%.3f\n', ...
                    t_now, S, norm(vrxy), norm(rxy), h);
            end

            % ON中の定期表示
            if obj.sway_on == 1 && (t_now - obj.last_print_time) >= obj.param.print_interval
                fprintf('[SWAY_REF] ACT t=%.2f  S=%.3f  h=%.3f  danger=%d\n', ...
                    t_now, S, h, danger);
                obj.last_print_time = t_now;
            end

            % 1秒に1回くらいのDBG（S表示の丸めミスを修正）
            if mod(round(t_now/obj.param.dt),40)==0
                fprintf('[DBG] t=%.2f |vxy|=%.3f |rxy|=%.3f S=%.3f Son=%.3f Soff=%.3f sway_on=%d danger=%d\n', ...
                    t_now, norm(vrxy), norm(rxy), S, obj.param.S_on, obj.param.S_off, obj.sway_on, danger);
            end

            obj.sway_on_prev = obj.sway_on;

            %--------------------------------------------------------------
            % 7) alpha（CBFゲートによる補正強化倍率）
            %--------------------------------------------------------------
            if danger
                alpha = obj.param.gain_boost;
            else
                alpha = 1.0;
            end

            %--------------------------------------------------------------
            % 8) 理想補正量 c* と一次遅れ更新
            %    c* = alpha*(kv*Tv*v_xy + kr*r_xy)
            %--------------------------------------------------------------
            if obj.sway_on == 1
                c_star = alpha*(obj.param.kv * obj.param.Tv * vrxy + obj.param.kr * rxy);
            else
                c_star = [0;0];
            end

            dt = obj.param.dt;
            if obj.sway_on == 1
                tau = obj.param.tau_on;
            else
                tau = obj.param.tau_off;
            end
            beta = dt/(tau + dt);
            obj.c = (1-beta)*obj.c + beta*c_star;

            %--------------------------------------------------------------
            % 9) 目標へ適用（追従用xd_cmd）
            %    ※元の目標 xd_origin は保存しておく
            %--------------------------------------------------------------
            xd_cmd = xd_origin;
            xd_cmd(1:2) = xd_cmd(1:2) - obj.c;

            if numel(xd_cmd) >= 7 && obj.param.apply_to_vref
                xd_cmd(5:6) = xd_cmd(5:6) - obj.param.kv_vref * obj.c;
            end

            %--------------------------------------------------------------
            % 10) 戻り値：従来通り「xdを上書きして返す」 + 「元目標を別名保存」
            %    - controller/下流は result.state.xd を見て動く（従来互換）
            %    - 比較用に res.xd_origin, res.xd_cmd を保持
            %--------------------------------------------------------------
            res = base.result;        % 返却用（base.resultをベースにする）

            % 追従用（適用）: xd を上書き（従来どおり）
            res.state.xd = xd_cmd;

            % 下流がp,vを参照する場合の整合
            if isprop(res.state,'p')
                res.state.p = xd_cmd(1:3);
            end
            if isprop(res.state,'v') && numel(xd_cmd) >= 7
                res.state.v = xd_cmd(5:7);
            end

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
