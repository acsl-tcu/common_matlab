classdef SWAY_REF_MOD < handle
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % SWAY_REF_MOD (with 1st-order lag smoothing)
    %
    % 既存reference(origin)が生成した目標xd(t)を基本として使用し、
    % 揺れが顕在化したときのみ、目標の水平成分を微修正する割り込みモジュール。
    %
    % ★今回追加：補正ベクトルcに一次遅れを入れて、目標のジャンプを抑制する。
    %
    % 目標修正：
    %   p_ref_new_xy = p_ref_xy - c
    %
    % 理想補正量：
    %   c* = alpha*(kv*v_xy + kr*r_xy)
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

        sway_on = 0         % 0:補正OFF, 1:補正ON（ヒステリシス）
        c = [0;0]           % 実際に適用する補正ベクトル（水平2次元）
    end

    methods
        function obj = SWAY_REF_MOD(self, param)
            obj.self = self;
            obj.param = param;
        end

        function result = do(obj, varargin)
            % varargin: (time, cha, logger, env, agent, i) が来る想定
            % origin（既存）が先に走って reference を更新している前提

            %---- base(reference origin) の取得（コンテナ構造に対応） ----
            if isstruct(obj.self.reference) && isfield(obj.self.reference,'origin')
                base = obj.self.reference.origin;   % 既存TIME_VARYING_REFERENCE
            else
                base = obj.self.reference;          % 単体構成
            end

            % originがまだ走っていない/xdが無い場合は何もしない（落とさない）
            if ~isprop(base,'result') || ~isprop(base.result,'state') || ~isprop(base.result.state,'xd')
                result = base.result;
                return;
            end

            %---- 既存が生成した目標xd（形式は絶対に変えない） ----
            xd = base.result.state.xd;

            %---- 推定状態から相対運動量を計算 ----
            model = obj.self.estimator.result;
            p  = model.state.p;   v  = model.state.v;
            pL = model.state.pL;  vL = model.state.vL;

            r  = pL - p;          % 相対位置（load - drone）
            vr = vL - v;          % 相対速度（load - drone）
            rxy  = r(1:2);
            vrxy = vr(1:2);

            %---- 揺れ指標 S（水平相対運動）----
            % S = ||v_xy|| + sr*||r_xy||
            S = norm(vrxy) + obj.param.sr * norm(rxy);

            %---- CBFゲート：水平距離による安全集合 h>=0 ----
            % rmax = L*sin(theta_max),  h = rmax^2 - ||rxy||^2
            L = obj.self.parameter.get("cableL");
            rmax = L * sin(obj.param.theta_max);
            h = rmax^2 - (rxy.'*rxy);
            danger = (h < obj.param.h_gate);

            %---- ヒステリシスで補正ON/OFF ----
            if obj.sway_on == 0
                % OFF->ON
                if (S > obj.param.S_on) || (danger && obj.param.force_on_when_danger)
                    obj.sway_on = 1;
                end
            else
                % ON->OFF（dangerならOFFに戻さない設定も可）
                if (S < obj.param.S_off) && ~(danger && obj.param.hold_on_when_danger)
                    obj.sway_on = 0;
                end
            end

            %---- alpha（CBFゲートによる補正強化倍率）----
            if danger
                alpha = obj.param.gain_boost;   % 例：2.0
            else
                alpha = 1.0;
            end

            %---- 理想補正量 c* を計算 ----
            % 補正ONなら c* = alpha*(kv*v_xy + kr*r_xy)
            % 補正OFFなら c* = 0（cを0へ戻す）
            if obj.sway_on == 1
                c_star = alpha*(obj.param.kv * vrxy + obj.param.kr * rxy);
            else
                c_star = [0;0];
            end

            %---- 一次遅れ（離散）で c を更新 ----
            dt = obj.param.dt; % 明示的にparamから取得（TIMEのdtでもOK）
            if obj.sway_on == 1
                tau = obj.param.tau_on;   % ON時：素早く
            else
                tau = obj.param.tau_off;  % OFF時：ゆっくり戻す
            end
            % beta = dt/(tau+dt)  (0<beta<1)
            beta = dt/(tau + dt);

            % c <- (1-beta)*c + beta*c*
            obj.c = (1-beta)*obj.c + beta*c_star;

            %---- 目標へ適用（水平位置のみ）----
            % p_ref_new_xy = p_ref_xy - c
            xd(1:2) = xd(1:2) - obj.c;

            % 速度目標がある場合、整合を取っておく（任意）
            if numel(xd) >= 7 && obj.param.apply_to_vref
                % vrefも滑らかにしたい場合： vref <- vref - c_dot 相当
                % ここでは簡易に、c_star（またはc）を使って弱く反映させる
                xd(5:6) = xd(5:6) - obj.param.kv_vref * obj.c;
            end

            %---- 上書き（既存構造を維持）----
            base.result.state.xd = xd;

            % loggerや下流がp,vを参照する場合に整合を取る（変数名は維持）
            if isprop(base.result.state,'p')
                base.result.state.p = xd(1:3);
            end
            if isprop(base.result.state,'v') && numel(xd) >= 7
                base.result.state.v = xd(5:7);
            end

            %---- 形式維持：必ずoriginのresultを返す ----
            result = base.result;
        end
    end
end
