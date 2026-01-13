classdef SWAY_REF_MOD < handle
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % SWAY_REF_INTERRUPT
    %
    % 【役割】
    %   既存の reference(origin) が生成した目標 xd(t) を基本として使用し，
    %   牽引物の揺れ（ドローンに対する相対運動）が顕在化したときだけ，
    %   目標の水平成分を微修正する「割り込み」モジュール．
    %
    % 【特徴】
    %   - 既存の変数名・構造（base.result.state.xd など）を壊さない
    %   - 修正しないときは完全に元のxdを通す（追従性能を維持）
    %   - ヒステリシスでON/OFFを安定化（チャタリング防止）
    %
    % 【今回追加：CBFゲート（安全判定）】
    %   角度θが直接取れない場合でも，ケーブル長Lと許容角θ_maxから
    %     ||r_xy|| <= L*sin(θ_max)
    %   を「揺れ角制約の代理」とみなし，安全集合
    %     h = r_max^2 - ||r_xy||^2 >= 0
    %   を評価する．
    %   実装としては：
    %     h が小さい（境界付近/危険域）ときにのみ
    %     相対運動ベースの目標修正を「強める／強制ONする」
    %   という “安全ゲート” として用いる．
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    properties
        self
        param
        sway_on = 0   % 0:介入OFF, 1:介入ON（ヒステリシスで更新）
    end

    methods
        function obj = SWAY_REF_MOD(self, param)
            obj.self = self;
            obj.param = param;
        end

        function result = do(obj, varargin)
            % varargin: (time, cha, logger, env, agent, i) が来る想定
            % origin（既存）が先に走って reference を更新している前提

            %--- base(reference origin) の取得（コンテナ構造に対応） ---
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

            %--- 既存が生成した目標 xd（形式は絶対に変えない） ---
            xd = base.result.state.xd;

            %--- 推定状態から相対運動量を計算 ---
            model = obj.self.estimator.result;
            p  = model.state.p;   v  = model.state.v;
            pL = model.state.pL;  vL = model.state.vL;

            r  = pL - p;   % 相対位置（load - drone）
            vr = vL - v;   % 相対速度（load - drone）

            %--- 揺れ指標 S（水平成分のみ）---
            % 相対速度ベースが主、相対位置は補助（折り返し点での誤判定を減らす）
            S = norm(vr(1:2)) + obj.param.sr * norm(r(1:2));

            %--- CBFゲート用：安全集合 h = r_max^2 - ||r_xy||^2 ---
            % 角度θが取れないため，水平距離で揺れ角制約を代理する
            L = obj.self.parameter.get("cableL");
            rmax = L * sin(obj.param.theta_max);
            rxy = r(1:2);
            h = rmax^2 - (rxy.'*rxy);  % h>=0:安全, h<0:超過

            % 危険域フラグ（境界付近）：
            % hが小さいときだけ「安全ゲート」をONにする
            danger = (h < obj.param.h_gate);

            %--- ヒステリシスで「揺れ顕在化」の判定 ---
            % ※CBFゲート(danger)がONなら，揺れ判定を強める（強制ONも可）
            if obj.sway_on == 0
                % OFF -> ON
                if (S > obj.param.S_on) || (danger && obj.param.force_on_when_danger)
                    obj.sway_on = 1;
                end
            else
                % ON -> OFF
                % 危険域にいる間はOFFにしない（安全側）
                if (S < obj.param.S_off) && ~(danger && obj.param.hold_on_when_danger)
                    obj.sway_on = 0;
                end
            end

            %--- 介入ONのときだけ目標を微修正 ---
            if obj.sway_on == 1
                % 危険域なら修正を強める（＝CBFを安全ゲートとして使用）
                if danger
                    gain_scale = obj.param.gain_boost;  % 例: 2.0
                else
                    gain_scale = 1.0;
                end

                % 目標位置（水平）を、相対運動が減る方向へ微修正
                % -kv*vr は「相対速度の減衰」
                % -kr*r  は「大きく離れているときに戻す」補助項（小さめ推奨）
                xd(1:2) = xd(1:2) ...
                          - gain_scale*obj.param.kv * vr(1:2) ...
                          - gain_scale*obj.param.kr * r(1:2);

                % xdに速度目標(5:7)が含まれる場合のみ，同様に微修正（任意）
                if numel(xd) >= 7
                    xd(5:6) = xd(5:6) ...
                              - gain_scale*obj.param.kdv * vr(1:2) ...
                              - gain_scale*obj.param.kdr * r(1:2);
                end

                %--- 割り込み上書き：xd（必要ならp,vも整合） ---
                base.result.state.xd = xd;

                % loggerや下流が p,v を参照する場合に整合を取る（変数名は維持）
                if isprop(base.result.state,'p')
                    base.result.state.p = xd(1:3);
                end
                if isprop(base.result.state,'v') && numel(xd) >= 7
                    base.result.state.v = xd(5:7);
                end
            end

            %--- 形式維持のため，必ずoriginのresultを返す ---
            result = base.result;
        end
    end
end
