classdef SUSPENDED_LOAD_REF_ADJUST < handle
    % Phase-aware modifier that offsets TIME_VARYING_REFERENCE outputs for suspended-load operations
    properties
        self
        td = 3 % 各種遷移時間
        takeoff_t0 % 離陸判定時の時刻
        landing_t0 % 接地判定時の時刻
        pL0 % pL の初期位置
        baseP % p-pLの初期配置
        cableL_L0 % landing phase移行時のケーブル長
        isGround = 1 % 接地しているか
    end

    methods

        function obj = SUSPENDED_LOAD_REF_ADJUST(self)
            obj.self = self;
        end

        function result = do(obj, varargin)
            % Override base reference positions in takeoff/landing to keep load aligned with drone frame
            time = varargin{1};
            cha = varargin{2};
            base = obj.self.reference;

            if isempty(base) || ~obj.has_field_or_prop(base, "result")
                error("SUSPENDED_LOAD_REF_ADJUST:MissingBase", "Base reference result is missing.");
            end
            base_result = base.result;
            if ~obj.has_field_or_prop(base_result, "state")
                error("SUSPENDED_LOAD_REF_ADJUST:MissingState", "Base reference state is missing.");
            end
            base_state = base_result.state;
            if ~obj.has_field_or_prop(base_state, "xd")
                error("SUSPENDED_LOAD_REF_ADJUST:MissingXd", "Base reference state.xd is missing.");
            end

            estContainer = obj.self.estimator;

            if isstruct(estContainer)
                hasResult = isfield(estContainer, "result");
            else
                hasResult = isprop(estContainer, "result");
            end

            if ~hasResult
                result = base.result;
                return
            end

            estResult = estContainer.result;

            if isstruct(estResult)
                hasState = isfield(estResult, "state");
            else
                hasState = isprop(estResult, "state");
            end

            if ~hasState
                result = base.result;
                return
            end

            model_state = estResult.state;

            if ~isprop(model_state, "p") || ~isprop(model_state, "pL")
                result = base.result;
                return
            end

            xd = base_state.xd;

            if numel(xd) < 3
                result = base.result;
                return
            end

            p = model_state.p;
            pL = model_state.pL;

            if isempty(obj.baseP) % 空回しで設定
                obj.baseP = p - pL; % 初期配置(dx,dy)を保存：着陸時その配置にする
            end

            if isempty(obj.pL0) % 空回しで設定
                obj.pL0 = pL; % 初期位置(x,y)保存：takeoffで利用
            end

            nxy = xd(1:3);
            L = obj.self.parameter.get("cableL");

            if cha == 't' || cha == '0' || cha == 'a' % 空回し含む
                if p(3) > obj.pL0(3) + L * 0.6 % 離陸判定
                    % 牽引物の初期高さ+0.6*ケーブル長より高度が上がったら
                    % 実際の牽引物位置に寄せていく
                    if obj.isGround % isempty(obj.tt0) %
                        obj.isGround = 0;
                        obj.takeoff_t0 = time.t; % 離陸時の時刻保存
                    end

                    tt = min((time.t - obj.takeoff_t0), obj.td);
                    k = tt/obj.td; %センサ値反映割合 [0,1]
                    nxy = p + k * (pL - p);
                    % p => pL へ変化
                    % z 方向は最後に更新する
                else %閾値を越えなかったら機体の真下に牽引物がいることにする
                    pL = p;
                    nxy = pL; %牽引物のreferenceのためpLにいるままになってしまうのでここで代入して下にいるようにする。
                end
                nxy(3) = xd(3) - L; % takeoff のリファレンスxdは機体位置を前提としているのでLをひく
            elseif cha == 'l'
                if isempty(obj.cableL_L0) % in landing phase
                    obj.cableL_L0 = norm(p - pL); % 飛行時のケーブル長
                end

                if p(3) < obj.cableL_L0 || obj.isGround == 1
                    if isempty(obj.landing_t0)
                        obj.landing_t0 = time.t;
                        obj.isGround = 1;
                    end
                    % 十分高度が下がったら有効になる
                    tt = min(time.t - obj.landing_t0, obj.td);
                    k = tt/obj.td;
                    nxy = nxy + k * obj.baseP; % 離陸前の相対関係を復元
                    nxy(3) = xd(3); % LANDING_REFERENCE : "zd",-L  が必要
                end
            else % flight phase
                nxy(3) = xd(3) - L; % flight のリファレンスxdは機体位置を前提としているのでLをひく
            end
            % [xd(1:3),nxy]
            xd(1:3) = nxy;
            base_result.state.xd = xd;
            base_result.state.p = xd(1:3);

            result = base_result;
        end

    end

    methods (Access = private)

        function tf = has_field_or_prop(~, target, name)
            if isempty(target)
                tf = false;
                return
            end
            if isstruct(target)
                tf = all(isfield(target, name));
            elseif isobject(target)
                tf = all(isprop(target, name));
            else
                tf = false;
            end
        end

    end

end
