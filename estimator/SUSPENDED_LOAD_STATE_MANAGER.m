classdef SUSPENDED_LOAD_STATE_MANAGER < handle
% Smoothly blends suspended-load states across phases so controller sees consistent pL/pT/mL
    properties
        self
        source = "ekf"
        result
        td = 5
        ratet
        tt0 = []
        isGround = 1
        pL0
        mLL
        cableLL
    end

    methods
        function obj = SUSPENDED_LOAD_STATE_MANAGER(self, opts)
            arguments
                self
                opts.source = "ekf"
            end
            obj.self = self;
            obj.source = opts.source;
            if ~isprop(obj.self.estimator,obj.source)
                error("SUSPENDED_LOAD_STATE_MANAGER:MissingSource", "Source class is missing.");
            end 
            obj.ratet = 1 / obj.td ^ 2;
            obj.result.state = [];
            
        end

        function result = do(obj, varargin)
            % EKF等の推定状態を基準にして、フェーズごとにpL/pT/mLを滑らかに補正する。
            % 目的:
            %  - 離陸/着陸時に吊り荷位置の接地に対応するため
            %  - controller へ常に整合したpL/pT/mLを提供する
            % 流れ:
            %  1) baseState から状態をコピー
            %  2) フェーズ(a/t/f/l)に応じてpLをブレンド
            %  3) 吊り荷接地時はpTを機体の真下方向で再計算
            %  4) mLも着地時は>0の範囲で減衰させる
            time = varargin{1};
            cha = varargin{2};
            baseState = obj.self.estimator.(obj.source).result.state;

            if isempty(baseState) || ~isprop(baseState, "pL")
                result = obj.result;
                return
            end
            state = state_copy(baseState);
            p = state.p;
            pL = state.pL;
            if isprop(state, "pT")
                pT = state.pT;
            else
                pT = [];
            end
            L = obj.self.parameter.get("cableL");
            mL = obj.get_and_set_mass(state);

            if isempty(obj.pL0)
                obj.pL0 = pL;
            end

            % 離陸/待機: 荷が地面にある間はpL=p。センサーを信用できるときだけ
            % pLをpからpLへ滑らかに遷移させる。
            if cha == 't' || cha == '0' || cha == 'a'
                if obj.can_use_sensor(mL, p, L)
                    if isempty(obj.tt0)
                        obj.isGround = 0;
                        obj.tt0 = time.t;
                    end
                    tt = min((time.t - obj.tt0), obj.td);
                    k = obj.ratet * tt ^ 2;
                    pL = p + k * (pL - p);
                else
                    pL = p;
                end
            % 着陸: ケーブル長と質量を記録し、接地検知後にpLをpへ戻す。
            % その際mLも同じ係数で減衰させ、地面接触後の変動を抑える。
            elseif cha == 'l'
                if isempty(obj.cableLL) || isempty(obj.mLL)
                    obj.cableLL = norm(p - pL);
                    obj.mLL = mL;
                    obj.tt0 = time.t;
                end
                if p(3) - pL(3) < obj.cableLL * 0.9 || obj.isGround == 1
                    obj.isGround = 1;
                    tt = min(time.t - obj.tt0, obj.td);
                    k = obj.ratet * tt ^ 2;
                    pL = p + (1 - k) * (pL - p);
                    obj.mLL = (1 - k) * obj.mLL;
                    mL = min(mL, obj.mLL);
                end
            end

            % 非飛行フェーズではpLのzをケーブル長で拘束し、pTを再計算。
            if cha ~= 'f'
                pL(3) = p(3) - L;
                delta = pL - p;
                if norm(delta) > 1e-9
                    pT = delta / norm(delta);
                end
            end

            state.set_state("pL", pL);
            if ~isempty(pT) && isprop(state, "pT")
                state.set_state("pT", pT);
            end
            if isprop(state, "mL")
                state.set_state("mL", max(0, mL));
            end
            obj.result.state = state;
            result = obj.result;
        end
    end

    methods (Access = private)

        function tf = can_use_sensor(obj, mL, p, L)
            if isempty(obj.pL0)
                tf = false;
                return
            end
            tf = (mL > 0.1) || (p(3) > obj.pL0(3) + L * 0.9);
        end

        function mL = get_and_set_mass(obj, state)
            % return mL 
            % if mL is estimated then it assigns to self.parameter
            if isprop(state, "mL")
                mL = max(0, state.mL);
                obj.self.parameter.set("loadmass",mL);
            else
                mL = obj.self.parameter.get("loadmass");
            end
        end
    end
end
