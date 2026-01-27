classdef SUSPENDED_LOAD_STATE_MANAGER < handle
% Smoothly blends suspended-load states across phases so controller sees consistent pL/pT/mL
    properties
        self
        result
        td = 10 % 各種遷移時間
        takeoff_t0 % 離陸判定時の時刻
        landing_t0 % 接地判定時の時刻
        pL0 % pL の初期位置
        baseP % p-pLの初期配置
        cableL_L0 % landing phase移行時のケーブル長
        mL_L0 % landing phase移行時の牽引物質量
        isGround = 1 % 接地しているか
    end

    methods
        function obj = SUSPENDED_LOAD_STATE_MANAGER(self,opts)
            arguments
                self
                opts.td = 5
            end
            obj.self = self;
            obj.td = opts.td;
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
            baseState = obj.self.estimator.result.state;

            if isempty(baseState) || ~isprop(baseState, "pL")
                error("SUSPENDED_LOAD_STATE_MANAGER:Missing_pL", "Base estimator pL is missing.");
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
            mL = obj.get_mass(state);
            if isempty(obj.pL0)
                obj.pL0 = pL;
            end

            % % 離陸/待機: 荷が地面にある間はpLはpの真下。
            % % センサーを信用できるときだけpLをpからpLへ滑らかに遷移させる。
            if cha == 't' || cha == '0' || cha == 'a'
            %     % if p(3) > obj.pL0(3) + L * 0.8 % 離陸判定
            %     if mL > 0.02 % 離陸判定
            %         if obj.isGround
                        obj.isGround = 0;
            %             obj.takeoff_t0 = time.t;
            %         end
            %         tt = min((time.t - obj.takeoff_t0), obj.td);
            %         k = tt/obj.td;
            %         pL = p + k * (pL - p);
            %     else
            %         obj.isGround = 1;
            %         pL = p;
            %     end
            %     pL(3) = p(3) - L;
            %     pT = [0;0;-1];

            % 着陸: ケーブル長と質量を記録し、接地検知後にpLをpへ戻す。
            % その際mLも同じ係数で減衰させ、地面接触後の変動を抑える。
            elseif cha == 'l'
                if isempty(obj.cableL_L0) % in landing phase
                    obj.cableL_L0 = norm(p - pL);
                    obj.mL_L0 = mL;
                end
                if p(3) < 1.2*obj.cableL_L0 || obj.isGround == 1
                    if isempty(obj.landing_t0)
                        obj.landing_t0 = time.t;
                        obj.isGround = 1;
                    end
                    tt = min(time.t - obj.landing_t0, obj.td);
                    k = tt/obj.td;
                    pL = [p(1:2);p(3)-L] + (1 - k) * (pL - [p(1:2);p(3)-L]); % pL : pL => pの真下
                    delta = pL - p;
                    pT = delta / norm(delta);
                    tmL = (1 - k) * obj.mL_L0; % mL : mL_L0 => 0
                    mL = min(mL, tmL);
                end
            end

            state.set_state("pL", pL);
            state.set_state("pT", pT);
            if isprop(state, "mL")
                state.set_state("mL", max(0, mL));
                obj.self.parameter.set("loadmass",mL);
            end
            [obj.self.estimator.ekf.result.state.get';state.get']
            obj.self.estimator.ekf.result.state.set_state(state);
            obj.result.state = state;
            result = obj.result;
        end
    end

    methods (Access = private)

        function mL = get_mass(obj, state)
            % return mL 
            % if mL is estimated then it assigns to self.parameter
            if isprop(state, "mL")
                mL = max(0, state.mL);
            else
                mL = obj.self.parameter.get("loadmass");
            end
        end
    end
end
