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
            obj.ratet = 1 / obj.td ^ 2;
            obj.result.state = [];
        end

        function result = do(obj, varargin)
            % Take EKF output, apply phase-dependent blending, and publish back via estimator result
            time = varargin{1};
            cha = varargin{2};
            baseState = obj.get_base_state();
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
            mL = obj.get_mass(state);

            if isempty(obj.pL0)
                obj.pL0 = pL;
            end

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
        function state = get_base_state(obj)
            est = obj.self.estimator;
            if isstruct(est)
                if isfield(est, obj.source)
                    state = est.(obj.source).result.state;
                    return
                end
            elseif isprop(est, obj.source)
                state = est.(obj.source).result.state;
                return
            end
            if isstruct(est) && isfield(est, "result")
                state = est.result.state;
            elseif isprop(est, "result")
                state = est.result.state;
            else
                state = [];
            end
        end

        function tf = can_use_sensor(obj, mL, p, L)
            if isempty(obj.pL0)
                tf = false;
                return
            end
            tf = (mL > 0.1) || (p(3) > obj.pL0(3) + L * 0.9);
        end

        function mL = get_mass(obj, state)
            if isprop(state, "mL")
                mL = max(0, state.mL);
            else
                mL = obj.self.parameter.get("loadmass");
            end
        end
    end
end
