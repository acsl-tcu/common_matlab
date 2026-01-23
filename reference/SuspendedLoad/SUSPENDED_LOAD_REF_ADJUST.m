classdef SUSPENDED_LOAD_REF_ADJUST < handle
% Phase-aware modifier that offsets TIME_VARYING_REFERENCE outputs for suspended-load operations
properties
    self
    base_field = "timevarying"
    td = 5
    ratet
    tt0 = []
    pL0
    baseP
    cableLL
    isLanding = 0
end

methods

    function obj = SUSPENDED_LOAD_REF_ADJUST(self, opts)

        arguments
            self
            opts.base = "timevarying"
        end

        obj.self = self;
        obj.base_field = opts.base;
        obj.ratet = 1 / obj.td ^ 2;
    end

    function result = do(obj, varargin)
        % Override base reference positions in takeoff/landing to keep load aligned with drone frame
        time = varargin{1};
        cha = varargin{2};
        base = obj.get_base_reference();

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
        mL = obj.get_mass(model_state);

        if cha == 't' || cha == '0' || cha == 'a'

            if ~(obj.can_use_sensor(mL, p, L))
                nxy = pL; % これだと trueになったときにリファレンスが飛ぶ
            end
            nxy(3) = xd(3) - L;           
        elseif cha == 'l'

            if isempty(obj.cableLL)
                obj.cableLL = norm(p - pL);
                obj.tt0 = time.t;
                obj.isLanding = 0;
            end

            if (~isempty(obj.cableLL) && (p(3) < obj.cableLL * 0.9)) || obj.isLanding == 1
                obj.isLanding = 1;
                tt = min(time.t - obj.tt0, obj.td);
                k = obj.ratet * tt ^ 2;

                if ~isempty(obj.baseP)
                    nxy = nxy + k * obj.baseP;
                end
                nxy(3) = xd(3);
            end
        else % flight phase
            nxy(3) = xd(3) - L;         
        end
        % [xd(1:3),nxy]
        xd(1:3) = nxy;
        base_result.state.xd = xd;
        base_result.state.p = xd(1:3);

        result = base_result;
    end

end

methods (Access = private)

    function base = get_base_reference(obj)
        ref = obj.self.reference;

        if isstruct(ref)

            if isfield(ref, obj.base_field)
                base = ref.(obj.base_field);
            else
                f = fieldnames(ref);
                base = ref.(f{1});
            end

        else
            base = ref;
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
