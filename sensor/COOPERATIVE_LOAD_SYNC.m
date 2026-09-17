classdef COOPERATIVE_LOAD_SYNC < handle
% Simulation-only sensor to sync drone plant state from payload dynamics.
properties
    name = "loadsync";
    result
    self
    payload_index = []
    rho = []
    li = []
end

methods

    function obj = COOPERATIVE_LOAD_SYNC(self, opts)
        arguments
            self
            opts.payload_index = []
            opts.rho = []
            opts.li = []
        end
        obj.self = self;
        obj.payload_index = opts.payload_index;
        obj.rho = opts.rho;
        obj.li = opts.li;
        obj.result.state = STATE_CLASS(struct('state_list', ["p", "q", "pL", "pT"], 'num_list', [3, 3, 3, 3]));
        obj.result.output = zeros(12, 1);
        obj.result.y = obj.result.output;
    end

    function result = do(obj, varargin)
        agent_list = varargin{5};
        payload_id = obj.payload_index;
        if isempty(payload_id)
            payload_id = numel(agent_list);
        end
        if isempty(obj.rho) || isempty(obj.li)
            result = obj.result;
            return
        end

        payload = agent_list(payload_id);
        load = payload.plant.state;

        R_load = RodriguesQuaternion(load.Q);
        R_load = R_load(:, :, 1);
        dR_load = R_load * Skew(load.O);
        idx = obj.self.id;
        wi_load = load.wi(3 * idx - 2:3 * idx, 1);

        pL = load.p + R_load * obj.rho;
        vL = load.v + dR_load * obj.rho;
        pT = load.qi(3 * idx - 2:3 * idx, 1);
        dpT = Skew(wi_load) * pT;

        p = load.p + R_load * obj.rho - obj.li * pT;
        v = load.v + dR_load * obj.rho - obj.li * dpT;
        q = Quat2Eul(load.Qi(4 * idx - 3:4 * idx, 1));
        w = load.Oi(3 * idx - 2:3 * idx, 1);

        obj.self.plant.state.set_state("pL", pL, "vL", vL);
        obj.self.plant.state.set_state("pT", pT, "wL", wi_load);
        obj.self.plant.state.set_state("p", p, "v", v);
        obj.self.plant.state.set_state("q", q, "w", w);

        obj.result.state.set_state("p", p, "q", q, "pL", pL, "pT", pT);
        obj.result.output = [p; q; pL; pT];
        obj.result.y = obj.result.output;
        result = obj.result;
    end
end

end
