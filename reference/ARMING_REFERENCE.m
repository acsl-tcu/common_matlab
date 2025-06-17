classdef ARMING_REFERENCE < handle
  properties
    self
    dt
    result
    base_state
    th_offset_idle = 300;
  end

  methods
    function obj = ARMING_REFERENCE(self, varargin)
      obj.self = self;
      obj.result.state = STATE_CLASS(struct('state_list', ["xd", "p", "v"], 'num_list', [20, 3, 3]));
    end
    function result = do(obj, varargin)
      if isempty(obj.result.state.xd)
        obj.base_state = obj.self.estimator.result.state.p;
        obj.result.state.xd = obj.gen_ref_for_arming(obj.base_state);
        if isprop(obj.self.input_transform, "param")
          obj.self.input_transform.param.th_offset = obj.th_offset_idle;
        end
      end
      obj.result.state.p = obj.result.state.xd(1:3, 1);
      obj.result.state.v = obj.result.state.xd(5:7, 1);
      result = obj.result;
    end

  
    function Xd = gen_ref_for_arming(obj, hold_position)
      Xd = zeros(20, 1);
      Xd(1:3, 1) = hold_position;
    end
  end
end