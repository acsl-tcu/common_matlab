classdef BEZIER_REFERENCE < handle
  properties
    self
    dt
    result   
    T_total = 5.0;
    P1 = [0.25, 0.25, 0.3];
    P2 = [0.45, 0.45, 0.2];
    P3 = [0.55, 0.55, 0.1];
    P4 = [0.6,  0.6,  0.0];
    ref_generator = [];
    count = 0;
  end

  methods
    function obj = BEZIER_REFERENCE(self, varargin)
      obj.self = self;
      obj.result.state = STATE_CLASS(struct('state_list', ["xd", "p", "v"], 'num_list', [20, 3, 3]));
      obj.dt = varargin{2}.dt;
    end

    function result = do(obj, varargin)
        t = varargin{1};
        if (obj.count < 2) 
             t.ts = t.t;
        obj.count = obj.count+1;
        end
        t.ts = t.t;
       P0 = varargin{1, 5}.estimator.result.state.p ;
        obj.ref_generator= bezier_curve(obj,P0,t);
        obj.result.state.xd = obj.ref_generator(t.t); % 目標重心位置（絶対座標）
        obj.result.state.p = obj.result.state.xd(1:3);
        if length(obj.result.state.xd)>4
            obj.result.state.v = obj.result.state.xd(5:7);
        else
            obj.result.state.v = [0;0;0];
        end
        
        result = obj.result;
    end

    function ref = bezier_curve(obj, p0, time)
      syms t real
       tt = t-time.ts;
      tau = tt/obj.T_total;
                          
      P0_vec = p0(:)';
      Path_sym = (1 - tau)^4 * P0_vec + ...
                 4 * (1 - tau)^3 * tau * obj.P1 + ...
                 6 * (1 - tau)^2 * tau^2 * obj.P2 + ...
                 4 * (1 - tau) * tau^3 * obj.P3 + ...
                 tau^4 * obj.P4;
                 
      Vel_sym = diff(Path_sym, t);
      Acc_sym = diff(Vel_sym, t);
      Xd_sym = sym(zeros(20, 1));
      Xd_sym(1:3)   = Path_sym.';
      Xd_sym(5:7)   = Vel_sym.';
      Xd_sym(9:11)  = Acc_sym.';
      
      ref = matlabFunction(Xd_sym, 'Vars', {t});
    end
  end
end