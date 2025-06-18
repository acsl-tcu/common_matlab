classdef BEZIER_REFERENCE < handle
  properties
    self
    dt
    result   
    T_total = 20.0;
    P1 = [0.25, 0.25, 0.5];
    P2 = [0.35, 0.35, 0.5];
    P3 = [0.5, 0.5, 0.3];
    P4 = [0.6,  0.6, 0.0];
    ref_generator = [];
    count = 0;
    count2 = 0;
    Pz_vec = 0;
    tss
    P0
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
           obj.tss = t.t;
        obj.count = obj.count+1;
        end
        % obj.tss = t.t;
        if isempty(obj.P0)
            obj.P0 = obj.self.estimator.result.state.p;
        else
            obj.P0 = [obj.self.estimator.result.state.p(1:2);0.6];  
        end
        obj.ref_generator= bezier_curve(obj,obj.P0);
        obj.result.state.xd = obj.ref_generator(t.t); % 目標重心位置（絶対座標）
        obj.result.state.p = obj.result.state.xd(1:3);
        if length(obj.result.state.xd)>4
            obj.result.state.v = obj.result.state.xd(5:7);
        else
            obj.result.state.v = [0;0;0];
        end
        
        result = obj.result;
    end

    function ref = bezier_curve(obj,p0)
      syms t_local real
       % tt = min(max(t - obj.tss, 0), 5);
      tau = t_local/obj.T_total;
      % if obj.count2 < 2 
      %   obj.Pz_vec=  p0(:)';
      %   obj.count = obj.count+1;
      % end
      P0_vec = p0(:)';
      % P0_vec =obj.Pz_vec;
      Path_sym = (1 - tau)^4 * P0_vec + ...
                 4 * (1 - tau)^3 * tau * obj.P1 + ...
                 6 * (1 - tau)^2 * tau^2 * obj.P2 + ...
                 4 * (1 - tau) * tau^3 * obj.P3 + ...
                 tau^4 * obj.P4;
                 
      Vel_sym = diff(Path_sym, t_local);
      Acc_sym = diff(Vel_sym, t_local);
      Xd_sym = sym(zeros(20, 1));
      Xd_sym(1:3)   = Path_sym.';
      Xd_sym(5:7)   = Vel_sym.';
      Xd_sym(9:11)  = Acc_sym.';
       calc_handle = matlabFunction(Xd_sym, 'Vars', {t_local});
        ref = @(t_global) calc_handle(min(max(t_global - obj.tss, 0), obj.T_total));
    end
  end
end