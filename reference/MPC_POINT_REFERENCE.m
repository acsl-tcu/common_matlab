classdef MPC_POINT_REFERENCE < handle
    properties
        param
        self
        cha
        fns
        length_fns
        ref_t
        fref_t
        t0
        i
        result
        func
        is_first_run
    end
    methods
        function obj = MPC_POINT_REFERENCE(self,varargin)
            obj.self = self;
            var=varargin{1};
            
            % 1. Parse Time
            if length(var) ==2
                obj.ref_t=var{2};
                obj.fref_t = (var{2}>=5 ) ;
            else
                obj.fref_t =false;
            end
            
            % 2. Parse Waypoints
            obj.param=var{1};
            obj.fns = fieldnames(obj.param);
            obj.length_fns=length(obj.fns);
            
            % 3. Complete Yaw data if missing
            for j = 1:obj.length_fns
                fn_name = obj.fns{j};
                if length(obj.param.(fn_name))==3
                    obj.param.(fn_name)= [obj.param.(fn_name);0];
                end
            end
            
            % 4. Plot Waypoints Immediately (Red Dots)
            hold on;
            for k = 1:obj.length_fns
                fn_name = obj.fns{k};
                pt = obj.param.(fn_name);
                plot3(pt(1), pt(2), pt(3), 'r.', 'MarkerSize', 20);
                text(pt(1), pt(2), pt(3)+0.15, num2str(k), 'Color', 'r', 'FontSize', 10, 'FontWeight', 'bold');
            end
            grid on; axis equal;
            xlabel('x [m]'); ylabel('y [m]');
            title('Manual Waypoints Reference');
            hold off;
            
            % 5. Initialize Variables
            obj.func = @(t) obj.get_ref_at_time(t);
            obj.i=1;
            obj.t0 = [];
            obj.is_first_run = true;
            
            % 6. Initialize Result Struct
            obj.result.state = STATE_CLASS(struct('state_list',["xd","p", "q","v"],'num_list',[20,3,3,3]));
            obj.result.state.set_state("xd",zeros(20,1));
            try
                obj.result.state.set_state("p",obj.self.estimator.result.state.get("p"));
                obj.result.state.set_state("q",obj.self.estimator.result.state.get("q"));
                obj.result.state.set_state("v",obj.self.estimator.result.state.get("v"));
            catch
                obj.result.state.set_state("p",zeros(3,1));
                obj.result.state.set_state("q",zeros(3,1));
                obj.result.state.set_state("v",zeros(3,1));
            end
        end
        
        function result = do(obj,varargin)
            % Skip the first run for initialization stability
            if obj.is_first_run
                obj.is_first_run = false;
                result = obj.result;
                return;
            end
            
            current_sys_t = varargin{1}.t;
            
            if obj.fref_t
                % Lock Start Time
                if isempty(obj.t0)
                    obj.t0 = current_sys_t;
                end
                
                t_rel = current_sys_t - obj.t0;
                
                % Calculate Index (Immediate Start)
                idx = floor(t_rel / obj.ref_t) + 1;
                
                % Clamp Index (Stop at the last point, do not loop)
                if idx > obj.length_fns
                    idx = obj.length_fns;
                end
                obj.i = idx;
                
                curr_fn = obj.fns{obj.i};
                target_vals = obj.param.(curr_fn);
                
                obj.result.state.p = target_vals(1:3);
                obj.result.state.q(3,1) = target_vals(4);
            else
                % Manual Selection Mode
                obj.cha=varargin{2};
                if isfield(obj.param,obj.cha)
                    vals = obj.param.(obj.cha);
                    obj.result.state.p = vals(1:3);
                    obj.result.state.q(3,1) = vals(4);
                end
            end
            
            obj.result.state.v = [0;0;0];
            obj.result.state.xd = obj.get_ref_at_time(current_sys_t);
            result = obj.result;
        end
        
        function show(obj, logger)
            % Only plot trajectory lines here
            rp = logger.data(1,"p","r");
            plot3(rp(:,1), rp(:,2), rp(:,3), 'b');
            daspect([1 1 1]);
            hold on
            ep = logger.data(1,"p","e");
            plot3(ep(:,1), ep(:,2), ep(:,3), 'g');
            hold off
        end
        
        function ref = get_ref_at_time(obj, t_input)
            if isempty(obj.t0)
                t_rel = 0;
            else
                t_rel = t_input - obj.t0;
            end
            
            if t_rel < 0; t_rel = 0; end
            
            if obj.fref_t
                idx = floor(t_rel / obj.ref_t) + 1;
                
                % Prevent Looping in Prediction
                if idx > obj.length_fns
                    idx = obj.length_fns;
                end
                current_idx = idx;
            else
                current_idx = 1;
                if ~isempty(obj.i) && obj.i > 0
                    current_idx = obj.i;
                end
            end
            
            fn_name = obj.fns{current_idx};
            target_vals = obj.param.(fn_name);
            % Return 20-dim vector [Pos(4); Zero_Derivatives(16)]
            ref = [target_vals; zeros(16, 1)];
        end
    end
end