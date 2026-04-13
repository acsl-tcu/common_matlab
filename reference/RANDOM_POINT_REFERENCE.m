classdef RANDOM_POINT_REFERENCE < handle
    properties
        param
        self
        fns
        length_fns
        ref_t
        t0
        i
        result
        func
        is_first_run
        waypoints_mat
    end
    methods
        function obj = RANDOM_POINT_REFERENCE(self,varargin)
            obj.self = self;
            config = varargin{1};
            
            % 1. Parse Inputs
            start_pt = config{1};    % Start Position
            end_pt = config{2};      % End Position
            num_random = config{3};  % Number of random waypoints
            obj.ref_t = config{4};   % Duration per point
            
            % 2. Define Random Area
            x_range = [-1.5, 1.5];
            y_range = [-1.5, 1.5];
            z_range = [0.3, 1.5];
            min_dist = 0.8;          % Minimum distance constraint
            
            % 3. Generate All Points (For Plotting: Start -> Random -> End)
            % Size: num_random + 2 (Start + N + End)
            all_points = zeros(3, num_random + 2);
            all_points(:, 1) = start_pt(:); 
            
            valid_count = 1;
            while valid_count <= num_random
                cand_x = x_range(1) + (x_range(2)-x_range(1)) * rand;
                cand_y = y_range(1) + (y_range(2)-y_range(1)) * rand;
                cand_z = z_range(1) + (z_range(2)-z_range(1)) * rand;
                candidate = [cand_x; cand_y; cand_z];
                
                prev_pt = all_points(:, valid_count); % Check distance from previous
                if norm(candidate - prev_pt) > min_dist
                    valid_count = valid_count + 1;
                    all_points(:, valid_count) = candidate;
                end
            end
            all_points(:, end) = end_pt(:);
            obj.waypoints_mat = all_points;
            
            % 4. Create Param Struct (TARGETS ONLY)
            % FIX 1: Exclude the Start Point from the target list.
            % This ensures the moment 'do' starts, the target is P1 (First Random), not Start.
            obj.param = struct();
            
            % Iterate from 2 to end (Skipping Start Point at index 1)
            target_indices = 2:size(all_points, 2);
            for k = 1:length(target_indices)
                idx = target_indices(k);
                
                % Name fields p1, p2, p3... (p1 is now the first random point)
                field_name = sprintf('p%d', k);
                val = all_points(:, idx);
                obj.param.(field_name) = [val; 0]; % [x; y; z; yaw]
            end
            
            obj.fns = fieldnames(obj.param);
            obj.length_fns = length(obj.fns);
            
            % =============================================================
            % Plotting (3D View)
            % =============================================================
            figure(101); 
            clf(101);
            hold on;
            
            % Plot Path Line
            plot3(all_points(1,:), all_points(2,:), all_points(3,:), ...
                'k--', 'LineWidth', 1.2);
            
            % Plot Start (Green)
            plot3(start_pt(1), start_pt(2), start_pt(3), ...
                'g.', 'MarkerSize', 40);
            text(start_pt(1), start_pt(2), start_pt(3)+0.15, 'START', ...
                'Color', 'g', 'FontSize', 12, 'FontWeight', 'bold');
            
            % Plot Random Targets (Red) - These correspond to p1, p2...
            for k = 1:num_random
                % These are indices 2 to N+1 in all_points
                pt = all_points(:, k+1);
                plot3(pt(1), pt(2), pt(3), 'r.', 'MarkerSize', 30);
                text(pt(1), pt(2), pt(3)+0.15, sprintf('R%d', k), ...
                    'Color', 'r', 'FontSize', 10);
            end
            
            % Plot End (Blue)
            plot3(end_pt(1), end_pt(2), end_pt(3), ...
                'b.', 'MarkerSize', 40);
            text(end_pt(1), end_pt(2), end_pt(3)+0.15, 'END', ...
                'Color', 'b', 'FontSize', 12, 'FontWeight', 'bold');
            
            grid on; axis equal; view(3); rotate3d on;
            xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
            title(['Random Trajectory (Immediate Start, Stops at End)']);
            box on;
            hold off;
            % =============================================================

            obj.func = @(t) obj.get_ref_at_time(t);
            obj.i = 1;
            obj.t0 = [];
            obj.is_first_run = true;
            
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
            if obj.is_first_run
                obj.is_first_run = false;
                result = obj.result;
                return;
            end
            
            current_sys_t = varargin{1}.t;
            
            % Lock start time
            if isempty(obj.t0)
                obj.t0 = current_sys_t;
            end
            
            % Calculate relative time
            t_rel = current_sys_t - obj.t0;
            
            % FIX 2: Stop Looping. Use min() to clamp index.
            % idx 1 = First Random Point (Immediate move)
            % idx Last = End Point (Stay there)
            idx = floor(t_rel / obj.ref_t) + 1;
            
            % Clamp index to not exceed total points
            if idx > obj.length_fns
                idx = obj.length_fns;
            end
            obj.i = idx; 
            
            curr_fn = obj.fns{obj.i};
            target_vals = obj.param.(curr_fn);
            
            obj.result.state.p = target_vals(1:3);
            obj.result.state.q(3,1) = target_vals(4);
            obj.result.state.v = [0;0;0];
            
            obj.result.state.xd = obj.get_ref_at_time(current_sys_t);
            result = obj.result;
        end
        
        function show(obj, logger)
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
            
            idx = floor(t_rel / obj.ref_t) + 1;
            
            % FIX 2: Stop Looping in Prediction Horizon as well
            if idx > obj.length_fns
                idx = obj.length_fns;
            end
            current_idx = idx;
            
            fn_name = obj.fns{current_idx};
            target_vals = obj.param.(fn_name);
            ref = [target_vals; zeros(16, 1)];
        end
    end
end