classdef REPLANNING_BSPLINE_HLC < handle
    % =========================================================================
    % REPLANNING_BSPLINE_HLC
    % Unified B-spline base online replanner for the simulator
    % =========================================================================

    properties
        self
        base_ref
        result
        
        % State
        t_start = []
        is_flight = false
        xd_nominal = zeros(28,1);

        % Physical parameters
        mQ = 1.0
        mL = 0.2
        L_cable = 1.0
        gravity = 9.81
        J = [1e-2;1e-2;2e-2]
        dstxy = [0;0]

        % B-spline
        degree = 7
        nCtrl = 24
        nSamples = 61

        % Control/replanning
        dt_control = 0.01
        max_replan_time = 0.007
        trigger_distance = 7.0
        min_replan_interval = 0.05
        horizon_default = 4.0

        % Geometric body approximations
        drone_radius = 0.30
        load_radius = 0.15
        cable_radius = 0.02

        % Dynamic/kinematic constraints
        max_load_speed = 5.0
        max_load_accel = 8.0
        max_load_jerk = 30.0
        max_load_snap = 100.0
        max_drone_speed = 8.0
        max_drone_accel = 15.0
        max_tilt_rad = deg2rad(35)
        max_cable_angle_rad = deg2rad(45)

        % Candidate search
        duration_scales = [0.75 1.0 1.25 1.6 2.0 3.0]
        deformation_scales = [0.25 0.5 1.0 1.5 2.0 3.0]
        max_candidates = 36

        % Tracking-envelope settings
        error_window = 2.0
        tracking_quantile = 0.99
        tracking_sigma_factor = 3.0
        tracking_floor = 0.30

        % No-solution behavior
        failsafe_mode = 'hold_if_safe_then_retry'
        retry_period = 0.10

        % HLC generated gains
        F1 = []
        F2 = []
        F3 = []
        F4 = []

        % Runtime state
        state = struct()
        obstacles = struct('id',{},'center',{},'radii',{},'R',{}, ...
                           'velocity',{},'timestamp',{},'enabled',{})
        active = []
        goal = struct('pos',[],'yaw',[],'pos_ders',[],'yaw_ders',[])
        last_replan_time = -inf
        last_attempt_time = -inf
        next_id = 1

        % Logs
        log = struct()
        replan_active = false
        warned_crash_load = []
        warned_crash_drone = []
        warned_margin_load = []
        warned_margin_drone = []
        last_warn_time = 0.0
    end
    
    methods
        function obj = REPLANNING_BSPLINE_HLC(self, base_ref, opts)
            arguments
                self
                base_ref
                opts = struct()
            end
            
            obj.self = self;
            
            if iscell(base_ref)
                base_ref_name = str2func(base_ref{1});
                obj.base_ref = base_ref_name(self, base_ref(2:end));
            else
                obj.base_ref = base_ref;
            end
            
            obj.result = struct();
            obj.result.state = STATE_CLASS(struct('state_list', ["xd", "p", "q", "v"], 'num_list', [28, 3, 3, 3]));
            
            if isfield(opts, 'trigger_dist')
                obj.trigger_distance = opts.trigger_dist;
            end
            if isfield(opts, 'r_load')
                obj.load_radius = opts.r_load;
            end
            if isfield(opts, 'r_drone')
                obj.drone_radius = opts.r_drone;
            end
            if isfield(opts, 'safe_margin')
                obj.tracking_floor = opts.safe_margin;
            end

            obj.reset_log();
        end
        
        function result_out = do(obj, varargin)
            time = varargin{1};
            cha = varargin{2};
            
            if length(varargin) >= 4
                env = varargin{4};
            else
                env = [];
            end
            
            base_res = obj.base_ref.do(varargin{:});
            xd_nom = base_res.state.xd;
            obj.xd_nominal = xd_nom;
            
            try obj.mQ = obj.self.parameter.get("mass"); catch, end
            try obj.L_cable = obj.self.parameter.get("cableL"); catch, end
            try obj.J = diag(obj.self.parameter.get("I")); catch, end
            try obj.dstxy = obj.self.parameter.get("dstxy"); catch, end
            
            if isprop(obj.self.estimator.result.state, "mL")
                obj.mL = obj.self.estimator.result.state.mL;
            else
                try obj.mL = obj.self.parameter.get("loadmass"); catch, end
            end
            
            try
                c_param = obj.self.controller.param;
                obj.F1 = c_param.F1; obj.F2 = c_param.F2; obj.F3 = c_param.F3; obj.F4 = c_param.F4;
            catch
            end
            
            if cha == 'f'
                if isempty(obj.t_start)
                    obj.t_start = time.t;
                end
                
                obs_list = [];
                if isstruct(env) && isfield(env, 'obstacle')
                    obs_list = env.obstacle;
                elseif isobject(env) && isprop(env, 'obstacle')
                    obs_list = env.obstacle;
                else
                    try
                        obs_list = ENVIRONMENT_OBSTACLE_ELLIPSE_MOVE(time.t);
                    catch
                        try
                            obs_list = ENVIRONMENT_OBSTACLE_ELLIPSE();
                        catch
                            try
                                obs_list = ENVIRONMENT_OBSTACLE_HOCBF_LINK_XY();
                            catch
                            end
                        end
                    end
                end
                
                obj.obstacles = struct('id',{},'center',{},'radii',{},'R',{},'velocity',{},'timestamp',{},'enabled',{});
                obj.next_id = 1;

                for i = 1:length(obs_list)
                    o = obs_list(i);
                    v = [0;0;0];
                    if isfield(o, 'v_center'), v = o.v_center(:);
                    elseif isfield(o, 'v'), v = o.v(:);
                    elseif isprop(o, 'v_center'), v = o.v_center(:);
                    elseif isprop(o, 'v'), v = o.v(:);
                    end
                    
                    R_o = eye(3);
                    if isfield(o, 'R_obs') && ~isempty(o.R_obs), R_o = o.R_obs;
                    elseif isfield(o, 'R') && ~isempty(o.R), R_o = o.R;
                    elseif isprop(o, 'R_obs') && ~isempty(o.R_obs), R_o = o.R_obs;
                    elseif isprop(o, 'R') && ~isempty(o.R), R_o = o.R;
                    end
                    
                    radii = [1; 1; 1];
                    if isfield(o, 'ellipsoid_radii') && ~isempty(o.ellipsoid_radii), radii = o.ellipsoid_radii(:);
                    elseif isprop(o, 'ellipsoid_radii') && ~isempty(o.ellipsoid_radii), radii = o.ellipsoid_radii(:);
                    elseif isfield(o, 'raw_param')
                        rp = o.raw_param;
                        typ = '';
                        if isfield(o, 'type'), typ = o.type; end
                        if isprop(o, 'type'), typ = o.type; end
                        switch typ
                            case 'cylinder'
                                radii = [rp(1)*sqrt(2); rp(1)*sqrt(2); (rp(2)/2)*sqrt(2)];
                            case 'box'
                                radii = [rp(1)/2; rp(2)/2; rp(3)/2]*sqrt(3);
                            case 'sphere'
                                radii = [rp(1); rp(1); rp(1)];
                            otherwise
                                radii = [0.5; 0.5; 0.5];
                        end
                    end
                    
                    p_c = [0;0;0];
                    if isfield(o, 'p_center'), p_c = o.p_center(:); end
                    if isprop(o, 'p_center'), p_c = o.p_center(:); end
                    
                    obj.add_obstacle(p_c, radii, R_o, v, time.t, i);
                end
                
                if isprop(obj.self.estimator.result.state, "pL")
                    pL_cur = obj.self.estimator.result.state.pL;
                    vL_cur = obj.self.estimator.result.state.vL;
                else
                    pL_cur = obj.self.estimator.result.state.p - [0; 0; obj.L_cable];
                    vL_cur = obj.self.estimator.result.state.v;
                end
                
                if isprop(obj.self.estimator.result.state, "p")
                    pQ_cur = obj.self.estimator.result.state.p;
                    vQ_cur = obj.self.estimator.result.state.v;
                else
                    pQ_cur = pL_cur + [0; 0; obj.L_cable];
                    vQ_cur = vL_cur;
                end

                s = struct();
                s.load_pos = pL_cur;
                s.load_vel = vL_cur;
                s.drone_pos = pQ_cur;
                try s.drone_yaw = obj.self.estimator.result.state.q(3); catch, s.drone_yaw = 0; end
                
                obj.update_state(s);
                
                pos_ders = zeros(3,7);
                yaw_ders = zeros(1,7);
                for k = 1:7
                    idx = (k-1)*4 + 1;
                    pos_ders(:,k) = xd_nom(idx:idx+2);
                    yaw_ders(k) = xd_nom(idx+3);
                end
                
                obj.set_goal(pos_ders(:,1), yaw_ders(1), pos_ders, yaw_ders);
                
                out = obj.step_algo(time.t);
                
                if out.reference.valid
                    ref = out.reference;
                    xd = zeros(28,1);
                    for k = 1:7
                        idx = (k-1)*4 + 1;
                        xd(idx:idx+2) = ref.ders(:, k);
                        xd(idx+3) = ref.yaw_ders(k);
                    end
                    obj.result.state.xd = xd;
                else
                    obj.result.state.xd = xd_nom;
                end
            else
                obj.result.state.xd = xd_nom;
            end
            
            obj.result.state.p = obj.result.state.xd(1:3);
            obj.result.state.v = obj.result.state.xd(5:7);
            obj.result.state.q = [0; 0; obj.result.state.xd(4)];
            
            result_out = obj.result;
        end
    end

    methods (Access = private)
        function set_goal(obj,pos,yaw,varargin)
            obj.goal.pos = pos(:);
            obj.goal.yaw = yaw;
            obj.goal.pos_ders = zeros(3,7);
            obj.goal.yaw_ders = zeros(1,7);
            obj.goal.pos_ders(:,1) = obj.goal.pos;
            obj.goal.yaw_ders(1) = yaw;
            if nargin >= 4 && ~isempty(varargin{1})
                gd = varargin{1};
                obj.goal.pos_ders = gd(:,1:7);
            end
            if nargin >= 5 && ~isempty(varargin{2})
                yd = varargin{2};
                obj.goal.yaw_ders = yd(1:7);
            end
        end
        
        function id = add_obstacle(obj,center,radii,varargin)
            R = eye(3); v = zeros(3,1); ts = 0; id = obj.next_id;
            if nargin >= 4 && ~isempty(varargin{1}), R = varargin{1}; end
            if nargin >= 5 && ~isempty(varargin{2}), v = varargin{2}(:); end
            if nargin >= 6 && ~isempty(varargin{3}), ts = varargin{3}; end
            if nargin >= 7 && ~isempty(varargin{4}), id = varargin{4}; end
            k = numel(obj.obstacles)+1;
            obj.obstacles(k) = struct('id',id,'center',center(:),'radii',radii(:), ...
                'R',R,'velocity',v,'timestamp',ts,'enabled',true);
            obj.next_id = max(obj.next_id,id+1);
        end
        
        function update_state(obj,s)
            obj.state = s;
            obj.state.time = obj.get_field(s,'time',0);
            if ~isfield(obj.state,'drone_R') || isempty(obj.state.drone_R)
                if isfield(s,'drone_quat')
                    obj.state.drone_R = REPLANNING_BSPLINE_HLC.quat2rotm_col(s.drone_quat(:));
                else
                    obj.state.drone_R = eye(3);
                end
            end
            if ~isfield(obj.state,'cable_dir') || isempty(obj.state.cable_dir)
                d = s.drone_pos(:)-s.load_pos(:);
                if norm(d)>1e-9
                    obj.state.cable_dir = d/norm(d);
                else
                    obj.state.cable_dir = [0;0;1];
                end
            end
        end
        
        function out = step_algo(obj,time_now)
            if isempty(obj.goal.pos)
                error('Call set_goal before step().');
            end
            if isempty(obj.state) || ~isfield(obj.state,'load_pos')
                error('Call update_state() before step().');
            end
    
            obj.state.time = time_now;
            t0 = tic;
    
            metrics = obj.current_metrics(time_now);
            trigger = obj.should_replan(metrics,time_now);
    
            replanned = false;
            reason = '';
            candidate = obj.active;
    
            if trigger
                if time_now-obj.last_attempt_time >= obj.min_replan_interval
                    obj.last_attempt_time = time_now;
                    [candidate,ok,reason,stats] = obj.replan(time_now);
                    if ok
                        if ~obj.replan_active
                            fprintf("\n=================================================================================\n");
                            fprintf(" [B-SPLINE BLUEPRINT REPLANNER 診断レポート]  t = %.3f s\n", time_now);
                            fprintf("=================================================================================\n");
                            fprintf(" 1. 真値状態取得      : 荷物 pL, 機体 pQ (推定期直接抽出: 正常)\n");
                            fprintf(" 2. 動的接近判定      : B-Spline軌道変形法による衝突検知 (PVO類似)\n");
                            fprintf(" 3. 軌道変形スケール  : 探索空間 %.2f 倍 (最大候補数 %d)\n", obj.deformation_scales(end), obj.max_candidates);
                            fprintf(" 4. 回避プラン策定    : 処理時間 = %6.2f ms (安全解発見)\n", stats.computeTime*1000);
                            fprintf(" 5. 軌道評価スコア    : コスト = %.3f\n", candidate.validation.cost);
                            fprintf("=================================================================================\n\n");
                        end
                        obj.active = candidate;
                        obj.last_replan_time = time_now;
                        obj.replan_active = true;
                        replanned = true;
                    end
                else
                    stats = struct('attempted',false,'success',false,'reason','rate_limited');
                end
            else
                stats = struct('attempted',false,'success',false,'reason','not_needed');
            end
    
            ref = obj.evaluate_active_reference(time_now);
            obj.update_log(time_now,metrics,ref,replanned,reason,stats,toc(t0));
    
            out = struct('reference',ref,'replanned',replanned, ...
                'trigger',trigger,'reason',reason,'stats',stats, ...
                'metrics',metrics);
        end
        
        function [traj,ok,reason,stats] = replan(obj,time_now)
            tStart = tic;
            reason = '';
            ok = false;
            traj = obj.active;
            stats = struct('attempted',true,'success',false,'reason','', ...
                'nCandidates',0,'nFeasible',0,'computeTime',0);
    
            T0 = obj.estimate_base_duration();
            if isfield(obj.active,'T') && ~isempty(obj.active.T)
                T0 = max(T0,0.5*obj.active.T);
            end
    
            [b0,bd0] = obj.start_boundary_derivatives(time_now);
            [bT,bdT] = obj.goal_boundary_derivatives();
    
            base = obj.solve_bspline(b0,bT,T0);
            if ~base.ok
                reason='boundary_solver_failed';
                stats.reason=reason; stats.computeTime=toc(tStart); return
            end
            base.yaw = obj.solve_bspline_yaw(bd0(4,:),bdT(4,:),T0);
    
            candList = obj.make_candidates(base,time_now);
    
            best = [];
            bestScore = inf;
            tBudget = obj.max_replan_time;
    
            for k=1:numel(candList)
                if toc(tStart) > tBudget
                    break
                end
                stats.nCandidates = stats.nCandidates+1;
                c = candList{k};
                val = obj.validate_trajectory(c,time_now);
                if val.feasible
                    score = val.cost;
                    if score < bestScore
                        bestScore = score;
                        best = c;
                        best.validation = val;
                    end
                    stats.nFeasible = stats.nFeasible+1;
                end
            end
    
            stats.computeTime = toc(tStart);
    
            if ~isempty(best)
                traj = best;
                ok = true;
                reason = 'feasible_candidate_found';
                stats.success = true;
                stats.reason = reason;
                return
            end
    
            if ~isempty(obj.active)
                av = obj.validate_trajectory(obj.active,time_now);
                if av.feasible
                    traj = obj.active;
                    reason = 'no_new_solution_active_trajectory_retained';
                    stats.reason=reason;
                    return
                end
            end
    
            reason = 'no_feasible_solution';
            stats.reason = reason;
            stats.failsafe = obj.failsafe_action(time_now);
        end
        
        function ref = evaluate_active_reference(obj,time_now)
            if isempty(obj.active)
                ref = struct('valid',false);
                return
            end
            tr = obj.active;
            
            % Check if trajectory is completed
            if time_now - tr.t0 >= tr.T
                if obj.replan_active
                    fprintf("[B-SPLINE C^6] 回避完了! 公称軌道へ完全復帰 (t=%.3f s)\n\n", time_now);
                end
                obj.replan_active = false;
                obj.active = [];
                ref = struct('valid',false);
                return
            end
            
            tau = min(max(time_now-tr.t0,0),tr.T);
            [p,d] = obj.bspline_eval(tr.P,tau,tr.T,6);
            if isfield(tr, 'Y')
                [y,yd] = obj.bspline_eval(tr.Y.P,tau,tr.T,6);
            else
                [y,yd] = obj.bspline_eval(tr.yaw.P,tau,tr.T,6);
            end
    
            ref.valid = true;
            ref.time = time_now;
            ref.tau = tau;
            ref.pos = p(:,1);
            ref.ders = p;
            ref.yaw = y(1,1);
            ref.yaw_ders = y;
            ref.drone = obj.flatness_from_load_reference(p,y);
        end
        
        function ref = make_reference_at(obj,tr,time_now)
            tau=min(max(time_now-tr.t0,0),tr.T);
            [p,~]=obj.bspline_eval(tr.P,tau,tr.T,6);
            if isfield(tr, 'Y')
                [y,~]=obj.bspline_eval(tr.Y.P,tau,tr.T,6);
            else
                [y,~]=obj.bspline_eval(tr.yaw.P,tau,tr.T,6);
            end
            ref=struct('valid',true,'time',time_now,'tau',tau, ...
                'pos',p(:,1),'ders',p,'yaw',y(1),'yaw_ders',y);
            ref.drone=obj.flatness_from_load_reference(p,y);
        end
        
        function reset_log(obj)
            obj.log=struct('time',[],'load_pos_error',[],'drone_pos_error',[], ...
                'cable_angle',[],'tracking_margin',[],'min_raw_clearance',[], ...
                'min_tube_clearance',[],'trigger',[],'replanned',[], ...
                'replan_failed',[],'compute_time',[],'reason',{{}}, ...
                'hlc_checked',[],'hlc_thrust',[],'hlc_torque',[]);
        end
        
        function [b0,bd0] = start_boundary_derivatives(obj,tNow)
            if ~isempty(obj.active) && isfield(obj.active,'P')
                r=obj.make_reference_at(obj.active,tNow);
                b0=[r.ders; r.yaw_ders];
                bd0=b0;
                return
            end
            p=obj.state.load_pos(:);
            v=obj.get_field(obj.state,'load_vel',zeros(3,1));
            b0=zeros(4,7);
            b0(1:3,1)=p;
            b0(1:3,2)=v(:);
            yaw=obj.get_field(obj.state,'drone_yaw',0);
            b0(4,1)=yaw;
            bd0=b0;
        end
    
        function [bT,bdT] = goal_boundary_derivatives(obj)
            bT=zeros(4,7);
            bT(1:3,:)=obj.goal.pos_ders;
            bT(4,:)=obj.goal.yaw_ders;
            bdT=bT;
        end
    
        function T=estimate_base_duration(obj)
            p0=obj.state.load_pos(:);
            d=norm(obj.goal.pos(:)-p0);
            T=max([obj.horizon_default,d/max(obj.max_load_speed,1e-3), ...
                sqrt(4*d/max(obj.max_load_accel,1e-3))]);
            if ~isempty(obj.active)
                T=max(T,0.5*obj.active.T);
            end
        end
    
        function tr=solve_bspline(obj,b0,bT,T)
            n=obj.nCtrl; p=obj.degree;
            U=obj.make_knots(n,p,T);
            A=zeros(14,n);
            for k=0:6
                A(k+1,:)=obj.basis_derivative_row(U,p,0,k,T);
                A(8+k,:)=obj.basis_derivative_row(U,p,T,k,T);
            end
            H=obj.minimum_snap_H(U,p,T);
            H=H+1e-9*eye(n);
            K=[H A'; A zeros(14)];
            bx=[b0(1,:)';bT(1,:)'];
            by=[b0(2,:)';bT(2,:)'];
            bz=[b0(3,:)';bT(3,:)'];
            Px=K\[zeros(n,1);bx];
            Py=K\[zeros(n,1);by];
            Pz=K\[zeros(n,1);bz];
            tr.ok=true; tr.T=T; tr.t0=obj.state.time; tr.U=U;
            tr.P=[Px(1:n),Py(1:n),Pz(1:n)];
            tr.degree=p;
        end
    
        function Y=solve_bspline_yaw(obj,y0,yT,T)
            n=obj.nCtrl; p=obj.degree; U=obj.make_knots(n,p,T);
            A=zeros(14,n);
            for k=0:6
                A(k+1,:)=obj.basis_derivative_row(U,p,0,k,T);
                A(8+k,:)=obj.basis_derivative_row(U,p,T,k,T);
            end
            H=obj.minimum_snap_H(U,p,T)+1e-9*eye(n);
            K=[H A';A zeros(14)];
            z=K\[zeros(n,1);[y0(:);yT(:)]];
            Y=struct('T',T,'t0',obj.state.time,'U',U,'P',z(1:n),'degree',p);
        end
    
        function cands=make_candidates(obj,base,tNow)
            cands={base};
            if isempty(obj.obstacles), return; end
            obs=obj.predicted_obstacles(tNow);
            baseEval=obj.sample_trajectory(base);
            dirs=[];
            for j=1:numel(obs)
                o=obs(j);
                [dmin,pt,normal]=obj.trajectory_obstacle_hint(baseEval,o);
                if dmin < obj.trigger_distance*1.5
                    dirs=[dirs,normal];
                end
            end
            if isempty(dirs), return; end
            dir=sum(dirs,2);
            if norm(dir)<1e-9, dir=[1;0;0]; end
            dir=dir/norm(dir);
    
            D=[dir,REPLANNING_BSPLINE_HLC.nullspace_direction(dir),cross(dir,[0;0;1])];
            for q=1:size(D,2)
                if norm(D(:,q))<1e-9, continue; end
                D(:,q)=D(:,q)/norm(D(:,q));
                for s=obj.deformation_scales
                    if numel(cands)>=obj.max_candidates, return; end
                    c=base;
                    c.P=obj.deform_preserving_C6(base.P,base.U,base.degree,D(:,q),s);
                    cands{end+1}=c;
                    if numel(cands)>=obj.max_candidates, return; end
                    c.P=obj.deform_preserving_C6(base.P,base.U,base.degree,-D(:,q),s);
                    cands{end+1}=c;
                end
            end
    
            relScale=obj.relative_speed_scale(tNow,base);
            for sc=obj.duration_scales*relScale
                if numel(cands)>=obj.max_candidates, return; end
                Tnew=max(0.25,base.T*sc);
                tr=obj.solve_bspline_from_boundary_of(base,Tnew);
                cands{end+1}=tr;
            end
        end
    
        function tr=solve_bspline_from_boundary_of(obj,base,T)
            [p,~]=obj.bspline_eval(base.P,0,base.T,6);
            [pe,~]=obj.bspline_eval(base.P,base.T,base.T,6);
            if isfield(base, 'Y')
                [y,~]=obj.bspline_eval(base.Y.P,0,base.Y.T,6);
                [ye,~]=obj.bspline_eval(base.Y.P,base.Y.T,base.Y.T,6);
            else
                [y,~]=obj.bspline_eval(base.yaw.P,0,base.yaw.T,6);
                [ye,~]=obj.bspline_eval(base.yaw.P,base.yaw.T,base.yaw.T,6);
            end
            b0=p; bT=pe; y0=y; yT=ye;
            tr=obj.solve_bspline(b0,bT,T);
            tr.yaw=obj.solve_bspline_yaw(y0(1,:),yT(1,:),T);
        end
    
        function Pnew=deform_preserving_C6(obj,P,U,p,dir,scale)
            n=size(P,1);
            A=zeros(14,n);
            for k=0:6
                A(k+1,:)=obj.basis_derivative_row(U,p,0,k,P);
                A(8+k,:)=obj.basis_derivative_row(U,p,U(end),k,P);
            end
            Z=null(A,'r');
            if isempty(Z), Pnew=P; return; end
            w=zeros(n,1);
            s=linspace(0,1,n)';
            w=(sin(pi*s).^2).^2;
            raw=w*dir(:)';
            D=Z*(Z'*raw);
            nr=norm(D,'fro');
            if nr<1e-12, Pnew=P; return; end
            Pnew=P+(0.25*scale)*D/nr*n;
        end
    
        function val=validate_trajectory(obj,tr,tNow)
            S=obj.sample_trajectory(tr);
            obs=obj.predicted_obstacles(tNow);
            val=struct('feasible',true,'cost',0,'minRaw',inf,'minTube',inf, ...
                'maxSpeed',0,'maxAccel',0,'maxJerk',0,'maxSnap',0, ...
                'maxDroneSpeed',0,'maxDroneAccel',0,'maxTilt',0, ...
                'maxCableAngle',0,'hlcChecked',false,'hlcThrust',NaN,'hlcTorque',NaN);
    
            val.maxSpeed=max(vecnorm(S.loadDers(:,2:end),2,1),[],'all');
            val.maxAccel=max(vecnorm(S.loadDers(:,3:end),2,1),[],'all');
            val.maxJerk=max(vecnorm(S.loadDers(:,4:end),2,1),[],'all');
            val.maxSnap=max(vecnorm(S.loadDers(:,5:end),2,1),[],'all');
    
            if val.maxSpeed>obj.max_load_speed || val.maxAccel>obj.max_load_accel || ...
                    val.maxJerk>obj.max_load_jerk || val.maxSnap>obj.max_load_snap
                val.feasible=false;
            end
    
            for k=1:numel(obs)
                o=obs(k);
                for i=1:size(S.load,2)
                    [rawd,inside]=obj.point_ellipsoid_signed_distance(S.load(:,i),o);
                    [~,din]=obj.point_ellipsoid_signed_distance(S.drone(:,i),o);
                    val.minRaw=min([val.minRaw,rawd,din]);
                    if inside || rawd<=0 || din<=0
                        val.feasible=false;
                    end
    
                    dl=obj.point_clearance_to_inflated_ellipsoid(S.load(:,i),o,obj.load_radius);
                    dq=obj.point_clearance_to_inflated_ellipsoid(S.drone(:,i),o,obj.drone_radius);
                    if dl<=0 || dq<=0, val.feasible=false; end
    
                    if i<size(S.load,2)
                        dc=obj.segment_ellipsoid_clearance(S.drone(:,i),S.load(:,i),o,obj.cable_radius);
                        val.minRaw=min(val.minRaw,dc);
                        if dc<=0, val.feasible=false; end
                    end
                    tube=obj.tracking_margin_at(tNow);
                    dl2=obj.point_clearance_to_inflated_ellipsoid(S.load(:,i),o,obj.load_radius+tube);
                    dq2=obj.point_clearance_to_inflated_ellipsoid(S.drone(:,i),o,obj.drone_radius+tube);
                    val.minTube=min([val.minTube,dl2,dq2]);
                    if i<size(S.load,2)
                        dc2=obj.segment_ellipsoid_clearance(S.drone(:,i),S.load(:,i),o,obj.cable_radius+tube);
                        val.minTube=min(val.minTube,dc2);
                    end
                end
            end
    
            val.maxDroneSpeed=max(vecnorm(S.droneDers(:,2:end),2,1),[],'all');
            val.maxDroneAccel=max(vecnorm(S.droneDers(:,3:end),2,1),[],'all');
            val.maxTilt=max(S.tilt);
            val.maxCableAngle=max(S.cableAngle);
    
            if val.maxDroneSpeed>obj.max_drone_speed || val.maxDroneAccel>obj.max_drone_accel || ...
                    val.maxTilt>obj.max_tilt_rad || val.maxCableAngle>obj.max_cable_angle_rad
                val.feasible=false;
            end
    
            if val.minTube < 0
                val.feasible=false;
            end
    
            val.cost=tr.T + 0.05*(val.maxAccel/obj.max_load_accel)^2 + ...
                0.05*(val.maxCableAngle/obj.max_cable_angle_rad)^2;
        end
    
        function S=sample_trajectory(obj,tr)
            tt=linspace(0,tr.T,obj.nSamples);
            [~,D]=obj.bspline_eval(tr.P,tt,tr.T,6);
            if isfield(tr, 'Y')
                [~,YD]=obj.bspline_eval(tr.Y.P,tt,tr.T,6);
            else
                [~,YD]=obj.bspline_eval(tr.yaw.P,tt,tr.T,6);
            end
            N=numel(tt);
            S.t=tt; 
            S.load = reshape(D(:,1,:), [3, N]); 
            S.loadDers=D; 
            S.yaw = YD(1, :); 
            S.yawDers=YD;
            S.drone=zeros(3,N); S.droneDers=zeros(3,7,N); S.cable=zeros(3,N);
            S.cableAngle=zeros(1,N); S.tilt=zeros(1,N);
            for i=1:N
                rr=obj.flatness_from_load_reference(D(:,:,i),YD(:,i));
                S.drone(:,i)=rr.pos; S.droneDers(:,:,i)=rr.ders;
                S.cable(:,i)=rr.cable_dir; S.cableAngle(i)=rr.cable_angle; S.tilt(i)=rr.tilt;
            end
        end
    
        function r=flatness_from_load_reference(obj,p,y)
            p0=p(1:3,1); v=p(1:3,2); a=p(1:3,3);
            gvec=[0;0;obj.gravity]; Fc=obj.mL*(a+gvec);
            if norm(Fc)<1e-8, pT=[0;0;1]; else, pT=-Fc/norm(Fc); end
            if size(p,2)>=4
                da=p(1:3,4); Fdot=obj.mL*da; n=norm(Fc);
                pTdot=-(Fdot/n-Fc*(Fc'*Fdot)/n^3);
            else
                pTdot=zeros(3,1);
            end
            pQ=p0-obj.L_cable*pT; vQ=v-obj.L_cable*pTdot;
            aQ=a;
            if size(p,2)>=4, jQ=p(1:3,4); else, jQ=zeros(3,1); end
            R=REPLANNING_BSPLINE_HLC.reference_rotation_from_force(aQ,y(1));
            r.pos=pQ; r.vel=vQ; r.acc=aQ; r.jerk=jQ;
            r.ders=zeros(3,7); r.ders(:,1)=pQ; r.ders(:,2)=vQ; r.ders(:,3)=aQ; r.ders(:,4)=jQ;
            r.cable_dir=pT; r.cable_angle=acos(max(-1,min(1,abs(pT(3)))));
            r.tilt=acos(max(-1,min(1,R(3,3)))); r.R=R; r.yaw=y(1);
        end
    
        function m=current_metrics(obj,tNow)
            m=struct('trigger',false,'droneDistance',inf,'loadDistance',inf, ...
                'minDistance',inf,'trackingMargin',obj.tracking_margin_at(tNow), ...
                'cableAngle',0,'loadError',0,'droneError',0);
            obs=obj.predicted_obstacles(tNow);
            % Initialize warning arrays if size changes
            n_obs = numel(obs);
            if length(obj.warned_crash_load) ~= n_obs
                obj.warned_crash_load = false(n_obs, 1);
                obj.warned_crash_drone = false(n_obs, 1);
                obj.warned_margin_load = false(n_obs, 1);
                obj.warned_margin_drone = false(n_obs, 1);
            end
            
            for i=1:n_obs
                [d1,~]=obj.point_ellipsoid_signed_distance(obj.state.drone_pos(:),obs(i));
                [d2,~]=obj.point_ellipsoid_signed_distance(obj.state.load_pos(:),obs(i));
                
                % Logging for drone
                if d1 <= 0 && ~obj.warned_crash_drone(i)
                    fprintf(2, "[CRITICAL ALARM] 機体が障害物%dに衝突! (t=%.3f s, 侵入深さ: %.3f m)\n", i, tNow, -d1);
                    obj.warned_crash_drone(i) = true;
                elseif d1 <= (obj.drone_radius + 0.5) && ~obj.warned_margin_drone(i) && d1 > 0
                    fprintf("[SAFETY WARN] 機体が障害物%dのマージン帯侵入 (t=%.3f s, 残余距離: %.3f m)\n", i, tNow, d1);
                    obj.warned_margin_drone(i) = true;
                end
                
                % Logging for load
                if d2 <= 0 && ~obj.warned_crash_load(i)
                    fprintf(2, "[CRITICAL ALARM] 荷物が障害物%dに衝突! (t=%.3f s, 侵入深さ: %.3f m)\n", i, tNow, -d2);
                    obj.warned_crash_load(i) = true;
                elseif d2 <= (obj.load_radius + 0.5) && ~obj.warned_margin_load(i) && d2 > 0
                    fprintf("[SAFETY WARN] 荷物が障害物%dのマージン帯侵入 (t=%.3f s, 残余距離: %.3f m)\n", i, tNow, d2);
                    obj.warned_margin_load(i) = true;
                end
                
                m.droneDistance=min(m.droneDistance,d1);
                m.loadDistance=min(m.loadDistance,d2);
            end
            m.minDistance=min(m.droneDistance,m.loadDistance);
            if m.minDistance<=obj.trigger_distance
                m.trigger=true;
            end
            if isfield(obj.state,'cable_dir')
                m.cableAngle=acos(max(-1,min(1,abs(obj.state.cable_dir(3)))));
            end
            if ~isempty(obj.active)
                rr=obj.evaluate_active_reference(tNow);
                m.loadError=norm(obj.state.load_pos(:)-rr.pos(:));
                m.droneError=norm(obj.state.drone_pos(:)-rr.drone.pos(:));
            end
        end
    
        function tf=should_replan(obj,m,tNow)
            if tNow-obj.last_replan_time < obj.min_replan_interval
                tf=false; return
            end
            tf=m.trigger;
            if ~tf && ~isempty(obj.active)
                tube=obj.tracking_margin_at(tNow);
                if tube>obj.tracking_floor
                    rr=obj.evaluate_active_reference(tNow);
                    obs=obj.predicted_obstacles(tNow);
                    for i=1:numel(obs)
                        [d,~]=obj.point_ellipsoid_signed_distance(rr.pos,obs(i));
                        if d<=tube, tf=true; break; end
                    end
                end
            end
        end
    
        function margin=tracking_margin_at(obj,~)
            if ~isfield(obj.log,'time') || numel(obj.log.time)<5
                margin=obj.tracking_floor;
                return
            end
            n=numel(obj.log.time); t0=obj.log.time(end);
            idx=obj.log.time>=t0-obj.error_window;
            e=[obj.log.load_pos_error(idx),obj.log.drone_pos_error(idx)]; e=e(isfinite(e));
            if isempty(e), margin=obj.tracking_floor; return; end
            q=quantile(e,obj.tracking_quantile); mu=mean(e); sig=std(e);
            margin=max([q,mu+obj.tracking_sigma_factor*sig,obj.tracking_floor]);
        end
    
        function obs=predicted_obstacles(obj,tNow)
            obs=obj.obstacles;
            for i=1:numel(obs)
                if ~obs(i).enabled, continue; end
                obs(i).center=obs(i).center(:)+obs(i).velocity(:)*(tNow-obs(i).timestamp);
            end
            obs=obs([obs.enabled]);
        end
    
        function [dmin,pt,n]=trajectory_obstacle_hint(obj,S,o)
            dmin=inf; pt=S.load(:,1); n=[1;0;0];
            for i=1:size(S.load,2)
                [d,inside]=obj.point_ellipsoid_signed_distance(S.load(:,i),o);
                if d<dmin, dmin=d; pt=S.load(:,i); n=obj.ellipsoid_outward_normal(pt,o); end
                [d2,~]=obj.point_ellipsoid_signed_distance(S.drone(:,i),o);
                if d2<dmin, dmin=d2; pt=S.drone(:,i); n=obj.ellipsoid_outward_normal(pt,o); end
            end
            if norm(n)<1e-9, n=[1;0;0]; end
        end
    
        function scale=relative_speed_scale(obj,tNow,tr)
            S=obj.sample_trajectory(tr); vref=S.loadDers(:,2);
            obs=obj.predicted_obstacles(tNow); closing=0;
            for i=1:numel(obs)
                [~,pt,n]=obj.trajectory_obstacle_hint(S,obs(i));
                vv=obs(i).velocity(:);
                k=max(0,dot(vv-vref(:,1),n));
                closing=max(closing,k);
            end
            scale=1+0.5*closing/max(obj.max_load_speed,1e-3);
        end
    
        function row=basis_derivative_row(obj,U,p,t,k,T)
            n=numel(U)-p-1; epsT=1e-9*max(1,T);
            if abs(t-T)<epsT, t=T-epsT; end
            row=zeros(1,n);
            for i=1:n, row(i)=obj.bspline_basis_derivative(i,p,k,t,U); end
        end
    
        function H=minimum_snap_H(obj,U,p,T)
            n=numel(U)-p-1; H=zeros(n);
            [x,w]=REPLANNING_BSPLINE_HLC.gauss8();
            for s=p+1:numel(U)-p-1
                a=U(s); b=U(s+1);
                if b<=a, continue; end
                for q=1:numel(x)
                    tq=(a+b)/2+(b-a)/2*x(q); B=zeros(1,n);
                    for i=1:n, B(i)=obj.bspline_basis_derivative(i,p,4,tq,U); end
                    H=H+w(q)*(b-a)/2*(B'*B);
                end
            end
            H=H/max(T,1e-9);
        end
    
        function U=make_knots(~,n,p,T)
            nInt=n-p-1; internal=linspace(0,T,nInt+2); internal=internal(2:end-1);
            U=[zeros(1,p+1),internal,T*ones(1,p+1)];
        end
    
        function [D,Db]=bspline_eval(obj,P,t,T,maxOrder)
            t=t(:).'; n=size(P,1); p=obj.degree; U=obj.make_knots(n,p,T);
            if size(P,2)==1
                D=zeros(maxOrder+1,numel(t));
                for j=1:numel(t)
                    for k=0:maxOrder
                        row=obj.basis_derivative_row(U,p,t(j),k,T);
                        D(k+1,j)=row*P;
                    end
                end
                Db=D; return
            end
            D=zeros(size(P,2),maxOrder+1,numel(t));
            for j=1:numel(t)
                for k=0:maxOrder
                    row=obj.basis_derivative_row(U,p,t(j),k,T);
                    D(:,k+1,j)=(row*P).';
                end
            end
            Db=D;
        end
    
        function v=bspline_basis_derivative(obj,i,p,k,t,U)
            if k==0, v=obj.bspline_basis(i,p,t,U); return; end
            if p==0, v=0; return; end
            a=U(i+p)-U(i); b=U(i+p+1)-U(i+1); v=0;
            if a>0, v=v+p/a*obj.bspline_basis_derivative(i,p-1,k-1,t,U); end
            if b>0, v=v-p/b*obj.bspline_basis_derivative(i+1,p-1,k-1,t,U); end
        end
    
        function v=bspline_basis(~,i,p,t,U)
            if p==0
                if (U(i)<=t && t<U(i+1)) || (t==U(end) && i==numel(U)-1), v=1; else, v=0; end
                return
            end
            v=0; a=U(i+p)-U(i); b=U(i+p+1)-U(i+1);
            if a>0, v=v+(t-U(i))/a*REPLANNING_BSPLINE_HLC.basis_static(i,p-1,t,U); end
            if b>0, v=v+(U(i+p+1)-t)/b*REPLANNING_BSPLINE_HLC.basis_static(i+1,p-1,t,U); end
        end
    
        function [d,inside]=point_ellipsoid_signed_distance(~,p,o)
            y=o.R'*(p(:)-o.center(:)); r=o.radii(:); q=sum((y./r).^2);
            inside=q<1;
            if norm(y)<1e-14, d=-min(r); return; end
            a2=r.^2; yy=y.^2;
            if q>1, lo=0; hi=max(abs(y).*r); else, lo=-min(a2)*(1-1e-12); hi=0; end
            f=@(lam)sum(a2.*yy./(lam+a2).^2)-1;
            for kk=1:80
                mid=(lo+hi)/2;
                if f(mid)>0, lo=mid; else, hi=mid; end
            end
            lam=(lo+hi)/2; x=a2.*y./(lam+a2);
            d0=norm(x-y); d=sign(q-1)*d0;
        end
    
        function d=point_clearance_to_inflated_ellipsoid(~,p,o,radd)
            oo=o; oo.radii=o.radii(:)+radd;
            [d,~]=REPLANNING_BSPLINE_HLC.point_ellipsoid_signed_distance_static(p,oo);
        end
    
        function d=segment_ellipsoid_clearance(~,a,b,o,radd)
            oo=o; oo.radii=o.radii(:)+radd;
            A=oo.R'*(a(:)-oo.center(:)); B=oo.R'*(b(:)-oo.center(:));
            d=B-A; aa=sum((d./oo.radii).^2); bb=2*sum((A./oo.radii).*(d./oo.radii));
            if aa<1e-14
                q=sum((A./oo.radii).^2);
                d=sign(1-q)*min(oo.radii); return
            end
            s=-bb/(2*aa); s=max(0,min(1,s));
            q=sum(((A+s*d)./oo.radii).^2);
            if q<=1, d=-min(oo.radii)*(1-q); else, d=(sqrt(q)-1)*min(oo.radii); end
        end
    
        function n=ellipsoid_outward_normal(~,p,o)
            y=o.R'*(p(:)-o.center(:)); n=o.R*(y./(o.radii(:).^2));
            if norm(n)>1e-12, n=n/norm(n); else, n=[1;0;0]; end
        end
    
        function act=failsafe_action(obj,tNow)
            act=struct('mode',obj.failsafe_mode,'time',tNow,'action','retry');
        end
    
        function update_log(obj,tNow,m,ref,replanned,reason,stats,comp)
            obj.log.time(end+1)=tNow;
            obj.log.load_pos_error(end+1)=m.loadError;
            obj.log.drone_pos_error(end+1)=m.droneError;
            obj.log.cable_angle(end+1)=m.cableAngle;
            obj.log.tracking_margin(end+1)=m.trackingMargin;
            obj.log.min_raw_clearance(end+1)=m.minDistance;
            obj.log.min_tube_clearance(end+1)=NaN;
            obj.log.trigger(end+1)=m.trigger;
            obj.log.replanned(end+1)=replanned;
            obj.log.replan_failed(end+1)=stats.attempted && ~stats.success;
            obj.log.compute_time(end+1)=comp;
            obj.log.reason{end+1}=reason;
        end
        
        function v=get_field(~,s,name,default)
            if isfield(s,name) && ~isempty(s.(name)), v=s.(name); else, v=default; end
        end
    end
    
    methods (Static)
        function v=basis_static(i,p,t,U)
            if p==0, v=double((U(i)<=t && t<U(i+1)) || (t==U(end)&&i==numel(U)-1)); return; end
            v=0; a=U(i+p)-U(i); b=U(i+p+1)-U(i+1);
            if a>0, v=v+(t-U(i))/a*REPLANNING_BSPLINE_HLC.basis_static(i,p-1,t,U); end
            if b>0, v=v+(U(i+p+1)-t)/b*REPLANNING_BSPLINE_HLC.basis_static(i+1,p-1,t,U); end
        end
    
        function [d,inside]=point_ellipsoid_signed_distance_static(p,o)
            y=o.R'*(p(:)-o.center(:)); r=o.radii(:); q=sum((y./r).^2); inside=q<1;
            if norm(y)<1e-14, d=-min(r); return; end
            a2=r.^2; yy=y.^2;
            if q>1, lo=0; hi=max(abs(y).*r); else, lo=-min(a2)*(1-1e-12); hi=0; end
            f=@(lam)sum(a2.*yy./(lam+a2).^2)-1;
            for kk=1:80
                mid=(lo+hi)/2; if f(mid)>0, lo=mid; else, hi=mid; end
            end
            lam=(lo+hi)/2; x=a2.*y./(lam+a2); d=sign(q-1)*norm(x-y);
        end
        
        function [x,w]=gauss8()
            x=[-0.9602898564975363 -0.7966664774136267 -0.5255324099163290 ...
               -0.1834346424956498 0.1834346424956498 0.5255324099163290 ...
                0.7966664774136267  0.9602898564975363]';
            w=[0.1012285362903763 0.2223810344533745 0.3137066458778873 ...
               0.3626837833783620 0.3626837833783620 0.3137066458778873 ...
               0.2223810344533745 0.1012285362903763]';
        end
        
        function Z=nullspace_direction(d)
            d=d(:); if abs(d(1))<0.9, a=[1;0;0]; else, a=[0;1;0]; end
            Z=a-d*(d'*a); if norm(Z)<1e-12, Z=[0;0;1]; else, Z=Z/norm(Z); end
        end
        
        function R=reference_rotation_from_force(a,yaw)
            g=[0;0;9.81]; F=a+g;
            if norm(F)<1e-9, b3=[0;0;1]; else, b3=F/norm(F); end
            b1d=[cos(yaw);sin(yaw);0]; b2=cross(b3,b1d);
            if norm(b2)<1e-9, b2=[-sin(yaw);cos(yaw);0]; else, b2=b2/norm(b2); end
            b1=cross(b2,b3); R=[b1 b2 b3];
        end
        
        function R=quat2rotm_col(q)
            q=q(:); q=q/norm(q); w=q(1); x=q(2); y=q(3); z=q(4);
            R=[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w); ...
               2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w); ...
               2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)];
        end
    end
end
