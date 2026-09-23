% classdef REPLANNING_BSPLINE < handle
%     % =========================================================================
%     % REPLANNING_BSPLINE_HLC
%     % Unified B-spline base online replanner for the simulator
%     % =========================================================================
% 
%     properties
%         self
%         base_ref
%         result
% 
%         % State
%         t_start = []
%         is_flight = false
%         xd_nominal = zeros(28,1);
% 
%         % Physical parameters
%         mQ = 1.0
%         mL = 0.2
%         L_cable = 1.0
%         gravity = 9.81
%         J = [1e-2;1e-2;2e-2]
%         dstxy = [0;0]
% 
%         % B-spline
%         degree = 7
%         nCtrl = 24
%         nSamples = 61
% 
%         % Control/replanning
%         dt_control = 0.01
%         max_replan_time = 0.007
%         trigger_distance = 7.0
%         min_replan_interval = 0.05
%         horizon_default = 4.0
% 
%         % Geometric body approximations
%         drone_radius = 0.30
%         load_radius = 0.15
%         cable_radius = 0.02
% 
%         % Dynamic/kinematic constraints
%         max_load_speed = 5.0
%         max_load_accel = 8.0
%         max_load_jerk = 30.0
%         max_load_snap = 100.0
%         max_drone_speed = 8.0
%         max_drone_accel = 15.0
%         max_tilt_rad = deg2rad(35)
%         max_cable_angle_rad = deg2rad(45)
% 
%         % Candidate search
%         duration_scales = [0.75 1.0 1.25 1.6 2.0 3.0]
%         deformation_scales = [0.25 0.5 1.0 1.5 2.0 3.0]
%         max_candidates = 36
% 
%         % Tracking-envelope settings
%         error_window = 2.0
%         tracking_quantile = 0.99
%         tracking_sigma_factor = 3.0
%         tracking_floor = 0.30
% 
%         % No-solution behavior
%         failsafe_mode = 'hold_if_safe_then_retry'
%         retry_period = 0.10
% 
%         % HLC generated gains
%         F1 = []
%         F2 = []
%         F3 = []
%         F4 = []
% 
%         % Runtime state
%         state = struct()
%         obstacles = struct('id',{},'center',{},'radii',{},'R',{}, ...
%                            'velocity',{},'timestamp',{},'enabled',{})
%         active = []
%         goal = struct('pos',[],'yaw',[],'pos_ders',[],'yaw_ders',[])
%         last_replan_time = -inf
%         last_attempt_time = -inf
%         next_id = 1
% 
%         % Logs
%         log = struct()
%         replan_active = false
%         warned_crash_load = []
%         warned_crash_drone = []
%         warned_margin_load = []
%         warned_margin_drone = []
%         last_warn_time = 0.0
%     end
% 
%     methods
%         function obj = REPLANNING_BSPLINE(self, base_ref, opts)
%             arguments
%                 self
%                 base_ref
%                 opts = struct()
%             end
% 
%             obj.self = self;
% 
%             if iscell(base_ref)
%                 base_ref_name = str2func(base_ref{1});
%                 obj.base_ref = base_ref_name(self, base_ref(2:end));
%             else
%                 obj.base_ref = base_ref;
%             end
% 
%             obj.result = struct();
%             obj.result.state = STATE_CLASS(struct('state_list', ["xd", "p", "q", "v"], 'num_list', [28, 3, 3, 3]));
% 
%             if isfield(opts, 'trigger_dist')
%                 obj.trigger_distance = opts.trigger_dist;
%             end
%             if isfield(opts, 'r_load')
%                 obj.load_radius = opts.r_load;
%             end
%             if isfield(opts, 'r_drone')
%                 obj.drone_radius = opts.r_drone;
%             end
%             if isfield(opts, 'safe_margin')
%                 obj.tracking_floor = opts.safe_margin;
%             end
% 
%             obj.reset_log();
%         end
% 
%         function result_out = do(obj, varargin)
%             time = varargin{1};
%             cha = varargin{2};
% 
%             if length(varargin) >= 4
%                 env = varargin{4};
%             else
%                 env = [];
%             end
% 
%             base_res = obj.base_ref.do(varargin{:});
%             xd_nom = base_res.state.xd;
%             obj.xd_nominal = xd_nom;
% 
%             try obj.mQ = obj.self.parameter.get("mass"); catch, end
%             try obj.L_cable = obj.self.parameter.get("cableL"); catch, end
%             try obj.J = diag(obj.self.parameter.get("I")); catch, end
%             try obj.dstxy = obj.self.parameter.get("dstxy"); catch, end
% 
%             if isprop(obj.self.estimator.result.state, "mL")
%                 obj.mL = obj.self.estimator.result.state.mL;
%             else
%                 try obj.mL = obj.self.parameter.get("loadmass"); catch, end
%             end
% 
%             try
%                 c_param = obj.self.controller.param;
%                 obj.F1 = c_param.F1; obj.F2 = c_param.F2; obj.F3 = c_param.F3; obj.F4 = c_param.F4;
%             catch
%             end
% 
%             if cha == 'f'
%                 if isempty(obj.t_start)
%                     obj.t_start = time.t;
%                 end
% 
%                 obs_list = [];
%                 if isstruct(env) && isfield(env, 'obstacle')
%                     obs_list = env.obstacle;
%                 elseif isobject(env) && isprop(env, 'obstacle')
%                     obs_list = env.obstacle;
%                 else
%                     try
%                         obs_list = ENVIRONMENT_OBSTACLE_ELLIPSE_MOVE(time.t);
%                     catch
%                         try
%                             obs_list = ENVIRONMENT_OBSTACLE_ELLIPSE();
%                         catch
%                             try
%                                 obs_list = ENVIRONMENT_OBSTACLE_HOCBF_LINK_XY();
%                             catch
%                             end
%                         end
%                     end
%                 end
% 
%                 obj.obstacles = struct('id',{},'center',{},'radii',{},'R',{},'velocity',{},'timestamp',{},'enabled',{});
%                 obj.next_id = 1;
% 
%                 for i = 1:length(obs_list)
%                     o = obs_list(i);
%                     v = [0;0;0];
%                     if isfield(o, 'v_center'), v = o.v_center(:);
%                     elseif isfield(o, 'v'), v = o.v(:);
%                     elseif isprop(o, 'v_center'), v = o.v_center(:);
%                     elseif isprop(o, 'v'), v = o.v(:);
%                     end
% 
%                     R_o = eye(3);
%                     if isfield(o, 'R_obs') && ~isempty(o.R_obs), R_o = o.R_obs;
%                     elseif isfield(o, 'R') && ~isempty(o.R), R_o = o.R;
%                     elseif isprop(o, 'R_obs') && ~isempty(o.R_obs), R_o = o.R_obs;
%                     elseif isprop(o, 'R') && ~isempty(o.R), R_o = o.R;
%                     end
% 
%                     radii = [1; 1; 1];
%                     if isfield(o, 'ellipsoid_radii') && ~isempty(o.ellipsoid_radii), radii = o.ellipsoid_radii(:);
%                     elseif isprop(o, 'ellipsoid_radii') && ~isempty(o.ellipsoid_radii), radii = o.ellipsoid_radii(:);
%                     elseif isfield(o, 'raw_param')
%                         rp = o.raw_param;
%                         typ = '';
%                         if isfield(o, 'type'), typ = o.type; end
%                         if isprop(o, 'type'), typ = o.type; end
%                         switch typ
%                             case 'cylinder'
%                                 radii = [rp(1)*sqrt(2); rp(1)*sqrt(2); (rp(2)/2)*sqrt(2)];
%                             case 'box'
%                                 radii = [rp(1)/2; rp(2)/2; rp(3)/2]*sqrt(3);
%                             case 'sphere'
%                                 radii = [rp(1); rp(1); rp(1)];
%                             otherwise
%                                 radii = [0.5; 0.5; 0.5];
%                         end
%                     end
% 
%                     p_c = [0;0;0];
%                     if isfield(o, 'p_center'), p_c = o.p_center(:); end
%                     if isprop(o, 'p_center'), p_c = o.p_center(:); end
% 
%                     obj.add_obstacle(p_c, radii, R_o, v, time.t, i);
%                 end
% 
%                 if isprop(obj.self.estimator.result.state, "pL")
%                     pL_cur = obj.self.estimator.result.state.pL;
%                     vL_cur = obj.self.estimator.result.state.vL;
%                 else
%                     pL_cur = obj.self.estimator.result.state.p - [0; 0; obj.L_cable];
%                     vL_cur = obj.self.estimator.result.state.v;
%                 end
% 
%                 if isprop(obj.self.estimator.result.state, "p")
%                     pQ_cur = obj.self.estimator.result.state.p;
%                     vQ_cur = obj.self.estimator.result.state.v;
%                 else
%                     pQ_cur = pL_cur + [0; 0; obj.L_cable];
%                     vQ_cur = vL_cur;
%                 end
% 
%                 s = struct();
%                 s.load_pos = pL_cur;
%                 s.load_vel = vL_cur;
%                 s.drone_pos = pQ_cur;
%                 try s.drone_yaw = obj.self.estimator.result.state.q(3); catch, s.drone_yaw = 0; end
% 
%                 obj.update_state(s);
% 
%                 pos_ders = zeros(3,7);
%                 yaw_ders = zeros(1,7);
%                 for k = 1:7
%                     idx = (k-1)*4 + 1;
%                     pos_ders(:,k) = xd_nom(idx:idx+2);
%                     yaw_ders(k) = xd_nom(idx+3);
%                 end
% 
%                 obj.set_goal(pos_ders(:,1), yaw_ders(1), pos_ders, yaw_ders);
% 
%                 out = obj.step_algo(time.t);
% 
%                 if out.reference.valid
%                     ref = out.reference;
%                     xd = zeros(28,1);
%                     for k = 1:7
%                         idx = (k-1)*4 + 1;
%                         xd(idx:idx+2) = ref.ders(:, k);
%                         xd(idx+3) = ref.yaw_ders(k);
%                     end
%                     obj.result.state.xd = xd;
%                 else
%                     obj.result.state.xd = xd_nom;
%                 end
%             else
%                 obj.result.state.xd = xd_nom;
%             end
% 
%             obj.result.state.p = obj.result.state.xd(1:3);
%             obj.result.state.v = obj.result.state.xd(5:7);
%             obj.result.state.q = [0; 0; obj.result.state.xd(4)];
% 
%             result_out = obj.result;
%         end
%     end
% 
%     methods (Access = private)
%         function set_goal(obj,pos,yaw,varargin)
%             obj.goal.pos = pos(:);
%             obj.goal.yaw = yaw;
%             obj.goal.pos_ders = zeros(3,7);
%             obj.goal.yaw_ders = zeros(1,7);
%             obj.goal.pos_ders(:,1) = obj.goal.pos;
%             obj.goal.yaw_ders(1) = yaw;
%             if nargin >= 4 && ~isempty(varargin{1})
%                 gd = varargin{1};
%                 obj.goal.pos_ders = gd(:,1:7);
%             end
%             if nargin >= 5 && ~isempty(varargin{2})
%                 yd = varargin{2};
%                 obj.goal.yaw_ders = yd(1:7);
%             end
%         end
% 
%         function id = add_obstacle(obj,center,radii,varargin)
%             R = eye(3); v = zeros(3,1); ts = 0; id = obj.next_id;
%             if nargin >= 4 && ~isempty(varargin{1}), R = varargin{1}; end
%             if nargin >= 5 && ~isempty(varargin{2}), v = varargin{2}(:); end
%             if nargin >= 6 && ~isempty(varargin{3}), ts = varargin{3}; end
%             if nargin >= 7 && ~isempty(varargin{4}), id = varargin{4}; end
%             k = numel(obj.obstacles)+1;
%             obj.obstacles(k) = struct('id',id,'center',center(:),'radii',radii(:), ...
%                 'R',R,'velocity',v,'timestamp',ts,'enabled',true);
%             obj.next_id = max(obj.next_id,id+1);
%         end
% 
%         function update_state(obj,s)
%             obj.state = s;
%             obj.state.time = obj.get_field(s,'time',0);
%             if ~isfield(obj.state,'drone_R') || isempty(obj.state.drone_R)
%                 if isfield(s,'drone_quat')
%                     obj.state.drone_R = REPLANNING_BSPLINE_HLC.quat2rotm_col(s.drone_quat(:));
%                 else
%                     obj.state.drone_R = eye(3);
%                 end
%             end
%             if ~isfield(obj.state,'cable_dir') || isempty(obj.state.cable_dir)
%                 d = s.drone_pos(:)-s.load_pos(:);
%                 if norm(d)>1e-9
%                     obj.state.cable_dir = d/norm(d);
%                 else
%                     obj.state.cable_dir = [0;0;1];
%                 end
%             end
%         end
% 
%         function out = step_algo(obj,time_now)
%             if isempty(obj.goal.pos)
%                 error('Call set_goal before step().');
%             end
%             if isempty(obj.state) || ~isfield(obj.state,'load_pos')
%                 error('Call update_state() before step().');
%             end
% 
%             obj.state.time = time_now;
%             t0 = tic;
% 
%             metrics = obj.current_metrics(time_now);
%             trigger = obj.should_replan(metrics,time_now);
% 
%             replanned = false;
%             reason = '';
%             candidate = obj.active;
% 
%             if trigger
%                 if time_now-obj.last_attempt_time >= obj.min_replan_interval
%                     obj.last_attempt_time = time_now;
%                     [candidate,ok,reason,stats] = obj.replan(time_now);
%                     if ok
%                         if ~obj.replan_active
%                             fprintf("\n=================================================================================\n");
%                             fprintf(" [B-SPLINE BLUEPRINT REPLANNER 診断レポート]  t = %.3f s\n", time_now);
%                             fprintf("=================================================================================\n");
%                             fprintf(" 1. 真値状態取得      : 荷物 pL, 機体 pQ (推定期直接抽出: 正常)\n");
%                             fprintf(" 2. 動的接近判定      : B-Spline軌道変形法による衝突検知 (PVO類似)\n");
%                             fprintf(" 3. 軌道変形スケール  : 探索空間 %.2f 倍 (最大候補数 %d)\n", obj.deformation_scales(end), obj.max_candidates);
%                             fprintf(" 4. 回避プラン策定    : 処理時間 = %6.2f ms (安全解発見)\n", stats.computeTime*1000);
%                             fprintf(" 5. 軌道評価スコア    : コスト = %.3f\n", candidate.validation.cost);
%                             fprintf("=================================================================================\n\n");
%                         end
%                         obj.active = candidate;
%                         obj.last_replan_time = time_now;
%                         obj.replan_active = true;
%                         replanned = true;
%                     end
%                 else
%                     stats = struct('attempted',false,'success',false,'reason','rate_limited');
%                 end
%             else
%                 stats = struct('attempted',false,'success',false,'reason','not_needed');
%             end
% 
%             ref = obj.evaluate_active_reference(time_now);
%             obj.update_log(time_now,metrics,ref,replanned,reason,stats,toc(t0));
% 
%             out = struct('reference',ref,'replanned',replanned, ...
%                 'trigger',trigger,'reason',reason,'stats',stats, ...
%                 'metrics',metrics);
%         end
% 
%         function [traj,ok,reason,stats] = replan(obj,time_now)
%             tStart = tic;
%             reason = '';
%             ok = false;
%             traj = obj.active;
%             stats = struct('attempted',true,'success',false,'reason','', ...
%                 'nCandidates',0,'nFeasible',0,'computeTime',0);
% 
%             T0 = obj.estimate_base_duration();
%             if isfield(obj.active,'T') && ~isempty(obj.active.T)
%                 T0 = max(T0,0.5*obj.active.T);
%             end
% 
%             [b0,bd0] = obj.start_boundary_derivatives(time_now);
%             [bT,bdT] = obj.goal_boundary_derivatives();
% 
%             base = obj.solve_bspline(b0,bT,T0);
%             if ~base.ok
%                 reason='boundary_solver_failed';
%                 stats.reason=reason; stats.computeTime=toc(tStart); return
%             end
%             base.yaw = obj.solve_bspline_yaw(bd0(4,:),bdT(4,:),T0);
% 
%             candList = obj.make_candidates(base,time_now);
% 
%             best = [];
%             bestScore = inf;
%             tBudget = obj.max_replan_time;
% 
%             for k=1:numel(candList)
%                 if toc(tStart) > tBudget
%                     break
%                 end
%                 stats.nCandidates = stats.nCandidates+1;
%                 c = candList{k};
%                 val = obj.validate_trajectory(c,time_now);
%                 if val.feasible
%                     score = val.cost;
%                     if score < bestScore
%                         bestScore = score;
%                         best = c;
%                         best.validation = val;
%                     end
%                     stats.nFeasible = stats.nFeasible+1;
%                 end
%             end
% 
%             stats.computeTime = toc(tStart);
% 
%             if ~isempty(best)
%                 traj = best;
%                 ok = true;
%                 reason = 'feasible_candidate_found';
%                 stats.success = true;
%                 stats.reason = reason;
%                 return
%             end
% 
%             if ~isempty(obj.active)
%                 av = obj.validate_trajectory(obj.active,time_now);
%                 if av.feasible
%                     traj = obj.active;
%                     reason = 'no_new_solution_active_trajectory_retained';
%                     stats.reason=reason;
%                     return
%                 end
%             end
% 
%             reason = 'no_feasible_solution';
%             stats.reason = reason;
%             stats.failsafe = obj.failsafe_action(time_now);
%         end
% 
%         function ref = evaluate_active_reference(obj,time_now)
%             if isempty(obj.active)
%                 ref = struct('valid',false);
%                 return
%             end
%             tr = obj.active;
% 
%             % Check if trajectory is completed
%             if time_now - tr.t0 >= tr.T
%                 if obj.replan_active
%                     fprintf("[B-SPLINE C^6] 回避完了! 公称軌道へ完全復帰 (t=%.3f s)\n\n", time_now);
%                 end
%                 obj.replan_active = false;
%                 obj.active = [];
%                 ref = struct('valid',false);
%                 return
%             end
% 
%             tau = min(max(time_now-tr.t0,0),tr.T);
%             [p,d] = obj.bspline_eval(tr.P,tau,tr.T,6);
%             if isfield(tr, 'Y')
%                 [y,yd] = obj.bspline_eval(tr.Y.P,tau,tr.T,6);
%             else
%                 [y,yd] = obj.bspline_eval(tr.yaw.P,tau,tr.T,6);
%             end
% 
%             ref.valid = true;
%             ref.time = time_now;
%             ref.tau = tau;
%             ref.pos = p(:,1);
%             ref.ders = p;
%             ref.yaw = y(1,1);
%             ref.yaw_ders = y;
%             ref.drone = obj.flatness_from_load_reference(p,y);
%         end
% 
%         function ref = make_reference_at(obj,tr,time_now)
%             tau=min(max(time_now-tr.t0,0),tr.T);
%             [p,~]=obj.bspline_eval(tr.P,tau,tr.T,6);
%             if isfield(tr, 'Y')
%                 [y,~]=obj.bspline_eval(tr.Y.P,tau,tr.T,6);
%             else
%                 [y,~]=obj.bspline_eval(tr.yaw.P,tau,tr.T,6);
%             end
%             ref=struct('valid',true,'time',time_now,'tau',tau, ...
%                 'pos',p(:,1),'ders',p,'yaw',y(1),'yaw_ders',y);
%             ref.drone=obj.flatness_from_load_reference(p,y);
%         end
% 
%         function reset_log(obj)
%             obj.log=struct('time',[],'load_pos_error',[],'drone_pos_error',[], ...
%                 'cable_angle',[],'tracking_margin',[],'min_raw_clearance',[], ...
%                 'min_tube_clearance',[],'trigger',[],'replanned',[], ...
%                 'replan_failed',[],'compute_time',[],'reason',{{}}, ...
%                 'hlc_checked',[],'hlc_thrust',[],'hlc_torque',[]);
%         end
% 
%         function [b0,bd0] = start_boundary_derivatives(obj,tNow)
%             if ~isempty(obj.active) && isfield(obj.active,'P')
%                 r=obj.make_reference_at(obj.active,tNow);
%                 b0=[r.ders; r.yaw_ders];
%                 bd0=b0;
%                 return
%             end
%             p=obj.state.load_pos(:);
%             v=obj.get_field(obj.state,'load_vel',zeros(3,1));
%             b0=zeros(4,7);
%             b0(1:3,1)=p;
%             b0(1:3,2)=v(:);
%             yaw=obj.get_field(obj.state,'drone_yaw',0);
%             b0(4,1)=yaw;
%             bd0=b0;
%         end
% 
%         function [bT,bdT] = goal_boundary_derivatives(obj)
%             bT=zeros(4,7);
%             bT(1:3,:)=obj.goal.pos_ders;
%             bT(4,:)=obj.goal.yaw_ders;
%             bdT=bT;
%         end
% 
%         function T=estimate_base_duration(obj)
%             p0=obj.state.load_pos(:);
%             d=norm(obj.goal.pos(:)-p0);
%             T=max([obj.horizon_default,d/max(obj.max_load_speed,1e-3), ...
%                 sqrt(4*d/max(obj.max_load_accel,1e-3))]);
%             if ~isempty(obj.active)
%                 T=max(T,0.5*obj.active.T);
%             end
%         end
% 
%         function tr=solve_bspline(obj,b0,bT,T)
%             n=obj.nCtrl; p=obj.degree;
%             U=obj.make_knots(n,p,T);
%             A=zeros(14,n);
%             for k=0:6
%                 A(k+1,:)=obj.basis_derivative_row(U,p,0,k,T);
%                 A(8+k,:)=obj.basis_derivative_row(U,p,T,k,T);
%             end
%             H=obj.minimum_snap_H(U,p,T);
%             H=H+1e-9*eye(n);
%             K=[H A'; A zeros(14)];
%             bx=[b0(1,:)';bT(1,:)'];
%             by=[b0(2,:)';bT(2,:)'];
%             bz=[b0(3,:)';bT(3,:)'];
%             Px=K\[zeros(n,1);bx];
%             Py=K\[zeros(n,1);by];
%             Pz=K\[zeros(n,1);bz];
%             tr.ok=true; tr.T=T; tr.t0=obj.state.time; tr.U=U;
%             tr.P=[Px(1:n),Py(1:n),Pz(1:n)];
%             tr.degree=p;
%         end
% 
%         function Y=solve_bspline_yaw(obj,y0,yT,T)
%             n=obj.nCtrl; p=obj.degree; U=obj.make_knots(n,p,T);
%             A=zeros(14,n);
%             for k=0:6
%                 A(k+1,:)=obj.basis_derivative_row(U,p,0,k,T);
%                 A(8+k,:)=obj.basis_derivative_row(U,p,T,k,T);
%             end
%             H=obj.minimum_snap_H(U,p,T)+1e-9*eye(n);
%             K=[H A';A zeros(14)];
%             z=K\[zeros(n,1);[y0(:);yT(:)]];
%             Y=struct('T',T,'t0',obj.state.time,'U',U,'P',z(1:n),'degree',p);
%         end
% 
%         function cands=make_candidates(obj,base,tNow)
%             cands={base};
%             if isempty(obj.obstacles), return; end
%             obs=obj.predicted_obstacles(tNow);
%             baseEval=obj.sample_trajectory(base);
%             dirs=[];
%             for j=1:numel(obs)
%                 o=obs(j);
%                 [dmin,pt,normal]=obj.trajectory_obstacle_hint(baseEval,o);
%                 if dmin < obj.trigger_distance*1.5
%                     dirs=[dirs,normal];
%                 end
%             end
%             if isempty(dirs), return; end
%             dir=sum(dirs,2);
%             if norm(dir)<1e-9, dir=[1;0;0]; end
%             dir=dir/norm(dir);
% 
%             D=[dir,REPLANNING_BSPLINE_HLC.nullspace_direction(dir),cross(dir,[0;0;1])];
%             for q=1:size(D,2)
%                 if norm(D(:,q))<1e-9, continue; end
%                 D(:,q)=D(:,q)/norm(D(:,q));
%                 for s=obj.deformation_scales
%                     if numel(cands)>=obj.max_candidates, return; end
%                     c=base;
%                     c.P=obj.deform_preserving_C6(base.P,base.U,base.degree,D(:,q),s);
%                     cands{end+1}=c;
%                     if numel(cands)>=obj.max_candidates, return; end
%                     c.P=obj.deform_preserving_C6(base.P,base.U,base.degree,-D(:,q),s);
%                     cands{end+1}=c;
%                 end
%             end
% 
%             relScale=obj.relative_speed_scale(tNow,base);
%             for sc=obj.duration_scales*relScale
%                 if numel(cands)>=obj.max_candidates, return; end
%                 Tnew=max(0.25,base.T*sc);
%                 tr=obj.solve_bspline_from_boundary_of(base,Tnew);
%                 cands{end+1}=tr;
%             end
%         end
% 
%         function tr=solve_bspline_from_boundary_of(obj,base,T)
%             [p,~]=obj.bspline_eval(base.P,0,base.T,6);
%             [pe,~]=obj.bspline_eval(base.P,base.T,base.T,6);
%             if isfield(base, 'Y')
%                 [y,~]=obj.bspline_eval(base.Y.P,0,base.Y.T,6);
%                 [ye,~]=obj.bspline_eval(base.Y.P,base.Y.T,base.Y.T,6);
%             else
%                 [y,~]=obj.bspline_eval(base.yaw.P,0,base.yaw.T,6);
%                 [ye,~]=obj.bspline_eval(base.yaw.P,base.yaw.T,base.yaw.T,6);
%             end
%             b0=p; bT=pe; y0=y; yT=ye;
%             tr=obj.solve_bspline(b0,bT,T);
%             tr.yaw=obj.solve_bspline_yaw(y0(1,:),yT(1,:),T);
%         end
% 
%         function Pnew=deform_preserving_C6(obj,P,U,p,dir,scale)
%             n=size(P,1);
%             A=zeros(14,n);
%             for k=0:6
%                 A(k+1,:)=obj.basis_derivative_row(U,p,0,k,P);
%                 A(8+k,:)=obj.basis_derivative_row(U,p,U(end),k,P);
%             end
%             Z=null(A,'r');
%             if isempty(Z), Pnew=P; return; end
%             w=zeros(n,1);
%             s=linspace(0,1,n)';
%             w=(sin(pi*s).^2).^2;
%             raw=w*dir(:)';
%             D=Z*(Z'*raw);
%             nr=norm(D,'fro');
%             if nr<1e-12, Pnew=P; return; end
%             Pnew=P+(0.25*scale)*D/nr*n;
%         end
% 
%         function val=validate_trajectory(obj,tr,tNow)
%             S=obj.sample_trajectory(tr);
%             obs=obj.predicted_obstacles(tNow);
%             val=struct('feasible',true,'cost',0,'minRaw',inf,'minTube',inf, ...
%                 'maxSpeed',0,'maxAccel',0,'maxJerk',0,'maxSnap',0, ...
%                 'maxDroneSpeed',0,'maxDroneAccel',0,'maxTilt',0, ...
%                 'maxCableAngle',0,'hlcChecked',false,'hlcThrust',NaN,'hlcTorque',NaN);
% 
%             val.maxSpeed=max(vecnorm(S.loadDers(:,2:end),2,1),[],'all');
%             val.maxAccel=max(vecnorm(S.loadDers(:,3:end),2,1),[],'all');
%             val.maxJerk=max(vecnorm(S.loadDers(:,4:end),2,1),[],'all');
%             val.maxSnap=max(vecnorm(S.loadDers(:,5:end),2,1),[],'all');
% 
%             if val.maxSpeed>obj.max_load_speed || val.maxAccel>obj.max_load_accel || ...
%                     val.maxJerk>obj.max_load_jerk || val.maxSnap>obj.max_load_snap
%                 val.feasible=false;
%             end
% 
%             for k=1:numel(obs)
%                 o=obs(k);
%                 for i=1:size(S.load,2)
%                     [rawd,inside]=obj.point_ellipsoid_signed_distance(S.load(:,i),o);
%                     [~,din]=obj.point_ellipsoid_signed_distance(S.drone(:,i),o);
%                     val.minRaw=min([val.minRaw,rawd,din]);
%                     if inside || rawd<=0 || din<=0
%                         val.feasible=false;
%                     end
% 
%                     dl=obj.point_clearance_to_inflated_ellipsoid(S.load(:,i),o,obj.load_radius);
%                     dq=obj.point_clearance_to_inflated_ellipsoid(S.drone(:,i),o,obj.drone_radius);
%                     if dl<=0 || dq<=0, val.feasible=false; end
% 
%                     if i<size(S.load,2)
%                         dc=obj.segment_ellipsoid_clearance(S.drone(:,i),S.load(:,i),o,obj.cable_radius);
%                         val.minRaw=min(val.minRaw,dc);
%                         if dc<=0, val.feasible=false; end
%                     end
%                     tube=obj.tracking_margin_at(tNow);
%                     dl2=obj.point_clearance_to_inflated_ellipsoid(S.load(:,i),o,obj.load_radius+tube);
%                     dq2=obj.point_clearance_to_inflated_ellipsoid(S.drone(:,i),o,obj.drone_radius+tube);
%                     val.minTube=min([val.minTube,dl2,dq2]);
%                     if i<size(S.load,2)
%                         dc2=obj.segment_ellipsoid_clearance(S.drone(:,i),S.load(:,i),o,obj.cable_radius+tube);
%                         val.minTube=min(val.minTube,dc2);
%                     end
%                 end
%             end
% 
%             val.maxDroneSpeed=max(vecnorm(S.droneDers(:,2:end),2,1),[],'all');
%             val.maxDroneAccel=max(vecnorm(S.droneDers(:,3:end),2,1),[],'all');
%             val.maxTilt=max(S.tilt);
%             val.maxCableAngle=max(S.cableAngle);
% 
%             if val.maxDroneSpeed>obj.max_drone_speed || val.maxDroneAccel>obj.max_drone_accel || ...
%                     val.maxTilt>obj.max_tilt_rad || val.maxCableAngle>obj.max_cable_angle_rad
%                 val.feasible=false;
%             end
% 
%             if val.minTube < 0
%                 val.feasible=false;
%             end
% 
%             val.cost=tr.T + 0.05*(val.maxAccel/obj.max_load_accel)^2 + ...
%                 0.05*(val.maxCableAngle/obj.max_cable_angle_rad)^2;
%         end
% 
%         function S=sample_trajectory(obj,tr)
%             tt=linspace(0,tr.T,obj.nSamples);
%             [~,D]=obj.bspline_eval(tr.P,tt,tr.T,6);
%             if isfield(tr, 'Y')
%                 [~,YD]=obj.bspline_eval(tr.Y.P,tt,tr.T,6);
%             else
%                 [~,YD]=obj.bspline_eval(tr.yaw.P,tt,tr.T,6);
%             end
%             N=numel(tt);
%             S.t=tt; 
%             S.load = reshape(D(:,1,:), [3, N]); 
%             S.loadDers=D; 
%             S.yaw = YD(1, :); 
%             S.yawDers=YD;
%             S.drone=zeros(3,N); S.droneDers=zeros(3,7,N); S.cable=zeros(3,N);
%             S.cableAngle=zeros(1,N); S.tilt=zeros(1,N);
%             for i=1:N
%                 rr=obj.flatness_from_load_reference(D(:,:,i),YD(:,i));
%                 S.drone(:,i)=rr.pos; S.droneDers(:,:,i)=rr.ders;
%                 S.cable(:,i)=rr.cable_dir; S.cableAngle(i)=rr.cable_angle; S.tilt(i)=rr.tilt;
%             end
%         end
% 
%         function r=flatness_from_load_reference(obj,p,y)
%             p0=p(1:3,1); v=p(1:3,2); a=p(1:3,3);
%             gvec=[0;0;obj.gravity]; Fc=obj.mL*(a+gvec);
%             if norm(Fc)<1e-8, pT=[0;0;1]; else, pT=-Fc/norm(Fc); end
%             if size(p,2)>=4
%                 da=p(1:3,4); Fdot=obj.mL*da; n=norm(Fc);
%                 pTdot=-(Fdot/n-Fc*(Fc'*Fdot)/n^3);
%             else
%                 pTdot=zeros(3,1);
%             end
%             pQ=p0-obj.L_cable*pT; vQ=v-obj.L_cable*pTdot;
%             aQ=a;
%             if size(p,2)>=4, jQ=p(1:3,4); else, jQ=zeros(3,1); end
%             R=REPLANNING_BSPLINE_HLC.reference_rotation_from_force(aQ,y(1));
%             r.pos=pQ; r.vel=vQ; r.acc=aQ; r.jerk=jQ;
%             r.ders=zeros(3,7); r.ders(:,1)=pQ; r.ders(:,2)=vQ; r.ders(:,3)=aQ; r.ders(:,4)=jQ;
%             r.cable_dir=pT; r.cable_angle=acos(max(-1,min(1,abs(pT(3)))));
%             r.tilt=acos(max(-1,min(1,R(3,3)))); r.R=R; r.yaw=y(1);
%         end
% 
%         function m=current_metrics(obj,tNow)
%             m=struct('trigger',false,'droneDistance',inf,'loadDistance',inf, ...
%                 'minDistance',inf,'trackingMargin',obj.tracking_margin_at(tNow), ...
%                 'cableAngle',0,'loadError',0,'droneError',0);
%             obs=obj.predicted_obstacles(tNow);
%             % Initialize warning arrays if size changes
%             n_obs = numel(obs);
%             if length(obj.warned_crash_load) ~= n_obs
%                 obj.warned_crash_load = false(n_obs, 1);
%                 obj.warned_crash_drone = false(n_obs, 1);
%                 obj.warned_margin_load = false(n_obs, 1);
%                 obj.warned_margin_drone = false(n_obs, 1);
%             end
% 
%             for i=1:n_obs
%                 [d1,~]=obj.point_ellipsoid_signed_distance(obj.state.drone_pos(:),obs(i));
%                 [d2,~]=obj.point_ellipsoid_signed_distance(obj.state.load_pos(:),obs(i));
% 
%                 % Logging for drone
%                 if d1 <= 0 && ~obj.warned_crash_drone(i)
%                     fprintf(2, "[CRITICAL ALARM] 機体が障害物%dに衝突! (t=%.3f s, 侵入深さ: %.3f m)\n", i, tNow, -d1);
%                     obj.warned_crash_drone(i) = true;
%                 elseif d1 <= (obj.drone_radius + 0.5) && ~obj.warned_margin_drone(i) && d1 > 0
%                     fprintf("[SAFETY WARN] 機体が障害物%dのマージン帯侵入 (t=%.3f s, 残余距離: %.3f m)\n", i, tNow, d1);
%                     obj.warned_margin_drone(i) = true;
%                 end
% 
%                 % Logging for load
%                 if d2 <= 0 && ~obj.warned_crash_load(i)
%                     fprintf(2, "[CRITICAL ALARM] 荷物が障害物%dに衝突! (t=%.3f s, 侵入深さ: %.3f m)\n", i, tNow, -d2);
%                     obj.warned_crash_load(i) = true;
%                 elseif d2 <= (obj.load_radius + 0.5) && ~obj.warned_margin_load(i) && d2 > 0
%                     fprintf("[SAFETY WARN] 荷物が障害物%dのマージン帯侵入 (t=%.3f s, 残余距離: %.3f m)\n", i, tNow, d2);
%                     obj.warned_margin_load(i) = true;
%                 end
% 
%                 m.droneDistance=min(m.droneDistance,d1);
%                 m.loadDistance=min(m.loadDistance,d2);
%             end
%             m.minDistance=min(m.droneDistance,m.loadDistance);
%             if m.minDistance<=obj.trigger_distance
%                 m.trigger=true;
%             end
%             if isfield(obj.state,'cable_dir')
%                 m.cableAngle=acos(max(-1,min(1,abs(obj.state.cable_dir(3)))));
%             end
%             if ~isempty(obj.active)
%                 rr=obj.evaluate_active_reference(tNow);
%                 m.loadError=norm(obj.state.load_pos(:)-rr.pos(:));
%                 m.droneError=norm(obj.state.drone_pos(:)-rr.drone.pos(:));
%             end
%         end
% 
%         function tf=should_replan(obj,m,tNow)
%             if tNow-obj.last_replan_time < obj.min_replan_interval
%                 tf=false; return
%             end
%             tf=m.trigger;
%             if ~tf && ~isempty(obj.active)
%                 tube=obj.tracking_margin_at(tNow);
%                 if tube>obj.tracking_floor
%                     rr=obj.evaluate_active_reference(tNow);
%                     obs=obj.predicted_obstacles(tNow);
%                     for i=1:numel(obs)
%                         [d,~]=obj.point_ellipsoid_signed_distance(rr.pos,obs(i));
%                         if d<=tube, tf=true; break; end
%                     end
%                 end
%             end
%         end
% 
%         function margin=tracking_margin_at(obj,~)
%             if ~isfield(obj.log,'time') || numel(obj.log.time)<5
%                 margin=obj.tracking_floor;
%                 return
%             end
%             n=numel(obj.log.time); t0=obj.log.time(end);
%             idx=obj.log.time>=t0-obj.error_window;
%             e=[obj.log.load_pos_error(idx),obj.log.drone_pos_error(idx)]; e=e(isfinite(e));
%             if isempty(e), margin=obj.tracking_floor; return; end
%             q=quantile(e,obj.tracking_quantile); mu=mean(e); sig=std(e);
%             margin=max([q,mu+obj.tracking_sigma_factor*sig,obj.tracking_floor]);
%         end
% 
%         function obs=predicted_obstacles(obj,tNow)
%             obs=obj.obstacles;
%             for i=1:numel(obs)
%                 if ~obs(i).enabled, continue; end
%                 obs(i).center=obs(i).center(:)+obs(i).velocity(:)*(tNow-obs(i).timestamp);
%             end
%             obs=obs([obs.enabled]);
%         end
% 
%         function [dmin,pt,n]=trajectory_obstacle_hint(obj,S,o)
%             dmin=inf; pt=S.load(:,1); n=[1;0;0];
%             for i=1:size(S.load,2)
%                 [d,inside]=obj.point_ellipsoid_signed_distance(S.load(:,i),o);
%                 if d<dmin, dmin=d; pt=S.load(:,i); n=obj.ellipsoid_outward_normal(pt,o); end
%                 [d2,~]=obj.point_ellipsoid_signed_distance(S.drone(:,i),o);
%                 if d2<dmin, dmin=d2; pt=S.drone(:,i); n=obj.ellipsoid_outward_normal(pt,o); end
%             end
%             if norm(n)<1e-9, n=[1;0;0]; end
%         end
% 
%         function scale=relative_speed_scale(obj,tNow,tr)
%             S=obj.sample_trajectory(tr); vref=S.loadDers(:,2);
%             obs=obj.predicted_obstacles(tNow); closing=0;
%             for i=1:numel(obs)
%                 [~,pt,n]=obj.trajectory_obstacle_hint(S,obs(i));
%                 vv=obs(i).velocity(:);
%                 k=max(0,dot(vv-vref(:,1),n));
%                 closing=max(closing,k);
%             end
%             scale=1+0.5*closing/max(obj.max_load_speed,1e-3);
%         end
% 
%         function row=basis_derivative_row(obj,U,p,t,k,T)
%             n=numel(U)-p-1; epsT=1e-9*max(1,T);
%             if abs(t-T)<epsT, t=T-epsT; end
%             row=zeros(1,n);
%             for i=1:n, row(i)=obj.bspline_basis_derivative(i,p,k,t,U); end
%         end
% 
%         function H=minimum_snap_H(obj,U,p,T)
%             n=numel(U)-p-1; H=zeros(n);
%             [x,w]=REPLANNING_BSPLINE_HLC.gauss8();
%             for s=p+1:numel(U)-p-1
%                 a=U(s); b=U(s+1);
%                 if b<=a, continue; end
%                 for q=1:numel(x)
%                     tq=(a+b)/2+(b-a)/2*x(q); B=zeros(1,n);
%                     for i=1:n, B(i)=obj.bspline_basis_derivative(i,p,4,tq,U); end
%                     H=H+w(q)*(b-a)/2*(B'*B);
%                 end
%             end
%             H=H/max(T,1e-9);
%         end
% 
%         function U=make_knots(~,n,p,T)
%             nInt=n-p-1; internal=linspace(0,T,nInt+2); internal=internal(2:end-1);
%             U=[zeros(1,p+1),internal,T*ones(1,p+1)];
%         end
% 
%         function [D,Db]=bspline_eval(obj,P,t,T,maxOrder)
%             t=t(:).'; n=size(P,1); p=obj.degree; U=obj.make_knots(n,p,T);
%             if size(P,2)==1
%                 D=zeros(maxOrder+1,numel(t));
%                 for j=1:numel(t)
%                     for k=0:maxOrder
%                         row=obj.basis_derivative_row(U,p,t(j),k,T);
%                         D(k+1,j)=row*P;
%                     end
%                 end
%                 Db=D; return
%             end
%             D=zeros(size(P,2),maxOrder+1,numel(t));
%             for j=1:numel(t)
%                 for k=0:maxOrder
%                     row=obj.basis_derivative_row(U,p,t(j),k,T);
%                     D(:,k+1,j)=(row*P).';
%                 end
%             end
%             Db=D;
%         end
% 
%         function v=bspline_basis_derivative(obj,i,p,k,t,U)
%             if k==0, v=obj.bspline_basis(i,p,t,U); return; end
%             if p==0, v=0; return; end
%             a=U(i+p)-U(i); b=U(i+p+1)-U(i+1); v=0;
%             if a>0, v=v+p/a*obj.bspline_basis_derivative(i,p-1,k-1,t,U); end
%             if b>0, v=v-p/b*obj.bspline_basis_derivative(i+1,p-1,k-1,t,U); end
%         end
% 
%         function v=bspline_basis(~,i,p,t,U)
%             if p==0
%                 if (U(i)<=t && t<U(i+1)) || (t==U(end) && i==numel(U)-1), v=1; else, v=0; end
%                 return
%             end
%             v=0; a=U(i+p)-U(i); b=U(i+p+1)-U(i+1);
%             if a>0, v=v+(t-U(i))/a*REPLANNING_BSPLINE_HLC.basis_static(i,p-1,t,U); end
%             if b>0, v=v+(U(i+p+1)-t)/b*REPLANNING_BSPLINE_HLC.basis_static(i+1,p-1,t,U); end
%         end
% 
%         function [d,inside]=point_ellipsoid_signed_distance(~,p,o)
%             y=o.R'*(p(:)-o.center(:)); r=o.radii(:); q=sum((y./r).^2);
%             inside=q<1;
%             if norm(y)<1e-14, d=-min(r); return; end
%             a2=r.^2; yy=y.^2;
%             if q>1, lo=0; hi=max(abs(y).*r); else, lo=-min(a2)*(1-1e-12); hi=0; end
%             f=@(lam)sum(a2.*yy./(lam+a2).^2)-1;
%             for kk=1:80
%                 mid=(lo+hi)/2;
%                 if f(mid)>0, lo=mid; else, hi=mid; end
%             end
%             lam=(lo+hi)/2; x=a2.*y./(lam+a2);
%             d0=norm(x-y); d=sign(q-1)*d0;
%         end
% 
%         function d=point_clearance_to_inflated_ellipsoid(~,p,o,radd)
%             oo=o; oo.radii=o.radii(:)+radd;
%             [d,~]=REPLANNING_BSPLINE_HLC.point_ellipsoid_signed_distance_static(p,oo);
%         end
% 
%         function d=segment_ellipsoid_clearance(~,a,b,o,radd)
%             oo=o; oo.radii=o.radii(:)+radd;
%             A=oo.R'*(a(:)-oo.center(:)); B=oo.R'*(b(:)-oo.center(:));
%             d=B-A; aa=sum((d./oo.radii).^2); bb=2*sum((A./oo.radii).*(d./oo.radii));
%             if aa<1e-14
%                 q=sum((A./oo.radii).^2);
%                 d=sign(1-q)*min(oo.radii); return
%             end
%             s=-bb/(2*aa); s=max(0,min(1,s));
%             q=sum(((A+s*d)./oo.radii).^2);
%             if q<=1, d=-min(oo.radii)*(1-q); else, d=(sqrt(q)-1)*min(oo.radii); end
%         end
% 
%         function n=ellipsoid_outward_normal(~,p,o)
%             y=o.R'*(p(:)-o.center(:)); n=o.R*(y./(o.radii(:).^2));
%             if norm(n)>1e-12, n=n/norm(n); else, n=[1;0;0]; end
%         end
% 
%         function act=failsafe_action(obj,tNow)
%             act=struct('mode',obj.failsafe_mode,'time',tNow,'action','retry');
%         end
% 
%         function update_log(obj,tNow,m,ref,replanned,reason,stats,comp)
%             obj.log.time(end+1)=tNow;
%             obj.log.load_pos_error(end+1)=m.loadError;
%             obj.log.drone_pos_error(end+1)=m.droneError;
%             obj.log.cable_angle(end+1)=m.cableAngle;
%             obj.log.tracking_margin(end+1)=m.trackingMargin;
%             obj.log.min_raw_clearance(end+1)=m.minDistance;
%             obj.log.min_tube_clearance(end+1)=NaN;
%             obj.log.trigger(end+1)=m.trigger;
%             obj.log.replanned(end+1)=replanned;
%             obj.log.replan_failed(end+1)=stats.attempted && ~stats.success;
%             obj.log.compute_time(end+1)=comp;
%             obj.log.reason{end+1}=reason;
%         end
% 
%         function v=get_field(~,s,name,default)
%             if isfield(s,name) && ~isempty(s.(name)), v=s.(name); else, v=default; end
%         end
%     end
% 
%     methods (Static)
%         function v=basis_static(i,p,t,U)
%             if p==0, v=double((U(i)<=t && t<U(i+1)) || (t==U(end)&&i==numel(U)-1)); return; end
%             v=0; a=U(i+p)-U(i); b=U(i+p+1)-U(i+1);
%             if a>0, v=v+(t-U(i))/a*REPLANNING_BSPLINE_HLC.basis_static(i,p-1,t,U); end
%             if b>0, v=v+(U(i+p+1)-t)/b*REPLANNING_BSPLINE_HLC.basis_static(i+1,p-1,t,U); end
%         end
% 
%         function [d,inside]=point_ellipsoid_signed_distance_static(p,o)
%             y=o.R'*(p(:)-o.center(:)); r=o.radii(:); q=sum((y./r).^2); inside=q<1;
%             if norm(y)<1e-14, d=-min(r); return; end
%             a2=r.^2; yy=y.^2;
%             if q>1, lo=0; hi=max(abs(y).*r); else, lo=-min(a2)*(1-1e-12); hi=0; end
%             f=@(lam)sum(a2.*yy./(lam+a2).^2)-1;
%             for kk=1:80
%                 mid=(lo+hi)/2; if f(mid)>0, lo=mid; else, hi=mid; end
%             end
%             lam=(lo+hi)/2; x=a2.*y./(lam+a2); d=sign(q-1)*norm(x-y);
%         end
% 
%         function [x,w]=gauss8()
%             x=[-0.9602898564975363 -0.7966664774136267 -0.5255324099163290 ...
%                -0.1834346424956498 0.1834346424956498 0.5255324099163290 ...
%                 0.7966664774136267  0.9602898564975363]';
%             w=[0.1012285362903763 0.2223810344533745 0.3137066458778873 ...
%                0.3626837833783620 0.3626837833783620 0.3137066458778873 ...
%                0.2223810344533745 0.1012285362903763]';
%         end
% 
%         function Z=nullspace_direction(d)
%             d=d(:); if abs(d(1))<0.9, a=[1;0;0]; else, a=[0;1;0]; end
%             Z=a-d*(d'*a); if norm(Z)<1e-12, Z=[0;0;1]; else, Z=Z/norm(Z); end
%         end
% 
%         function R=reference_rotation_from_force(a,yaw)
%             g=[0;0;9.81]; F=a+g;
%             if norm(F)<1e-9, b3=[0;0;1]; else, b3=F/norm(F); end
%             b1d=[cos(yaw);sin(yaw);0]; b2=cross(b3,b1d);
%             if norm(b2)<1e-9, b2=[-sin(yaw);cos(yaw);0]; else, b2=b2/norm(b2); end
%             b1=cross(b2,b3); R=[b1 b2 b3];
%         end
% 
%         function R=quat2rotm_col(q)
%             q=q(:); q=q/norm(q); w=q(1); x=q(2); y=q(3); z=q(4);
%             R=[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w); ...
%                2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w); ...
%                2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)];
%         end
%     end
% end

% classdef REPLANNING_BSPLINE < handle
%     % =========================================================================
%     % REPLANNING_BSPLINE
%     % 1.模擬センサー：機体重心 pQ および荷物位置 pL を点（Point）として扱い、楕円体境界面までのユークリッド最短距離を厳密に計算して検知を判定
% 
%     % =========================================================================
%     properties
%         self % ドローンエージェント自身 (推定器 estimator やパラメータ parameter を保持) 
%         base_ref % 公称参照軌道生成オブジェクト
%         result                % 出力結果構造体 (目標状態 xd, pRef, vRef, yawRef, 検知情報)
% 
%         trigger_dist = 7.0;   % 接近検知の閾値 [m] (表面間距離がこれ以下になるとアラート)
%         % r_load       = 0.15;  % 荷物保護半径 [m] (必要に応じて将来のマージン計算等に使用)
%         % r_drone      = 0.30;  % 機体保護半径 [m] (必要に応じて将来のマージン計算等に使用)
%     end
% 
%     methods (Access = public)
%         % =====================================================================
%         % コンストラクタ: クラスの初期化と外部設定 (opts) の反映
%         % =====================================================================
%         function obj = REPLANNING_BSPLINE(self, base_ref, opts)
%             arguments
%                 self                  % 必須: エージェントインスタンス
%                 base_ref              % 必須: 通常飛行用の公称軌道インスタンス
%                 opts = struct()       % 任意: 外部からパラメータを変更するための構造体
%             end
% 
%             obj.self = self;
%             obj.base_ref = base_ref;
% 
%             if isfield(opts, 'trigger_dist'), obj.trigger_dist = opts.trigger_dist; end % 外で定義されていたらデフォルト値を上書き　センサー代わりの検知範囲
%             % if isfield(opts, 'r_load'),       obj.r_load       = opts.r_load;       end % 外で定義されていたらデフォルト値を上書き　牽引物を近似した球体
%             % if isfield(opts, 'r_drone'),      obj.r_drone      = opts.r_drone;      end % 外で定義されていたらデフォルト値を上書き　機体を近似した球体
% 
%             % base_ref の result 構造体をそのまま継承 (直下は state のみ)
%             obj.result = base_ref.result;
%             % --- 【重要】STATE_CLASS に検知用プロパティを動的追加 (dynamicprops) ---
%             % これにより、state 内に正式な記録領域が作成され、ロガーで抽出可能になる
%             sensor_props = ["time", "pQ", "pL", "detected_point", ...
%                 "drone_inside_obstacle_point", "load_inside_obstacle_point", ...
%                 "drone_min_dist_point", "load_min_dist_point", "min_dist_point", ...
%                 "drone_obstacle_id_point", "load_obstacle_id_point", "min_obstacle_id_point", ...
%                 "min_source_point", "detected_obstacle_count_point"];
%             for p_name = sensor_props
%                 if ~isprop(obj.result.state, p_name)
%                     addprop(obj.result.state, p_name);
%                 end
%             end
% 
%             % 初期ダミー値のセット
%             obj.clear_state_sensor_values(0.0);
%         end
% 
%         % =====================================================================
%         % do: 制御周期ごと (例: 25ms周期) にメインループから呼び出される実行メソッド
%         % 入力: 
%         %   varargin{1}: time (現在の時刻 struct: time.t, time.dt など)
%         %   varargin{2}: cha  (フェーズ文字列: 'f' = 飛行中, 't' = 離陸など)
%         %   varargin{4}: env  (環境構造体、障害物リストを内包する場合あり)
%         % 出力:
%         %   result_out : 下流のコントローラやロガーが受け取る目標状態・検知結果
%         % =====================================================================
%         function result_out = do(obj, varargin)
%             time = varargin{1}; % varargin{1}: time (現在の時刻 struct: time.t, time.dt など)
%             cha = varargin{2}; % varargin{2}: cha  (フェーズ文字列: 'f' = 飛行中, 't' = 離陸など)
% 
%             % --- 公称目標軌道 (Nominal Reference) の算出 ---
%             % 本クラスが障害物を回避する新軌道を生成しない間は、公称軌道生成器の出力をそのまま踏襲する
%             % 公称参照軌道の取得
%             base_res = obj.base_ref.do(varargin{:}); % 公称軌道の抜き出し
%             xd_nom = base_res.state.xd; % 牽引物の目標３次元位置・yaw角からその６階微分まで [pL(3); yaw(1); vL(3); yaw_dot(1); aL(3)...]
% 
%             % 毎ステップ、検知プロパティの初期値をリセット
%             obj.clear_state_sensor_values(time.t);
% 
%             % obj.result.state に公称軌道を反映 (base_res 全体の上書きは行わない)
%             obj.result.state.xd = xd_nom; % 公称軌道保存
% 
%             % --- 2. 空の検知構造体を用意 (非飛行フェーズ用) ---
%             detection = struct();
%             detection.time                          = time.t;             % [s] 現在のシミュレーション時刻
%             detection.pQ                            = [NaN; NaN; NaN];    % [m] ドローン機体重心の3次元位置ベクトル [x; y; z]
%             detection.pL                            = [NaN; NaN; NaN];    % [m] 牽引荷物の3次元位置ベクトル [x; y; z]
%             detection.detected_point                = false;              % [bool] 7m近接検知フラグ (true: 検知, false: 未検知)
%             detection.drone_inside_obstacle_point   = false;              % [bool] 機体の障害物楕円体内部侵入フラグ (true: 侵入, false: 外部)
%             detection.load_inside_obstacle_point    = false;              % [bool] 荷物の障害物楕円体内部侵入フラグ (true: 侵入, false: 外部)
%             detection.drone_min_dist_point          = inf;                % [m] 機体から全障害物表面までの最短幾何学距離
%             detection.load_min_dist_point           = inf;                % [m] 荷物から全障害物表面までの最短幾何学距離
%             detection.min_dist_point                = inf;                % [m] システム全体(機体・荷物)で最も近い表面距離 min(dQ, dL)
%             detection.drone_obstacle_id_point       = NaN;                % [ID] 機体にとって最短距離を与えている障害物インデックス番号
%             detection.load_obstacle_id_point        = NaN;                % [ID] 荷物にとって最短距離を与えている障害物インデックス番号
%             detection.min_obstacle_id_point         = NaN;                % [ID] システム全体で最短距離を与えている障害物インデックス番号
%             detection.min_source_point              = "none";             % [string] 最短距離をもたらしたセンサ種別 ("drone", "load", "none")
%             detection.detected_obstacle_count_point = 0;                  % [個] 7m以内に検知された障害物の総数
% 
%             % --- 飛行フェーズ ('f') 時の近接障害物スキャン ---
%             if cha == 'f'
%                 % --- 機体・荷物の現在位置の取得 ---
%                 % (A) 状態推定器 (estimator) から真値・推定位置を直接取得
%                 % ※ フォールバックを排除しているため、estimator にプロパティが存在しない場合は即座にエラー停止
%                 pL_cur = obj.self.estimator.result.state.pL(:); % 荷物位置の現在3次元位置 [x; y; z]
%                 pQ_cur = obj.self.estimator.result.state.p(:); % ドローン機体の現在3次元位置 [x; y; z]
% 
%                 % (B) 現在時刻 t における動的障害物配置を取得
%                 obs_list = obj.get_obstacles_at_time(time.t);
% 
% 
%                 % (C) 機体の重心および荷物の取り付け位置から障害物表面までの最短ユークリッド距離を幾何計算
%                 detection = obj.check_detection_simulated_sensor(pQ_cur, pL_cur, obs_list, time.t);
% 
%                 % 7m以内に侵入した場合のコンソール警告
%                 if detection.detected_point
%                     fprintf("[PROXIMITY ALERT] t=%.3f s | 7m近接検知! (機体表面間: %.2f m, 荷物表面間: %.2f m, 最短: %.2f m, 最寄センサ: %s, ID: %d)\n", ...
%                         time.t, detection.drone_min_dist_point, detection.load_min_dist_point, detection.min_dist_point, detection.min_source_point, detection.min_obstacle_id_point);
%                 end
%             end
% 
%             % --- 4. 【最重要】state 内の各プロパティに代入して app.logger に完全保存 ---
%             st = obj.result.state;
%             st.xd                            = xd_nom;                                  % [28x1 double] 目標軌道全状態
%             st.p                             = xd_nom(1:3);                             % [m] 目標位置 [x; y; z]
%             st.v                             = xd_nom(5:7);                             % [m/s] 目標速度 [vx; vy; vz]
%             st.q                             = [0; 0; xd_nom(4)];                       % [rad] 目標姿勢 (yaw角)
% 
%             st.time                          = detection.time;                          % [s] 計測時刻 (double)
%             st.pQ                            = detection.pQ;                            % [m] ドローン機体重心位置 (3x1 double)
%             st.pL                            = detection.pL;                            % [m] 荷物位置 (3x1 double)
%             st.detected_point                = detection.detected_point;                % [bool] 7m近接検知判定フラグ (true / false)
%             st.drone_inside_obstacle_point   = detection.drone_inside_obstacle_point;   % [bool] 機体侵入フラグ (true: 侵入, false: 外部)
%             st.load_inside_obstacle_point    = detection.load_inside_obstacle_point;    % [bool] 荷物侵入フラグ (true: 侵入, false: 外部)
%             st.drone_min_dist_point          = detection.drone_min_dist_point;          % [m] 機体の最短表面距離 (正: 外部, 負: 侵入深さ)
%             st.load_min_dist_point           = detection.load_min_dist_point;           % [m] 荷物の最短表面距離 (正: 外部, 負: 侵入深さ)
%             st.min_dist_point                = detection.min_dist_point;                % [m] システム全体の最短幾何表面距離 min(dQ, dL)
%             st.drone_obstacle_id_point       = detection.drone_obstacle_id_point;       % [ID] 機体にとって最短の障害物インデックス番号
%             st.load_obstacle_id_point        = detection.load_obstacle_id_point;        % [ID] 荷物にとって最短の障害物インデックス番号
%             st.min_obstacle_id_point         = detection.min_obstacle_id_point;         % [ID] 全体で最短の障害物インデックス番号
%             st.min_source_point              = detection.min_source_point;              % [string] 最短距離センサ種別 ("drone" または "load")
%             st.detected_obstacle_count_point = detection.detected_obstacle_count_point; % [個] 7m以内に検知された障害物の総数
% 
%             result_out = obj.result;
%         end
%     end
% 
%     methods (Access = private)
%         % =====================================================================
%         % clear_state_sensor_values: state 内の検知プロパティを初期化
%         % =====================================================================
%         function clear_state_sensor_values(obj, t_now)
%             st = obj.result.state;
%             st.time                          = t_now;              % [s] 現在時刻
%             st.pQ                            = [NaN; NaN; NaN];    % [m] ドローン位置初期値
%             st.pL                            = [NaN; NaN; NaN];    % [m] 荷物位置初期値
%             st.detected_point                = false;              % 検知なし
%             st.drone_inside_obstacle_point   = false;              % 機体侵入なし
%             st.load_inside_obstacle_point    = false;              % 荷物侵入なし
%             st.drone_min_dist_point          = inf;                % 最短距離初期値 (無限大)
%             st.load_min_dist_point           = inf;                % 最短距離初期値 (無限大)
%             st.min_dist_point                = inf;                % 最短距離初期値 (無限大)
%             st.drone_obstacle_id_point       = NaN;                % 最短障害物ID初期値
%             st.load_obstacle_id_point        = NaN;                % 最短障害物ID初期値
%             st.min_obstacle_id_point         = NaN;                % 最短障害物ID初期値
%             st.min_source_point              = "none";             % 最短センサ初期値
%             st.detected_obstacle_count_point = 0;                  % 検知個数初期値
%         end
% 
%         % =====================================================================
%         % get_obstacles_at_time: 環境関数を叩き、時刻 t_now での障害物リストを取得
%         % =====================================================================
%         function list = get_obstacles_at_time(~,t_now)
%             % ENVIRONMENT_OBSTACLE_ELLIPSE_MOVE 内ですでに「p_center = p0 + v*t」
%             % のように時刻 t_now に応じた現在位置が計算されている
%             list = ENVIRONMENT_OBSTACLE_ELLIPSE_MOVE(t_now);
% 
%         end
% 
%         % =====================================================================
%         % check_detection_simulated_sensor: 全障害物を走査し、機体・荷物の点と障害物の表面との最短距離を評価
%         % =====================================================================
%         function det = check_detection_simulated_sensor(obj, pQ, pL, obs_list, t_now)
%             det = struct();
% 
%             % --- センサ状態・時刻 ---
%             det.time                          = t_now;              % [s] 現在のシミュレーション時刻 (double)
%             det.pQ                            = pQ;                 % [m] ドローン機体重心の3次元位置ベクトル [x; y; z] (3x1 double)
%             det.pL                            = pL;                 % [m] 牽引荷物の3次元位置ベクトル [x; y; z] (3x1 double)
%             det.trigger_dist                  = obj.trigger_dist;   % [m] 近接検知判定の閾値距離 (例: 7.0 m) (double)
% 
%             % --- 全体判定フラグ ---
%             det.detected_point                = false;              % [bool] 機体または荷物が障害物から7m以内に接近したか (true: 検知, false: 未検知)
%             det.drone_inside_obstacle_point   = false;              % [bool] 機体が障害物楕円体の内部へ侵入/衝突したか (true: 侵入, false: 外部)
%             det.load_inside_obstacle_point    = false;              % [bool] 荷物が障害物楕円体の内部へ侵入/衝突したか (true: 侵入, false: 外部)
% 
%             % --- 最短距離 ---
%             det.drone_min_dist_point          = inf;                % [m] 環境内の全障害物の中で、機体から表面までの最短幾何学距離 (double, 内部時は負値)
%             det.load_min_dist_point           = inf;                % [m] 環境内の全障害物の中で、荷物から表面までの最短幾何学距離 (double, 内部時は負値)
%             det.min_dist_point                = inf;                % [m] システム全体(機体・荷物の双方)で最も近い表面距離 min(dQ, dL) (double)
% 
%             % --- 識別情報・統計 ---
%             det.drone_obstacle_id_point       = [];                 % [ID] 機体にとって最短距離を与えている障害物のインデックス番号 (integer)
%             det.load_obstacle_id_point        = [];                 % [ID] 荷物にとって最短距離を与えている障害物のインデックス番号 (integer)
%             det.min_obstacle_id_point         = [];                 % [ID] システム全体で最短距離を与えている障害物のインデックス番号 (integer)
%             det.min_source_point              = "none";                 % [string] 最短距離をもたらしたセンサ種別 ("drone": 機体, "load": 荷物)
%             det.detected_obstacles_point      = [];                 % [struct配列] 7m以内に検知された全障害物の詳細情報リスト (各要素は下記2を参照)
%             det.detected_obstacle_count_point = 0;                  % [個] 7m以内に検知された障害物の総数 (integer, 未検知時は0)
% 
%             if isempty(obs_list)
%                 return;
%             end
% 
%             detected_obs_point = [];
% 
%             for i = 1:length(obs_list)
%                 o = obs_list(i);
% 
%                 % --- 障害物パラメータの抽出 (推測フォールバックなし) ---
%                 if ~isfield(o, 'R_obs') || ~isfield(o, 'ellipsoid_radii') || ~isfield(o, 'p_center')
%                     error('インデックス %d の障害物に必須フィールド (R_obs, ellipsoid_radii, p_center) が不足しています.', i);
%                 end
% 
%                 % --- 障害物の姿勢回転行列 R_obs の抽出 ---
%                 R_obs = o.R_obs;
%                 % --- 外接楕円体の主軸半径 [a; b; c] の抽出 ---
%                 radii_obs = o.ellipsoid_radii(:);
%                 % --- 障害物の中心位置 ---
%                 % get_obstacles_at_time(t_now) で既に t_now 時点の中心位置が得られているため、
%                 % timestamp フィールドが明示的に別時刻として存在する場合のみ差分時間で補正
%                 p_obs = o.p_center(:);
% 
%                 obs_parsed = struct('center', p_obs, 'radii', radii_obs, 'R', R_obs);
% 
%                 % --- E. 機体 pQ および荷物 pL から楕円体表面への符号付き最短幾何距離 ---
%                 % d > 0: 表面の外側にある (表面までの最短距離 [m])
%                 % d < 0: 内部に侵入している (侵入深さ [m])
%                 % ここでは機体は重心点・牽引物は取り付け点の点と考えて，その表面からの障害物を近似した楕円表面への最短距離が検知範囲内かどうかの判定に使用
%                 [d_drone_point, inside_drone_point, cpQ_local_point, ~] = obj.point_ellipsoid_signed_distance(pQ, obs_parsed);
%                 [d_load_point,  inside_load_point,  cpL_local_point, ~] = obj.point_ellipsoid_signed_distance(pL, obs_parsed);
% 
%                 % ワールド座標系における表面最近接点: x_world = center + R * x_local
%                 cpQ_world_point = p_obs + R_obs * cpQ_local_point;
%                 cpL_world_point = p_obs + R_obs * cpL_local_point;
% 
%                 % --- 障害物表面における真の外向き単位法線ベクトル (ワールド座標系) ---
%                 % 幾何学的定義: 楕円体表面の陰関数 F(x) = (x/rx)^2 + (y/ry)^2 + (z/rz)^2 - 1 = 0
%                 % 局所法線ベクトルは勾配 grad(F) = 2 * [x/rx^2; y/ry^2; z/rz^2] に等しい。
%                 % これにより、点 p が表面上に完全一致 (d = 0) している場合でもフォールバックなしで厳密に求まる。
%                 nQ_local_point = cpQ_local_point ./ (radii_obs.^2);
%                 if norm(nQ_local_point) < 1e-12
%                     error('機体の最近接点における法線ベクトルが退化 (ゼロベクトル) しました.');
%                 end
%                 normal_drone_point = R_obs * (nQ_local_point / norm(nQ_local_point));
% 
%                 nL_local_point = cpL_local_point ./ (radii_obs.^2);
%                 if norm(nL_local_point) < 1e-12
%                     error('機体の最近接点における法線ベクトルが退化 (ゼロベクトル) しました.');
%                 end
%                 normal_load_point = R_obs * (nL_local_point / norm(nL_local_point));
% 
%                 % 幾何学的侵入判定の論理和更新
%                 if inside_drone_point, det.drone_inside_obstacle_point = true; end
%                 if inside_load_point,  det.load_inside_obstacle_point  = true; end
% 
%                 % 機体側の最小値更新
%                 if d_drone_point < det.drone_min_dist_point
%                     det.drone_min_dist_point = d_drone_point;
%                     det.drone_obstacle_id_point    = i;
%                 end
%                 % 荷物側の最小値更新
%                 if d_load_point < det.load_min_dist_point
%                     det.load_min_dist_point = d_load_point;
%                     det.load_obstacle_id_point    = i;
%                 end
% 
%                 obs_min_point = min(d_drone_point, d_load_point);
% 
%                 % 該当障害物の詳細データ構造体
%                 obs_info_point = struct( ...
%                     'id',                        i, ...                  % [ID] 障害物のインデックス番号 (integer)
%                     'dist_drone_point',          d_drone_point, ...      % [m] 機体重心からこの障害物表面までの符号付き最短距離 (正: 外部, 負: 侵入深さ)
%                     'dist_load_point',           d_load_point, ...       % [m] 荷物位置からこの障害物表面までの符号付き最短距離 (正: 外部, 負: 侵入深さ)
%                     'min_dist_point',            obs_min_point, ...      % [m] この障害物に対する機体・荷物双方の最小表面間距離 min(dQ, dL)
%                     'drone_inside_point',        inside_drone_point, ... % [bool] 機体がこの障害物の内部に侵入しているか (true: 侵入, false: 外部)
%                     'load_inside_point',         inside_load_point, ...  % [bool] 荷物がこの障害物の内部に侵入しているか (true: 侵入, false: 外部)
%                     'p_obs',                     p_obs, ...              % [m] 時刻 t における障害物の中心位置ベクトル [x; y; z] (3x1 double)
%                     'radii_obs',                 radii_obs, ...          % [m] 障害物の3軸半径 [rx; ry; rz] (3x1 double)
%                     'R_obs',                     R_obs, ...              % [-] 障害物の主軸姿勢を表す3x3回転行列 SO(3) (3x3 double)
%                     'closest_drone_world_point', cpQ_world_point, ...    % [m] 機体に対する障害物表面上の最短最近接点 (ワールド座標系 [x; y; z])
%                     'closest_load_world_point',  cpL_world_point, ...    % [m] 荷物に対する障害物表面上の最短最近接点 (ワールド座標系 [x; y; z])
%                     'normal_drone_point',        normal_drone_point, ... % [-] 機体側最近接点における障害物表面の真の外向き単位法線ベクトル (ワールド系, 単位ノルム)
%                     'normal_load_point',         normal_load_point ...   % [-] 荷物側最近接点における障害物表面の真の外向き単位法線ベクトル (ワールド系, 単位ノルム)
%                 );
% 
%                 % --- 接近検知判定 (7m以内) ---
%                 if obs_min_point <= obj.trigger_dist
%                     det.detected_point = true;
%                     detected_obs_point = [detected_obs_point; obs_info_point];
%                 end
%             end
% 
%             % システム全体での最短距離および検知元の確定
%             if det.drone_min_dist_point <= det.load_min_dist_point
%                 det.min_dist_point  = det.drone_min_dist_point;
%                 det.min_obstacle_id_point = det.drone_obstacle_id_point;
%                 det.min_source_point      = "drone";
%             else
%                 det.min_dist_point  = det.load_min_dist_point;
%                 det.min_obstacle_id_point = det.load_obstacle_id_point;
%                 det.min_source_point      = "load";
%             end
% 
%             det.detected_obstacles_point = detected_obs_point;
%             det.detected_obstacle_count_point = numel(detected_obs_point);  % 検知された障害物の個数を記録
%         end
% 
%         % =====================================================================
%         % point_ellipsoid_signed_distance
%         % 任意の3次元点 p と回転楕円体 (中心 c, 半径 r=[a;b;c], 回転行列 R) の
%         % 【外郭表面】までの真の最短ユークリッド距離を算出する。
%         %
%         % [数学的アルゴリズム]:
%         % 楕円方程式: (x/a)^2 + (y/b)^2 + (z/c)^2 = 1
%         % 点 p から楕円面上の最近接点 x への最短距離は、ラグランジュ未定乗数法:
%         %   f(lambda) = sum( (a_i^2 * y_i^2) / (lambda + a_i^2)^2 ) - 1 = 0
%         % を満たす単一根 lambda を二分探索法 (Binary Search) で 80 回反復して求め、
%         % 表面点 x = (a_i^2 * y_i) / (lambda + a_i^2) と点 y の距離 ||x - y|| を計算する。
%         % =====================================================================
%         function [d, inside, closest_local, lambda] = point_ellipsoid_signed_distance(~, p, o)
%             p = p(:);
%             c = o.center(:);
%             r = o.radii(:);
%             R = o.R;
% 
%             % --- 入力チェック: 不正ならフォールバックせず即座に停止 ---
%             if numel(p) ~= 3 || numel(c) ~= 3 || numel(r) ~= 3
%                 error('点 p、中心 c、および半径 r はすべて3x1ベクトルである必要があります。');
%             end
%             if any(~isfinite(p)) || any(~isfinite(c)) || any(~isfinite(r))
%                 error('点または楕円体の定義に非有限値 (NaN や Inf) が検出されました。');
%             end
%             if any(r <= 0)
%                 error('楕円体の半径はすべて厳密に正の値 (r > 0) である必要があります。');
%             end
%             if ~isequal(size(R), [3 3]) || any(~isfinite(R(:)))
%                 error('回転行列 R は有限な3x3行列である必要があります。');
%             end
%             if norm(R'*R - eye(3), 'fro') > 1e-6 || abs(det(R) - 1.0) > 1e-6
%                 error('行列 R は有効な直交回転行列 SO(3) ではありません。');
%             end
% 
%             % 1. ワールド座標系の点 p を障害物の局所主軸座標系 (Local Frame) に変換
%             y  = R' * (p - c);
%             r2 = r.^2;
% 
%             % 2. 楕円代数判定値 q:
%             %    q < 1.0 -> 点は楕円体の内部にある (衝突・侵入状態)
%             %    q = 1.0 -> 点は楕円体の表面上にある
%             %    q > 1.0 -> 点は楕円体の外部にある
%             q      = sum((y ./ r).^2);
%             inside = (q < 1.0);
% 
%             % 特異点処理: 点が障害物の中心そのものにある場合 (最も近い表面は最短半径の主軸上)
%             if norm(y) < 1e-14
%                 [min_r, min_idx] = min(r);
%                 closest_local = zeros(3, 1);
%                 closest_local(min_idx) = min_r;
%                 d      = -min_r;
%                 lambda = -min(r2);
%                 return;
%             end
% 
%             % ラグランジュ未定乗数の非線形方程式 f(lambda) = 0
%             f = @(lam) sum(r2 .* (y.^2) ./ ((lam + r2).^2)) - 1.0;
% 
%             % 3. ラグランジュ乗数 lambda の探索範囲 (Bracketing) の設定と根の存在検証
%             if q > 1.0
%                 % 外部点の場合: lambda >= 0
%                 lo = 0.0;
%                 hi = max(r) * norm(y);
% 
%                 if f(lo) < 0 || f(hi) > 0
%                     error('外部点に対する楕円体距離の求根ブラケット設定に失敗しました (解を挟み込めていません)。');
%                 end
%             else
%                 % 内部点の場合: -min(r_i^2) < lambda < 0
%                 lo = -min(r2) * (1.0 - 1e-12);
%                 hi = 0.0;
% 
%                 if f(lo) < 0 || f(hi) > 0
%                     error('内部点に対する楕円体距離の求根ブラケット設定に失敗しました (解を挟み込めていません)。');
%                 end
%             end
% 
%             % 4. 二分探索により lambda を数値的に収束させる
%             %    80回反復して根を高精度に求める
%             for kk = 1:80
%                 mid = 0.5 * (lo + hi);
%                 if f(mid) > 0
%                     lo = mid;
%                 else
%                     hi = mid;
%                 end
%             end
%             lambda = 0.5 * (lo + hi);
% 
%             % 5. 局所座標系における楕円体表面上の最近接点 closest_local
%             closest_local = r2 .* y ./ (lambda + r2);
%             d_abs         = norm(closest_local - y);
% 
%             % 6. 表面までの最短ユークリッド距離
%             %    外部なら正値 (+), 内部なら侵入深さとして負値 (-) を返す
%             if inside
%                 d = -d_abs;
%             else
%                 d = d_abs;
%             end
%         end
%     end
% end
% 


% classdef REPLANNING_BSPLINE < handle
%     % =========================================================================
%     % REPLANNING_BSPLINE
%     % 1.模擬センサー：機体重心 pQ を点（Point）として扱い、楕円体境界面までのユークリッド最短距離を厳密に計算して検知を判定
%     % 2.障害物評価：
%     % 【索（テザー）幾何モデル化モード (cable_model_mode)】:
%     %   1 : システム全体を「索姿勢付き単一指向性楕円体」として一括包絡 (案1: 超高速・保守的)
%     %   2 : UAV/Payloadを楕円体、索を可変サンプリング線分カプセルで評価 (案2: 高精度・推奨)
%     %   3 : 機体・荷物・索を「球体プリミティブ群 (Sphere Decomposition)」に統一 (案3)
%     % 【安全マージン (d_margin)】:
%     %   ENVIRONMENT_OBSTACLE_ELLIPSE_MOVE 内の各障害物が保持する d_margin を
%     %   動的に読み込んで適用 (Park & Cho, 2020; Geetha et al., 2026)
%     % [依拠理論]:
%     % 1. Geetha et al. (2026): 索線分サンプリングによる離散包括クリアランス評価 (式4)
%     % 2. Woo et al. (2026): 解析的CPA (式8-11) & 複数エンティティ連立衝突判定
%     % 3. Park & Cho (2020): バウンディングボックス幾何生成 & 予測エンベロープ評価 (Sec. 5)
%     % =========================================================================
%     properties
%         self % ドローンエージェント自身 (推定器 estimator やパラメータ parameter を保持) 
%         base_ref % 公称参照軌道生成オブジェクト
%         result                % 出力結果構造体 (目標状態 xd, pRef, vRef, yawRef, 検知情報)
% 
%         trigger_dist = 15.0;   % 接近検知の閾値 [m] (表面間距離がこれ以下になるとアラート)
%         r_load       = 0.15;  % 荷物保護半径 [m]
%         r_drone      = 0.30;  % 機体保護半径 [m]
%         r_cable      = 0.08;  % 索（ケーブル）等価保護半径 [m]
%         default_margin = 0.30;% 障害物に d_margin が未定義の場合のフォールバックマージン [m]
% 
%         % --- 索幾何モデル切り替え設定 ---
%         cable_model_mode = 2; % 1: 単一楕円体, 2: 楕円体+カプセル, 3: 球体分解
%         cable_ds_sample  = 0.40; % 索サンプリング間隔 [m] (長さ L に応じて点数を動的スケール)
%         min_cable_pts    = 3;    % 索上の最小サンプリング点数
% 
%         t_horizon_eval   = 6.0;  % 将来予測評価ホライズン [s] (回避から合流までの最大時間)
%     end
% 
%     methods (Access = public)
%         % =====================================================================
%         % コンストラクタ: クラスの初期化と外部設定 (opts) の反映
%         % =====================================================================
%         function obj = REPLANNING_BSPLINE(self, base_ref, opts)
%             arguments
%                 self                  % 必須: エージェントインスタンス
%                 base_ref              % 必須: 通常飛行用の公称軌道インスタンス
%                 opts = struct()       % 任意: 外部からパラメータを変更するための構造体
%             end
% 
%             obj.self = self;
%             obj.base_ref = base_ref;
% 
%             if isfield(opts, 'trigger_dist'), obj.trigger_dist = opts.trigger_dist; end % 外で定義されていたらデフォルト値を上書き　センサー代わりの検知範囲
%             if isfield(opts, 'r_load'),       obj.r_load       = opts.r_load;       end % 外で定義されていたらデフォルト値を上書き　牽引物を近似した球体
%             if isfield(opts, 'r_drone'),      obj.r_drone      = opts.r_drone;      end % 外で定義されていたらデフォルト値を上書き　機体を近似した球体
%             if isfield(opts, 'r_cable'),           obj.r_cable           = opts.r_cable;           end
%             if isfield(opts, 'default_margin'),    obj.default_margin    = opts.default_margin;    end
%             if isfield(opts, 'cable_model_mode'),  obj.cable_model_mode  = opts.cable_model_mode;  end
%             if isfield(opts, 'cable_ds_sample'),   obj.cable_ds_sample   = opts.cable_ds_sample;   end
%             if isfield(opts, 't_horizon_eval'),    obj.t_horizon_eval    = opts.t_horizon_eval;    end
% 
%             % base_ref の result 構造体をそのまま継承 (直下は state のみ)
%             obj.result = base_ref.result;
%             % --- 【重要】STATE_CLASS に検知用プロパティを動的追加 (dynamicprops) ---
%             % これにより、state 内に正式な記録領域が作成され、ロガーで抽出可能になる
%             sensor_props = [ ...
%                 "time", "pQ", "pL", ...                                         % 基本計測状態
%                 "detected_point", ...                                           % 15m検知フラグ
%                 "drone_inside_obstacle_point", "load_inside_obstacle_point", ...% 内部侵入フラグ
%                 "drone_min_dist_point", "load_min_dist_point", "min_dist_point",... % 表面間最短幾何距離
%                 "drone_obstacle_id_point", "load_obstacle_id_point", "min_obstacle_id_point", ... % 最寄ID
%                 "min_source_point", ...                                         % 最短距離をもたらした部位 ("drone" / "load")
%                 "detected_obstacle_count_point", ...                            % 15m以内の検知総数
%                 "active_threat_count", ...                                      % 真に衝突危険のある連立脅威数
%                 "primary_threat_id", ...                                        % 最も切迫している障害物ID
%                 "primary_threat_tcpa", ...                                      % 最優先障害物の衝突到達予測時間 [s]
%                 "primary_threat_dcpa" ...                                       % 最優先障害物の最接近予測距離 [m]
%             ];
%             for p_name = sensor_props
%                 if ~isprop(obj.result.state, p_name)
%                     addprop(obj.result.state, p_name);
%                 end
%             end
% 
%             % 初期ダミー値のセット
%             obj.clear_state_sensor_values(0.0);
%         end
% 
%         % =====================================================================
%         % do: 制御周期ごと (例: 25ms周期) にメインループから呼び出される実行メソッド
%         % 入力: 
%         %   varargin{1}: time (現在の時刻 struct: time.t, time.dt など)
%         %   varargin{2}: cha  (フェーズ文字列: 'f' = 飛行中, 't' = 離陸など)
%         %   varargin{4}: env  (環境構造体、障害物リストを内包する場合あり)
%         % 出力:
%         %   result_out : 下流のコントローラやロガーが受け取る目標状態・検知結果
%         % =====================================================================
%         function result_out = do(obj, varargin)
%             time = varargin{1}; % varargin{1}: time (現在の時刻 struct: time.t, time.dt など)
%             cha = varargin{2}; % varargin{2}: cha  (フェーズ文字列: 'f' = 飛行中, 't' = 離陸など)
% 
%             % --- 公称目標軌道 (Nominal Reference) の算出 ---
%             % 本クラスが障害物を回避する新軌道を生成しない間は、公称軌道生成器の出力をそのまま踏襲する
%             % 公称参照軌道の取得
%             base_res = obj.base_ref.do(varargin{:}); % 公称軌道の抜き出し
%             xd_nom = base_res.state.xd; % 牽引物の目標３次元位置・yaw角からその６階微分まで [pL(3); yaw(1); vL(3); yaw_dot(1); aL(3)...]
% 
%             % 毎ステップ、検知プロパティの初期値をリセット
%             obj.clear_state_sensor_values(time.t);
% 
%             % obj.result.state に公称軌道を反映 (base_res 全体の上書きは行わない)
%             obj.result.state.xd = xd_nom; % 公称軌道保存
% 
%             % --- 空の検知構造体を用意 (非飛行フェーズ用) ---
%             detection = struct();
%             detection.time                          = time.t;             % [s] 現在のシミュレーション時刻
%             detection.pQ                            = [NaN; NaN; NaN];    % [m] ドローン機体重心の3次元位置ベクトル [x; y; z]
%             detection.pL                            = [NaN; NaN; NaN];    % [m] 牽引荷物の3次元位置ベクトル [x; y; z]
%             detection.detected_point                = false;              % [bool] センサー検知範囲近接検知フラグ (true: 検知, false: 未検知)
%             detection.drone_inside_obstacle_point   = false;              % [bool] 機体の障害物楕円体内部侵入フラグ (true: 侵入, false: 外部)
%             detection.load_inside_obstacle_point    = false;              % [bool] 荷物の障害物楕円体内部侵入フラグ (true: 侵入, false: 外部)
%             detection.drone_min_dist_point          = inf;                % [m] 機体から全障害物表面までの最短幾何学距離
%             detection.load_min_dist_point           = inf;                % [m] 荷物から全障害物表面までの最短幾何学距離
%             detection.min_dist_point                = inf;                % [m] システム全体(機体・荷物)で最も近い表面距離 min(dQ, dL)
%             detection.drone_obstacle_id_point       = NaN;                % [ID] 機体にとって最短距離を与えている障害物インデックス番号
%             detection.load_obstacle_id_point        = NaN;                % [ID] 荷物にとって最短距離を与えている障害物インデックス番号
%             detection.min_obstacle_id_point         = NaN;                % [ID] システム全体で最短距離を与えている障害物インデックス番号
%             detection.min_source_point              = "none";             % [string] 最短距離をもたらしたセンサ種別 ("drone", "load", "none")
%             detection.detected_obstacle_count_point = 0;                  % [個] センサー検知範囲以内に検知された障害物の総数
% 
%             % 複数障害物統合データ構造体 (軌道生成へ引き渡す完全情報)
%             threat_group = struct();
%             threat_group.has_threat           = false;
%             threat_group.threat_count         = 0;
%             threat_group.items                = [];     % 危険な全障害物の詳細リスト (除外なし)
%             threat_group.primary_id           = NaN;    % 今最も切迫している障害物ID
%             threat_group.primary_tcpa         = inf;    % 最優先障害物の到達時間 [s]
%             threat_group.primary_dcpa         = inf;    % 最優先障害物の最接近距離 [m]
%             threat_group.secondary_ids        = [];     % 回避・復帰軌道上で衝突する障害物ID群
%             threat_group.union_center         = [0;0;0];% 複数障害物の合成幾何中心
%             threat_group.req_clearance_max    = 0.0;    % 最大必要退避クリアランス [m]
%             threat_group.suggested_escape_dir = [0;0;0]; % 全障害物を一括回避する推奨法線方向
% 
%             % --- 飛行フェーズ ('f') 時の近接障害物スキャン ---
%             if cha == 'f'
%                 % --- 機体・荷物の現在位置の取得 ---
%                 % (A) 状態推定器 (estimator) から真値・推定位置を直接取得
%                 % ※ フォールバックを排除しているため、estimator にプロパティが存在しない場合は即座にエラー停止
%                 pL_cur = obj.self.estimator.result.state.pL(:); % 荷物位置の現在3次元位置 [x; y; z]
%                 pQ_cur = obj.self.estimator.result.state.p(:); % ドローン機体の現在3次元位置 [x; y; z]
%                 vL_cur = obj.self.estimator.result.state.vL(:); % 荷物の現在3次元速度 [vx; vy; vz]
%                 vQ_cur = obj.self.estimator.result.state.v(:); % ドローン機体の現在3次元速度 [vx; vy; vz]
% 
%                 % (B) 現在時刻 t における動的障害物配置を取得
%                 obs_list = obj.get_obstacles_at_time(time.t);
% 
% 
%                 % (C) 機体の重心および荷物の取り付け位置から障害物表面までの最短ユークリッド距離を幾何計算
%                 detection = obj.check_detection_simulated_sensor(pQ_cur, pL_cur, obs_list, time.t);
% 
%                 % (D) 幾何モデル切り替え ＆ 環境マージン連動型 マルチスキャンフィルタ
%                 threat_group = obj.filter_multi_obstacle_threats( ...
%                     pQ_cur, vQ_cur, pL_cur, vL_cur, detection.detected_obstacles_point, xd_nom, time.t);
% 
%                 % (E) コンソール診断ログ出力
%                 if threat_group.has_threat
%                     primary = threat_group.items(1);
%                     fprintf("[PROXIMITY ALERT | Drone-LiDAR 15m] t=%.3f s | センサ視界内: %d 個 | 衝突脅威: %d 個\n", ...
%                         time.t, detection.detected_obstacle_count_point, threat_group.threat_count);
%                     fprintf("  -> [最切迫脅威 ID:%d] 危険部位: %s (機体表面間: %.2f m, 荷物表面間: %.2f m)\n", ...
%                         threat_group.primary_id, primary.threat_target, ...
%                         detection.drone_min_dist_point, detection.load_min_dist_point);
%                     fprintf("  -> 到達予測: %.2f s後, CPA予測離隔: %.2f m (適用マージン: %.2f m)\n", ...
%                         primary.t_cpa, primary.d_cpa, primary.d_margin);
%                     if ~isempty(threat_group.secondary_ids)
%                         fprintf("  -> [復帰路の二次警戒] ID: %s (回避後の追突を警戒)\n", mat2str(threat_group.secondary_ids));
%                     end
%                 end
%             end
% 
%             % --- 4. 【最重要】state 内の各プロパティに代入して app.logger に完全保存 ---
%             st = obj.result.state;
%             st.xd                            = xd_nom;                                  % [28x1 double] 目標軌道全状態
%             st.p                             = xd_nom(1:3);                             % [m] 目標位置 [x; y; z]
%             st.v                             = xd_nom(5:7);                             % [m/s] 目標速度 [vx; vy; vz]
%             st.q                             = [0; 0; xd_nom(4)];                       % [rad] 目標姿勢 (yaw角)
% 
%             st.time                          = detection.time;                          % [s] 計測時刻 (double)
%             st.pQ                            = detection.pQ;                            % [m] ドローン機体重心位置 (3x1 double)
%             st.pL                            = detection.pL;                            % [m] 荷物位置 (3x1 double)
%             st.detected_point                = detection.detected_point;                % [bool] センサー範囲内近接検知判定フラグ (true / false)
%             st.drone_inside_obstacle_point   = detection.drone_inside_obstacle_point;   % [bool] 機体侵入フラグ (true: 侵入, false: 外部)
%             st.load_inside_obstacle_point    = detection.load_inside_obstacle_point;    % [bool] 荷物侵入フラグ (true: 侵入, false: 外部)
%             st.drone_min_dist_point          = detection.drone_min_dist_point;          % [m] 機体の最短表面距離 (正: 外部, 負: 侵入深さ)
%             st.load_min_dist_point           = detection.load_min_dist_point;           % [m] 荷物の最短表面距離 (正: 外部, 負: 侵入深さ)
%             st.min_dist_point                = detection.min_dist_point;                % [m] システム全体の最短幾何表面距離 min(dQ, dL)
%             st.drone_obstacle_id_point       = detection.drone_obstacle_id_point;       % [ID] 機体にとって最短の障害物インデックス番号
%             st.load_obstacle_id_point        = detection.load_obstacle_id_point;        % [ID] 荷物にとって最短の障害物インデックス番号
%             st.min_obstacle_id_point         = detection.min_obstacle_id_point;         % [ID] 全体で最短の障害物インデックス番号
%             st.min_source_point              = detection.min_source_point;              % [string] 最短距離センサ種別 ("drone" または "load")
%             st.detected_obstacle_count_point = detection.detected_obstacle_count_point; % [個] センサー範囲以内に検知された障害物の総数
% 
%             % 新規マルチ脅威指標の代入 (ロガーで完全記録)
%             st.active_threat_count           = threat_group.threat_count;
%             st.primary_threat_id             = threat_group.primary_id;
%             st.primary_threat_tcpa           = threat_group.primary_tcpa;
%             st.primary_threat_dcpa           = threat_group.primary_dcpa;
% 
%             obj.result.state                 = st;
%             obj.result.obstacle_detection    = detection;
%             obj.result.threat_group          = threat_group; % 軌道生成側へ渡す構造体
% 
%             result_out = obj.result;
%         end
%     end
% 
%     methods (Access = private)
%         % =====================================================================
%         % clear_state_sensor_values: state 内の検知プロパティを初期化
%         % =====================================================================
%         function clear_state_sensor_values(obj, t_now)
%             st = obj.result.state;
%             st.time                          = t_now;              % [s] 現在時刻
%             st.pQ                            = [NaN; NaN; NaN];    % [m] ドローン位置初期値
%             st.pL                            = [NaN; NaN; NaN];    % [m] 荷物位置初期値
%             st.detected_point                = false;              % 検知なし
%             st.drone_inside_obstacle_point   = false;              % 機体侵入なし
%             st.load_inside_obstacle_point    = false;              % 荷物侵入なし
%             st.drone_min_dist_point          = inf;                % 最短距離初期値 (無限大)
%             st.load_min_dist_point           = inf;                % 最短距離初期値 (無限大)
%             st.min_dist_point                = inf;                % 最短距離初期値 (無限大)
%             st.drone_obstacle_id_point       = NaN;                % 最短障害物ID初期値
%             st.load_obstacle_id_point        = NaN;                % 最短障害物ID初期値
%             st.min_obstacle_id_point         = NaN;                % 最短障害物ID初期値
%             st.min_source_point              = "none";             % 最短センサ初期値
%             st.detected_obstacle_count_point = 0;                  % 検知個数初期値
%             st.active_threat_count           = 0;
%             st.primary_threat_id             = NaN;
%             st.primary_threat_tcpa           = inf;
%             st.primary_threat_dcpa           = inf;
%         end
% 
%         % =====================================================================
%         % get_obstacles_at_time: 環境関数を叩き、時刻 t_now での障害物リストを取得
%         % =====================================================================
%         function list = get_obstacles_at_time(~,t_now)
%             % ENVIRONMENT_OBSTACLE_ELLIPSE_MOVE 内ですでに「p_center = p0 + v*t」
%             % のように時刻 t_now に応じた現在位置が計算されている
%             list = ENVIRONMENT_OBSTACLE_ELLIPSE_MOVE(t_now);
% 
%         end
% 
%         % =====================================================================
%         % check_detection_simulated_sensor: 全障害物を走査し、機体・荷物の点と障害物の表面との最短距離を評価
%         % =====================================================================
%         function det = check_detection_simulated_sensor(obj, pQ, pL, obs_list, t_now)
%             det = struct();
% 
%             % --- センサ状態・時刻 ---
%             det.time                          = t_now;              % [s] 現在のシミュレーション時刻 (double)
%             det.pQ                            = pQ;                 % [m] ドローン機体重心の3次元位置ベクトル [x; y; z] (3x1 double)
%             det.pL                            = pL;                 % [m] 牽引荷物の3次元位置ベクトル [x; y; z] (3x1 double)
%             det.trigger_dist                  = obj.trigger_dist;   % [m] 近接検知判定の閾値距離 (例: 7.0 m) (double)
% 
%             % --- 全体判定フラグ ---
%             det.detected_point                = false;              % [bool] 機体または荷物が障害物からセンサー検知範囲以内に接近したか (true: 検知, false: 未検知)
%             det.drone_inside_obstacle_point   = false;              % [bool] 機体が障害物楕円体の内部へ侵入/衝突したか (true: 侵入, false: 外部)
%             det.load_inside_obstacle_point    = false;              % [bool] 荷物が障害物楕円体の内部へ侵入/衝突したか (true: 侵入, false: 外部)
% 
%             % --- 最短距離 ---
%             det.drone_min_dist_point          = inf;                % [m] 環境内の全障害物の中で、機体から表面までの最短幾何学距離 (double, 内部時は負値)
%             det.load_min_dist_point           = inf;                % [m] 環境内の全障害物の中で、荷物から表面までの最短幾何学距離 (double, 内部時は負値)
%             det.min_dist_point                = inf;                % [m] システム全体(機体・荷物の双方)で最も近い表面距離 min(dQ, dL) (double)
% 
%             % --- 識別情報・統計 ---
%             det.drone_obstacle_id_point       = [];                 % [ID] 機体にとって最短距離を与えている障害物のインデックス番号 (integer)
%             det.load_obstacle_id_point        = [];                 % [ID] 荷物にとって最短距離を与えている障害物のインデックス番号 (integer)
%             det.min_obstacle_id_point         = [];                 % [ID] システム全体で最短距離を与えている障害物のインデックス番号 (integer)
%             det.min_source_point              = "none";                 % [string] 最短距離をもたらしたセンサ種別 ("drone": 機体, "load": 荷物)
%             det.detected_obstacles_point      = [];                 % [struct配列] センサー検知範囲以内に検知された全障害物の詳細情報リスト (各要素は下記2を参照)
%             det.detected_obstacle_count_point = 0;                  % [個] センサー感知範囲以内に検知された障害物の総数 (integer, 未検知時は0)
% 
%             if isempty(obs_list)
%                 return;
%             end
% 
%             detected_obs_point = [];
% 
%             for i = 1:length(obs_list)
%                 o = obs_list(i);
% 
%                 % --- 障害物パラメータの抽出 (推測フォールバックなし) ---
%                 if ~isfield(o, 'R_obs') || ~isfield(o, 'ellipsoid_radii') || ~isfield(o, 'p_center')
%                     error('インデックス %d の障害物に必須フィールド (R_obs, ellipsoid_radii, p_center) が不足しています.', i);
%                 end
% 
%                 % --- 障害物の姿勢回転行列 R_obs の抽出 ---
%                 R_obs = o.R_obs;
%                 % --- 外接楕円体の主軸半径 [a; b; c] の抽出 ---
%                 radii_obs = o.ellipsoid_radii(:);
%                 % --- 障害物の中心位置 ---
%                 % get_obstacles_at_time(t_now) で既に t_now 時点の中心位置が得られているため、
%                 % timestamp フィールドが明示的に別時刻として存在する場合のみ差分時間で補正
%                 p_obs = o.p_center(:);
%                 % --- 障害物の速度 ---
%                 v_obs = o.v_center(:);
%                 %--- 障害物の安全マージン ---
%                 d_margin_obs = obj.default_margin;
% 
%                 obs_parsed = struct('center', p_obs, 'radii', radii_obs, 'R', R_obs);
% 
%                 % --- E. 機体 pQ および荷物 pL から楕円体表面への符号付き最短幾何距離 ---
%                 % d > 0: 表面の外側にある (表面までの最短距離 [m])
%                 % d < 0: 内部に侵入している (侵入深さ [m])
%                 % ここでは機体は重心点・牽引物は取り付け点の点と考えて，その表面からの障害物を近似した楕円表面への最短距離が検知範囲内かどうかの判定に使用
%                 [d_drone_point, inside_drone_point, cpQ_local_point, ~] = obj.point_ellipsoid_signed_distance(pQ, obs_parsed);
%                 [d_load_point,  inside_load_point,  cpL_local_point, ~] = obj.point_ellipsoid_signed_distance(pL, obs_parsed);
% 
%                 % ワールド座標系における表面最近接点: x_world = center + R * x_local
%                 cpQ_world_point = p_obs + R_obs * cpQ_local_point;
%                 cpL_world_point = p_obs + R_obs * cpL_local_point;
% 
%                 % --- 障害物表面における真の外向き単位法線ベクトル (ワールド座標系) ---
%                 % 幾何学的定義: 楕円体表面の陰関数 F(x) = (x/rx)^2 + (y/ry)^2 + (z/rz)^2 - 1 = 0
%                 % 局所法線ベクトルは勾配 grad(F) = 2 * [x/rx^2; y/ry^2; z/rz^2] に等しい。
%                 % これにより、点 p が表面上に完全一致 (d = 0) している場合でもフォールバックなしで厳密に求まる。
%                 nQ_local_point = cpQ_local_point ./ (radii_obs.^2);
%                 if norm(nQ_local_point) < 1e-12
%                     error('機体の最近接点における法線ベクトルが退化 (ゼロベクトル) しました.');
%                 end
%                 normal_drone_point = R_obs * (nQ_local_point / norm(nQ_local_point));
% 
%                 nL_local_point = cpL_local_point ./ (radii_obs.^2);
%                 if norm(nL_local_point) < 1e-12
%                     error('機体の最近接点における法線ベクトルが退化 (ゼロベクトル) しました.');
%                 end
%                 normal_load_point = R_obs * (nL_local_point / norm(nL_local_point));
% 
%                 % 幾何学的侵入判定の論理和更新
%                 if inside_drone_point, det.drone_inside_obstacle_point = true; end
%                 if inside_load_point,  det.load_inside_obstacle_point  = true; end
% 
%                 % 機体側の最小値更新
%                 if d_drone_point < det.drone_min_dist_point
%                     det.drone_min_dist_point = d_drone_point;
%                     det.drone_obstacle_id_point    = i;
%                 end
%                 % 荷物側の最小値更新
%                 if d_load_point < det.load_min_dist_point
%                     det.load_min_dist_point = d_load_point;
%                     det.load_obstacle_id_point    = i;
%                 end
% 
%                 obs_min_point = min(d_drone_point, d_load_point);
% 
%                 % 該当障害物の詳細データ構造体
%                 obs_info_point = struct( ...
%                     'id',                        i, ...                  % [ID] 障害物のインデックス番号 (integer)
%                     'dist_drone_point',          d_drone_point, ...      % [m] 機体重心からこの障害物表面までの符号付き最短距離 (正: 外部, 負: 侵入深さ)
%                     'dist_load_point',           d_load_point, ...       % [m] 荷物位置からこの障害物表面までの符号付き最短距離 (正: 外部, 負: 侵入深さ)
%                     'min_dist_point',            obs_min_point, ...      % [m] この障害物に対する機体・荷物双方の最小表面間距離 min(dQ, dL)
%                     'drone_inside_point',        inside_drone_point, ... % [bool] 機体がこの障害物の内部に侵入しているか (true: 侵入, false: 外部)
%                     'load_inside_point',         inside_load_point, ...  % [bool] 荷物がこの障害物の内部に侵入しているか (true: 侵入, false: 外部)
%                     'p_obs',                     p_obs, ...              % [m] 時刻 t における障害物の中心位置ベクトル [x; y; z] (3x1 double)
%                     'v_obs',                     v_obs, ...
%                     'radii_obs',                 radii_obs, ...          % [m] 障害物の3軸半径 [rx; ry; rz] (3x1 double)
%                     'R_obs',                     R_obs, ...              % [-] 障害物の主軸姿勢を表す3x3回転行列 SO(3) (3x3 double)
%                     'd_margin',                  d_margin_obs, ...
%                     'closest_drone_world_point', cpQ_world_point, ...    % [m] 機体に対する障害物表面上の最短最近接点 (ワールド座標系 [x; y; z])
%                     'closest_load_world_point',  cpL_world_point, ...    % [m] 荷物に対する障害物表面上の最短最近接点 (ワールド座標系 [x; y; z])
%                     'normal_drone_point',        normal_drone_point, ... % [-] 機体側最近接点における障害物表面の真の外向き単位法線ベクトル (ワールド系, 単位ノルム)
%                     'normal_load_point',         normal_load_point ...   % [-] 荷物側最近接点における障害物表面の真の外向き単位法線ベクトル (ワールド系, 単位ノルム)
%                 );
% 
%                 % --- 接近検知判定 (センサー検知範囲以内) ---
%                 if d_drone_point <= obj.trigger_dist
%                     det.detected_point = true;
%                     detected_obs_point = [detected_obs_point; obs_info_point];
%                 end
%             end
% 
%             % システム全体での最短距離および検知元の確定
%             % ドローンセンサが捉えた障害物群の中で、機体・荷物のどちらが最も危険域に近いか
%             if det.drone_min_dist_point <= det.load_min_dist_point
%                 det.min_dist_point  = det.drone_min_dist_point;
%                 det.min_obstacle_id_point = det.drone_obstacle_id_point;
%                 det.min_source_point      = "drone";
%             else
%                 det.min_dist_point  = det.load_min_dist_point;
%                 det.min_source_point      = "load";
%             end
% 
%             % センサ自身の最短探知距離 (ドローンから最も近い障害物表面までの距離)
%             det.sensor_min_dist = det.drone_min_dist_point;
% 
%             det.detected_obstacles_point = detected_obs_point;
%             det.detected_obstacle_count_point = numel(detected_obs_point);  % 検知された障害物の個数を記録
%         end
%         % =====================================================================
%         % build_system_representation
%         % 選択されたモード (1, 2, 3) と障害物固有の d_margin に基づき自機エンティティ群を構築
%         % =====================================================================
%         function entities = build_system_representation(obj, pQ, vQ, pL, vL, d_margin_obs)
%             vec_cable = pQ - pL;
%             L_cable = norm(vec_cable);
% 
%             switch obj.cable_model_mode
%                 % -------------------------------------------------------------
%                 % 案1: システム全体を「索姿勢付き単一指向性楕円体」で一括包絡
%                 % -------------------------------------------------------------
%                 case 1
%                     c_sys = 0.5 * (pQ + pL);
%                     v_sys = 0.5 * (vQ + vL);
%                     a_long  = 0.5 * L_cable + max(obj.r_drone, obj.r_load) + d_margin_obs;
%                     b_short = max(obj.r_drone, obj.r_load) + d_margin_obs;
%                     r_equiv = max(a_long, b_short);
% 
%                     entities = struct( ...
%                         'name',   {"system_oriented_ellipsoid"}, ...
%                         'p',      {c_sys}, ...
%                         'v',      {v_sys}, ...
%                         'r_safe', {r_equiv} ...
%                     );
% 
%                 % -------------------------------------------------------------
%                 % 案2: UAV/Payloadを楕円体、索を可変サンプリング線分カプセルで評価 (推奨)
%                 % -------------------------------------------------------------
%                 case 2
%                     K_pts = max(obj.min_cable_pts, ceil(L_cable / obj.cable_ds_sample));
%                     s_samples = linspace(0.15, 0.85, K_pts);
% 
%                     ent_names = ["drone", "load"];
%                     ent_p     = {pQ, pL};
%                     ent_v     = {vQ, vL};
%                     ent_r     = {obj.r_drone + d_margin_obs, obj.r_load + d_margin_obs};
% 
%                     for k = 1:K_pts
%                         s_k = s_samples(k);
%                         p_k = (1.0 - s_k) * pL + s_k * pQ;
%                         v_k = (1.0 - s_k) * vL + s_k * vQ;
% 
%                         ent_names(end+1) = sprintf("cable_capsule_pt_%d", k); %#ok<AGROW>
%                         ent_p{end+1}     = p_k;                                %#ok<AGROW>
%                         ent_v{end+1}     = v_k;                                %#ok<AGROW>
%                         ent_r{end+1}     = obj.r_cable + d_margin_obs;         %#ok<AGROW>
%                     end
% 
%                     entities = struct('name', cellstr(ent_names), 'p', ent_p, 'v', ent_v, 'r_safe', ent_r);
% 
%                 % -------------------------------------------------------------
%                 % 案3: 機体・荷物・索を「球体プリミティブ群 (Sphere Decomposition)」に統一
%                 % -------------------------------------------------------------
%                 case 3
%                     d_ball = 2.0 * obj.r_cable;
%                     K_balls = max(obj.min_cable_pts, ceil(L_cable / max(0.15, d_ball * 1.5)));
%                     s_balls = linspace(0.1, 0.9, K_balls);
% 
%                     ent_names = ["drone_sphere", "load_sphere"];
%                     ent_p     = {pQ, pL};
%                     ent_v     = {vQ, vL};
%                     ent_r     = {obj.r_drone + d_margin_obs, obj.r_load + d_margin_obs};
% 
%                     for k = 1:K_balls
%                         s_b = s_balls(k);
%                         p_b = (1.0 - s_b) * pL + s_b * pQ;
%                         v_b = (1.0 - s_b) * vL + s_b * vQ;
% 
%                         ent_names(end+1) = sprintf("cable_sphere_%d", k); %#ok<AGROW>
%                         ent_p{end+1}     = p_b;                           %#ok<AGROW>
%                         ent_v{end+1}     = v_b;                           %#ok<AGROW>
%                         ent_r{end+1}     = obj.r_cable + d_margin_obs;    %#ok<AGROW>
%                     end
% 
%                     entities = struct('name', cellstr(ent_names), 'p', ent_p, 'v', ent_v, 'r_safe', ent_r);
% 
%                 otherwise
%                     error('未定義の cable_model_mode です: %d (1, 2, 3 のいずれかを指定してください)', obj.cable_model_mode);
%             end
%         end
% 
%         % =====================================================================
%         % filter_multi_obstacle_threats
%         % 複数障害物をソートから破棄せず、直近衝突と復帰路干渉を包括分類する
%         % =====================================================================
%         function tg = filter_multi_obstacle_threats(obj, pQ, vQ, pL, vL, detected_obs, xd_nom, ~)
%             tg = struct();
%             tg.has_threat           = false;
%             tg.threat_count         = 0;
%             tg.items                = [];
%             tg.primary_id           = NaN;
%             tg.primary_tcpa         = inf;
%             tg.primary_dcpa         = inf;
%             tg.secondary_ids        = [];
%             tg.union_center         = [0; 0; 0];
%             tg.req_clearance_max    = 0.0;
%             tg.suggested_escape_dir = [0; 0; 0];
% 
%             if isempty(detected_obs), return; end
% 
%             v_nom = xd_nom(5:7);
%             spd_nom = norm(v_nom);
%             if spd_nom < 0.1, dir_nom = [0; 0; 1]; else, dir_nom = v_nom / spd_nom; end
% 
%             all_threat_items = [];
%             combined_escape_vec = [0; 0; 0];
% 
%             for k = 1:numel(detected_obs)
%                 obs = detected_obs(k);
%                 p_o = obs.p_obs;
%                 v_o = obs.v_obs;
%                 r_bound = max(obs.radii_obs);
%                 d_m = obs.d_margin;
% 
%                 % 障害物ごとのマージンを反映して自機エンティティ群を構築
%                 entities = obj.build_system_representation(pQ, vQ, pL, vL, d_m);
% 
%                 % --- (A) 解析的 CPA 衝突判定 (Woo et al., 2026, Eq. 8-11) ---
%                 min_t_cpa = inf;
%                 min_d_cpa = inf;
%                 is_direct_threat = false;
%                 target_hit = "none";
% 
%                 for e_idx = 1:numel(entities)
%                     ent = entities(e_idx);
%                     r_rel = ent.p - p_o;
%                     v_rel = ent.v - v_o;
%                     v_rel_sq = dot(v_rel, v_rel);
% 
%                     if v_rel_sq < 1e-4
%                         t_cpa_e = 0.0;
%                         d_cpa_e = norm(r_rel) - r_bound;
%                     else
%                         t_cpa_e = -dot(r_rel, v_rel) / v_rel_sq;
%                         r_at_cpa = r_rel + v_rel * t_cpa_e;
%                         d_cpa_e = norm(r_at_cpa) - r_bound;
%                     end
% 
%                     % 未来 (-0.2s以降) に最接近し、かつ離隔が安全マージンを割り込むか
%                     if (t_cpa_e > -0.2) && (d_cpa_e < ent.r_safe)
%                         is_direct_threat = true;
%                         if t_cpa_e < min_t_cpa
%                             min_t_cpa = t_cpa_e;
%                             min_d_cpa = d_cpa_e;
%                             target_hit = ent.name;
%                         end
%                     end
%                 end
% 
%                 % --- (B) 将来復帰軌道での二次衝突判定 (Park & Cho, 2020; Woo et al., Sec 4.1) ---
%                 is_secondary_threat = false;
%                 t_future_samples = linspace(1.5, obj.t_horizon_eval, 10);
%                 min_future_corridor_dist = inf;
% 
%                 for tf = t_future_samples
%                     p_obs_f = p_o + v_o * tf;
%                     p_nom_f = pL + dir_nom * (spd_nom * tf);
% 
%                     dist_to_corridor = norm(p_nom_f - p_obs_f) - r_bound;
%                     if dist_to_corridor < min_future_corridor_dist
%                         min_future_corridor_dist = dist_to_corridor;
%                     end
% 
%                     % 荷物保護半径 + 障害物マージン + 余裕代(0.5m)
%                     if dist_to_corridor < (obj.r_load + d_m + 0.5)
%                         is_secondary_threat = true;
%                     end
%                 end
% 
%                 % --- (C) 脅威の登録（直近衝突 または 将来衝突のどちらかに該当すれば保持） ---
%                 if is_direct_threat || is_secondary_threat
%                     % 切迫度スコア (Woo et al., 2026, Algorithm 2)
%                     urgency_score = max(0.0, min_t_cpa) + 2.0 * max(0.0, min_d_cpa);
%                     if ~is_direct_threat && is_secondary_threat
%                         % 二次衝突障害物は一次衝突の直後に並べる
%                         urgency_score = urgency_score + 10.0;
%                     end
% 
%                     vec_from_obs = pL - p_o;
%                     if norm(vec_from_obs) > 1e-3
%                         n_away = vec_from_obs / norm(vec_from_obs);
%                     else
%                         n_away = obs.normal_load;
%                     end
%                     combined_escape_vec = combined_escape_vec + n_away * (1.0 / max(0.5, norm(vec_from_obs)));
% 
%                     req_clearance_i = r_bound + max(obj.r_drone, obj.r_load) + d_m;
% 
%                     item = struct();
%                     item.id                  = obs.id;
%                     item.urgency_score       = urgency_score;
%                     item.is_direct_threat    = is_direct_threat;
%                     item.is_secondary_threat = is_secondary_threat;
%                     item.t_cpa               = max(0.0, min_t_cpa);
%                     item.d_cpa               = min_d_cpa;
%                     item.threat_target       = target_hit;
%                     item.p_obs               = p_o;
%                     item.v_obs               = v_o;
%                     item.radii_obs           = obs.radii_obs;
%                     item.d_margin            = d_m;
%                     item.req_clearance       = req_clearance_i;
%                     item.n_away              = n_away;
% 
%                     all_threat_items = [all_threat_items; item];
%                 end
%             end
% 
%             % 3. 複数障害物の統合構造体を作成 (ソートで排除せず全件保持)
%             if ~isempty(all_threat_items)
%                 [~, sort_idx] = sort([all_threat_items.urgency_score], 'ascend');
%                 sorted_items = all_threat_items(sort_idx);
% 
%                 tg.has_threat         = true;
%                 tg.threat_count       = numel(sorted_items);
%                 tg.items              = sorted_items;     % 危険な障害物の完全配列
%                 tg.primary_id         = sorted_items(1).id;
%                 tg.primary_tcpa       = sorted_items(1).t_cpa;
%                 tg.primary_dcpa       = sorted_items(1).d_cpa;
% 
%                 sec_ids = [];
%                 for m = 2:numel(sorted_items)
%                     sec_ids = [sec_ids, sorted_items(m).id];
%                 end
%                 tg.secondary_ids      = sec_ids;
% 
%                 all_centers = reshape([sorted_items.p_obs], 3, []);
%                 tg.union_center       = mean(all_centers, 2);
%                 tg.req_clearance_max  = max([sorted_items.req_clearance]);
% 
%                 if norm(combined_escape_vec) > 1e-3
%                     tg.suggested_escape_dir = combined_escape_vec / norm(combined_escape_vec);
%                 else
%                     tg.suggested_escape_dir = sorted_items(1).n_away;
%                 end
%             end
%         end
%         % =====================================================================
%         % point_ellipsoid_signed_distance
%         % 任意の3次元点 p と回転楕円体 (中心 c, 半径 r=[a;b;c], 回転行列 R) の
%         % 【外郭表面】までの真の最短ユークリッド距離を算出する。
%         %
%         % [数学的アルゴリズム]:
%         % 楕円方程式: (x/a)^2 + (y/b)^2 + (z/c)^2 = 1
%         % 点 p から楕円面上の最近接点 x への最短距離は、ラグランジュ未定乗数法:
%         %   f(lambda) = sum( (a_i^2 * y_i^2) / (lambda + a_i^2)^2 ) - 1 = 0
%         % を満たす単一根 lambda を二分探索法 (Binary Search) で 80 回反復して求め、
%         % 表面点 x = (a_i^2 * y_i) / (lambda + a_i^2) と点 y の距離 ||x - y|| を計算する。
%         % =====================================================================
%         function [d, inside, closest_local, lambda] = point_ellipsoid_signed_distance(~, p, o)
%             p = p(:);
%             c = o.center(:);
%             r = o.radii(:);
%             R = o.R;
% 
%             % --- 入力チェック: 不正ならフォールバックせず即座に停止 ---
%             if numel(p) ~= 3 || numel(c) ~= 3 || numel(r) ~= 3
%                 error('点 p、中心 c、および半径 r はすべて3x1ベクトルである必要があります。');
%             end
%             if any(~isfinite(p)) || any(~isfinite(c)) || any(~isfinite(r))
%                 error('点または楕円体の定義に非有限値 (NaN や Inf) が検出されました。');
%             end
%             if any(r <= 0)
%                 error('楕円体の半径はすべて厳密に正の値 (r > 0) である必要があります。');
%             end
%             if ~isequal(size(R), [3 3]) || any(~isfinite(R(:)))
%                 error('回転行列 R は有限な3x3行列である必要があります。');
%             end
%             if norm(R'*R - eye(3), 'fro') > 1e-6 || abs(det(R) - 1.0) > 1e-6
%                 error('行列 R は有効な直交回転行列 SO(3) ではありません。');
%             end
% 
%             % 1. ワールド座標系の点 p を障害物の局所主軸座標系 (Local Frame) に変換
%             y  = R' * (p - c);
%             r2 = r.^2;
% 
%             % 2. 楕円代数判定値 q:
%             %    q < 1.0 -> 点は楕円体の内部にある (衝突・侵入状態)
%             %    q = 1.0 -> 点は楕円体の表面上にある
%             %    q > 1.0 -> 点は楕円体の外部にある
%             q      = sum((y ./ r).^2);
%             inside = (q < 1.0);
% 
%             % 特異点処理: 点が障害物の中心そのものにある場合 (最も近い表面は最短半径の主軸上)
%             if norm(y) < 1e-14
%                 [min_r, min_idx] = min(r);
%                 closest_local = zeros(3, 1);
%                 closest_local(min_idx) = min_r;
%                 d      = -min_r;
%                 lambda = -min(r2);
%                 return;
%             end
% 
%             % ラグランジュ未定乗数の非線形方程式 f(lambda) = 0
%             f = @(lam) sum(r2 .* (y.^2) ./ ((lam + r2).^2)) - 1.0;
% 
%             % 3. ラグランジュ乗数 lambda の探索範囲 (Bracketing) の設定と根の存在検証
%             if q > 1.0
%                 % 外部点の場合: lambda >= 0
%                 lo = 0.0;
%                 hi = max(r) * norm(y);
% 
%                 if f(lo) < 0 || f(hi) > 0
%                     error('外部点に対する楕円体距離の求根ブラケット設定に失敗しました (解を挟み込めていません)。');
%                 end
%             else
%                 % 内部点の場合: -min(r_i^2) < lambda < 0
%                 lo = -min(r2) * (1.0 - 1e-12);
%                 hi = 0.0;
% 
%                 if f(lo) < 0 || f(hi) > 0
%                     error('内部点に対する楕円体距離の求根ブラケット設定に失敗しました (解を挟み込めていません)。');
%                 end
%             end
% 
%             % 4. 二分探索により lambda を数値的に収束させる
%             %    80回反復して根を高精度に求める
%             for kk = 1:80
%                 mid = 0.5 * (lo + hi);
%                 if f(mid) > 0
%                     lo = mid;
%                 else
%                     hi = mid;
%                 end
%             end
%             lambda = 0.5 * (lo + hi);
% 
%             % 5. 局所座標系における楕円体表面上の最近接点 closest_local
%             closest_local = r2 .* y ./ (lambda + r2);
%             d_abs         = norm(closest_local - y);
% 
%             % 6. 表面までの最短ユークリッド距離
%             %    外部なら正値 (+), 内部なら侵入深さとして負値 (-) を返す
%             if inside
%                 d = -d_abs;
%             else
%                 d = d_abs;
%             end
%         end
%     end
% end

% classdef REPLANNING_BSPLINE < handle
%     % =========================================================================
%     % REPLANNING_BSPLINE
%     % 1.模擬センサー：機体重心 pQ を点（Point）として扱い、楕円体境界面までのユークリッド最短距離を厳密に計算して検知を判定
%     % 2.障害物評価：
%     % 【索（テザー）幾何モデル化モード (cable_model_mode)】:
%     %   1 : システム全体を「索姿勢付き単一指向性楕円体」として一括包絡 (案1: 超高速・保守的)
%     %   2 : UAV/Payloadを楕円体、索を可変サンプリング線分カプセルで評価 (案2: 高精度・推奨)
%     %   3 : 機体・荷物・索を「球体プリミティブ群 (Sphere Decomposition)」に統一 (案3)
%     % 【安全マージン (d_margin)】:
%     %   ENVIRONMENT_OBSTACLE_ELLIPSE_MOVE 内の各障害物が保持する d_margin を
%     %   動的に読み込んで適用 (Park & Cho, 2020; Geetha et al., 2026)
%     % [依拠理論]:
%     % 1. Geetha et al. (2026): 索線分サンプリングによる離散包括クリアランス評価 (式4)
%     % 2. Woo et al. (2026): 解析的CPA (式8-11) & 複数エンティティ連立衝突判定
%     % 3. Park & Cho (2020): バウンディングボックス幾何生成 & 予測エンベロープ評価 (Sec. 5)
%     % =========================================================================
%     properties
%         self % ドローンエージェント自身 (推定器 estimator やパラメータ parameter を保持) 
%         base_ref % 公称参照軌道生成オブジェクト
%         result                % 出力結果構造体 (目標状態 xd, pRef, vRef, yawRef, 検知情報)
% 
%         trigger_dist = 15.0;   % 接近検知の閾値 [m] (表面間距離がこれ以下になるとアラート)
%         r_load       = 0.15;  % 荷物保護半径 [m]
%         r_drone      = 0.30;  % 機体保護半径 [m]
%         r_cable      = 0.08;  % 索（ケーブル）等価保護半径 [m]
%         default_margin = 0.30;% 障害物に d_margin が未定義の場合のフォールバックマージン [m]
% 
%         % --- 索幾何モデル切り替え設定 ---
%         cable_model_mode = 2; % 1: 単一楕円体, 2: 楕円体+カプセル, 3: 球体分解
%         cable_ds_sample  = 0.40; % 索サンプリング間隔 [m] (長さ L に応じて点数を動的スケール)
%         min_cable_pts    = 3;    % 索上の最小サンプリング点数
% 
%         t_horizon_eval   = 6.0;  % 将来予測評価ホライズン [s] (回避から合流までの最大時間)
%     end
% 
%     methods (Access = public)
%         % =====================================================================
%         % コンストラクタ: クラスの初期化と外部設定 (opts) の反映
%         % =====================================================================
%         function obj = REPLANNING_BSPLINE(self, base_ref, opts)
%             arguments
%                 self                  % 必須: エージェントインスタンス
%                 base_ref              % 必須: 通常飛行用の公称軌道インスタンス
%                 opts = struct()       % 任意: 外部からパラメータを変更するための構造体
%             end
% 
%             obj.self = self;
%             obj.base_ref = base_ref;
% 
%             if isfield(opts, 'trigger_dist'), obj.trigger_dist = opts.trigger_dist; end % 外で定義されていたらデフォルト値を上書き　センサー代わりの検知範囲
%             if isfield(opts, 'r_load'),       obj.r_load       = opts.r_load;       end % 外で定義されていたらデフォルト値を上書き　牽引物を近似した球体
%             if isfield(opts, 'r_drone'),      obj.r_drone      = opts.r_drone;      end % 外で定義されていたらデフォルト値を上書き　機体を近似した球体
%             if isfield(opts, 'r_cable'),           obj.r_cable           = opts.r_cable;           end
%             if isfield(opts, 'default_margin'),    obj.default_margin    = opts.default_margin;    end
%             if isfield(opts, 'cable_model_mode'),  obj.cable_model_mode  = opts.cable_model_mode;  end
%             if isfield(opts, 'cable_ds_sample'),   obj.cable_ds_sample   = opts.cable_ds_sample;   end
%             if isfield(opts, 't_horizon_eval'),    obj.t_horizon_eval    = opts.t_horizon_eval;    end
% 
%             % base_ref の result 構造体をそのまま継承 (直下は state のみ)
%             obj.result = base_ref.result;
%             % --- 【重要】STATE_CLASS に検知用プロパティを動的追加 (dynamicprops) ---
%             % これにより、state 内に正式な記録領域が作成され、ロガーで抽出可能になる
%             sensor_props = [ ...
%                 "time", "pQ", "pL", ...                                         % 基本計測状態
%                 "detected_point", ...                                           % 15m検知フラグ
%                 "drone_inside_obstacle_point", "load_inside_obstacle_point", ...% 内部侵入フラグ
%                 "drone_min_dist_point", "load_min_dist_point", "min_dist_point",... % 表面間最短幾何距離
%                 "drone_obstacle_id_point", "load_obstacle_id_point", "min_obstacle_id_point", ... % 最寄ID
%                 "min_source_point", ...                                         % 最短距離をもたらした部位 ("drone" / "load")
%                 "detected_obstacle_count_point", ...                            % 15m以内の検知総数
%                 "active_threat_count", ...                                      % 真に衝突危険のある連立脅威数
%                 "primary_threat_id", ...                                        % 最も切迫している障害物ID
%                 "primary_threat_tcpa", ...                                      % 最優先障害物の衝突到達予測時間 [s]
%                 "primary_threat_dcpa" ...                                       % 最優先障害物の最接近予測距離 [m]
%             ];
%             for p_name = sensor_props
%                 if ~isprop(obj.result.state, p_name)
%                     addprop(obj.result.state, p_name);
%                 end
%             end
% 
%             % 初期ダミー値のセット
%             obj.clear_state_sensor_values(0.0);
%         end
% 
%         % =====================================================================
%         % do: 制御周期ごと (例: 25ms周期) にメインループから呼び出される実行メソッド
%         % 入力: 
%         %   varargin{1}: time (現在の時刻 struct: time.t, time.dt など)
%         %   varargin{2}: cha  (フェーズ文字列: 'f' = 飛行中, 't' = 離陸など)
%         %   varargin{4}: env  (環境構造体、障害物リストを内包する場合あり)
%         % 出力:
%         %   result_out : 下流のコントローラやロガーが受け取る目標状態・検知結果
%         % =====================================================================
%         function result_out = do(obj, varargin)
%             time = varargin{1}; % varargin{1}: time (現在の時刻 struct: time.t, time.dt など)
%             cha = varargin{2}; % varargin{2}: cha  (フェーズ文字列: 'f' = 飛行中, 't' = 離陸など)
% 
%             % --- 公称目標軌道 (Nominal Reference) の算出 ---
%             % 本クラスが障害物を回避する新軌道を生成しない間は、公称軌道生成器の出力をそのまま踏襲する
%             % 公称参照軌道の取得
%             base_res = obj.base_ref.do(varargin{:}); % 公称軌道の抜き出し
%             xd_nom = base_res.state.xd; % 牽引物の目標３次元位置・yaw角からその６階微分まで [pL(3); yaw(1); vL(3); yaw_dot(1); aL(3)...]
% 
%             % 毎ステップ、検知プロパティの初期値をリセット
%             obj.clear_state_sensor_values(time.t);
% 
%             % obj.result.state に公称軌道を反映 (base_res 全体の上書きは行わない)
%             obj.result.state.xd = xd_nom; % 公称軌道保存
% 
%             % --- 空の検知構造体を用意 (非飛行フェーズ用) ---
%             detection = struct();
%             detection.time                          = time.t;             % [s] 現在のシミュレーション時刻
%             detection.pQ                            = [NaN; NaN; NaN];    % [m] ドローン機体重心の3次元位置ベクトル [x; y; z]
%             detection.pL                            = [NaN; NaN; NaN];    % [m] 牽引荷物の3次元位置ベクトル [x; y; z]
%             detection.detected_point                = false;              % [bool] センサー検知範囲近接検知フラグ (true: 検知, false: 未検知)
%             detection.drone_inside_obstacle_point   = false;              % [bool] 機体の障害物楕円体内部侵入フラグ (true: 侵入, false: 外部)
%             detection.load_inside_obstacle_point    = false;              % [bool] 荷物の障害物楕円体内部侵入フラグ (true: 侵入, false: 外部)
%             detection.drone_min_dist_point          = inf;                % [m] 機体から全障害物表面までの最短幾何学距離
%             detection.load_min_dist_point           = inf;                % [m] 荷物から全障害物表面までの最短幾何学距離
%             detection.min_dist_point                = inf;                % [m] システム全体(機体・荷物)で最も近い表面距離 min(dQ, dL)
%             detection.drone_obstacle_id_point       = NaN;                % [ID] 機体にとって最短距離を与えている障害物インデックス番号
%             detection.load_obstacle_id_point        = NaN;                % [ID] 荷物にとって最短距離を与えている障害物インデックス番号
%             detection.min_obstacle_id_point         = NaN;                % [ID] システム全体で最短距離を与えている障害物インデックス番号
%             detection.min_source_point              = "none";             % [string] 最短距離をもたらしたセンサ種別 ("drone", "load", "none")
%             detection.detected_obstacle_count_point = 0;                  % [個] センサー検知範囲以内に検知された障害物の総数
% 
%             % 複数障害物統合データ構造体 (軌道生成へ引き渡す完全情報)
%             threat_group = struct();
%             threat_group.has_threat           = false;
%             threat_group.threat_count         = 0;
%             threat_group.items                = [];     % 危険な全障害物の詳細リスト (除外なし)
%             threat_group.primary_id           = NaN;    % 今最も切迫している障害物ID
%             threat_group.primary_tcpa         = inf;    % 最優先障害物の到達時間 [s]
%             threat_group.primary_dcpa         = inf;    % 最優先障害物の最接近距離 [m]
%             threat_group.secondary_ids        = [];     % 回避・復帰軌道上で衝突する障害物ID群
%             threat_group.union_center         = [0;0;0];% 複数障害物の合成幾何中心
%             threat_group.req_clearance_max    = 0.0;    % 最大必要退避クリアランス [m]
%             threat_group.suggested_escape_dir = [0;0;0]; % 全障害物を一括回避する推奨法線方向
% 
%             % --- 飛行フェーズ ('f') 時の近接障害物スキャン ---
%             if cha == 'f'
%                 % --- 機体・荷物の現在位置の取得 ---
%                 % (A) 状態推定器 (estimator) から真値・推定位置を直接取得
%                 % ※ フォールバックを排除しているため、estimator にプロパティが存在しない場合は即座にエラー停止
%                 pL_cur = obj.self.estimator.result.state.pL(:); % 荷物位置の現在3次元位置 [x; y; z]
%                 pQ_cur = obj.self.estimator.result.state.p(:); % ドローン機体の現在3次元位置 [x; y; z]
%                 vL_cur = obj.self.estimator.result.state.vL(:); % 荷物の現在3次元速度 [vx; vy; vz]
%                 vQ_cur = obj.self.estimator.result.state.v(:); % ドローン機体の現在3次元速度 [vx; vy; vz]
% 
%                 % (B) 現在時刻 t における動的障害物配置を取得
%                 obs_list = obj.get_obstacles_at_time(time.t);
% 
% 
%                 % (C) 機体の重心および荷物の取り付け位置から障害物表面までの最短ユークリッド距離を幾何計算
%                 detection = obj.check_detection_simulated_sensor(pQ_cur, pL_cur, obs_list, time.t);
% 
%                 % (D) 幾何モデル切り替え ＆ 環境マージン連動型 マルチスキャンフィルタ
%                 threat_group = obj.filter_multi_obstacle_threats( ...
%                     pQ_cur, vQ_cur, pL_cur, vL_cur, detection.detected_obstacles_point, xd_nom, time.t);
% 
%                 % (E) コンソール診断ログ出力
%                 if threat_group.has_threat
%                     primary = threat_group.items(1);
%                     fprintf("[PROXIMITY ALERT | Drone-LiDAR 15m] t=%.3f s | センサ視界内: %d 個 | 衝突脅威: %d 個\n", ...
%                         time.t, detection.detected_obstacle_count_point, threat_group.threat_count);
%                     fprintf("  -> [最切迫脅威 ID:%d] 危険部位: %s (機体表面間: %.2f m, 荷物表面間: %.2f m)\n", ...
%                         threat_group.primary_id, primary.threat_target, ...
%                         detection.drone_min_dist_point, detection.load_min_dist_point);
%                     fprintf("  -> 到達予測: %.2f s後, CPA予測離隔: %.2f m (適用マージン: %.2f m)\n", ...
%                         primary.t_cpa, primary.d_cpa, primary.d_margin);
%                     if ~isempty(threat_group.secondary_ids)
%                         fprintf("  -> [復帰路の二次警戒] ID: %s (回避後の追突を警戒)\n", mat2str(threat_group.secondary_ids));
%                     end
%                 end
%             end
% 
%             % --- 4. 【最重要】state 内の各プロパティに代入して app.logger に完全保存 ---
%             st = obj.result.state;
%             st.xd                            = xd_nom;                                  % [28x1 double] 目標軌道全状態
%             st.p                             = xd_nom(1:3);                             % [m] 目標位置 [x; y; z]
%             st.v                             = xd_nom(5:7);                             % [m/s] 目標速度 [vx; vy; vz]
%             st.q                             = [0; 0; xd_nom(4)];                       % [rad] 目標姿勢 (yaw角)
% 
%             st.time                          = detection.time;                          % [s] 計測時刻 (double)
%             st.pQ                            = detection.pQ;                            % [m] ドローン機体重心位置 (3x1 double)
%             st.pL                            = detection.pL;                            % [m] 荷物位置 (3x1 double)
%             st.detected_point                = detection.detected_point;                % [bool] センサー範囲内近接検知判定フラグ (true / false)
%             st.drone_inside_obstacle_point   = detection.drone_inside_obstacle_point;   % [bool] 機体侵入フラグ (true: 侵入, false: 外部)
%             st.load_inside_obstacle_point    = detection.load_inside_obstacle_point;    % [bool] 荷物侵入フラグ (true: 侵入, false: 外部)
%             st.drone_min_dist_point          = detection.drone_min_dist_point;          % [m] 機体の最短表面距離 (正: 外部, 負: 侵入深さ)
%             st.load_min_dist_point           = detection.load_min_dist_point;           % [m] 荷物の最短表面距離 (正: 外部, 負: 侵入深さ)
%             st.min_dist_point                = detection.min_dist_point;                % [m] システム全体の最短幾何表面距離 min(dQ, dL)
%             st.drone_obstacle_id_point       = detection.drone_obstacle_id_point;       % [ID] 機体にとって最短の障害物インデックス番号
%             st.load_obstacle_id_point        = detection.load_obstacle_id_point;        % [ID] 荷物にとって最短の障害物インデックス番号
%             st.min_obstacle_id_point         = detection.min_obstacle_id_point;         % [ID] 全体で最短の障害物インデックス番号
%             st.min_source_point              = detection.min_source_point;              % [string] 最短距離センサ種別 ("drone" または "load")
%             st.detected_obstacle_count_point = detection.detected_obstacle_count_point; % [個] センサー範囲以内に検知された障害物の総数
% 
%             % 新規マルチ脅威指標の代入 (ロガーで完全記録)
%             st.active_threat_count           = threat_group.threat_count;
%             st.primary_threat_id             = threat_group.primary_id;
%             st.primary_threat_tcpa           = threat_group.primary_tcpa;
%             st.primary_threat_dcpa           = threat_group.primary_dcpa;
% 
%             obj.result.state                 = st;
%             obj.result.obstacle_detection    = detection;
%             obj.result.threat_group          = threat_group; % 軌道生成側へ渡す構造体
% 
%             result_out = obj.result;
%         end
%     end
% 
%     methods (Access = private)
%         % =====================================================================
%         % clear_state_sensor_values: state 内の検知プロパティを初期化
%         % =====================================================================
%         function clear_state_sensor_values(obj, t_now)
%             st = obj.result.state;
%             st.time                          = t_now;              % [s] 現在時刻
%             st.pQ                            = [NaN; NaN; NaN];    % [m] ドローン位置初期値
%             st.pL                            = [NaN; NaN; NaN];    % [m] 荷物位置初期値
%             st.detected_point                = false;              % 検知なし
%             st.drone_inside_obstacle_point   = false;              % 機体侵入なし
%             st.load_inside_obstacle_point    = false;              % 荷物侵入なし
%             st.drone_min_dist_point          = inf;                % 最短距離初期値 (無限大)
%             st.load_min_dist_point           = inf;                % 最短距離初期値 (無限大)
%             st.min_dist_point                = inf;                % 最短距離初期値 (無限大)
%             st.drone_obstacle_id_point       = NaN;                % 最短障害物ID初期値
%             st.load_obstacle_id_point        = NaN;                % 最短障害物ID初期値
%             st.min_obstacle_id_point         = NaN;                % 最短障害物ID初期値
%             st.min_source_point              = "none";             % 最短センサ初期値
%             st.detected_obstacle_count_point = 0;                  % 検知個数初期値
%             st.active_threat_count           = 0;
%             st.primary_threat_id             = NaN;
%             st.primary_threat_tcpa           = inf;
%             st.primary_threat_dcpa           = inf;
%         end
% 
%         % =====================================================================
%         % get_obstacles_at_time: 環境関数を叩き、時刻 t_now での障害物リストを取得
%         % =====================================================================
%         function list = get_obstacles_at_time(~,t_now)
%             % ENVIRONMENT_OBSTACLE_ELLIPSE_MOVE 内ですでに「p_center = p0 + v*t」
%             % のように時刻 t_now に応じた現在位置が計算されている
%             list = ENVIRONMENT_OBSTACLE_ELLIPSE_MOVE(t_now);
% 
%         end
% 
%         % =====================================================================
%         % check_detection_simulated_sensor: 全障害物を走査し、機体・荷物の点と障害物の表面との最短距離を評価
%         % =====================================================================
%         function det = check_detection_simulated_sensor(obj, pQ, pL, obs_list, t_now)
%             det = struct();
% 
%             % --- センサ状態・時刻 ---
%             det.time                          = t_now;              % [s] 現在のシミュレーション時刻 (double)
%             det.pQ                            = pQ;                 % [m] ドローン機体重心の3次元位置ベクトル [x; y; z] (3x1 double)
%             det.pL                            = pL;                 % [m] 牽引荷物の3次元位置ベクトル [x; y; z] (3x1 double)
%             det.trigger_dist                  = obj.trigger_dist;   % [m] 近接検知判定の閾値距離 (例: 7.0 m) (double)
% 
%             % --- 全体判定フラグ ---
%             det.detected_point                = false;              % [bool] 機体または荷物が障害物からセンサー検知範囲以内に接近したか (true: 検知, false: 未検知)
%             det.drone_inside_obstacle_point   = false;              % [bool] 機体が障害物楕円体の内部へ侵入/衝突したか (true: 侵入, false: 外部)
%             det.load_inside_obstacle_point    = false;              % [bool] 荷物が障害物楕円体の内部へ侵入/衝突したか (true: 侵入, false: 外部)
% 
%             % --- 最短距離 ---
%             det.drone_min_dist_point          = inf;                % [m] 環境内の全障害物の中で、機体から表面までの最短幾何学距離 (double, 内部時は負値)
%             det.load_min_dist_point           = inf;                % [m] 環境内の全障害物の中で、荷物から表面までの最短幾何学距離 (double, 内部時は負値)
%             det.min_dist_point                = inf;                % [m] システム全体(機体・荷物の双方)で最も近い表面距離 min(dQ, dL) (double)
% 
%             % --- 識別情報・統計 ---
%             det.drone_obstacle_id_point       = [];                 % [ID] 機体にとって最短距離を与えている障害物のインデックス番号 (integer)
%             det.load_obstacle_id_point        = [];                 % [ID] 荷物にとって最短距離を与えている障害物のインデックス番号 (integer)
%             det.min_obstacle_id_point         = [];                 % [ID] システム全体で最短距離を与えている障害物のインデックス番号 (integer)
%             det.min_source_point              = "none";                 % [string] 最短距離をもたらしたセンサ種別 ("drone": 機体, "load": 荷物)
%             det.detected_obstacles_point      = [];                 % [struct配列] センサー検知範囲以内に検知された全障害物の詳細情報リスト (各要素は下記2を参照)
%             det.detected_obstacle_count_point = 0;                  % [個] センサー感知範囲以内に検知された障害物の総数 (integer, 未検知時は0)
% 
%             if isempty(obs_list)
%                 return;
%             end
% 
%             detected_obs_point = [];
% 
%             for i = 1:length(obs_list)
%                 o = obs_list(i);
% 
%                 % --- 障害物パラメータの抽出 (推測フォールバックなし) ---
%                 if ~isfield(o, 'R_obs') || ~isfield(o, 'ellipsoid_radii') || ~isfield(o, 'p_center')
%                     error('インデックス %d の障害物に必須フィールド (R_obs, ellipsoid_radii, p_center) が不足しています.', i);
%                 end
% 
%                 % --- 障害物の姿勢回転行列 R_obs の抽出 ---
%                 R_obs = o.R_obs;
%                 % --- 外接楕円体の主軸半径 [a; b; c] の抽出 ---
%                 radii_obs = o.ellipsoid_radii(:);
%                 % --- 障害物の中心位置 ---
%                 % get_obstacles_at_time(t_now) で既に t_now 時点の中心位置が得られているため、
%                 % timestamp フィールドが明示的に別時刻として存在する場合のみ差分時間で補正
%                 p_obs = o.p_center(:);
%                 % --- 障害物の速度 ---
%                 v_obs = o.v_center(:);
%                 %--- 障害物の安全マージン ---
%                 d_margin_obs = obj.default_margin;
% 
%                 obs_parsed = struct('center', p_obs, 'radii', radii_obs, 'R', R_obs);
% 
%                 % --- E. 機体 pQ および荷物 pL から楕円体表面への符号付き最短幾何距離 ---
%                 % d > 0: 表面の外側にある (表面までの最短距離 [m])
%                 % d < 0: 内部に侵入している (侵入深さ [m])
%                 % ここでは機体は重心点・牽引物は取り付け点の点と考えて，その表面からの障害物を近似した楕円表面への最短距離が検知範囲内かどうかの判定に使用
%                 [d_drone_point, inside_drone_point, cpQ_local_point, ~] = obj.point_ellipsoid_signed_distance(pQ, obs_parsed);
%                 [d_load_point,  inside_load_point,  cpL_local_point, ~] = obj.point_ellipsoid_signed_distance(pL, obs_parsed);
% 
%                 % ワールド座標系における表面最近接点: x_world = center + R * x_local
%                 cpQ_world_point = p_obs + R_obs * cpQ_local_point;
%                 cpL_world_point = p_obs + R_obs * cpL_local_point;
% 
%                 % --- 障害物表面における真の外向き単位法線ベクトル (ワールド座標系) ---
%                 % 幾何学的定義: 楕円体表面の陰関数 F(x) = (x/rx)^2 + (y/ry)^2 + (z/rz)^2 - 1 = 0
%                 % 局所法線ベクトルは勾配 grad(F) = 2 * [x/rx^2; y/ry^2; z/rz^2] に等しい。
%                 % これにより、点 p が表面上に完全一致 (d = 0) している場合でもフォールバックなしで厳密に求まる。
%                 nQ_local_point = cpQ_local_point ./ (radii_obs.^2);
%                 if norm(nQ_local_point) < 1e-12
%                     error('機体の最近接点における法線ベクトルが退化 (ゼロベクトル) しました.');
%                 end
%                 normal_drone_point = R_obs * (nQ_local_point / norm(nQ_local_point));
% 
%                 nL_local_point = cpL_local_point ./ (radii_obs.^2);
%                 if norm(nL_local_point) < 1e-12
%                     error('機体の最近接点における法線ベクトルが退化 (ゼロベクトル) しました.');
%                 end
%                 normal_load_point = R_obs * (nL_local_point / norm(nL_local_point));
% 
%                 % 幾何学的侵入判定の論理和更新
%                 if inside_drone_point, det.drone_inside_obstacle_point = true; end
%                 if inside_load_point,  det.load_inside_obstacle_point  = true; end
% 
%                 % 機体側の最小値更新
%                 if d_drone_point < det.drone_min_dist_point
%                     det.drone_min_dist_point = d_drone_point;
%                     det.drone_obstacle_id_point    = i;
%                 end
%                 % 荷物側の最小値更新
%                 if d_load_point < det.load_min_dist_point
%                     det.load_min_dist_point = d_load_point;
%                     det.load_obstacle_id_point    = i;
%                 end
% 
%                 obs_min_point = min(d_drone_point, d_load_point);
% 
%                 % 該当障害物の詳細データ構造体
%                 obs_info_point = struct( ...
%                     'id',                        i, ...                  % [ID] 障害物のインデックス番号 (integer)
%                     'dist_drone_point',          d_drone_point, ...      % [m] 機体重心からこの障害物表面までの符号付き最短距離 (正: 外部, 負: 侵入深さ)
%                     'dist_load_point',           d_load_point, ...       % [m] 荷物位置からこの障害物表面までの符号付き最短距離 (正: 外部, 負: 侵入深さ)
%                     'min_dist_point',            obs_min_point, ...      % [m] この障害物に対する機体・荷物双方の最小表面間距離 min(dQ, dL)
%                     'drone_inside_point',        inside_drone_point, ... % [bool] 機体がこの障害物の内部に侵入しているか (true: 侵入, false: 外部)
%                     'load_inside_point',         inside_load_point, ...  % [bool] 荷物がこの障害物の内部に侵入しているか (true: 侵入, false: 外部)
%                     'p_obs',                     p_obs, ...              % [m] 時刻 t における障害物の中心位置ベクトル [x; y; z] (3x1 double)
%                     'v_obs',                     v_obs, ...
%                     'radii_obs',                 radii_obs, ...          % [m] 障害物の3軸半径 [rx; ry; rz] (3x1 double)
%                     'R_obs',                     R_obs, ...              % [-] 障害物の主軸姿勢を表す3x3回転行列 SO(3) (3x3 double)
%                     'd_margin',                  d_margin_obs, ...
%                     'closest_drone_world_point', cpQ_world_point, ...    % [m] 機体に対する障害物表面上の最短最近接点 (ワールド座標系 [x; y; z])
%                     'closest_load_world_point',  cpL_world_point, ...    % [m] 荷物に対する障害物表面上の最短最近接点 (ワールド座標系 [x; y; z])
%                     'normal_drone_point',        normal_drone_point, ... % [-] 機体側最近接点における障害物表面の真の外向き単位法線ベクトル (ワールド系, 単位ノルム)
%                     'normal_load_point',         normal_load_point ...   % [-] 荷物側最近接点における障害物表面の真の外向き単位法線ベクトル (ワールド系, 単位ノルム)
%                 );
% 
%                 % --- 接近検知判定 (センサー検知範囲以内) ---
%                 if d_drone_point <= obj.trigger_dist
%                     det.detected_point = true;
%                     detected_obs_point = [detected_obs_point; obs_info_point];
%                 end
%             end
% 
%             % システム全体での最短距離および検知元の確定
%             % ドローンセンサが捉えた障害物群の中で、機体・荷物のどちらが最も危険域に近いか
%             if det.drone_min_dist_point <= det.load_min_dist_point
%                 det.min_dist_point  = det.drone_min_dist_point;
%                 det.min_obstacle_id_point = det.drone_obstacle_id_point;
%                 det.min_source_point      = "drone";
%             else
%                 det.min_dist_point  = det.load_min_dist_point;
%                 det.min_source_point      = "load";
%             end
% 
%             % センサ自身の最短探知距離 (ドローンから最も近い障害物表面までの距離)
%             det.sensor_min_dist = det.drone_min_dist_point;
% 
%             det.detected_obstacles_point = detected_obs_point;
%             det.detected_obstacle_count_point = numel(detected_obs_point);  % 検知された障害物の個数を記録
%         end
%         % =====================================================================
%         % build_system_representation
%         % 選択されたモード (1, 2, 3) と障害物固有の d_margin に基づき自機エンティティ群を構築
%         % =====================================================================
%         function entities = build_system_representation(obj, pQ, vQ, pL, vL, d_margin_obs)
%             vec_cable = pQ - pL;
%             L_cable = norm(vec_cable);
% 
%             switch obj.cable_model_mode
%                 % -------------------------------------------------------------
%                 % 案1: システム全体を「索姿勢付き単一指向性楕円体」で一括包絡
%                 % -------------------------------------------------------------
%                 case 1
%                     c_sys = 0.5 * (pQ + pL);
%                     v_sys = 0.5 * (vQ + vL);
%                     a_long  = 0.5 * L_cable + max(obj.r_drone, obj.r_load) + d_margin_obs;
%                     b_short = max(obj.r_drone, obj.r_load) + d_margin_obs;
%                     r_equiv = max(a_long, b_short);
% 
%                     entities = struct( ...
%                         'name',   {"system_oriented_ellipsoid"}, ...
%                         'p',      {c_sys}, ...
%                         'v',      {v_sys}, ...
%                         'r_safe', {r_equiv} ...
%                     );
% 
%                 % -------------------------------------------------------------
%                 % 案2: UAV/Payloadを楕円体、索を可変サンプリング線分カプセルで評価 (推奨)
%                 % -------------------------------------------------------------
%                 case 2
%                     K_pts = max(obj.min_cable_pts, ceil(L_cable / obj.cable_ds_sample));
%                     s_samples = linspace(0.15, 0.85, K_pts);
% 
%                     ent_names = ["drone", "load"];
%                     ent_p     = {pQ, pL};
%                     ent_v     = {vQ, vL};
%                     ent_r     = {obj.r_drone + d_margin_obs, obj.r_load + d_margin_obs};
% 
%                     for k = 1:K_pts
%                         s_k = s_samples(k);
%                         p_k = (1.0 - s_k) * pL + s_k * pQ;
%                         v_k = (1.0 - s_k) * vL + s_k * vQ;
% 
%                         ent_names(end+1) = sprintf("cable_capsule_pt_%d", k); %#ok<AGROW>
%                         ent_p{end+1}     = p_k;                                %#ok<AGROW>
%                         ent_v{end+1}     = v_k;                                %#ok<AGROW>
%                         ent_r{end+1}     = obj.r_cable + d_margin_obs;         %#ok<AGROW>
%                     end
% 
%                     entities = struct('name', cellstr(ent_names), 'p', ent_p, 'v', ent_v, 'r_safe', ent_r);
% 
%                 % -------------------------------------------------------------
%                 % 案3: 機体・荷物・索を「球体プリミティブ群 (Sphere Decomposition)」に統一
%                 % -------------------------------------------------------------
%                 case 3
%                     d_ball = 2.0 * obj.r_cable;
%                     K_balls = max(obj.min_cable_pts, ceil(L_cable / max(0.15, d_ball * 1.5)));
%                     s_balls = linspace(0.1, 0.9, K_balls);
% 
%                     ent_names = ["drone_sphere", "load_sphere"];
%                     ent_p     = {pQ, pL};
%                     ent_v     = {vQ, vL};
%                     ent_r     = {obj.r_drone + d_margin_obs, obj.r_load + d_margin_obs};
% 
%                     for k = 1:K_balls
%                         s_b = s_balls(k);
%                         p_b = (1.0 - s_b) * pL + s_b * pQ;
%                         v_b = (1.0 - s_b) * vL + s_b * vQ;
% 
%                         ent_names(end+1) = sprintf("cable_sphere_%d", k); %#ok<AGROW>
%                         ent_p{end+1}     = p_b;                           %#ok<AGROW>
%                         ent_v{end+1}     = v_b;                           %#ok<AGROW>
%                         ent_r{end+1}     = obj.r_cable + d_margin_obs;    %#ok<AGROW>
%                     end
% 
%                     entities = struct('name', cellstr(ent_names), 'p', ent_p, 'v', ent_v, 'r_safe', ent_r);
% 
%                 otherwise
%                     error('未定義の cable_model_mode です: %d (1, 2, 3 のいずれかを指定してください)', obj.cable_model_mode);
%             end
%         end
% 
%         % =====================================================================
%         % filter_multi_obstacle_threats
%         % 複数障害物をソートから破棄せず、直近衝突と復帰路干渉を包括分類する
%         % =====================================================================
%         function tg = filter_multi_obstacle_threats(obj, pQ, vQ, pL, vL, detected_obs, xd_nom, ~)
%             tg = struct();
%             tg.has_threat           = false;
%             tg.threat_count         = 0;
%             tg.items                = [];
%             tg.primary_id           = NaN;
%             tg.primary_tcpa         = inf;
%             tg.primary_dcpa         = inf;
%             tg.secondary_ids        = [];
%             tg.union_center         = [0; 0; 0];
%             tg.req_clearance_max    = 0.0;
%             tg.suggested_escape_dir = [0; 0; 0];
% 
%             if isempty(detected_obs), return; end
% 
%             v_nom = xd_nom(5:7);
%             spd_nom = norm(v_nom);
%             if spd_nom < 0.1, dir_nom = [0; 0; 1]; else, dir_nom = v_nom / spd_nom; end
% 
%             all_threat_items = [];
%             combined_escape_vec = [0; 0; 0];
% 
%             for k = 1:numel(detected_obs)
%                 obs = detected_obs(k);
%                 p_o = obs.p_obs;
%                 v_o = obs.v_obs;
%                 r_bound = max(obs.radii_obs);
%                 d_m = obs.d_margin;
% 
%                 % 障害物ごとのマージンを反映して自機エンティティ群を構築
%                 entities = obj.build_system_representation(pQ, vQ, pL, vL, d_m);
% 
%                 % --- (A) 解析的 CPA 衝突判定 (Woo et al., 2026, Eq. 8-11) ---
%                 min_t_cpa = inf;
%                 min_d_cpa = inf;
%                 is_direct_threat = false;
%                 target_hit = "none";
% 
%                 for e_idx = 1:numel(entities)
%                     ent = entities(e_idx);
%                     r_rel = ent.p - p_o;
%                     v_rel = ent.v - v_o;
%                     v_rel_sq = dot(v_rel, v_rel);
% 
%                     if v_rel_sq < 1e-4
%                         t_cpa_e = 0.0;
%                         d_cpa_e = norm(r_rel) - r_bound;
%                     else
%                         t_cpa_e = -dot(r_rel, v_rel) / v_rel_sq;
%                         r_at_cpa = r_rel + v_rel * t_cpa_e;
%                         d_cpa_e = norm(r_at_cpa) - r_bound;
%                     end
% 
%                     % 未来 (-0.2s以降) に最接近し、かつ離隔が安全マージンを割り込むか
%                     if (t_cpa_e > -0.2) && (d_cpa_e < ent.r_safe)
%                         is_direct_threat = true;
%                         if t_cpa_e < min_t_cpa
%                             min_t_cpa = t_cpa_e;
%                             min_d_cpa = d_cpa_e;
%                             target_hit = ent.name;
%                         end
%                     end
%                 end
% 
%                 % --- (B) 将来復帰軌道での二次衝突判定 (Park & Cho, 2020; Woo et al., Sec 4.1) ---
%                 is_secondary_threat = false;
%                 t_future_samples = linspace(1.5, obj.t_horizon_eval, 10);
%                 min_future_corridor_dist = inf;
% 
%                 for tf = t_future_samples
%                     p_obs_f = p_o + v_o * tf;
%                     p_nom_f = pL + dir_nom * (spd_nom * tf);
% 
%                     dist_to_corridor = norm(p_nom_f - p_obs_f) - r_bound;
%                     if dist_to_corridor < min_future_corridor_dist
%                         min_future_corridor_dist = dist_to_corridor;
%                     end
% 
%                     % 荷物保護半径 + 障害物マージン + 余裕代(0.5m)
%                     if dist_to_corridor < (obj.r_load + d_m + 0.5)
%                         is_secondary_threat = true;
%                     end
%                 end
% 
%                 % --- (C) 脅威の登録（直近衝突 または 将来衝突のどちらかに該当すれば保持） ---
%                 if is_direct_threat || is_secondary_threat
%                     % 切迫度スコア (Woo et al., 2026, Algorithm 2)
%                     urgency_score = max(0.0, min_t_cpa) + 2.0 * max(0.0, min_d_cpa);
%                     if ~is_direct_threat && is_secondary_threat
%                         % 二次衝突障害物は一次衝突の直後に並べる
%                         urgency_score = urgency_score + 10.0;
%                     end
% 
%                     vec_from_obs = pL - p_o;
%                     if norm(vec_from_obs) > 1e-3
%                         n_away = vec_from_obs / norm(vec_from_obs);
%                     else
%                         n_away = obs.normal_load;
%                     end
%                     combined_escape_vec = combined_escape_vec + n_away * (1.0 / max(0.5, norm(vec_from_obs)));
% 
%                     req_clearance_i = r_bound + max(obj.r_drone, obj.r_load) + d_m;
% 
%                     item = struct();
%                     item.id                  = obs.id;
%                     item.urgency_score       = urgency_score;
%                     item.is_direct_threat    = is_direct_threat;
%                     item.is_secondary_threat = is_secondary_threat;
%                     item.t_cpa               = max(0.0, min_t_cpa);
%                     item.d_cpa               = min_d_cpa;
%                     item.threat_target       = target_hit;
%                     item.p_obs               = p_o;
%                     item.v_obs               = v_o;
%                     item.radii_obs           = obs.radii_obs;
%                     item.d_margin            = d_m;
%                     item.req_clearance       = req_clearance_i;
%                     item.n_away              = n_away;
% 
%                     all_threat_items = [all_threat_items; item];
%                 end
%             end
% 
%             % 3. 複数障害物の統合構造体を作成 (ソートで排除せず全件保持)
%             if ~isempty(all_threat_items)
%                 [~, sort_idx] = sort([all_threat_items.urgency_score], 'ascend');
%                 sorted_items = all_threat_items(sort_idx);
% 
%                 tg.has_threat         = true;
%                 tg.threat_count       = numel(sorted_items);
%                 tg.items              = sorted_items;     % 危険な障害物の完全配列
%                 tg.primary_id         = sorted_items(1).id;
%                 tg.primary_tcpa       = sorted_items(1).t_cpa;
%                 tg.primary_dcpa       = sorted_items(1).d_cpa;
% 
%                 sec_ids = [];
%                 for m = 2:numel(sorted_items)
%                     sec_ids = [sec_ids, sorted_items(m).id];
%                 end
%                 tg.secondary_ids      = sec_ids;
% 
%                 all_centers = reshape([sorted_items.p_obs], 3, []);
%                 tg.union_center       = mean(all_centers, 2);
%                 tg.req_clearance_max  = max([sorted_items.req_clearance]);
% 
%                 if norm(combined_escape_vec) > 1e-3
%                     tg.suggested_escape_dir = combined_escape_vec / norm(combined_escape_vec);
%                 else
%                     tg.suggested_escape_dir = sorted_items(1).n_away;
%                 end
%             end
%         end
%         % =====================================================================
%         % point_ellipsoid_signed_distance
%         % 任意の3次元点 p と回転楕円体 (中心 c, 半径 r=[a;b;c], 回転行列 R) の
%         % 【外郭表面】までの真の最短ユークリッド距離を算出する。
%         %
%         % [数学的アルゴリズム]:
%         % 楕円方程式: (x/a)^2 + (y/b)^2 + (z/c)^2 = 1
%         % 点 p から楕円面上の最近接点 x への最短距離は、ラグランジュ未定乗数法:
%         %   f(lambda) = sum( (a_i^2 * y_i^2) / (lambda + a_i^2)^2 ) - 1 = 0
%         % を満たす単一根 lambda を二分探索法 (Binary Search) で 80 回反復して求め、
%         % 表面点 x = (a_i^2 * y_i) / (lambda + a_i^2) と点 y の距離 ||x - y|| を計算する。
%         % =====================================================================
%         function [d, inside, closest_local, lambda] = point_ellipsoid_signed_distance(~, p, o)
%             p = p(:);
%             c = o.center(:);
%             r = o.radii(:);
%             R = o.R;
% 
%             % --- 入力チェック: 不正ならフォールバックせず即座に停止 ---
%             if numel(p) ~= 3 || numel(c) ~= 3 || numel(r) ~= 3
%                 error('点 p、中心 c、および半径 r はすべて3x1ベクトルである必要があります。');
%             end
%             if any(~isfinite(p)) || any(~isfinite(c)) || any(~isfinite(r))
%                 error('点または楕円体の定義に非有限値 (NaN や Inf) が検出されました。');
%             end
%             if any(r <= 0)
%                 error('楕円体の半径はすべて厳密に正の値 (r > 0) である必要があります。');
%             end
%             if ~isequal(size(R), [3 3]) || any(~isfinite(R(:)))
%                 error('回転行列 R は有限な3x3行列である必要があります。');
%             end
%             if norm(R'*R - eye(3), 'fro') > 1e-6 || abs(det(R) - 1.0) > 1e-6
%                 error('行列 R は有効な直交回転行列 SO(3) ではありません。');
%             end
% 
%             % 1. ワールド座標系の点 p を障害物の局所主軸座標系 (Local Frame) に変換
%             y  = R' * (p - c);
%             r2 = r.^2;
% 
%             % 2. 楕円代数判定値 q:
%             %    q < 1.0 -> 点は楕円体の内部にある (衝突・侵入状態)
%             %    q = 1.0 -> 点は楕円体の表面上にある
%             %    q > 1.0 -> 点は楕円体の外部にある
%             q      = sum((y ./ r).^2);
%             inside = (q < 1.0);
% 
%             % 特異点処理: 点が障害物の中心そのものにある場合 (最も近い表面は最短半径の主軸上)
%             if norm(y) < 1e-14
%                 [min_r, min_idx] = min(r);
%                 closest_local = zeros(3, 1);
%                 closest_local(min_idx) = min_r;
%                 d      = -min_r;
%                 lambda = -min(r2);
%                 return;
%             end
% 
%             % ラグランジュ未定乗数の非線形方程式 f(lambda) = 0
%             f = @(lam) sum(r2 .* (y.^2) ./ ((lam + r2).^2)) - 1.0;
% 
%             % 3. ラグランジュ乗数 lambda の探索範囲 (Bracketing) の設定と根の存在検証
%             if q > 1.0
%                 % 外部点の場合: lambda >= 0
%                 lo = 0.0;
%                 hi = max(r) * norm(y);
% 
%                 if f(lo) < 0 || f(hi) > 0
%                     error('外部点に対する楕円体距離の求根ブラケット設定に失敗しました (解を挟み込めていません)。');
%                 end
%             else
%                 % 内部点の場合: -min(r_i^2) < lambda < 0
%                 lo = -min(r2) * (1.0 - 1e-12);
%                 hi = 0.0;
% 
%                 if f(lo) < 0 || f(hi) > 0
%                     error('内部点に対する楕円体距離の求根ブラケット設定に失敗しました (解を挟み込めていません)。');
%                 end
%             end
% 
%             % 4. 二分探索により lambda を数値的に収束させる
%             %    80回反復して根を高精度に求める
%             for kk = 1:80
%                 mid = 0.5 * (lo + hi);
%                 if f(mid) > 0
%                     lo = mid;
%                 else
%                     hi = mid;
%                 end
%             end
%             lambda = 0.5 * (lo + hi);
% 
%             % 5. 局所座標系における楕円体表面上の最近接点 closest_local
%             closest_local = r2 .* y ./ (lambda + r2);
%             d_abs         = norm(closest_local - y);
% 
%             % 6. 表面までの最短ユークリッド距離
%             %    外部なら正値 (+), 内部なら侵入深さとして負値 (-) を返す
%             if inside
%                 d = -d_abs;
%             else
%                 d = d_abs;
%             end
%         end
%     end
% end

% classdef REPLANNING_BSPLINE < handle
%     % =========================================================================
%     % REPLANNING_BSPLINE
%     % 索結合系（UAV・荷物・索）の複数球モデルおよび動的不確実性包絡に基づく
%     % 動的障害物センシング＆脅威候補フィルタリング（Threat Candidate Filter）
%     %
%     % 【本クラスの役割と設計思想】:
%     %   ・本クラスは「最終的な衝突判定」や「回避軌道生成」は行わない。
%     %   ・15m LiDARセンサ視野内の障害物に対し、自機の制御遅れ・追従誤差・
%     %     ペイロード揺動マージンを包括した複数球モデル（Sphere-Chain）を用いて、
%     %     後段の軌道最適化モジュールへ渡すべき「Active Threat Candidates」を
%     %     保守的（見逃しゼロ）かつ高速に抽出・管理する。
%     %   ・障害物1個あたりK個の球体に対して各々解析解 O(1) でCPAを算出するため、
%     %     全体計算量は O(N_obs * K) となり、40 Hz制御ループ内で極めて軽量に完結する。
%     %
%     % 【幾何・不確実性モデル】:
%     %   1. 隙間なし複数球体表現 (Sphere-Chain):
%     %      機体球、荷物球、および索上の球体列 (Delta s <= sqrt(2)*r_cable)
%     %   2. 制御遅れ等方包絡 (Delay Envelope):
%     %      速度方向の遅れ変位 ||v|| * tau_delay を包含する最悪値バウンド
%     %   3. 追従誤差バウンド (Tracking Error Bound):
%     %      コントローラ位置偏差の上限値 epsilon_track
%     %   4. 揺動幾何包絡 (Swing Uncertainty Envelope):
%     %      索・荷物の動的振れ幅を包含する幾何マージン r_swing(s)
%     %
%     % [参考文献]:
%     %   Woo et al. (2026), Nonlinear Dynamics, 114, 613 (CPA Conflict Determination)
%     % =========================================================================
% 
%     properties
%         self                  % ドローンエージェント自身 (estimator, parameter 保持)
%         base_ref              % 公称参照軌道生成オブジェクト
%         result                % 出力結果構造体 (xd, pRef, vRef, yawRef, threat_candidates 等)
% 
%         % --- センサおよび実体幾何パラメータ ---
%         trigger_dist = 15.0;  % 機体搭載センサの検知閾値 [m]
%         r_load       = 0.15;  % 荷物実体保護半径 [m]
%         r_drone      = 0.30;  % 機体実体保護半径 [m]
%         r_cable      = 0.08;  % 索（ケーブル）実体等価半径 [m]
%         default_margin = 0.30;% 障害物に d_margin が未定義の場合のフォールバック値 [m]
% 
%         % --- 幾何モデル設定 (複数球モデルを本命として固定) ---
%         cable_model_mode = 3; % 1: 包絡楕円体, 2: 線分カプセル, 3: 隙間なし複数球モデル(本命)
%         min_cable_pts    = 5; % 索上の最小球体配置数
% 
%         % --- 自機系の動的・制御不確実性マージン設定 ---
%         tau_delay_drone  = 0.05; % 機体制御応答遅延時間 [s] (40Hz制御の2ステップ分)
%         tau_delay_load   = 0.10; % 荷物等価応答遅延時間 [s]
%         eps_track_drone  = 0.15; % 機体位置追従誤差の上限バウンド [m]
%         eps_track_load   = 0.20; % 荷物位置追従誤差の上限バウンド [m]
%         r_swing_max      = 0.35; % 荷物・索の最大想定揺動幅マージン [m]
% 
%         % --- CPA 候補抽出フィルタ設定 ---
%         t_preview_end    = 6.0;  % 脅威評価ホライズン [s]
%         d_gate_margin    = 2.0;  % CPA 一次スクリーニング用予備ゲート幅 [m]
%     end
% 
%     methods (Access = public)
%         % =====================================================================
%         % コンストラクタ: クラス初期化とパラメータ反映
%         % =====================================================================
%         function obj = REPLANNING_BSPLINE(self, base_ref, opts)
%             arguments
%                 self
%                 base_ref
%                 opts = struct()
%             end
%             obj.self     = self;
%             obj.base_ref = base_ref;
% 
%             if isfield(opts, 'trigger_dist'),      obj.trigger_dist      = opts.trigger_dist;      end
%             if isfield(opts, 'r_load'),            obj.r_load            = opts.r_load;            end
%             if isfield(opts, 'r_drone'),           obj.r_drone           = opts.r_drone;           end
%             if isfield(opts, 'r_cable'),           obj.r_cable           = opts.r_cable;           end
%             if isfield(opts, 'default_margin'),    obj.default_margin    = opts.default_margin;    end
%             if isfield(opts, 'cable_model_mode'),  obj.cable_model_mode  = opts.cable_model_mode;  end
%             if isfield(opts, 'min_cable_pts'),     obj.min_cable_pts     = opts.min_cable_pts;     end
%             if isfield(opts, 'tau_delay_drone'),   obj.tau_delay_drone   = opts.tau_delay_drone;   end
%             if isfield(opts, 'tau_delay_load'),    obj.tau_delay_load    = opts.tau_delay_load;    end
%             if isfield(opts, 'eps_track_drone'),   obj.eps_track_drone   = opts.eps_track_drone;   end
%             if isfield(opts, 'eps_track_load'),    obj.eps_track_load    = opts.eps_track_load;    end
%             if isfield(opts, 'r_swing_max'),       obj.r_swing_max       = opts.r_swing_max;       end
%             if isfield(opts, 't_preview_end'),     obj.t_preview_end     = opts.t_preview_end;     end
%             if isfield(opts, 'd_gate_margin'),     obj.d_gate_margin     = opts.d_gate_margin;     end
% 
%             obj.result = base_ref.result;
% 
%             % 外部ロガー用プロパティの動的登録 (dynamicprops)
%             sensor_props = [ ...
%                 "time", "pQ", "pL", ...
%                 "detected_point", ...
%                 "drone_inside_obstacle_point", "load_inside_obstacle_point", ...
%                 "drone_min_dist_point", "load_min_dist_point", "min_dist_point", ...
%                 "drone_obstacle_id_point", "load_obstacle_id_point", "min_obstacle_id_point", ...
%                 "min_source_point", ...
%                 "detected_obstacle_count_point", ...
%                 "threat_candidate_count", ...
%                 "primary_threat_id", ...
%                 "primary_threat_tcpa", ...
%                 "primary_threat_dcpa" ...
%             ];
% 
%             for p_name = sensor_props
%                 if ~isprop(obj.result.state, p_name)
%                     addprop(obj.result.state, p_name);
%                 end
%             end
% 
%             obj.clear_state_sensor_values(0.0);
%         end
% 
%         % =====================================================================
%         % do: 制御周期ごと (25ms / 40Hz) の実行メソッド
%         % =====================================================================
%         function result_out = do(obj, varargin)
%             time = varargin{1};
%             cha  = varargin{2};
% 
%             % 1. 公称目標軌道の取得 (公称値を維持)
%             base_res = obj.base_ref.do(varargin{:});
%             xd_nom = base_res.state.xd;
%             obj.clear_state_sensor_values(time.t);
%             obj.result.state.xd = xd_nom;
% 
%             % センサ検知初期構造体
%             detection = struct();
%             detection.time                          = time.t;
%             detection.pQ                            = [NaN; NaN; NaN];
%             detection.pL                            = [NaN; NaN; NaN];
%             detection.detected_point                = false;
%             detection.drone_inside_obstacle_point   = false;
%             detection.load_inside_obstacle_point    = false;
%             detection.drone_min_dist_point          = inf;
%             detection.load_min_dist_point           = inf;
%             detection.min_dist_point                = inf;
%             detection.drone_obstacle_id_point       = NaN;
%             detection.load_obstacle_id_point        = NaN;
%             detection.min_obstacle_id_point         = NaN;
%             detection.min_source_point              = "none";
%             detection.detected_obstacles_point      = [];
%             detection.detected_obstacle_count_point = 0;
% 
%             % 抽出された脅威候補構造体 (Active Threat Set)
%             threat_candidates = struct();
%             threat_candidates.count                 = 0;
%             threat_candidates.items                 = [];
%             threat_candidates.primary_id            = NaN;
%             threat_candidates.primary_tcpa          = inf;
%             threat_candidates.primary_dcpa          = inf;
%             threat_candidates.threat_centroid       = [0;0;0];
% 
%             % 2. 飛行フェーズ ('f') における検知と脅威候補フィルタリング
%             if cha == 'f'
%                 pL_cur = obj.self.estimator.result.state.pL(:);
%                 pQ_cur = obj.self.estimator.result.state.p(:);
%                 vL_cur = obj.self.estimator.result.state.vL(:);
%                 vQ_cur = obj.self.estimator.result.state.v(:);
% 
%                 % (A) 現在時刻の動的障害物配置を取得 (固有マージン含む)
%                 obs_list = obj.get_obstacles_at_time(time.t);
% 
%                 % (B) 機体搭載センサによる 15m 幾何表面距離スキャン (Phase 0)
%                 detection = obj.check_drone_sensor_15m(pQ_cur, pL_cur, obs_list, time.t);
% 
%                 % (C) 複数球不確実性包絡モデルによる CPA 脅威候補フィルタリング
%                 threat_candidates = obj.filter_threat_candidates_sphere_chain( ...
%                     pQ_cur, vQ_cur, pL_cur, vL_cur, detection.detected_obstacles_point);
% 
%                 % (D) コンソール診断ログ出力
%                 if threat_candidates.count > 0
%                     fprintf("[THREAT FILTER | 40Hz] t=%.3f s | 15m視野内: %d 個 | 抽出候補数: %d 個\n", ...
%                         time.t, detection.detected_obstacle_count_point, threat_candidates.count);
%                     primary = threat_candidates.items(1);
%                     fprintf("  -> [最優先候補 ID:%d] 危険部位: %s | 最接近 CPA: %.2f s後 | 予測クリアランス: %.2f m\n", ...
%                         primary.id, primary.critical_entity, primary.t_cpa, primary.d_cpa);
%                 end
%             end
% 
%             % 3. 状態の基本反映
%             st = obj.result.state;
%             st.xd                            = xd_nom;
%             st.p                             = xd_nom(1:3);
%             st.v                             = xd_nom(5:7);
%             st.q                             = [0; 0; xd_nom(4)];
%             st.pRef                          = xd_nom(1:3);
%             st.vRef                          = xd_nom(5:7);
%             st.yawRef                        = xd_nom(4);
% 
%             st.time                          = detection.time;
%             st.pQ                            = detection.pQ;
%             st.pL                            = detection.pL;
%             st.detected_point                = detection.detected_point;
%             st.drone_inside_obstacle_point   = detection.drone_inside_obstacle_point;
%             st.load_inside_obstacle_point    = detection.load_inside_obstacle_point;
%             st.drone_min_dist_point          = detection.drone_min_dist_point;
%             st.load_min_dist_point           = detection.load_min_dist_point;
%             st.min_dist_point                = detection.min_dist_point;
%             st.drone_obstacle_id_point       = detection.drone_obstacle_id_point;
%             st.load_obstacle_id_point        = detection.load_obstacle_id_point;
%             st.min_obstacle_id_point         = detection.min_obstacle_id_point;
%             st.min_source_point              = detection.min_source_point;
%             st.detected_obstacle_count_point = detection.detected_obstacle_count_point;
% 
%             st.threat_candidate_count        = threat_candidates.count;
%             st.primary_threat_id             = threat_candidates.primary_id;
%             st.primary_threat_tcpa           = threat_candidates.primary_tcpa;
%             st.primary_threat_dcpa           = threat_candidates.primary_dcpa;
% 
%             obj.result.state                 = st;
%             obj.result.obstacle_detection    = detection;
%             obj.result.threat_candidates     = threat_candidates; % 後段の軌道最適化器へ渡す候補セット
% 
%             result_out = obj.result;
%         end
%     end
% 
%     methods (Access = private)
%         % =====================================================================
%         % clear_state_sensor_values: プロパティの初期化
%         % =====================================================================
%         function clear_state_sensor_values(obj, t_now)
%             st = obj.result.state;
%             st.time                          = t_now;
%             st.pQ                            = [NaN; NaN; NaN];
%             st.pL                            = [NaN; NaN; NaN];
%             st.detected_point                = false;
%             st.drone_inside_obstacle_point   = false;
%             st.load_inside_obstacle_point    = false;
%             st.drone_min_dist_point          = inf;
%             st.load_min_dist_point           = inf;
%             st.min_dist_point                = inf;
%             st.drone_obstacle_id_point       = NaN;
%             st.load_obstacle_id_point        = NaN;
%             st.min_obstacle_id_point         = NaN;
%             st.min_source_point              = "none";
%             st.detected_obstacle_count_point = 0;
%             st.threat_candidate_count        = 0;
%             st.primary_threat_id             = NaN;
%             st.primary_threat_tcpa           = inf;
%             st.primary_threat_dcpa           = inf;
% 
%             obj.result.state = st;
%         end
% 
%         % =====================================================================
%         % get_obstacles_at_time
%         % =====================================================================
%         function list = get_obstacles_at_time(~, t_now)
%             list = ENVIRONMENT_OBSTACLE_ELLIPSE_MOVE(t_now);
%         end
% 
%         % =====================================================================
%         % check_drone_sensor_15m: 機体搭載センサ基準の 15m 表面距離スキャン (Phase 0)
%         % =====================================================================
%         function det = check_drone_sensor_15m(obj, pQ, pL, obs_list, t_now)
%             det = struct();
%             det.time                          = t_now;
%             det.pQ                            = pQ;
%             det.pL                            = pL;
%             det.trigger_dist                  = obj.trigger_dist;
%             det.detected_point                = false;
%             det.drone_inside_obstacle_point   = false;
%             det.load_inside_obstacle_point    = false;
%             det.drone_min_dist_point          = inf;
%             det.load_min_dist_point           = inf;
%             det.min_dist_point                = inf;
%             det.drone_obstacle_id_point       = NaN;
%             det.load_obstacle_id_point        = NaN;
%             det.min_obstacle_id_point         = NaN;
%             det.min_source_point              = "none";
%             det.detected_obstacles_point      = [];
%             det.detected_obstacle_count_point = 0;
% 
%             if isempty(obs_list), return; end
%             detected_obs = [];
% 
%             for i = 1:length(obs_list)
%                 o = obs_list(i);
%                 obs_id = i;
%                 if isfield(o, 'id') && ~isempty(o.id), obs_id = o.id; end
% 
%                 R_obs     = o.R_obs;
%                 radii_obs = o.ellipsoid_radii(:);
%                 p_obs     = o.p_center(:);
% 
%                 v_obs = [0; 0; 0];
%                 if isfield(o, 'v_center') && ~isempty(o.v_center),     v_obs = o.v_center(:);
%                 elseif isfield(o, 'v') && ~isempty(o.v),               v_obs = o.v(:); end
% 
%                 d_margin_obs = obj.default_margin;
%                 if isfield(o, 'd_margin') && ~isempty(o.d_margin),     d_margin_obs = o.d_margin; end
% 
%                 obs_parsed = struct('center', p_obs, 'radii', radii_obs, 'R', R_obs);
% 
%                 [d_drone, in_drone, cpQ_loc, ~] = obj.point_ellipsoid_signed_distance(pQ, obs_parsed);
%                 [d_load, in_load, cpL_loc, ~]   = obj.point_ellipsoid_signed_distance(pL, obs_parsed);
% 
%                 if in_drone, det.drone_inside_obstacle_point = true; end
%                 if in_load,  det.load_inside_obstacle_point  = true; end
% 
%                 if d_drone < det.drone_min_dist_point
%                     det.drone_min_dist_point = d_drone;
%                     det.drone_obstacle_id_point = obs_id;
%                 end
%                 if d_load < det.load_min_dist_point
%                     det.load_min_dist_point = d_load;
%                     det.load_obstacle_id_point = obs_id;
%                 end
% 
%                 obs_min = min(d_drone, d_load);
% 
%                 nQ = cpQ_loc ./ (radii_obs.^2);
%                 nL = cpL_loc ./ (radii_obs.^2);
%                 if norm(nQ) < 1e-12, normal_drone = [0; 0; 0]; else, normal_drone = R_obs * (nQ / norm(nQ)); end
%                 if norm(nL) < 1e-12, normal_load = [0; 0; 0];  else, normal_load = R_obs * (nL / norm(nL)); end
% 
%                 obs_item = struct( ...
%                     'id',               obs_id, ...
%                     'dist_drone_point', d_drone, ...
%                     'dist_load_point',  d_load, ...
%                     'min_dist_point',   obs_min, ...
%                     'p_obs',            p_obs, ...
%                     'v_obs',            v_obs, ...
%                     'radii_obs',        radii_obs, ...
%                     'R_obs',            R_obs, ...
%                     'd_margin',         d_margin_obs, ...
%                     'normal_drone',     normal_drone, ...
%                     'normal_load',      normal_load ...
%                 );
% 
%                 if d_drone <= obj.trigger_dist
%                     det.detected_point = true;
%                     detected_obs = [detected_obs; obs_item];
%                 end
%             end
% 
%             if det.drone_min_dist_point <= det.load_min_dist_point
%                 det.min_dist_point   = det.drone_min_dist_point;
%                 det.min_obstacle_id_point = det.drone_obstacle_id_point;
%                 det.min_source_point = "drone";
%             else
%                 det.min_dist_point   = det.load_min_dist_point;
%                 det.min_obstacle_id_point = det.load_obstacle_id_point;
%                 det.min_source_point = "load";
%             end
% 
%             det.detected_obstacles_point      = detected_obs;
%             det.detected_obstacle_count_point = numel(detected_obs);
%         end
% 
%         % =====================================================================
%         % build_sphere_chain_model
%         % 隙間なし幾何被覆条件を満たす自機複数球モデルを構築
%         % 制御遅れ等方上界・位置追従誤差・揺動包絡マージンを各球体の有効保護半径に加算
%         % =====================================================================
%         function sphere_chain = build_sphere_chain_model(obj, pQ, vQ, pL, vL, d_margin_obs)
%             vec_cable = pQ - pL;
%             L_cable = norm(vec_cable);
% 
%             % 幾何学的に隙間が原理上ゼロになる球配置間隔 (Delta s <= sqrt(2)*r_cable)
%             ds_geom_safe = sqrt(2.0) * obj.r_cable;
%             K_balls = max(obj.min_cable_pts, ceil(L_cable / ds_geom_safe) + 1);
%             s_samples = linspace(0.0, 1.0, K_balls);
% 
%             % 制御遅延に伴う最悪変位の等方的一様バウンド (||v|| * tau_delay)
%             spd_Q = norm(vQ);
%             spd_L = norm(vL);
%             d_delay_bound_Q = spd_Q * obj.tau_delay_drone;
%             d_delay_bound_L = spd_L * obj.tau_delay_load;
% 
%             sphere_chain = cell(1, K_balls);
% 
%             for b = 1:K_balls
%                 s_b = s_samples(b);
%                 p_b = (1.0 - s_b) * pL + s_b * pQ;
%                 v_b = (1.0 - s_b) * vL + s_b * vQ;
% 
%                 if b == 1
%                     % 荷物端点 (s=0): 制御遅延上界 + 追従誤差 + 最大揺動不確実性マージン
%                     b_name = "load_sphere";
%                     r_effective = obj.r_load + d_delay_bound_L + obj.eps_track_load + obj.r_swing_max + d_margin_obs;
%                 elseif b == K_balls
%                     % 機体端点 (s=1): 制御遅延上界 + 追従誤差マージン
%                     b_name = "drone_sphere";
%                     r_effective = obj.r_drone + d_delay_bound_Q + obj.eps_track_drone + d_margin_obs;
%                 else
%                     % 索中間球: 索上の位置に応じた揺動包絡マージン (sin形状上界)
%                     b_name = sprintf("cable_sphere_%d", b);
%                     swing_local = obj.r_swing_max * sin(pi * s_b);
%                     % 中間球の等価遅れバウンド (線形補間)
%                     d_delay_mid = (1.0 - s_b) * d_delay_bound_L + s_b * d_delay_bound_Q;
%                     eps_track_mid = (1.0 - s_b) * obj.eps_track_load + s_b * obj.eps_track_drone;
%                     r_effective = obj.r_cable + d_delay_mid + eps_track_mid + swing_local + d_margin_obs;
%                 end
% 
%                 sphere_chain{b} = struct( ...
%                     'name',        b_name, ...
%                     'p',           p_b, ...
%                     'v',           v_b, ...
%                     'r_effective', r_effective ...
%                 );
%             end
%         end
% 
%         % =====================================================================
%         % filter_threat_candidates_sphere_chain
%         % Woo et al. (2026) の相対運動 CPA 概念に基づく脅威候補フィルタリング
%         % (障害物1個あたりK個の球体について各々解析解 O(1) でCPAを算出: 全体 O(N_obs * K))
%         % =====================================================================
%         function tc = filter_threat_candidates_sphere_chain(obj, pQ, vQ, pL, vL, detected_obs)
%             tc = struct();
%             tc.count           = 0;
%             tc.items           = [];
%             tc.primary_id      = NaN;
%             tc.primary_tcpa    = inf;
%             tc.primary_dcpa    = inf;
%             tc.threat_centroid = [0;0;0];
% 
%             if isempty(detected_obs), return; end
% 
%             candidates = [];
% 
%             for k = 1:numel(detected_obs)
%                 obs = detected_obs(k);
%                 p_o = obs.p_obs;
%                 v_o = obs.v_obs;
%                 d_m = obs.d_margin;
%                 r_bound_obs = max(obs.radii_obs); % 障害物外接球半径 (保守的スクリーニング用)
% 
%                 % 自機系の隙間なし複数球モデルを生成 (不確実性包絡マージン付加済み)
%                 chain = obj.build_sphere_chain_model(pQ, vQ, pL, vL, d_m);
% 
%                 min_d_cpa = inf;
%                 crit_t_cpa = inf;
%                 critical_entity = "none";
%                 threat_detected = false;
% 
%                 % 各球体に対して相対運動 CPA を評価 (各球 O(1) 閉形式[cite: 1])
%                 for b = 1:numel(chain)
%                     sph = chain{b};
%                     r_rel = sph.p - p_o;
%                     v_rel = sph.v - v_o;
%                     v_rel_sq = dot(v_rel, v_rel);
% 
%                     % 相対運動に基づく最接近時間 t_cpa の閉形式算出 (Woo et al., 式 9)[cite: 1]
%                     if v_rel_sq < 1e-4
%                         t_cpa_b = 0.0;
%                         center_dist_cpa = norm(r_rel);
%                     else
%                         t_cpa_b = -dot(r_rel, v_rel) / v_rel_sq;
%                         center_dist_cpa = norm(r_rel + v_rel * t_cpa_b);
%                     end
% 
%                     % 有効保護半径を考慮した最接近予測クリアランス
%                     d_cpa_b = center_dist_cpa - (r_bound_obs + sph.r_effective);
% 
%                     % フィルタ通過条件: 将来 (0〜t_preview) に最接近し、かつ予備ゲート以内
%                     if (t_cpa_b >= 0.0) && (t_cpa_b <= obj.t_preview_end) && (d_cpa_b <= obj.d_gate_margin)
%                         threat_detected = true;
%                         if d_cpa_b < min_d_cpa
%                             min_d_cpa = d_cpa_b;
%                             crit_t_cpa = t_cpa_b;
%                             critical_entity = sph.name;
%                         end
%                     end
%                 end
% 
%                 % ★ 現在時刻 (t=0) における全球体チェーン侵入救済判定 (UAVだけでなく索・荷物も包含)
%                 curr_dist_min = inf;
%                 curr_entity = "none";
%                 for b = 1:numel(chain)
%                     sph = chain{b};
%                     d_now = norm(sph.p - p_o) - (r_bound_obs + sph.r_effective);
%                     if d_now < curr_dist_min
%                         curr_dist_min = d_now;
%                         curr_entity = sph.name;
%                     end
%                 end
% 
%                 if curr_dist_min <= 0.0
%                     threat_detected = true;
%                     crit_t_cpa = 0.0;
%                     min_d_cpa = curr_dist_min;
%                     critical_entity = curr_entity;
%                 end
% 
%                 % フィルタを通過した障害物を後段用の候補リストへ登録
%                 if threat_detected
%                     item = struct();
%                     item.id               = obs.id;
%                     item.critical_entity  = critical_entity;   % 最も接近する部位
%                     item.t_cpa            = max(0.0, crit_t_cpa); % 最接近時刻 [s]
%                     item.d_cpa            = min_d_cpa;         % 予測最小離隔 [m] (不確実性包絡込み)
%                     item.p_obs            = p_o;               % 障害物中心位置
%                     item.v_obs            = v_o;               % 障害物速度
%                     item.radii_obs        = obs.radii_obs;     % 障害物3軸半径
%                     item.R_obs            = obs.R_obs;         % 障害物姿勢行列
%                     item.d_margin         = d_m;               % 適用安全マージン
% 
%                     candidates = [candidates; item]; %#ok<AGROW>
%                 end
%             end
% 
%             % 候補障害物群の優先度ソート (最接近時間 t_cpa 昇順 -> 予測離隔 d_cpa 昇順)
%             if ~isempty(candidates)
%                 sort_matrix = [[candidates.t_cpa]', [candidates.d_cpa]'];
%                 [~, sort_idx] = sortrows(sort_matrix, [1, 2]);
%                 sorted_candidates = candidates(sort_idx);
% 
%                 tc.items           = sorted_candidates;
%                 tc.count           = numel(sorted_candidates);
%                 tc.primary_id      = sorted_candidates(1).id;
%                 tc.primary_tcpa    = sorted_candidates(1).t_cpa;
%                 tc.primary_dcpa    = sorted_candidates(1).d_cpa;
% 
%                 all_centers = reshape([sorted_candidates.p_obs], 3, []);
%                 tc.threat_centroid = mean(all_centers, 2);
%             end
%         end
% 
%         % =====================================================================
%         % point_ellipsoid_signed_distance: ラグランジュ未定乗数・二分探索
%         % =====================================================================
%         function [d, inside, closest_local, lambda] = point_ellipsoid_signed_distance(~, p, o)
%             p = p(:); c = o.center(:); r = o.radii(:); R = o.R;
%             y = R' * (p - c);
%             r2 = r.^2;
%             q = sum((y ./ r).^2);
%             inside = (q < 1.0);
% 
%             if norm(y) < 1e-14
%                 [min_r, min_idx] = min(r);
%                 closest_local = zeros(3, 1); closest_local(min_idx) = min_r;
%                 d = -min_r; lambda = -min(r2);
%                 return;
%             end
% 
%             f = @(lam) sum(r2 .* (y.^2) ./ ((lam + r2).^2)) - 1.0;
%             if q > 1.0
%                 lo = 0.0; hi = max(r) * norm(y);
%             else
%                 lo = -min(r2) * (1.0 - 1e-12); hi = 0.0;
%             end
% 
%             for kk = 1:80
%                 mid = 0.5 * (lo + hi);
%                 if f(mid) > 0, lo = mid; else, hi = mid; end
%             end
%             lambda = 0.5 * (lo + hi);
%             closest_local = r2 .* y ./ (lambda + r2);
%             d_abs = norm(closest_local - y);
%             if inside, d = -d_abs; else, d = d_abs; end
%         end
%     end
% end

% classdef REPLANNING_BSPLINE < handle
%     % =========================================================================
%     % REPLANNING_BSPLINE
%     % 索結合系（UAV・荷物・索）の複数球モデルおよび動的不確実性包絡に基づく
%     % 動的障害物センシング＆脅威候補フィルタリング（Threat Candidate Filter）
%     %
%     % 【本クラスの役割と設計思想】:
%     %   ・本クラスは「最終的な衝突判定」や「回避軌道生成」は行わない。
%     %   ・15m LiDARセンサ視野内の障害物に対し、自機の制御遅れ・追従誤差・
%     %     ペイロード揺動マージンを包括した複数球モデル（Sphere-Chain）を用いて、
%     %     後段の軌道最適化モジュールへ渡すべき「Active Threat Candidates」を
%     %     保守的（見逃しゼロ）かつ高速に抽出・管理する。
%     %   ・障害物1個あたりK個の球体に対して各々解析解 O(1) でCPAを算出するため、
%     %     全体計算量は O(N_obs * K) となり、40 Hz制御ループ内で極めて軽量に完結する。
%     %
%     % 【幾何・不確実性モデル】:
%     %   1. 隙間なし複数球体表現 (Sphere-Chain):
%     %      機体球、荷物球、および索上の球体列 (Delta s <= sqrt(2)*r_cable)
%     %   2. 制御遅れ等方包絡 (Delay Envelope):
%     %      速度方向の遅れ変位 ||v|| * tau_delay を包含する最悪値バウンド
%     %   3. 追従誤差バウンド (Tracking Error Bound):
%     %      コントローラ位置偏差の上限値 epsilon_track
%     %   4. 揺動幾何包絡 (Swing Uncertainty Envelope):
%     %      索・荷物の動的振れ幅を包含する幾何マージン r_swing(s)
%     %
%     % [参考文献]:
%     %   Woo et al. (2026), Nonlinear Dynamics, 114, 613 (CPA Conflict Determination)
%     % =========================================================================
% 
%     properties
%         self                  % ドローンエージェント自身 (estimator, parameter 保持)
%         base_ref              % 公称参照軌道生成オブジェクト
%         result                % 出力結果構造体 (xd, pRef, vRef, yawRef, threat_candidates 等)
% 
%         % --- センサおよび実体幾何パラメータ ---
%         trigger_dist = 15.0;  % 機体搭載センサの検知閾値 [m]
%         r_load       = 0.15;  % 荷物実体保護半径 [m]
%         r_drone      = 0.30;  % 機体実体保護半径 [m]
%         r_cable      = 0.08;  % 索（ケーブル）実体等価半径 [m]
%         default_margin = 0.30;% 障害物に d_margin が未定義の場合のフォールバック値 [m]
% 
%         % --- 幾何モデル設定 (複数球モデルを本命として固定) ---
%         cable_model_mode = 3; % 1: 包絡楕円体, 2: 線分カプセル, 3: 隙間なし複数球モデル(本命)
%         min_cable_pts    = 5; % 索上の最小球体配置数
% 
%         % --- 自機系の動的・制御不確実性マージン設定 ---
%         tau_delay_drone  = 0.05; % 機体制御応答遅延時間 [s] (40Hz制御の2ステップ分)
%         tau_delay_load   = 0.10; % 荷物等価応答遅延時間 [s]
%         eps_track_drone  = 0.15; % 機体位置追従誤差の上限バウンド [m]
%         eps_track_load   = 0.20; % 荷物位置追従誤差の上限バウンド [m]
%         r_swing_max      = 0.35; % 荷物・索の最大想定揺動幅マージン [m]
% 
%         % --- CPA 候補抽出フィルタ設定 ---
%         t_preview_end    = 6.0;  % 脅威評価ホライズン [s]
%         d_gate_margin    = 2.0;  % CPA 一次スクリーニング用予備ゲート幅 [m]
%     end
% 
%     methods (Access = public)
%         % =====================================================================
%         % コンストラクタ: クラス初期化とパラメータ反映
%         % =====================================================================
%         function obj = REPLANNING_BSPLINE(self, base_ref, opts)
%             arguments
%                 self
%                 base_ref
%                 opts = struct()
%             end
%             obj.self     = self;
%             obj.base_ref = base_ref;
% 
%             if isfield(opts, 'trigger_dist'),      obj.trigger_dist      = opts.trigger_dist;      end
%             if isfield(opts, 'r_load'),            obj.r_load            = opts.r_load;            end
%             if isfield(opts, 'r_drone'),           obj.r_drone           = opts.r_drone;           end
%             if isfield(opts, 'r_cable'),           obj.r_cable           = opts.r_cable;           end
%             if isfield(opts, 'default_margin'),    obj.default_margin    = opts.default_margin;    end
%             if isfield(opts, 'cable_model_mode'),  obj.cable_model_mode  = opts.cable_model_mode;  end
%             if isfield(opts, 'min_cable_pts'),     obj.min_cable_pts     = opts.min_cable_pts;     end
%             if isfield(opts, 'tau_delay_drone'),   obj.tau_delay_drone   = opts.tau_delay_drone;   end
%             if isfield(opts, 'tau_delay_load'),    obj.tau_delay_load    = opts.tau_delay_load;    end
%             if isfield(opts, 'eps_track_drone'),   obj.eps_track_drone   = opts.eps_track_drone;   end
%             if isfield(opts, 'eps_track_load'),    obj.eps_track_load    = opts.eps_track_load;    end
%             if isfield(opts, 'r_swing_max'),       obj.r_swing_max       = opts.r_swing_max;       end
%             if isfield(opts, 't_preview_end'),     obj.t_preview_end     = opts.t_preview_end;     end
%             if isfield(opts, 'd_gate_margin'),     obj.d_gate_margin     = opts.d_gate_margin;     end
% 
%             obj.result = base_ref.result;
% 
%             % 外部ロガー用プロパティの動的登録 (dynamicprops)
%             sensor_props = [ ...
%                 "time", "pQ", "pL", ...
%                 "detected_point", ...
%                 "drone_inside_obstacle_point", "load_inside_obstacle_point", ...
%                 "drone_min_dist_point", "load_min_dist_point", "min_dist_point", ...
%                 "drone_obstacle_id_point", "load_obstacle_id_point", "min_obstacle_id_point", ...
%                 "min_source_point", ...
%                 "detected_obstacle_count_point", ...
%                 "threat_candidate_count", ...
%                 "primary_threat_id", ...
%                 "primary_threat_tcpa", ...
%                 "primary_threat_dcpa" ...
%             ];
% 
%             for p_name = sensor_props
%                 if ~isprop(obj.result.state, p_name)
%                     addprop(obj.result.state, p_name);
%                 end
%             end
% 
%             obj.clear_state_sensor_values(0.0);
%         end
% 
%         % =====================================================================
%         % do: 制御周期ごと (25ms / 40Hz) の実行メソッド
%         % =====================================================================
%         function result_out = do(obj, varargin)
%             time = varargin{1};
%             cha  = varargin{2};
% 
%             % 1. 公称目標軌道の取得 (公称値を維持)
%             base_res = obj.base_ref.do(varargin{:});
%             xd_nom = base_res.state.xd;
%             obj.clear_state_sensor_values(time.t);
%             obj.result.state.xd = xd_nom;
% 
%             % センサ検知初期構造体
%             detection = struct();
%             detection.time                          = time.t;
%             detection.pQ                            = [NaN; NaN; NaN];
%             detection.pL                            = [NaN; NaN; NaN];
%             detection.detected_point                = false;
%             detection.drone_inside_obstacle_point   = false;
%             detection.load_inside_obstacle_point    = false;
%             detection.drone_min_dist_point          = inf;
%             detection.load_min_dist_point           = inf;
%             detection.min_dist_point                = inf;
%             detection.drone_obstacle_id_point       = NaN;
%             detection.load_obstacle_id_point        = NaN;
%             detection.min_obstacle_id_point         = NaN;
%             detection.min_source_point              = "none";
%             detection.detected_obstacles_point      = [];
%             detection.detected_obstacle_count_point = 0;
% 
%             % 抽出された脅威候補構造体 (Active Threat Set)
%             threat_candidates = struct();
%             threat_candidates.count                 = 0;
%             threat_candidates.items                 = [];
%             threat_candidates.primary_id            = NaN;
%             threat_candidates.primary_tcpa          = inf;
%             threat_candidates.primary_dcpa          = inf;
%             threat_candidates.threat_centroid       = [0;0;0];
% 
%             % 2. 飛行フェーズ ('f') における検知と脅威候補フィルタリング
%             if cha == 'f'
%                 pL_cur = obj.self.estimator.result.state.pL(:);
%                 pQ_cur = obj.self.estimator.result.state.p(:);
%                 vL_cur = obj.self.estimator.result.state.vL(:);
%                 vQ_cur = obj.self.estimator.result.state.v(:);
% 
%                 % (A) 現在時刻の動的障害物配置を取得 (固有マージン含む)
%                 obs_list = obj.get_obstacles_at_time(time.t);
% 
%                 % (B) 機体搭載センサによる 15m 幾何表面距離スキャン (Phase 0)
%                 detection = obj.check_drone_sensor_15m(pQ_cur, pL_cur, obs_list, time.t);
% 
%                 % (C) 複数球不確実性包絡モデルによる CPA 脅威候補フィルタリング
%                 threat_candidates = obj.filter_threat_candidates_sphere_chain( ...
%                     pQ_cur, vQ_cur, pL_cur, vL_cur, detection.detected_obstacles_point);
% 
%                 % (D) コンソール診断ログ出力
%                 if threat_candidates.count > 0
%                     fprintf("[THREAT FILTER | 40Hz] t=%.3f s | 15m視野内: %d 個 | 抽出候補数: %d 個\n", ...
%                         time.t, detection.detected_obstacle_count_point, threat_candidates.count);
%                     primary = threat_candidates.items(1);
%                     fprintf("  -> [最優先候補 ID:%d] 危険部位: %s | 最接近 CPA: %.2f s後 | 予測クリアランス: %.2f m\n", ...
%                         primary.id, primary.critical_entity, primary.t_cpa, primary.d_cpa);
%                 end
%             end
% 
%             % 3. 状態の基本反映
%             st = obj.result.state;
%             st.xd                            = xd_nom;
%             st.p                             = xd_nom(1:3);
%             st.v                             = xd_nom(5:7);
%             st.q                             = [0; 0; xd_nom(4)];
%             st.pRef                          = xd_nom(1:3);
%             st.vRef                          = xd_nom(5:7);
%             st.yawRef                        = xd_nom(4);
% 
%             st.time                          = detection.time;
%             st.pQ                            = detection.pQ;
%             st.pL                            = detection.pL;
%             st.detected_point                = detection.detected_point;
%             st.drone_inside_obstacle_point   = detection.drone_inside_obstacle_point;
%             st.load_inside_obstacle_point    = detection.load_inside_obstacle_point;
%             st.drone_min_dist_point          = detection.drone_min_dist_point;
%             st.load_min_dist_point           = detection.load_min_dist_point;
%             st.min_dist_point                = detection.min_dist_point;
%             st.drone_obstacle_id_point       = detection.drone_obstacle_id_point;
%             st.load_obstacle_id_point        = detection.load_obstacle_id_point;
%             st.min_obstacle_id_point         = detection.min_obstacle_id_point;
%             st.min_source_point              = detection.min_source_point;
%             st.detected_obstacle_count_point = detection.detected_obstacle_count_point;
% 
%             st.threat_candidate_count        = threat_candidates.count;
%             st.primary_threat_id             = threat_candidates.primary_id;
%             st.primary_threat_tcpa           = threat_candidates.primary_tcpa;
%             st.primary_threat_dcpa           = threat_candidates.primary_dcpa;
% 
%             obj.result.state                 = st;
%             obj.result.obstacle_detection    = detection;
%             obj.result.threat_candidates     = threat_candidates; % 後段の軌道最適化器へ渡す候補セット
% 
%             result_out = obj.result;
%         end
%     end
% 
%     methods (Access = private)
%         % =====================================================================
%         % clear_state_sensor_values: プロパティの初期化
%         % =====================================================================
%         function clear_state_sensor_values(obj, t_now)
%             st = obj.result.state;
%             st.time                          = t_now;
%             st.pQ                            = [NaN; NaN; NaN];
%             st.pL                            = [NaN; NaN; NaN];
%             st.detected_point                = false;
%             st.drone_inside_obstacle_point   = false;
%             st.load_inside_obstacle_point    = false;
%             st.drone_min_dist_point          = inf;
%             st.load_min_dist_point           = inf;
%             st.min_dist_point                = inf;
%             st.drone_obstacle_id_point       = NaN;
%             st.load_obstacle_id_point        = NaN;
%             st.min_obstacle_id_point         = NaN;
%             st.min_source_point              = "none";
%             st.detected_obstacle_count_point = 0;
%             st.threat_candidate_count        = 0;
%             st.primary_threat_id             = NaN;
%             st.primary_threat_tcpa           = inf;
%             st.primary_threat_dcpa           = inf;
% 
%             obj.result.state = st;
%         end
% 
%         % =====================================================================
%         % get_obstacles_at_time
%         % =====================================================================
%         function list = get_obstacles_at_time(~, t_now)
%             list = ENVIRONMENT_OBSTACLE_ELLIPSE_MOVE(t_now);
%         end
% 
%         % =====================================================================
%         % check_drone_sensor_15m: 機体搭載センサ基準の 15m 表面距離スキャン (Phase 0)
%         % =====================================================================
%         function det = check_drone_sensor_15m(obj, pQ, pL, obs_list, t_now)
%             det = struct();
%             det.time                          = t_now;
%             det.pQ                            = pQ;
%             det.pL                            = pL;
%             det.trigger_dist                  = obj.trigger_dist;
%             det.detected_point                = false;
%             det.drone_inside_obstacle_point   = false;
%             det.load_inside_obstacle_point    = false;
%             det.drone_min_dist_point          = inf;
%             det.load_min_dist_point           = inf;
%             det.min_dist_point                = inf;
%             det.drone_obstacle_id_point       = NaN;
%             det.load_obstacle_id_point        = NaN;
%             det.min_obstacle_id_point         = NaN;
%             det.min_source_point              = "none";
%             det.detected_obstacles_point      = [];
%             det.detected_obstacle_count_point = 0;
% 
%             if isempty(obs_list), return; end
%             detected_obs = [];
% 
%             for i = 1:length(obs_list)
%                 o = obs_list(i);
%                 obs_id = i;
%                 if isfield(o, 'id') && ~isempty(o.id), obs_id = o.id; end
% 
%                 R_obs     = o.R_obs;
%                 radii_obs = o.ellipsoid_radii(:);
%                 p_obs     = o.p_center(:);
% 
%                 v_obs = [0; 0; 0];
%                 if isfield(o, 'v_center') && ~isempty(o.v_center),     v_obs = o.v_center(:);
%                 elseif isfield(o, 'v') && ~isempty(o.v),               v_obs = o.v(:); end
% 
%                 d_margin_obs = obj.default_margin;
%                 if isfield(o, 'd_margin') && ~isempty(o.d_margin),     d_margin_obs = o.d_margin; end
% 
%                 obs_parsed = struct('center', p_obs, 'radii', radii_obs, 'R', R_obs);
% 
%                 [d_drone, in_drone, cpQ_loc, ~] = obj.point_ellipsoid_signed_distance(pQ, obs_parsed);
%                 [d_load, in_load, cpL_loc, ~]   = obj.point_ellipsoid_signed_distance(pL, obs_parsed);
% 
%                 if in_drone, det.drone_inside_obstacle_point = true; end
%                 if in_load,  det.load_inside_obstacle_point  = true; end
% 
%                 if d_drone < det.drone_min_dist_point
%                     det.drone_min_dist_point = d_drone;
%                     det.drone_obstacle_id_point = obs_id;
%                 end
%                 if d_load < det.load_min_dist_point
%                     det.load_min_dist_point = d_load;
%                     det.load_obstacle_id_point = obs_id;
%                 end
% 
%                 obs_min = min(d_drone, d_load);
% 
%                 nQ = cpQ_loc ./ (radii_obs.^2);
%                 nL = cpL_loc ./ (radii_obs.^2);
%                 if norm(nQ) < 1e-12, normal_drone = [0; 0; 0]; else, normal_drone = R_obs * (nQ / norm(nQ)); end
%                 if norm(nL) < 1e-12, normal_load = [0; 0; 0];  else, normal_load = R_obs * (nL / norm(nL)); end
% 
%                 obs_item = struct( ...
%                     'id',               obs_id, ...
%                     'dist_drone_point', d_drone, ...
%                     'dist_load_point',  d_load, ...
%                     'min_dist_point',   obs_min, ...
%                     'p_obs',            p_obs, ...
%                     'v_obs',            v_obs, ...
%                     'radii_obs',        radii_obs, ...
%                     'R_obs',            R_obs, ...
%                     'd_margin',         d_margin_obs, ...
%                     'normal_drone',     normal_drone, ...
%                     'normal_load',      normal_load ...
%                 );
% 
%                 if d_drone <= obj.trigger_dist
%                     det.detected_point = true;
%                     detected_obs = [detected_obs; obs_item];
%                 end
%             end
% 
%             if det.drone_min_dist_point <= det.load_min_dist_point
%                 det.min_dist_point   = det.drone_min_dist_point;
%                 det.min_obstacle_id_point = det.drone_obstacle_id_point;
%                 det.min_source_point = "drone";
%             else
%                 det.min_dist_point   = det.load_min_dist_point;
%                 det.min_obstacle_id_point = det.load_obstacle_id_point;
%                 det.min_source_point = "load";
%             end
% 
%             det.detected_obstacles_point      = detected_obs;
%             det.detected_obstacle_count_point = numel(detected_obs);
%         end
% 
%         % =====================================================================
%         % build_sphere_chain_model
%         %
%         % 【理論的位置付け】
%         %   UAVと荷物を直線索で結ぶ taut cable を仮定し、
%         %
%         %       p_C(s) = (1-s) p_L + s p_Q,  0 <= s <= 1
%         %
%         %   として索中心線を構成する。
%         %
%         %   重要：
%         %   UAVが荷物の真上にいない場合、索は鉛直ではなく傾斜する。
%         %   本モデルでは「傾斜量」を単純な半径マージンとして加算しない。
%         %   実際の端点 p_L, p_Q から構成した p_C(s) 自体が傾斜した
%         %   公称索中心線になるためである。
%         %
%         %   これは Tian et al. (2026),
%         %   "Safe motion planning of underactuated rope-suspended payload
%         %    systems: A safety preview-based approach"
%         %   Automatica, 193, 113227
%         %   が rope inclination を考慮した rope collision avoidance を
%         %   明示的に扱っている考え方と整合する。
%         %
%         %   また、tether segment を機体とは独立した衝突対象として扱う
%         %   geometric tether control の考え方は、
%         %   "Autonomous Navigation of Tethered Drone Strings in Cluttered
%         %    Environments through Real-Time Obstacle Detection and Geometric
%         %    Tether Control" (2026)
%         %   と対応する。
%         %
%         % 【索上不確実性の導出】
%         %   UAV端点誤差を ||e_Q|| <= eps_Q、
%         %   荷物端点誤差を ||e_L|| <= eps_L とする。
%         %
%         %   索上点の誤差は
%         %
%         %       e_C(s) = (1-s)e_L + s e_Q
%         %
%         %   である。
%         %
%         %   三角不等式より
%         %
%         %       ||e_C(s)||
%         %       <= (1-s)||e_L|| + s||e_Q||
%         %       <= (1-s)eps_L + s eps_Q
%         %
%         %   したがって、索上点の追従誤差上界として
%         %
%         %       eps_C(s) = (1-s)eps_L + s eps_Q
%         %
%         %   を使用する。
%         %
%         %   同じ考え方で、端点の遅延変位上界も
%         %
%         %       d_delay_C(s)
%         %         = (1-s)d_delay_L + s d_delay_Q
%         %
%         %   とする。
%         %
%         % 【ケーブル球の幾何被覆】
%         %   単に Δs <= 2*r_cable とすると、中心線上では球間に隙間が
%         %   無くても、半径 r_cable の連続円柱（capsule）全体を数学的に
%         %   被覆できるとは限らない。
%         %
%         %   隣接球中心間距離を d とすると、線分区間の中央点までを
%         %   cable radius r_cable で被覆する十分条件は
%         %
%         %       R_sphere >= sqrt(r_cable^2 + (d/2)^2)
%         %
%         %   である。
%         %
%         %   よって離散化による追加半径を
%         %
%         %       r_disc =
%         %         sqrt(r_cable^2 + (d/2)^2) - r_cable
%         %
%         %   とする。
%         %
%         %   これを cable sphere の有効半径へ追加する。
%         %
%         % 【最終的な索球半径】
%         %
%         %   R_C(s) =
%         %       r_cable
%         %       + r_disc
%         %       + d_delay_C(s)
%         %       + eps_C(s)
%         %       + r_swing_C(s)
%         %       + d_margin_obs
%         %
%         %   ここで r_swing_C(s) は索の揺動による残差包絡であり、
%         %   現在の実装では実験データから得た r_swing_max を
%         %   sin(pi*s) 型で端点ゼロ・中央最大として配分している。
%         %
%         %   【注意】
%         %   r_swing_C(s) = r_swing_max*sin(pi*s) は現時点では
%         %   ケーブル内部点の実測データから直接同定したものではない。
%         %   したがって「理論的に導出された厳密上界」ではなく、
%         %   実験で妥当性検証すべき reduced-order envelope である。
%         % Tian et al., "Safe motion planning of underactuated rope-suspended payload systems: A safety preview-based approach", Automatica, 2026
%         % → rope inclination を含む rope collision avoidance を扱う。今回の「索を点ではなく幾何形状として扱う」根拠。
%         % "Autonomous Navigation of Tethered Drone Strings in Cluttered Environments through Real-Time Obstacle Detection and Geometric Tether Control", 2026
%         % → drone body と tether segment を同時に obstacle avoidance の対象として扱う。今回の Sphere-Chain における「索を独立した衝突対象にする」考え方に対応。
%         % Qian & Liu, "Path-Following Control of A Quadrotor UAV With A Cable-Suspended Payload Under Wind Disturbances", IEEE TIE, 2020
%         % → UAV–cable–payload 系で外乱・不確実性を扱う基礎研究。今回の eps_track 等の不確実性包絡を設定する背景として引用可能。
%         % =====================================================================
%         function sphere_chain = build_sphere_chain_model( ...
%                 obj, pQ, vQ, pL, vL, d_margin_obs)
% 
%             % -------------------------------------------------------------
%             % 1. 実際の UAV - 荷物間ベクトル
%             %
%             %    UAVが荷物の真上にいなければ、
%             %    このベクトル自体が傾いた cable centerline になる。
%             % -------------------------------------------------------------
%             vec_cable = pQ - pL;
%             L_cable = norm(vec_cable);
% 
%             if L_cable < 1e-9
%                 cable_dir = [0; 0; 1];
%             else
%                 cable_dir = vec_cable / L_cable;
%             end
% 
%             % 鉛直方向との傾斜角 [rad]
%             e_z = [0; 0; 1];
%             cable_tilt_angle = acos(max(-1.0, ...
%                 min(1.0, dot(cable_dir, e_z))));
% 
%             % -------------------------------------------------------------
%             % 2. Sphere-Chain の球数
%             %
%             %    幾何上の中心間隔 d を決定する。
%             %    「sqrt(2)*r_cable」は単独で gapless coverage を
%             %    保証するものではないため、実際の d を用いて
%             %    r_disc を別途計算する。
%             % -------------------------------------------------------------
%             ds_geom_safe = sqrt(2.0) * obj.r_cable;
% 
%             K_balls = max( ...
%                 obj.min_cable_pts, ...
%                 ceil(L_cable / ds_geom_safe) + 1);
% 
%             s_samples = linspace(0.0, 1.0, K_balls);
% 
%             % 実際の隣接球中心間距離
%             if K_balls > 1
%                 ds_actual = L_cable / (K_balls - 1);
%             else
%                 ds_actual = 0.0;
%             end
% 
%             % -------------------------------------------------------------
%             % 3. 連続 cable capsule を離散球で被覆するための
%             %    discretization inflation
%             %
%             %    R >= sqrt(r_cable^2 + (d/2)^2)
%             % -------------------------------------------------------------
%             if ds_actual > 0.0
%                 r_disc = sqrt( ...
%                     obj.r_cable^2 + (0.5 * ds_actual)^2) ...
%                     - obj.r_cable;
%             else
%                 r_disc = 0.0;
%             end
% 
%             % -------------------------------------------------------------
%             % 4. UAV / 荷物端点の遅延変位上界
%             %
%             %    d_delay <= ||v|| * tau
%             %
%             %    これは速度方向を限定せず、等方球として扱う
%             %    conservative bound。
%             % -------------------------------------------------------------
%             spd_Q = norm(vQ);
%             spd_L = norm(vL);
% 
%             d_delay_bound_Q = spd_Q * obj.tau_delay_drone;
%             d_delay_bound_L = spd_L * obj.tau_delay_load;
% 
%             sphere_chain = cell(1, K_balls);
% 
%             for b = 1:K_balls
% 
%                 s_b = s_samples(b);
% 
%                 % ---------------------------------------------------------
%                 % 5. 傾斜した cable centerline
%                 %
%                 %    p_C(s) = (1-s)p_L + s p_Q
%                 %
%                 %    ここが今回の重要点。
%                 %    「荷物の真上からの横ずれ」を別マージンとして
%                 %    加えるのではなく、索そのものを傾けている。
%                 % ---------------------------------------------------------
%                 p_b = (1.0 - s_b) * pL + s_b * pQ;
%                 v_b = (1.0 - s_b) * vL + s_b * vQ;
% 
%                 % ---------------------------------------------------------
%                 % 6. 端点 → 索内部への不確実性伝播
%                 %
%                 %    eps_C(s) = (1-s)eps_L + s eps_Q
%                 %
%                 %    端点の球状誤差集合の凸結合による保守的上界。
%                 % ---------------------------------------------------------
%                 d_delay_mid = ...
%                     (1.0 - s_b) * d_delay_bound_L ...
%                     + s_b * d_delay_bound_Q;
% 
%                 eps_track_mid = ...
%                     (1.0 - s_b) * obj.eps_track_load ...
%                     + s_b * obj.eps_track_drone;
% 
%                 % ---------------------------------------------------------
%                 % 7. 索の揺動残差包絡
%                 %
%                 %    現在の reduced-order model：
%                 %        r_swing_C(s)
%                 %          = r_swing_max sin(pi*s)
%                 %
%                 %    中央部最大、両端0。
%                 % ---------------------------------------------------------
%                 swing_local = ...
%                     obj.r_swing_max * sin(pi * s_b);
% 
%                 % ---------------------------------------------------------
%                 % 8. 各球の有効半径
%                 % ---------------------------------------------------------
%                 if b == 1
% 
%                     % 荷物端点
%                     b_name = "load_sphere";
% 
%                     r_effective = ...
%                         obj.r_load ...
%                         + d_delay_bound_L ...
%                         + obj.eps_track_load ...
%                         + d_margin_obs;
% 
%                 elseif b == K_balls
% 
%                     % UAV端点
%                     b_name = "drone_sphere";
% 
%                     r_effective = ...
%                         obj.r_drone ...
%                         + d_delay_bound_Q ...
%                         + obj.eps_track_drone ...
%                         + d_margin_obs;
% 
%                 else
% 
%                     % 索内部
%                     b_name = sprintf("cable_sphere_%d", b);
% 
%                     r_effective = ...
%                         obj.r_cable ...
%                         + r_disc ...
%                         + d_delay_mid ...
%                         + eps_track_mid ...
%                         + swing_local ...
%                         + d_margin_obs;
%                 end
% 
%                 % ---------------------------------------------------------
%                 % 9. デバッグ・検証用情報も保存
%                 % ---------------------------------------------------------
%                 sphere_chain{b} = struct( ...
%                     'name',             b_name, ...
%                     'p',                p_b, ...
%                     'v',                v_b, ...
%                     's',                s_b, ...
%                     'r_effective',      r_effective, ...
%                     'r_disc',            r_disc, ...
%                     'eps_track',         eps_track_mid, ...
%                     'delay_bound',       d_delay_mid, ...
%                     'swing_bound',       swing_local, ...
%                     'cable_tilt_angle',  cable_tilt_angle, ...
%                     'cable_dir',         cable_dir);
%             end
%         end
% 
%         % =====================================================================
%         % filter_threat_candidates_sphere_chain
%         % Woo et al. (2026) の相対運動 CPA 概念に基づく脅威候補フィルタリング
%         % (障害物1個あたりK個の球体について各々解析解 O(1) でCPAを算出: 全体 O(N_obs * K))
%         % =====================================================================
%         function tc = filter_threat_candidates_sphere_chain(obj, pQ, vQ, pL, vL, detected_obs)
%             tc = struct();
%             tc.count           = 0;
%             tc.items           = [];
%             tc.primary_id      = NaN;
%             tc.primary_tcpa    = inf;
%             tc.primary_dcpa    = inf;
%             tc.threat_centroid = [0;0;0];
% 
%             if isempty(detected_obs), return; end
% 
%             candidates = [];
% 
%             for k = 1:numel(detected_obs)
%                 obs = detected_obs(k);
%                 p_o = obs.p_obs;
%                 v_o = obs.v_obs;
%                 d_m = obs.d_margin;
%                 r_bound_obs = max(obs.radii_obs); % 障害物外接球半径 (保守的スクリーニング用)
% 
%                 % 自機系の隙間なし複数球モデルを生成 (不確実性包絡マージン付加済み)
%                 chain = obj.build_sphere_chain_model(pQ, vQ, pL, vL, d_m);
% 
%                 min_d_cpa = inf;
%                 crit_t_cpa = inf;
%                 critical_entity = "none";
%                 threat_detected = false;
% 
%                 % 各球体に対して相対運動 CPA を評価 (各球 O(1) 閉形式[cite: 1])
%                 for b = 1:numel(chain)
%                     sph = chain{b};
%                     r_rel = sph.p - p_o;
%                     v_rel = sph.v - v_o;
%                     v_rel_sq = dot(v_rel, v_rel);
% 
%                     % 相対運動に基づく最接近時間 t_cpa の閉形式算出 (Woo et al., 式 9)[cite: 1]
%                     if v_rel_sq < 1e-4
%                         t_cpa_b = 0.0;
%                         center_dist_cpa = norm(r_rel);
%                     else
%                         t_cpa_b = -dot(r_rel, v_rel) / v_rel_sq;
%                         center_dist_cpa = norm(r_rel + v_rel * t_cpa_b);
%                     end
% 
%                     % 有効保護半径を考慮した最接近予測クリアランス
%                     d_cpa_b = center_dist_cpa - (r_bound_obs + sph.r_effective);
% 
%                     % フィルタ通過条件: 将来 (0〜t_preview) に最接近し、かつ予備ゲート以内
%                     if (t_cpa_b >= 0.0) && (t_cpa_b <= obj.t_preview_end) && (d_cpa_b <= obj.d_gate_margin)
%                         threat_detected = true;
%                         if d_cpa_b < min_d_cpa
%                             min_d_cpa = d_cpa_b;
%                             crit_t_cpa = t_cpa_b;
%                             critical_entity = sph.name;
%                         end
%                     end
%                 end
% 
%                 % ★ 現在時刻 (t=0) における全球体チェーン侵入救済判定 (UAVだけでなく索・荷物も包含)
%                 curr_dist_min = inf;
%                 curr_entity = "none";
%                 for b = 1:numel(chain)
%                     sph = chain{b};
%                     d_now = norm(sph.p - p_o) - (r_bound_obs + sph.r_effective);
%                     if d_now < curr_dist_min
%                         curr_dist_min = d_now;
%                         curr_entity = sph.name;
%                     end
%                 end
% 
%                 if curr_dist_min <= 0.0
%                     threat_detected = true;
%                     crit_t_cpa = 0.0;
%                     min_d_cpa = curr_dist_min;
%                     critical_entity = curr_entity;
%                 end
% 
%                 % フィルタを通過した障害物を後段用の候補リストへ登録
%                 if threat_detected
%                     item = struct();
%                     item.id               = obs.id;
%                     item.critical_entity  = critical_entity;   % 最も接近する部位
%                     item.t_cpa            = max(0.0, crit_t_cpa); % 最接近時刻 [s]
%                     item.d_cpa            = min_d_cpa;         % 予測最小離隔 [m] (不確実性包絡込み)
%                     item.p_obs            = p_o;               % 障害物中心位置
%                     item.v_obs            = v_o;               % 障害物速度
%                     item.radii_obs        = obs.radii_obs;     % 障害物3軸半径
%                     item.R_obs            = obs.R_obs;         % 障害物姿勢行列
%                     item.d_margin         = d_m;               % 適用安全マージン
% 
%                     candidates = [candidates; item]; %#ok<AGROW>
%                 end
%             end
% 
%             % 候補障害物群の優先度ソート (最接近時間 t_cpa 昇順 -> 予測離隔 d_cpa 昇順)
%             if ~isempty(candidates)
%                 sort_matrix = [[candidates.t_cpa]', [candidates.d_cpa]'];
%                 [~, sort_idx] = sortrows(sort_matrix, [1, 2]);
%                 sorted_candidates = candidates(sort_idx);
% 
%                 tc.items           = sorted_candidates;
%                 tc.count           = numel(sorted_candidates);
%                 tc.primary_id      = sorted_candidates(1).id;
%                 tc.primary_tcpa    = sorted_candidates(1).t_cpa;
%                 tc.primary_dcpa    = sorted_candidates(1).d_cpa;
% 
%                 all_centers = reshape([sorted_candidates.p_obs], 3, []);
%                 tc.threat_centroid = mean(all_centers, 2);
%             end
%         end
% 
%         % =====================================================================
%         % point_ellipsoid_signed_distance: ラグランジュ未定乗数・二分探索
%         % =====================================================================
%         function [d, inside, closest_local, lambda] = point_ellipsoid_signed_distance(~, p, o)
%             p = p(:); c = o.center(:); r = o.radii(:); R = o.R;
%             y = R' * (p - c);
%             r2 = r.^2;
%             q = sum((y ./ r).^2);
%             inside = (q < 1.0);
% 
%             if norm(y) < 1e-14
%                 [min_r, min_idx] = min(r);
%                 closest_local = zeros(3, 1); closest_local(min_idx) = min_r;
%                 d = -min_r; lambda = -min(r2);
%                 return;
%             end
% 
%             f = @(lam) sum(r2 .* (y.^2) ./ ((lam + r2).^2)) - 1.0;
%             if q > 1.0
%                 lo = 0.0; hi = max(r) * norm(y);
%             else
%                 lo = -min(r2) * (1.0 - 1e-12); hi = 0.0;
%             end
% 
%             for kk = 1:80
%                 mid = 0.5 * (lo + hi);
%                 if f(mid) > 0, lo = mid; else, hi = mid; end
%             end
%             lambda = 0.5 * (lo + hi);
%             closest_local = r2 .* y ./ (lambda + r2);
%             d_abs = norm(closest_local - y);
%             if inside, d = -d_abs; else, d = d_abs; end
%         end
%     end
% end

classdef REPLANNING_BSPLINE < handle
    % =========================================================================
    % REPLANNING_BSPLINE
    % 1. 模擬センサー：機体重心 pQ を点（Point）として扱い、機体搭載センサーによる
    %    楕円体境界面までの最短ユークリッド距離計算および 7m 接近検知判定を実行。
    % 2. 衝突診断系：荷物位置 pL についてはセンサーとしては扱わず、機体センサー情報や
    %    安全評価（最短表面距離、最近接点、法線、侵入有無）の診断値としてのみ記録。
    % =========================================================================
    properties
        self % ドローンエージェント自身 (推定器 estimator やパラメータ parameter を保持) 
        base_ref % 公称参照軌道生成オブジェクト
        result                % 出力結果構造体 (目標状態 xd, pRef, vRef, yawRef, 検知・診断情報)
        trigger_dist = 7.0;   % 機体搭載センサーによる接近検知の閾値 [m] (表面間距離がこれ以下になるとアラート)
        % r_load       = 0.15;  % 荷物保護半径 [m] (必要に応じて将来のマージン計算等に使用)
        % r_drone      = 0.30;  % 機体保護半径 [m] (必要に応じて将来のマージン計算等に使用)
    end
    methods (Access = public)
        % =====================================================================
        % コンストラクタ: クラスの初期化と外部設定 (opts) の反映
        % =====================================================================
        function obj = REPLANNING_BSPLINE(self, base_ref, opts)
            arguments
                self                  % 必須: エージェントインスタンス
                base_ref              % 必須: 通常飛行用の公称軌道インスタンス
                opts = struct()       % 任意: 外部からパラメータを変更するための構造体
            end
            obj.self = self;
            obj.base_ref = base_ref;
            if isfield(opts, 'trigger_dist'), obj.trigger_dist = opts.trigger_dist; end % 外で定義されていたらデフォルト値を上書き 機体センサー検知範囲
            % if isfield(opts, 'r_load'),       obj.r_load       = opts.r_load;       end % 外で定義されていたらデフォルト値を上書き 牽引物を近似した球体
            % if isfield(opts, 'r_drone'),      obj.r_drone      = opts.r_drone;      end % 外で定義されていたらデフォルト値を上書き 機体を近似した球体
            % base_ref の result 構造体をそのまま継承 (直下は state のみ)
            obj.result = base_ref.result;
            % --- 【重要】STATE_CLASS に検知・診断用プロパティを動的追加 (dynamicprops) ---
            % これにより、state 内に正式な記録領域が作成され、ロガーで抽出可能になる
            sensor_props = ["time", "pQ", "pL", "detected_point", ...
                "drone_inside_obstacle_point", "load_inside_obstacle_point", ...
                "drone_min_dist_point", "load_min_dist_point", "min_dist_point", ...
                "drone_obstacle_id_point", "load_obstacle_id_point", "min_obstacle_id_point", ...
                "min_source_point", "detected_obstacle_count_point"];
            for p_name = sensor_props
                if ~isprop(obj.result.state, p_name)
                    addprop(obj.result.state, p_name);
                end
            end
            % 初期ダミー値のセット
            obj.clear_state_sensor_values(0.0);
        end
        % =====================================================================
        % do: 制御周期ごと (例: 25ms周期) にメインループから呼び出される実行メソッド
        % 入力: 
        %   varargin{1}: time (現在の時刻 struct: time.t, time.dt など)
        %   varargin{2}: cha  (フェーズ文字列: 'f' = 飛行中, 't' = 離陸など)
        %   varargin{4}: env  (環境構造体、障害物リストを内包する場合あり)
        % 出力:
        %   result_out : 下流のコントローラやロガーが受け取る目標状態・検知・診断結果
        % =====================================================================
        function result_out = do(obj, varargin)
            time = varargin{1}; % varargin{1}: time (現在の時刻 struct: time.t, time.dt など)
            cha = varargin{2}; % varargin{2}: cha  (フェーズ文字列: 'f' = 飛行中, 't' = 離陸など)
            % --- 公称目標軌道 (Nominal Reference) の算出 ---
            % 本クラスが障害物を回避する新軌道を生成しない間は、公称軌道生成器の出力をそのまま踏襲する
            % 公称参照軌道の取得
            base_res = obj.base_ref.do(varargin{:}); % 公称軌道の抜き出し
            xd_nom = base_res.state.xd; % 牽引物の目標３次元位置・yaw角からその６階微分まで [pL(3); yaw(1); vL(3); yaw_dot(1); aL(3)...]
            
            % 毎ステップ、検知・診断プロパティの初期値をリセット
            obj.clear_state_sensor_values(time.t);
            
            % obj.result.state に公称軌道を反映 (base_res 全体の上書きは行わない)
            obj.result.state.xd = xd_nom; % 公称軌道保存
            % --- 2. 空の検知・診断構造体を用意 (非飛行フェーズ用) ---
            detection = struct();
            detection.time                          = time.t;             % [s] 現在のシミュレーション時刻
            detection.pQ                            = [NaN; NaN; NaN];    % [m] ドローン機体重心の3次元位置ベクトル [x; y; z]
            detection.pL                            = [NaN; NaN; NaN];    % [m] 牽引荷物の3次元位置ベクトル [x; y; z] (状態診断用)
            detection.detected_point                = false;              % [bool] 機体センサー7m近接検知フラグ (true: 検知, false: 未検知)
            detection.drone_inside_obstacle_point   = false;              % [bool] 機体の障害物楕円体内部侵入フラグ (true: 侵入, false: 外部)
            detection.load_inside_obstacle_point    = false;              % [bool] 荷物の障害物楕円体内部侵入フラグ (true: 侵入, false: 外部)
            detection.drone_min_dist_point          = inf;                % [m] 機体センサーから全障害物表面までの最短幾何学距離
            detection.load_min_dist_point           = inf;                % [m] 荷物から全障害物表面までの最短幾何学距離 (診断値)
            detection.min_dist_point                = inf;                % [m] 機体センサーによる最短表面距離 (機体基準)
            detection.drone_obstacle_id_point       = NaN;                % [ID] 機体にとって最短距離を与えている障害物インデックス番号
            detection.load_obstacle_id_point        = NaN;                % [ID] 荷物にとって最短距離を与えている障害物インデックス番号 (診断値)
            detection.min_obstacle_id_point         = NaN;                % [ID] 機体センサーが捉えた最短障害物インデックス番号
            detection.min_source_point              = "none";             % [string] 最短距離センサ種別 (検知なし: "none", 検知時: "drone")
            detection.detected_obstacle_count_point = 0;                  % [個] 機体センサーにより7m以内に検知された障害物の総数
            % --- 飛行フェーズ ('f') 時の近接障害物スキャン & 荷物状態診断 ---
            if cha == 'f'
                % --- 機体・荷物の現在位置の取得 ---
                % (A) 状態推定器 (estimator) から真値・推定位置を直接取得
                % ※ フォールバックを排除しているため、estimator にプロパティが存在しない場合は即座にエラー停止
                pL_cur = obj.self.estimator.result.state.pL(:); % 荷物位置の現在3次元位置 [x; y; z]
                pQ_cur = obj.self.estimator.result.state.p(:); % ドローン機体の現在3次元位置 [x; y; z]
                % (B) 現在時刻 t における動的障害物配置を取得
                obs_list = obj.get_obstacles_at_time(time.t);
                % (C) 機体センサーによる最短距離評価・検知判定、および荷物の幾何診断値を計算
                detection = obj.check_detection_simulated_sensor(pQ_cur, pL_cur, obs_list, time.t);
                % 機体センサーが7m以内に侵入した場合のコンソール警告
                if detection.detected_point
                    fprintf("[PROXIMITY ALERT] t=%.3f s | 機体センサー7m近接検知! (機体表面間: %.2f m, 荷物表面間診断: %.2f m, 最寄障害物ID: %d)\n", ...
                        time.t, detection.drone_min_dist_point, detection.load_min_dist_point, detection.min_obstacle_id_point);
                end
            end
            
            % --- 4. 【最重要】state 内の各プロパティに代入して app.logger に完全保存 ---
            st = obj.result.state;
            st.xd                            = xd_nom;                                  % [28x1 double] 目標軌道全状態
            st.p                             = xd_nom(1:3);                             % [m] 目標位置 [x; y; z]
            st.v                             = xd_nom(5:7);                             % [m/s] 目標速度 [vx; vy; vz]
            st.q                             = [0; 0; xd_nom(4)];                       % [rad] 目標姿勢 (yaw角)
            
            st.time                          = detection.time;                          % [s] 計測時刻 (double)
            st.pQ                            = detection.pQ;                            % [m] ドローン機体重心位置 (3x1 double)
            st.pL                            = detection.pL;                            % [m] 荷物位置 (3x1 double, 診断用)
            st.detected_point                = detection.detected_point;                % [bool] 機体センサー7m近接検知判定フラグ (true / false)
            st.drone_inside_obstacle_point   = detection.drone_inside_obstacle_point;   % [bool] 機体侵入フラグ (true: 侵入, false: 外部)
            st.load_inside_obstacle_point    = detection.load_inside_obstacle_point;    % [bool] 荷物侵入フラグ (診断用, true: 侵入, false: 外部)
            st.drone_min_dist_point          = detection.drone_min_dist_point;          % [m] 機体センサー最短表面距離 (正: 外部, 負: 侵入深さ)
            st.load_min_dist_point           = detection.load_min_dist_point;           % [m] 荷物の最短表面距離 (診断値, 正: 外部, 負: 侵入深さ)
            st.min_dist_point                = detection.min_dist_point;                % [m] 機体センサーによる最短幾何表面距離 (= dQ)
            st.drone_obstacle_id_point       = detection.drone_obstacle_id_point;       % [ID] 機体にとって最短の障害物インデックス番号
            st.load_obstacle_id_point        = detection.load_obstacle_id_point;        % [ID] 荷物にとって最短の障害物インデックス番号 (診断値)
            st.min_obstacle_id_point         = detection.min_obstacle_id_point;         % [ID] 機体センサーが捉えた最短障害物インデックス番号
            st.min_source_point              = detection.min_source_point;              % [string] 最短距離センサ種別 (検知なし: "none", 検知時: "drone")
            st.detected_obstacle_count_point = detection.detected_obstacle_count_point; % [個] 機体センサーにより7m以内に検知された障害物の総数
            
            result_out = obj.result;
        end
    end
    methods (Access = private)
        % =====================================================================
        % clear_state_sensor_values: state 内の検知・診断プロパティを初期化
        % =====================================================================
        function clear_state_sensor_values(obj, t_now)
            st = obj.result.state;
            st.time                          = t_now;              % [s] 現在時刻
            st.pQ                            = [NaN; NaN; NaN];    % [m] ドローン位置初期値
            st.pL                            = [NaN; NaN; NaN];    % [m] 荷物位置初期値 (診断用)
            st.detected_point                = false;              % 検知なし
            st.drone_inside_obstacle_point   = false;              % 機体侵入なし
            st.load_inside_obstacle_point    = false;              % 荷物侵入なし (診断用)
            st.drone_min_dist_point          = inf;                % 最短距離初期値 (無限大)
            st.load_min_dist_point           = inf;                % 最短距離初期値 (無限大, 診断用)
            st.min_dist_point                = inf;                % 最短距離初期値 (無限大)
            st.drone_obstacle_id_point       = NaN;                % 最短障害物ID初期値
            st.load_obstacle_id_point        = NaN;                % 最短障害物ID初期値 (診断用)
            st.min_obstacle_id_point         = NaN;                % 最短障害物ID初期値
            st.min_source_point              = "none";             % 最短センサ初期値
            st.detected_obstacle_count_point = 0;                  % 検知個数初期値
        end
        % =====================================================================
        % get_obstacles_at_time: 環境関数を叩き、時刻 t_now での障害物リストを取得
        % =====================================================================
        function list = get_obstacles_at_time(~,t_now)
            % ENVIRONMENT_OBSTACLE_ELLIPSE_MOVE 内ですでに「p_center = p0 + v*t」
            % のように時刻 t_now に応じた現在位置が計算されている
            list = ENVIRONMENT_OBSTACLE_ELLIPSE_MOVE(t_now);
        end
        % =====================================================================
        % check_detection_simulated_sensor: 
        % 機体搭載センサーによる接近検知判定（7m以内）を実行し、
        % 荷物についてはシステム状態診断値（距離・最近接点・法線・侵入有無）として計算・記録する
        % =====================================================================
        function det = check_detection_simulated_sensor(obj, pQ, pL, obs_list, t_now)
            det = struct();
            % --- センサ状態・時刻 ---
            det.time                          = t_now;              % [s] 現在のシミュレーション時刻 (double)
            det.pQ                            = pQ;                 % [m] ドローン機体重心の3次元位置ベクトル [x; y; z] (3x1 double)
            det.pL                            = pL;                 % [m] 牽引荷物の3次元位置ベクトル [x; y; z] (3x1 double, 診断用)
            det.trigger_dist                  = obj.trigger_dist;   % [m] 機体センサーの近接検知閾値距離 (7.0 m) (double)
            % --- 判定フラグ ---
            det.detected_point                = false;              % [bool] 機体センサーが障害物から7m以内に接近したか (true: 検知, false: 未検知)
            det.drone_inside_obstacle_point   = false;              % [bool] 機体が障害物楕円体の内部へ侵入/衝突したか (true: 侵入, false: 外部)
            det.load_inside_obstacle_point    = false;              % [bool] 荷物が障害物楕円体の内部へ侵入/衝突したか (診断用, true: 侵入, false: 外部)
            % --- 最短距離 ---
            det.drone_min_dist_point          = inf;                % [m] 機体センサーから表面までの最短幾何学距離 (double, 内部時は負値)
            det.load_min_dist_point           = inf;                % [m] 荷物から表面までの最短幾何学距離 (診断値, double, 内部時は負値)
            det.min_dist_point                = inf;                % [m] 機体センサーによる最短幾何表面距離 (= dQ)
            % --- 識別情報・統計 ---
            det.drone_obstacle_id_point       = [];                 % [ID] 機体にとって最短距離を与えている障害物のインデックス番号 (integer)
            det.load_obstacle_id_point        = [];                 % [ID] 荷物にとって最短距離を与えている障害物のインデックス番号 (診断値, integer)
            det.min_obstacle_id_point         = [];                 % [ID] 機体センサーが捉えた最短障害物のインデックス番号 (integer)
            det.min_source_point              = "none";             % [string] 最短距離センサ種別 (検知なし: "none", 検知時: "drone")
            det.detected_obstacles_point      = [];                 % [struct配列] 機体センサーにより7m以内に検知された全障害物の詳細情報リスト
            det.detected_obstacle_count_point = 0;                  % [個] 機体センサーにより7m以内に検知された障害物の総数 (integer)
            if isempty(obs_list)
                return;
            end
            detected_obs_point = [];
            for i = 1:length(obs_list)
                o = obs_list(i);
                % --- 障害物パラメータの抽出 (推測フォールバックなし) ---
                if ~isfield(o, 'R_obs') || ~isfield(o, 'ellipsoid_radii') || ~isfield(o, 'p_center')
                    error('インデックス %d の障害物に必須フィールド (R_obs, ellipsoid_radii, p_center) が不足しています.', i);
                end
                % --- 障害物の姿勢回転行列 R_obs の抽出 ---
                R_obs = o.R_obs;
                % --- 外接楕円体の主軸半径 [a; b; c] の抽出 ---
                radii_obs = o.ellipsoid_radii(:);
                % --- 障害物の中心位置 ---
                p_obs = o.p_center(:);
                obs_parsed = struct('center', p_obs, 'radii', radii_obs, 'R', R_obs);
                
                % --- 機体 pQ (センサー) および荷物 pL (診断用) から楕円体表面への符号付き最短幾何距離 ---
                % d > 0: 表面の外側にある (表面までの最短距離 [m])
                % d < 0: 内部に侵入している (侵入深さ [m])
                [d_drone_point, inside_drone_point, cpQ_local_point, ~] = obj.point_ellipsoid_signed_distance(pQ, obs_parsed);
                [d_load_point,  inside_load_point,  cpL_local_point, ~] = obj.point_ellipsoid_signed_distance(pL, obs_parsed);
                
                % ワールド座標系における表面最近接点: x_world = center + R * x_local
                cpQ_world_point = p_obs + R_obs * cpQ_local_point;
                cpL_world_point = p_obs + R_obs * cpL_local_point;
                
                % --- 障害物表面における外向き単位法線ベクトル (ワールド座標系) ---
                nQ_local_point = cpQ_local_point ./ (radii_obs.^2);
                if norm(nQ_local_point) < 1e-12
                    error('機体の最近接点における法線ベクトルが退化 (ゼロベクトル) しました.');
                end
                normal_drone_point = R_obs * (nQ_local_point / norm(nQ_local_point));
                
                nL_local_point = cpL_local_point ./ (radii_obs.^2);
                if norm(nL_local_point) < 1e-12
                    error('荷物の最近接点における法線ベクトルが退化 (ゼロベクトル) しました.');
                end
                normal_load_point = R_obs * (nL_local_point / norm(nL_local_point));
                
                % 幾何学的侵入判定（衝突判定）の論理和更新
                if inside_drone_point, det.drone_inside_obstacle_point = true; end
                if inside_load_point,  det.load_inside_obstacle_point  = true; end
                
                % 機体側（センサー）の最小値更新
                if d_drone_point < det.drone_min_dist_point
                    det.drone_min_dist_point = d_drone_point;
                    det.drone_obstacle_id_point    = i;
                end
                % 荷物側（診断値）の最小値更新
                if d_load_point < det.load_min_dist_point
                    det.load_min_dist_point = d_load_point;
                    det.load_obstacle_id_point    = i;
                end
                
                % 該当障害物の詳細データ構造体 (荷物データはシステム診断値として包含)
                obs_info_point = struct( ...
                    'id',                        i, ...                  % [ID] 障害物のインデックス番号 (integer)
                    'dist_drone_point',          d_drone_point, ...      % [m] 機体重心(センサー)からこの障害物表面までの符号付き最短距離 (正: 外部, 負: 侵入深さ)
                    'dist_load_point',           d_load_point, ...       % [m] 荷物位置からこの障害物表面までの符号付き最短距離 (診断値, 正: 外部, 負: 侵入深さ)
                    'min_dist_point',            d_drone_point, ...      % [m] 機体センサー基準の表面間距離
                    'drone_inside_point',        inside_drone_point, ... % [bool] 機体がこの障害物の内部に侵入しているか (true: 侵入, false: 外部)
                    'load_inside_point',         inside_load_point, ...  % [bool] 荷物がこの障害物の内部に侵入しているか (診断用, true: 侵入, false: 外部)
                    'p_obs',                     p_obs, ...              % [m] 時刻 t における障害物の中心位置ベクトル [x; y; z] (3x1 double)
                    'radii_obs',                 radii_obs, ...          % [m] 障害物の3軸半径 [rx; ry; rz] (3x1 double)
                    'R_obs',                     R_obs, ...              % [-] 障害物の主軸姿勢を表す3x3回転行列 SO(3) (3x3 double)
                    'closest_drone_world_point', cpQ_world_point, ...    % [m] 機体に対する障害物表面上の最短最近接点 (ワールド座標系 [x; y; z])
                    'closest_load_world_point',  cpL_world_point, ...    % [m] 荷物に対する障害物表面上の最短最近接点 (診断用, ワールド座標系 [x; y; z])
                    'normal_drone_point',        normal_drone_point, ... % [-] 機体側最近接点における障害物表面の外向き単位法線ベクトル (ワールド系, 単位ノルム)
                    'normal_load_point',         normal_load_point ...   % [-] 荷物側最近接点における障害物表面の外向き単位法線ベクトル (診断用, ワールド系, 単位ノルム)
                );
                
                % --- 機体センサーによる接近検知判定 (7m以内) ---
                if d_drone_point <= obj.trigger_dist
                    det.detected_point = true;
                    detected_obs_point = [detected_obs_point; obs_info_point];
                end
            end
            
            % システム全体での最短距離の確定 (機体センサー基準)
            det.min_dist_point                = det.drone_min_dist_point;
            det.min_obstacle_id_point         = det.drone_obstacle_id_point;
            % 検知ありの場合は "drone"、検知なし（7m超過または障害物なし）の場合は初期値 "none" を保持
            if det.detected_point
                det.min_source_point          = "drone";
            else
                det.min_source_point          = "none";
            end
            det.detected_obstacles_point      = detected_obs_point;
            det.detected_obstacle_count_point = numel(detected_obs_point);  % 機体センサーが検知した障害物個数
        end
        % =====================================================================
        % point_ellipsoid_signed_distance
        % 任意の3次元点 p と回転楕円体 (中心 c, 半径 r=[a;b;c], 回転行列 R) の
        % 【外郭表面】までの真の最短ユークリッド距離を算出する。
        %
        % [数学的アルゴリズム]:
        % 楕円方程式: (x/a)^2 + (y/b)^2 + (z/c)^2 = 1
        % 点 p から楕円面上の最近接点 x への最短距離は、ラグランジュ未定乗数法:
        %   f(lambda) = sum( (a_i^2 * y_i^2) / (lambda + a_i^2)^2 ) - 1 = 0
        % を満たす単一根 lambda を二分探索法 (Binary Search) で 80 回反復して求め、
        % 表面点 x = (a_i^2 * y_i) / (lambda + a_i^2) と点 y の距離 ||x - y|| を計算する。
        % =====================================================================
        function [d, inside, closest_local, lambda] = point_ellipsoid_signed_distance(~, p, o)
            p = p(:);
            c = o.center(:);
            r = o.radii(:);
            R = o.R;
            % --- 入力チェック: 不正ならフォールバックせず即座に停止 ---
            if numel(p) ~= 3 || numel(c) ~= 3 || numel(r) ~= 3
                error('点 p、中心 c、および半径 r はすべて3x1ベクトルである必要があります。');
            end
            if any(~isfinite(p)) || any(~isfinite(c)) || any(~isfinite(r))
                error('点または楕円体の定義に非有限値 (NaN や Inf) が検出されました。');
            end
            if any(r <= 0)
                error('楕円体の半径はすべて厳密に正の値 (r > 0) である必要があります。');
            end
            if ~isequal(size(R), [3 3]) || any(~isfinite(R(:)))
                error('回転行列 R は有限な3x3行列である必要があります。');
            end
            if norm(R'*R - eye(3), 'fro') > 1e-6 || abs(det(R) - 1.0) > 1e-6
                error('行列 R は有効な直交回転行列 SO(3) ではありません。');
            end
            % 1. ワールド座標系の点 p を障害物の局所主軸座標系 (Local Frame) に変換
            y  = R' * (p - c);
            r2 = r.^2;
            % 2. 楕円代数判定値 q:
            %    q < 1.0 -> 点は楕円体の内部にある (衝突・侵入状態)
            %    q = 1.0 -> 点は楕円体の表面上にある
            %    q > 1.0 -> 点は楕円体の外部にある
            q      = sum((y ./ r).^2);
            inside = (q < 1.0);
            % 特異点処理: 点が障害物の中心そのものにある場合 (最も近い表面は最短半径の主軸上)
            if norm(y) < 1e-14
                [min_r, min_idx] = min(r);
                closest_local = zeros(3, 1);
                closest_local(min_idx) = min_r;
                d      = -min_r;
                lambda = -min(r2);
                return;
            end
            % ラグランジュ未定乗数の非線形方程式 f(lambda) = 0
            f = @(lam) sum(r2 .* (y.^2) ./ ((lam + r2).^2)) - 1.0;
            % 3. ラグランジュ乗数 lambda の探索範囲 (Bracketing) の設定と根の存在検証
            if q > 1.0
                % 外部点の場合: lambda >= 0
                lo = 0.0;
                hi = max(r) * norm(y);
                if f(lo) < 0 || f(hi) > 0
                    error('外部点に対する楕円体距離の求根ブラケット設定に失敗しました (解を挟み込めていません)。');
                end
            else
                % 内部点の場合: -min(r_i^2) < lambda < 0
                lo = -min(r2) * (1.0 - 1e-12);
                hi = 0.0;
                if f(lo) < 0 || f(hi) > 0
                    error('内部点に対する楕円体距離の求根ブラケット設定に失敗しました (解を挟み込めていません)。');
                end
            end
            % 4. 二分探索により lambda を数値的に収束させる
            %    80回反復して根を高精度に求める
            for kk = 1:80
                mid = 0.5 * (lo + hi);
                if f(mid) > 0
                    lo = mid;
                else
                    hi = mid;
                end
            end
            lambda = 0.5 * (lo + hi);
            % 5. 局所座標系における楕円体表面上の最近接点 closest_local
            closest_local = r2 .* y ./ (lambda + r2);
            d_abs         = norm(closest_local - y);
            % 6. 表面までの最短ユークリッド距離
            %    外部なら正値 (+), 内部なら侵入深さとして負値 (-) を返す
            if inside
                d = -d_abs;
            else
                d = d_abs;
            end
        end
    end
end