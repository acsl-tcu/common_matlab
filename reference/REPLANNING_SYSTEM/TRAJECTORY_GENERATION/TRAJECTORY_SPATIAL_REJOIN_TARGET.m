function R=TRAJECTORY_SPATIAL_REJOIN_TARGET(nominal_controls,ekf_position_m,progress_lower_u,options)
% TRAJECTORY_SPATIAL_REJOIN_TARGET Time-FREE geometric rejoin target.
% Select a spatial point on the current nominal Bezier path at parameter
% u in [progress_lower_u,1], minimizing Euclidean distance to the current
% EKF position. NEVER match a nominal wall-clock time, original trajectory
% time index, or require catch-up. No 'target_time_s' is returned.
%
% The nominal path may be a C6-smooth time-varying reference. This helper
% does not alter its derivatives and is NOT a trajectory planner: to join
% smoothly the actual planner must enforce the original 0..6 derivative
% jets at a selected spatial station with a new, freely optimized time
% law. If 7th derivatives are required for mathematically C7 joins, the
% authoritative source must supply or certify them; do not invent them.
%
% A rotation-equivariant sphere around EACH Bernstein polygon is used
% ONLY as a LOWER BOUND for geometric nearest-point search. It is NEVER
% a collision model, obstacle approximation, or physical body envelope.
% Every [a,b] subcurve is inside conv(control_points) and hence inside
% B(mean(controls),max_i ||control_i-mean||). Thus
%   max(0,||EKF-mean||-radius-guard) <= min_{u in [a,b]} ||x(u)-EKF||.
% With complete branch-and-bound, a returned distance is at most the
% caller's tolerance above the global closest forward-path distance.
% If the node budget is exhausted, return UNKNOWN and DO NOT certify.
%
% Output is GEOMETRY-ONLY, never permission to commit or fly. Collision
% freedom, cable/force constraints, and rejoin timing remain separate.
start=tic;
assert(isnumeric(nominal_controls)&&isreal(nominal_controls)&& ...
    size(nominal_controls,1)==3&&size(nominal_controls,2)>=2&& ...
    all(isfinite(nominal_controls(:)))&& ...
    isnumeric(ekf_position_m)&&isreal(ekf_position_m)&& ...
    isequal(size(ekf_position_m),[3 1])&&all(isfinite(ekf_position_m))&& ...
    isnumeric(progress_lower_u)&&isreal(progress_lower_u)&& ...
    isscalar(progress_lower_u)&&isfinite(progress_lower_u)&& ...
    progress_lower_u>=0&&progress_lower_u<=1, ...
    'TRAJECTORY_SPATIAL_REJOIN:Input','Finite path, EKF, progress required.');
assert(isstruct(options)&&isscalar(options)&& ...
    all(isfield(options,{'position_tolerance_m','max_node_count', ...
                         'numerical_guard_m'}))&& ...
    isnumeric(options.position_tolerance_m)&& ...
    isreal(options.position_tolerance_m)&& ...
    isscalar(options.position_tolerance_m)&& ...
    isfinite(options.position_tolerance_m)&& ...
    options.position_tolerance_m>=0&& ...
    isnumeric(options.numerical_guard_m)&& ...
    isreal(options.numerical_guard_m)&& ...
    isscalar(options.numerical_guard_m)&& ...
    isfinite(options.numerical_guard_m)&& ...
    options.numerical_guard_m>=0&& ...
    isnumeric(options.max_node_count)&& ...
    isreal(options.max_node_count)&& ...
    isscalar(options.max_node_count)&& ...
    isfinite(options.max_node_count)&& ...
    options.max_node_count>=1&& ...
    options.max_node_count==fix(options.max_node_count), ...
    'TRAJECTORY_SPATIAL_REJOIN:Budget','Explicit bound tolerance and node limit required.');
P=double(nominal_controls);x=double(ekf_position_m);
a=double(progress_lower_u);
R=struct('status','UNKNOWN_SPATIAL_SEARCH_NOT_FINISHED', ...
    'certified_spatial_nearest',false,'candidate_available',false, ...
    'safe_to_commit',false,'spatial_target_m',[NaN;NaN;NaN], ...
    'curve_parameter_u',NaN,'distance_upper_bound_m',Inf, ...
    'distance_lower_bound_m',0,'search_node_count',0, ...
    'target_time_s',[], ... % explicit absence of timed recovery
    'rejoin_timing_policy','FREE_TIME_SPATIAL_ONLY', ...
    'meaning','Geometric forward-path target; no dynamic, heading, obstacle, HLC or tracking certificate', ...
    'elapsed_ms',NaN);
if a==1
    R.candidate_available=true;
    R.certified_spatial_nearest=true;
    R.status='CERTIFIED_SPATIAL_ENDPOINT_ONLY__NO_HLC';
    R.spatial_target_m=P(:,end);
    R.curve_parameter_u=1;
    R.distance_upper_bound_m=norm(P(:,end)-x);
    R.distance_lower_bound_m=R.distance_upper_bound_m;
    R.elapsed_ms=toc(start)*1000;
    return
end
if a>0,[~,P]=split(P,a);end
% Exact endpoints give an initial feasible bound.
d0=norm(P(:,1)-x);d1=norm(P(:,end)-x);
if d0<=d1,best=d0;u=a;point=P(:,1);
else,best=d1;u=1;point=P(:,end);end
Q=struct('a',a,'b',1,'controls',P,'lb',lower_bound(P,x,options.numerical_guard_m));
nodes=0;tol=double(options.position_tolerance_m);
while ~isempty(Q)
    [small,k]=min([Q.lb]);
    if small>=best-tol
        R.certified_spatial_nearest=true;
        R.status='CERTIFIED_SPATIAL_NEAREST_WITH_TOLERANCE__NO_HLC';
        break
    end
    if nodes>=options.max_node_count,break;end
    node=Q(k);Q(k)=[];
    [A,B]=split(node.controls,0.5);
    midu=(node.a+node.b)*0.5;
    midp=A(:,end);
    d=norm(midp-x);
    if d<best,best=d;u=midu;point=midp;end
    child1=struct('a',node.a,'b',midu,'controls',A, ...
        'lb',lower_bound(A,x,options.numerical_guard_m));
    child2=struct('a',midu,'b',node.b,'controls',B, ...
        'lb',lower_bound(B,x,options.numerical_guard_m));
    % A box whose lower bound >= best-tolerance cannot improve accuracy.
    if child1.lb<best-tol,Q(end+1)=child1;end %#ok<AGROW>
    if child2.lb<best-tol,Q(end+1)=child2;end %#ok<AGROW>
    nodes=nodes+1;
end
if isempty(Q)
    R.certified_spatial_nearest=true;
    R.status='CERTIFIED_SPATIAL_NEAREST_WITH_TOLERANCE__NO_HLC';
    global_lower=max(0,best-tol);
else
    global_lower=min([Q.lb]);
end
R.candidate_available=true;
R.spatial_target_m=point;
R.curve_parameter_u=u;
R.distance_upper_bound_m=best;
R.distance_lower_bound_m=min(best,global_lower);
R.search_node_count=nodes;
R.elapsed_ms=toc(start)*1000;
end

function d=lower_bound(P,x,guard)
c=mean(P,2);
r=max(sqrt(sum((P-c).^2,1)));
d=max(0,norm(x-c)-r-guard);
end

function [L,R]=split(P,t)
n=size(P,2);V=P;L=zeros(size(P));R=zeros(size(P));
L(:,1)=V(:,1);R(:,end)=V(:,end);
for k=2:n
    V=(1-t)*V(:,1:end-1)+t*V(:,2:end);
    L(:,k)=V(:,1);R(:,n-k+1)=V(:,end);
end
end
