function O=AF7R58R117_DUAL_REPLACEMENT(action,request)
% NEW independent two-engine research implementation; NOT R115 production commit.
% action 'trigger': actual EKF-anchored HLC/plant rollout with externally
% CERTIFIED per-interval speed bounds. No legacy reference trigger fallback.
% action 'plan': general degree-15, position-C7 minimum eighth-derivative,
% oriented ellipsoid support seeds, all-segment yaw-C6, vehicle/cable/load
% continuous reference geometric/kinematic certificates. No hard maneuver
% duration cap, only bounded numerical SEARCH WORK. No fixed-clock rejoin.
% Both stages fail closed; never mark an uncertified candidate committable.
switch lower(char(action))
    case 'trigger'
        O=physical_trigger(request);
    case 'plan'
        O=plan_route(request);
    case 'decide'
        O=replan_decision(request);
    otherwise
        error('R117:UnknownAction','Expected trigger, plan or decide.');
end
end

function T=physical_trigger(A)
T=struct('status','BLOCKED_MISSING_PHYSICAL_CERTIFICATE', ...
 'request',false,'proved_safe',false,'certified_unsafe',false, ...
 'unknown',true,'first_inconclusive_interval',NaN, ...
 'obstacle_id',NaN,'body','','time_s',NaN, ...
 'checked_interval_count',0,'event_time_s',NaN, ...
 'reference_version',NaN,'old_reference_trigger_used',false, ...
 'production_commit_authorized',false);
fields={'event_time_s','reference_version','EKF_x24','EKF_mL','horizon_s', ...
 'dt_s','rollout','known_obstacles','radii_m'};
require_fields(A,fields,'R117:MissingTriggerInput');
x=double(A.EKF_x24(:));
assert(numel(x)==24&&all(isfinite(x)),'R117:EKF24');
assert(isscalar(A.EKF_mL)&&isfinite(A.EKF_mL)&&A.EKF_mL>0, ...
 'R117:EstimatedMassRequired');
assert(isscalar(A.horizon_s)&&A.horizon_s>0 && ...
 isscalar(A.dt_s)&&A.dt_s>0, 'R117:LiveHorizonAndDt');
T.event_time_s=double(A.event_time_s);
T.reference_version=double(A.reference_version);
obs=A.known_obstacles;
assert(isstruct(obs),'R117:KnownObstaclesStruct');
validate_obstacles(obs);
r=double(A.radii_m(:));
assert(isequal(size(r),[3 1])&&all(isfinite(r))&&all(r>=0), ...
 'R117:SeparateBodyRadii');
if isempty(obs)
    T.status='NO_SENSOR_KNOWN_OBSTACLES';
    T.unknown=false;T.proved_safe=true;return;
end
P=A.rollout;
if ~isstruct(P)||~isscalar(P)||~all(isfield(P, ...
 {'source','time_s','reference_version','pQ','pL','t_s','EKF_mL'}))
    T.status='BLOCKED_EXACT_HLC_PLANT_ROLLOUT_MISSING';return;
end
if ~strcmp(char(P.source),'LIVE_EKF_HLC_PLANT_ROLLOUT') || ...
 P.time_s~=A.event_time_s||P.reference_version~=A.reference_version|| ...
 P.EKF_mL~=A.EKF_mL
    T.status='BLOCKED_ROLLOUT_PROVENANCE_OR_MASS';return;
end
pQ=double(P.pQ);pL=double(P.pL);tt=double(P.t_s(:).');
N=numel(tt);H=double(A.horizon_s);dt=double(A.dt_s);
if N<2||~isequal(size(pQ),[3,N])||~isequal(size(pL),[3,N]) || ...
 any(~isfinite([pQ(:);pL(:);tt(:)]))|| ...
 abs(tt(1)-A.event_time_s)>128*eps(max(1,abs(tt(1))))|| ...
 abs(tt(end)-(tt(1)+H))>128*eps(max(1,abs(tt(end))))|| ...
 any(diff(tt)<=0)||max(diff(tt))>dt+128*eps(max(1,dt))
    T.status='BLOCKED_INCOMPLETE_OR_STALE_TIME_GRID';return;
end
seam=128*eps(max([1;abs(x(:))]));
if any(abs(pQ(:,1)-x(1:3))>seam)|| ...
 any(abs(pL(:,1)-x(13:15))>seam)
    T.status='BLOCKED_ROLLOUT_NOT_EKF_INITIALIZED';return;
end
% A sampled future cannot PROVE intersample clearance. Certified uniform
% speed bounds must come from an independently validated continuous dynamics
% enclosure (not finite-difference speeds or a chosen tuning constant).
if ~all(isfield(P,{'interval_speed_upper_m_s','bound_certificate'}))
    T.status='BLOCKED_NO_CONTINUOUS_VELOCITY_ENCLOSURE';return;
end
C=P.bound_certificate;
if ~isstruct(C)||~isscalar(C)||~all(isfield(C, ...
 {'certified','event_time_s','reference_version','method'}))|| ...
 ~isequal(C.certified,true)||C.event_time_s~=A.event_time_s|| ...
 C.reference_version~=A.reference_version||isempty(char(C.method))
    T.status='BLOCKED_VELOCITY_ENCLOSURE_UNAUTHENTICATED';return;
end
V=double(P.interval_speed_upper_m_s);
if ~isequal(size(V),[2,N-1])||any(~isfinite(V(:)))||any(V(:)<0)
    T.status='BLOCKED_INVALID_SPEED_BOUNDS';return;
end
for j=1:N-1
    h=tt(j+1)-tt(j);
    % Continuous-time Lipschitz enclosures: for any t in this interval,
    % ||pQ(t)-pQ(t_j)|| <= VQ*h and analogous payload/cable bound.
    eQ=V(1,j)*h;eL=V(2,j)*h;eC=max(eQ,eL);
    q=pQ(:,j);l=pL(:,j);
    for k=1:numel(obs)
        o=obs(k);
        [okL,~]=separation(l,l,o,r(3)+eL,'point');
        [okQ,~]=separation(q,q,o,r(1)+eQ,'point');
        [okC,~]=separation(l,q,o,r(2)+eC,'cable');
        if ~(okL&&okQ&&okC)
            % INCONCLUSIVE does not assert real collision, but requires
            % new planning/closer inspection; never silently returns safe.
            T.status='REPLAN_REQUIRED__CONTINUOUS_CLEARANCE_NOT_PROVED';
            T.request=true;T.unknown=true;
            T.first_inconclusive_interval=j;
            T.obstacle_id=double(o.id);
            if ~okL,T.body='PAYLOAD';
            elseif ~okQ,T.body='UAV';else,T.body='CABLE';end
            T.time_s=tt(j);
            T.checked_interval_count=j;
            return;
        end
    end
    T.checked_interval_count=j;
end
T.status='CONDITIONAL_CONTINUOUS_CLEAR__REQUIRES_VERIFIED_EXTERNAL_BOUNDS';
T.proved_safe=true;T.unknown=false;
end

function R=plan_route(A)
R=struct('status','NO_CERTIFIED_REFERENCE_CANDIDATE', ...
 'found',false,'reference_certified',false, ...
 'execution_certified',false,'production_commit_authorized',false, ...
 'route',struct([]),'certificate',struct([]),'attempts',0, ...
 'selected_duration_s',NaN,'selected_nominal_phase_s',NaN, ...
 'known_obstacle_count',0,'old_generator_used',false, ...
 'continuity_order',7,'yaw_order',6,'stop_or_singular_yaw_rejected',false);
fields={'time_s','start_jets07','nominal_jets07_fn', ...
 'known_obstacles','live','search_work_limit','branch_work_limit', ...
 'certificate_work_limit','duration_scale_s','radii_m','max_uav_speed_m_s'};
require_fields(A,fields,'R117:MissingPlannerAuthority');
assert(isa(A.nominal_jets07_fn,'function_handle'), ...
 'R117:LiveNominalJetFunctionRequired');
start=double(A.start_jets07);
assert(isequal(size(start),[3 8])&&all(isfinite(start(:))), ...
 'R117:StartJets07');
obs=A.known_obstacles;validate_obstacles(obs);
R.known_obstacle_count=numel(obs);
assert(~isempty(obs),'R117:NoTriggeredKnownObstacle');
liveKeys={'cable_length_m','gravity_m_s2','latest_EKF_mL_kg', ...
 'vehicle_mass_kg','thrust_bounds_N','torque_bounds_Nm'};
require_fields(A.live,liveKeys,'R117:MissingLivePhysicalAuthority');
assert(isfinite(A.live.latest_EKF_mL_kg)&&A.live.latest_EKF_mL_kg>0&& ...
 isfinite(A.live.vehicle_mass_kg)&&A.live.vehicle_mass_kg>0, ...
 'R117:LiveEstimatedLoadMassAndUAVMassRequired');
L=double(A.live.cable_length_m);g=double(A.live.gravity_m_s2);
r=double(A.radii_m(:));
assert(isequal(size(r),[3 1]) && all(isfinite(r))&&all(r>=0), ...
 'R117:SeparateRadii');
assert(isfinite(L)&&L>0&&isfinite(g)&&g>0, ...
 'R117:ExternalPhysicalAuthority');
for field={'search_work_limit','branch_work_limit','certificate_work_limit'}
    z=double(A.(field{1}));
    assert(isscalar(z)&&isfinite(z)&&z>=1&&z==fix(z), ...
     'R117:PositiveFiniteSolverWorkBudget');
end
assert(isscalar(A.duration_scale_s)&&A.duration_scale_s>0 && ...
 isfinite(A.duration_scale_s)&&isfinite(A.max_uav_speed_m_s)&& ...
 A.max_uav_speed_m_s>0,'R117:LivePhysicalTimeAndSpeed');
% These are computational search samples, not an upper bound on maneuver
% duration. In successive calls the search may be resumed with more work.
ratios=[1/4 1/2 1];
for attempt=1:double(A.search_work_limit)
    level=floor((attempt-1)/numel(ratios));
    fraction=ratios(mod(attempt-1,numel(ratios))+1);
    T=double(A.duration_scale_s)*2^level;
    if ~isfinite(T)||~isfinite(A.time_s+T),break;end
    phaseAdvance=T*fraction;
    nominalTime=double(A.time_s)+phaseAdvance;
    terminal=double(A.nominal_jets07_fn(nominalTime));
    if ~isequal(size(terminal),[3 8])||any(~isfinite(terminal(:)))
        R.status='MISSING_NOMINAL_PHASE_JETS';return;
    end
    seed=AF7R58R92_SUPPORT_SEED_ROUTES(start(:,1),terminal(:,1), ...
       obs,max(r),double(A.branch_work_limit));
    if isempty(seed.routes),R.attempts=attempt;continue;end
    for branch=1:numel(seed.routes)
        waypoints=[start(:,1),seed.routes{branch},terminal(:,1)];
        % Independent natural segment times, based on segment arc lengths
        % of the seed polygon; no special circular/path assumptions.
        lengths=vecnorm(diff(waypoints,1,2),2,1);
        if any(lengths<=0),continue;end
        knot=[0,cumsum(lengths)/sum(lengths)]*T+double(A.time_s);
        route=AF7R58R92_C7_MINIMUM8_ROUTE(knot,waypoints, ...
          start,terminal,T-phaseAdvance,L);
        if ~route.reference_candidate_found,continue;end
        cert=cert_route(route,obs,g,L,r,A.max_uav_speed_m_s, ...
          double(A.certificate_work_limit));
        if ~cert.all_pass,continue;end
        % Reference-only result. In particular a C7 reference and geometry
        % do not imply HLC/plant tracking/input feasibility.
        R.status='C7_REFERENCE_NUMERIC_WHOLE_BODY_CERTIFIED__EXECUTION_PENDING';
        R.found=true;R.reference_certified=true;
        R.route=route;R.certificate=cert;R.attempts=attempt;
        R.selected_duration_s=T;R.selected_nominal_phase_s=phaseAdvance;
        return;
    end
    R.attempts=attempt;
end
R.status='NO_PROVED_C7_ROUTE_WITHIN_COMPUTATION_BUDGET';
end

function B=cert_route(route,obs,g,L,r,vLimit,maxNodes)
B=struct('all_pass',false,'geometry_pass',false, ...
 'yaw_c6_pass',false,'speed_pass',false,'details',struct([]), ...
 'input_tension_certified',false,'tracking_tube_certified',false, ...
 'status','NOT_PROVED');
D=repmat(struct('geom',[],'yaw',[],'speed',[]),1,numel(route.segments));
for i=1:numel(route.segments)
    seg=route.segments(i);
    G=AF7R58R80_CONTINUOUS_SEPARATE_BODY_CERT( ...
        seg,obs,g,L,r(1),r(2),r(3),maxNodes);
    K=AF7R58R91_STABLE_KINEMATIC_CERT( ...
        seg,g,L,vLimit,maxNodes);
    Y=heading_check(seg.payload_cp,seg.duration_s,maxNodes);
    D(i)=struct('geom',G,'yaw',Y,'speed',K);
    if ~(G.certified&&K.certified&&Y.certified)
        B.details=D;B.status='SEGMENT_REFERENCE_CERTIFICATE_INCONCLUSIVE';return;
    end
end
B.details=D;B.geometry_pass=true;B.yaw_c6_pass=true;
B.speed_pass=true;B.all_pass=true;
B.status='ALL_SEGMENTS_REFERENCE_CONTINUOUS_CERTIFIED';
end

function Y=heading_check(P,T,maxNodes)
% p is global C7; p'(t) is C6. If the XY speed never vanishes, atan2
% of the tangent is C6 in any locally unwrapped angular chart.
% For each leaf, a separating positive projection n'*p'_xy > 0 is a
% SUFFICIENT exact Bernstein certificate of nonzero horizontal speed.
Y=struct('certified',false,'status','HEADING_TANGENT_UNKNOWN', ...
 'nodes',0,'minimum_projection_m_s',Inf);
D=(size(P,2)-1)/T*diff(P(1:2,:),1,2);
stack={D};
while ~isempty(stack)
    if Y.nodes>=maxNodes
        Y.status='HEADING_PROJECTION_WORK_BUDGET';return;
    end
    F=stack{end};stack(end)=[];Y.nodes=Y.nodes+1;
    a=sum(F,2);na=norm(a);
    if na>0
        n=a/na;lower=min(n'*F);
        guard=256*eps(max([1;abs(F(:))]));
        if lower>guard
            Y.minimum_projection_m_s=min(Y.minimum_projection_m_s,lower-guard);
            continue;
        end
    end
    if all(F(:)==0)
        Y.status='STOP_REQUIRES_SEPARATE_YAW_HOLD_CERTIFICATE';return;
    end
    [L,R]=split_half(F);
    if isequal(L,F)||isequal(R,F)
        Y.status='HEADING_FLOAT_RESOLUTION';return;
    end
    stack{end+1}=R;stack{end+1}=L;
end
Y.certified=true;Y.status='PATH_TANGENT_YAW_C6_FROM_NONZERO_XY_SPEED';
end

function [A,B]=split_half(P)
n=size(P,2);A=zeros(size(P));B=zeros(size(P));
A(:,1)=P(:,1);B(:,end)=P(:,end);
for j=2:n
    P=0.5*(P(:,1:end-1)+P(:,2:end));
    A(:,j)=P(:,1);B(:,n-j+1)=P(:,end);
end
end

function [pass,best]=separation(u,v,obs,radius,whichBody)
% Analytic support of arbitrary oriented static ellipsoid:
% h_E(n)=n'*c+||diag(a)*R'*n||. For a cable, BOTH endpoints must satisfy
% the same plane so EVERY lambda in [0,1] is separated.
pass=false;best=-Inf;c=double(obs.center(:));
w=v-u;ww=w'*w;
if strcmp(whichBody,'cable')&&ww>0
    lambda=max(0,min(1,((c-u)'*w)/ww));p=u+lambda*w;
else
    p=u;
end
N=[u-c,v-c,p-c];
for k=1:size(N,2)
    n=N(:,k);nn=norm(n);
    if nn==0||~isfinite(nn),continue;end
    n=n/nn;
    h=dot(n,c)+norm(double(obs.axes(:)).*(double(obs.R)'*n));
    margin=min(dot(n,u),dot(n,v))-h-radius;
    margin=margin-256*eps(max([1;abs([u;v;c]);abs(h);radius]));
    best=max(best,margin);
    if margin>0,pass=true;return;end
end
end

function O=replan_decision(A)
% All decisions pertain to the CURRENT EKF-initialized forecast, rather
% than obstacle-identity-change alone. Same obstacle can replan only when
% current accepted trajectory is no longer certified safe/feasible.
require_fields(A,{'physical_trigger','active_plan_certificate', ...
 'same_reference_version'},'R117:DecisionPacket');
T=A.physical_trigger;C=A.active_plan_certificate;
O=struct('replan',false,'status','', 'keep_active',false, ...
 'same_obstacle_repeat_due_to_detection_only',false);
if T.proved_safe && isstruct(C) && ...
    isfield(C,'still_feasible') && isequal(C.still_feasible,true) && ...
    isequal(A.same_reference_version,true)
    O.keep_active=true;
    O.status='KEEP_ACTIVE_CERTIFIED_SAFE_NO_DUPLICATE_REPLAN';
elseif T.request
    O.replan=true;O.status='REPLAN_GENUINE_FUTURE_GEOMETRIC_RISK';
else
    O.status='UNKNOWN_PHYSICAL_SAFETY_FAIL_CLOSED_NO_NEW_COMMIT';
end
end

function require_fields(S,fields,id)
assert(isstruct(S)&&isscalar(S)&&all(isfield(S,fields)),id, ...
 'Missing required runtime source inputs: %s',strjoin(fields,', '));
end
function validate_obstacles(obs)
assert(isstruct(obs),'R117:ObstacleType');
for k=1:numel(obs)
    o=obs(k);
    require_fields(o,{'id','center','R','axes'},'R117:ObstacleAuthority');
    c=double(o.center(:));a=double(o.axes(:));M=double(o.R);
    assert(isequal(size(c),[3,1])&&isequal(size(a),[3,1])&& ...
     isequal(size(M),[3,3])&&all(isfinite([c;a;M(:)]))&& ...
     all(a>0)&&norm(M'*M-eye(3),'fro')<1e-9&& ...
     abs(det(M)-1)<1e-9,'R117:InvalidOrientedEllipsoid');
end
end
