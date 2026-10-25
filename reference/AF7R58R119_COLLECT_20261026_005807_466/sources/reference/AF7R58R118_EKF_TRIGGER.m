function T=AF7R58R118_EKF_TRIGGER(A)
% Independent physical trigger. NEVER invokes old reference-trigger kernel.
% Samples are a risk WITNESS, never a continuous-time safe certificate.
% A certified external differential enclosure must be supplied to declare
% 4-s clearance. Without one, request is UNKNOWN/PROTECTIVE (not 'safe').
% Current EKF endpoints are used directly, without imposing taut geometry.
req={'event_time_s','reference_version','EKF_x24','EKF_mL', ...
    'horizon_s','dt_s','known_obstacles','radii_m','rollout'};
for i=1:numel(req)
    assert(isfield(A,req{i}),'R118:MissingEKFTriggerAuthority', ...
        'Missing runtime input %s',req{i});
end
T=struct('status','UNKNOWN','request',false,'proved_clear',false, ...
    'reference_trigger_used',false,'physical_full_interval_proved',false, ...
    'sampled_risk_witness',false,'uncertain_interval',NaN, ...
    'obstacle_id',NaN,'body','','relative_time_s',NaN, ...
    'reference_version',A.reference_version, ...
    'production_commit_authorized',false, ...
    'continuous_proof_conditioned_on_external_bound_provider',true);
if isempty(A.known_obstacles)
    T.status='NO_SENSOR_KNOWN_OBSTACLE__NOT_GLOBAL_SAFETY';
    T.proved_clear=true;return;
end
P=A.rollout;x=double(A.EKF_x24(:));
assert(numel(x)==24&&all(isfinite(x))&& ...
    isscalar(A.EKF_mL)&&isfinite(A.EKF_mL)&&A.EKF_mL>0, ...
    'R118:SameTickEKF24AndMass');
if ~isstruct(P)||~all(isfield(P, ...
        {'source','time_s','reference_version','pQ','pL','t_s','EKF_mL'}))
    T.status='BLOCKED_NO_EKF_INITIALIZED_PHYSICAL_HLC_ROLLOUT';
    T.request=true;return;
end
if ~strcmp(char(P.source),'LIVE_EKF_HLC_PLANT_ROLLOUT')|| ...
    P.time_s~=A.event_time_s||P.reference_version~=A.reference_version|| ...
    P.EKF_mL~=A.EKF_mL
    T.status='BLOCKED_STALE_OR_NOT_PHYSICAL_ROLLOUT';T.request=true;return;
end
t=double(P.t_s(:).');N=numel(t);q=double(P.pQ);l=double(P.pL);
if N<2||~isequal(size(q),[3,N])||~isequal(size(l),[3,N])|| ...
    any(~isfinite([t(:);q(:);l(:)]))||any(diff(t)<=0)|| ...
    abs(t(1)-A.event_time_s)>128*eps(max(1,abs(t(1))))|| ...
    abs(t(end)-t(1)-A.horizon_s)>128*eps(max(1,abs(t(end))))|| ...
    max(diff(t))>A.dt_s+128*eps(max(1,A.dt_s))
    T.status='BLOCKED_INCOMPLETE_4S_HLC_ROLLOUT';T.request=true;return;
end
seam=256*eps(max([1;abs(x(:))]));
if any(abs(q(:,1)-x(1:3))>seam)|| ...
        any(abs(l(:,1)-x(13:15))>seam)
    T.status='BLOCKED_EKF_ROLLOUT_SEAM_MISMATCH';T.request=true;return;
end
r=double(A.radii_m(:));
assert(numel(r)==3&&all(isfinite(r))&&all(r>=0), ...
    'R118:BodyRadiiMustComeFromLiveSystem');
% First, direct same-event physical sample risk witnesses. These do NOT
% certify contact or clearance between samples. Unknown => request.
for j=1:N
    for k=1:numel(A.known_obstacles)
        ob=A.known_obstacles(k);
        bodies={q(:,j),q(:,j),'UAV',r(1); ...
                l(:,j),l(:,j),'PAYLOAD',r(3); ...
                l(:,j),q(:,j),'CABLE',r(2)};
        for b=1:3
            if ~separated(bodies{b,1},bodies{b,2},ob,bodies{b,4})
                T.status='PHYSICAL_SAMPLE_UNSEPARATED__REQUEST_NOT_COLLISION_PROOF';
                T.request=true;T.sampled_risk_witness=true;
                T.obstacle_id=double(ob.id);T.body=bodies{b,3};
                T.relative_time_s=t(j)-t(1);T.uncertain_interval=max(1,j-1);
                return;
            end
        end
    end
end
% Uniform speed bounds for the exact physical continuous trajectory
% permit the R117 conservative interval proof. This data must be sourced
% from validated reachable-state enclosures (not numerical differences).
if isfield(P,'interval_speed_upper_m_s')&& ...
        isfield(P,'bound_certificate')
    R=AF7R58R117_DUAL_REPLACEMENT('trigger',A);
    if R.proved_safe
        T.status='CONDITIONAL_PHYSICAL_4S_CLEAR__VERIFY_EXTERNAL_BOUND_PROVIDER';
        T.proved_clear=true;T.physical_full_interval_proved=true;
        return;
    elseif R.request
        T.status='PHYSICAL_INTERVAL_NOT_SEPARATED__REPLAN_REQUIRED';
        T.request=true;T.obstacle_id=R.obstacle_id;
        T.body=R.body;T.relative_time_s=R.time_s-A.event_time_s;
        T.uncertain_interval=R.first_inconclusive_interval;return;
    end
    T.status=char(R.status);
else
    T.status='PHYSICAL_NUMERIC_ROLLOUT_ONLY__MISSING_INTERVAL_PROOF';
end
T.request=true; % Unknown physical clearance is NOT treated as a safe active plan.
end
function ok=separated(u,v,o,r)
% Robustly SUFFICIENT support-plane separation of static SO(3) ellipsoid.
% No separating plane found ==> unknown/non-separated, not a proven impact.
assert(isfield(o,'center')&&isfield(o,'R')&&isfield(o,'axes')&& ...
    isfield(o,'id'),'R118:KnownOrientedEllipsoidRequired');
c=double(o.center(:));R=double(o.R);a=double(o.axes(:));
assert(isequal(size(c),[3 1])&&isequal(size(R),[3 3])&& ...
    isequal(size(a),[3 1])&&all(isfinite([c;a;R(:)]))&& ...
    all(a>0)&&norm(R'*R-eye(3),'fro')<1e-9&& ...
    abs(det(R)-1)<1e-9,'R118:InvalidStaticOrientedEllipsoid');
U=v-u;den=U'*U;
if den>0,p=u+max(0,min(1,((c-u)'*U)/den))*U;
else,p=u;end
N=[u-c,v-c,p-c,R,-R];ok=false;
for k=1:size(N,2)
    n=N(:,k);z=norm(n);
    if z==0||~isfinite(z),continue;end
    n=n/z;
    support=n'*c+norm(a.*(R'*n));
    margin=min(n'*u,n'*v)-support-r;
    guard=512*eps(max([1;abs(u);abs(v);abs(c);abs(support);r]));
    if margin>guard
        ok=true;return;
    end
end
end
