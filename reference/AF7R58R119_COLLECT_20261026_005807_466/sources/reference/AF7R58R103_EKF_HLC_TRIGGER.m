function T = AF7R58R103_EKF_HLC_TRIGGER(F,P)
% Dual-trigger physical preview interface. F is one immutable Flight event
% packet captured from EKF and the SENSOR-KNOWN ellipsoid packet. P is the
% full, SAME-EVENT, current-EKF-initialized HLC+plant rollout. This code
% never converts reference-only prediction into a physical forecast.
% The existing MEX must retain its conservative sweep-inflation authority.
T = struct('status','INCONCLUSIVE_PHYSICAL_PREVIEW_UNAVAILABLE', ...
    'request',false,'complete_preview',false,'triggered',false, ...
    'collision_ttc_s',NaN,'collision_obstacle_id',NaN, ...
    'collision_component_code',NaN,'physical_forecast_safety_proved',false, ...
    'commit_authorized',false,'time_s',NaN,'reference_version',NaN, ...
    'known_mask',false(1,0),'reason','');
assert(isstruct(F)&&isscalar(F),'R103:FlightFrameRequired', ...
    'Supply one Flight EKF and sensed-obstacle frame.');
names={'time_s','reference_version','horizon_s','dt_s','cable_length_m', ...
    'pL_EKF','pQ_EKF','known_mask','C','R','Araw','ids','Dcomp'};
for i=1:numel(names)
    assert(isfield(F,names{i}),'R103:MissingFlightFrameField', ...
        'Missing Flight frame field %s.',names{i});
end
H=double(F.horizon_s);dt=double(F.dt_s);L=double(F.cable_length_m);
assert(isscalar(H)&&H==4&&isscalar(dt)&&dt>0&&isfinite(dt)&& ...
    isscalar(L)&&isfinite(L)&&L>0&& ...
    abs(H/dt-round(H/dt))<1e-9,'R103:InvalidFixedHorizon', ...
    'Exactly 4 s and the live prediction dt are required.');
T.time_s=double(F.time_s);T.reference_version=double(F.reference_version);
assert(isscalar(T.time_s)&&isfinite(T.time_s)&& ...
    isscalar(T.reference_version)&&isfinite(T.reference_version)&& ...
    T.reference_version==fix(T.reference_version), ...
    'R103:TimeAndVersionRequired','Event time and reference version required.');
a=double(F.pL_EKF(:));b=double(F.pQ_EKF(:));
assert(numel(a)==3&&numel(b)==3&&all(isfinite([a;b]))&& ...
    abs(norm(a-b)-L)<=1e-5*max(1,L), ...
    'R103:EKFEndpoints','EKF payload/UAV endpoints must match live cable length.');
mask=logical(F.known_mask(:).');T.known_mask=mask;
C=double(F.C);R=double(F.R);A=double(F.Araw);ids=double(F.ids(:).');
M=numel(ids);D=double(F.Dcomp(:));
assert(isequal(size(C),[3,M])&&isequal(size(R),[9,M])&& ...
    isequal(size(A),[3,M])&&numel(mask)==M&&numel(D)==3&& ...
    all(isfinite([C(:);R(:);A(:);ids(:);D(:)]))&& ...
    all(A(:)>0)&&all(D>=0), ...
    'R103:KnownObstaclePacket','Invalid sensor-known oriented ellipsoid packet.');
if ~any(mask)
    T.status='NO_SENSOR_KNOWN_OBSTACLE';
    T.complete_preview=false; % absence of seen obstacles is not global safety
    return;
end
if nargin<2||isempty(P)
    T.status='BLOCKED_NO_EKF_INITIALIZED_HLC_PLANT_PREVIEW';
    return;
end
assert(isstruct(P)&&isscalar(P),'R103:PhysicalPredictionStruct', ...
    'HLC physical forecast must be one struct.');
needed={'time_s','reference_version','pL','pQ','source'};
for i=1:numel(needed)
    assert(isfield(P,needed{i}),'R103:PhysicalPredictionMissingField', ...
        'Physical preview lacks %s.',needed{i});
end
if ~(isequal(double(P.time_s),T.time_s)&& ...
        isequal(double(P.reference_version),T.reference_version))
    T.status='BLOCKED_STALE_HLC_PREVIEW';return;
end
if ~strcmp(char(P.source),'LIVE_EKF_HLC_PLANT_ROLLOUT')
    T.status='BLOCKED_NOT_PHYSICAL_HLC_PLANT';return;
end
pL=double(P.pL);pQ=double(P.pQ);N=round(H/dt)+1;
assert(isequal(size(pL),[3,N])&&isequal(size(pQ),[3,N])&& ...
    all(isfinite([pL(:);pQ(:)])), ...
    'R103:PreviewDimensions','Physical preview must cover the complete fixed 4 s horizon.');
% Exact same time/event. Ref-table packets can never pass this EKF seam.
seamTol=256*eps(max([1;abs([a;b])]));
if any(abs(pL(:,1)-a)>seamTol)||any(abs(pQ(:,1)-b)>seamTol)
    T.status='BLOCKED_PREVIEW_NOT_EKF_ANCHORED';return;
end
q=(pL-pQ)/L;
if any(abs(vecnorm(q,2,1)-1)>1e-6)
    T.status='BLOCKED_PHYSICAL_CABLE_INCONSISTENT';return;
end
if exist('g26_future_trigger_kernel_mex','file')~=3
    T.status='BLOCKED_NATIVE_TRIGGER_KERNEL_MISSING';return;
end
[hit,j,id,comp]=g26_future_trigger_kernel_mex( ...
    H,dt,L,pL,q,C,R,A,D,ids,double(mask));
T.complete_preview=true;
T.triggered=logical(hit);T.request=logical(hit);
if T.triggered
    assert(isscalar(j)&&j>=1&&j<=N&&j==fix(j), ...
        'R103:InvalidNativeHitIndex','Native trigger returned an invalid sample.');
    T.collision_ttc_s=(double(j)-1)*dt;
    T.collision_obstacle_id=double(id);
    T.collision_component_code=double(comp);
    T.status='PHYSICAL_PREVIEW_REPLAN_REQUEST__NOT_A_SAFETY_PROOF';
else
    T.status='PHYSICAL_PREVIEW_NO_NATIVE_HIT__NOT_A_CONTINUOUS_PROOF';
end
% A numerical HLC rollout and MEX reference sweep are not a certified
% reachability tube / actuator / IEEE outward interval proof.
end
