function [F,D]=AF7R58R106_SOURCE_EVENT(O,eventTime,expectedVersion)
% Read immutable R106 SAME-TICK R88/EKF snapshot, never a nominal surrogate.
% Returned F is the same packet required by AF7R58R103_EKF_HLC_TRIGGER.
F=struct([]);
D=struct('status','NO_AUTHENTIC_SAME_TICK_EKF', ...
    'source','NONE','time_s',double(eventTime), ...
    'reference_version',double(expectedVersion), ...
    'r88_snapshot_present',false,'ekf_capture_status','NOT_FOUND', ...
    'authenticated',false,'raw_state_present',false, ...
    'geometry_residual_m',NaN,'pT_norm_error',NaN, ...
    'predictor_attached',false, ...
    'production_commit_authorized',false);
assert(isscalar(eventTime)&&isfinite(eventTime)&& ...
    isscalar(expectedVersion)&&isfinite(expectedVersion), ...
    'R106:FiniteIdentity','Exact source event identity required.');
assert(isstruct(O)||isobject(O), ...
    'R106:SourceType','Need live REPLANNING_SYSTEM or source struct.');
if isstruct(O)
    assert(isfield(O,'r88_snapshots'),'R106:MissingSnapshotLedger', ...
        'R88 source snapshot ledger required.');
else
    assert(isprop(O,'r88_snapshots'),'R106:MissingSnapshotLedger', ...
        'R88 source snapshot ledger required.');
end
S=O.r88_snapshots;
if isempty(S)
    D.status='R88_SNAPSHOT_LEDGER_EMPTY';return;
end
assert(isstruct(S)&&all(isfield(S,{'time_s','reference_version'})), ...
    'R106:SnapshotSchema','Invalid immutable R88 source ledger.');
idx=find([S.time_s]==double(eventTime) & ...
    [S.reference_version]==double(expectedVersion));
if isempty(idx)
    D.status='NO_R88_SNAPSHOT_WITH_EXACT_TIME_VERSION';return;
elseif numel(idx)~=1
    D.status='AMBIGUOUS_R88_EVENT_IDENTITY';return;
end
Z=S(idx);D.r88_snapshot_present=true;
if ~isfield(Z,'ekf_event')||~isstruct(Z.ekf_event)|| ...
        ~isscalar(Z.ekf_event)
    D.status='OLDER_R88_WITHOUT_EKF_EVENT';return;
end
E=Z.ekf_event;
if isfield(E,'status'),D.ekf_capture_status=char(E.status);end
requiredE={'available','time_s','reference_version','known_mask', ...
    'state_x24'};
if ~all(isfield(E,requiredE))
    D.status='R106_EKF_CAPTURE_UNAVAILABLE';return;
end
% A producer's cached "available" flag is NOT an EKF measurement and
% must not be the gate.  The previous producer checked attitude q instead
% of physical cable direction pT.  Re-assess the immutable, full raw x24
% independently, with original event identity, obstacle packet, and all
% physical consistency checks.  No raw x24 => never reconstruct from a ref.
legacyMisclassified=~logical(E.available);
D.raw_state_present=logical(isnumeric(E.state_x24) && ...
    numel(E.state_x24)==24 && all(isfinite(double(E.state_x24(:)))));
if ~D.raw_state_present
    D.status='R106_RAW_EKF_24_NOT_SAVED';return;
end
if ~isequal(double(E.time_s),double(Z.time_s))|| ...
   ~isequal(double(E.reference_version),double(Z.reference_version))|| ...
   ~isequal(logical(E.known_mask(:).'),logical(Z.known_mask(:).'))
    D.status='R106_EKF_SNAPSHOT_IDENTITY_MISMATCH';return;
end
x=double(E.state_x24(:)); L=double(Z.cable_length_m);
if ~(isfinite(L)&&isscalar(L)&&L>0)
    D.status='R106_LIVE_CABLE_LENGTH_INVALID';return;
end
D.pT_norm_error=abs(norm(x(19:21))-1);
D.geometry_residual_m=norm(x(13:15)-x(1:3)-L*x(19:21));
if D.pT_norm_error>1e-5
    D.status='R106_EKF_PT_NONUNIT__PHYSICAL_TRIGGER_BLOCKED';return;
end
if D.geometry_residual_m>1e-5*max(1,L)
    % An EKF has observer error.  Do NOT fix an inconsistent EKF by
    % moving either endpoint, normalizing pT, or inventing a tracking tube.
    D.status='R106_EKF_CABLE_RESIDUAL_NEEDS_OBSERVER_TUBE';return;
end
mandatory={'obstacle_packet','known_mask','horizon_s','prediction_dt_s', ...
    'rL_m','rQ_m','rC_m','d_margin_m','estimated_mass_kg'};
if ~all(isfield(Z,mandatory))
    D.status='R106_MISSING_LIVE_GEOMETRY_AUTHORITY';return;
end
Q=Z.obstacle_packet;
if ~isstruct(Q)||~isscalar(Q)|| ...
    ~all(isfield(Q,{'C','R','Araw','ids','delta_payload_m', ...
        'delta_uav_m','delta_cable_m'}))
    D.status='R106_MALFORMED_ELLIPSOID_PACKET';return;
end
ids=double(Q.ids(:).');M=numel(ids);
mask=logical(Z.known_mask(:).');
C=double(Q.C);R=double(Q.R);axes=double(Q.Araw);
if ~(numel(mask)==M && isequal(size(C),[3 M]) && ...
     isequal(size(R),[9 M]) && isequal(size(axes),[3 M]) && ...
     all(isfinite([C(:);R(:);axes(:);ids(:)])) && all(axes(:)>0) && ...
     any(mask) && numel(unique(ids))==M)
    D.status='R106_ELLIPSOID_MASK_OR_DIMENSION_INVALID';return;
end
for k=1:M
    Qk=reshape(R(:,k),3,3);
    if norm(Qk'*Qk-eye(3),'fro')>1e-6 || det(Qk)<0
        D.status='R106_ELLIPSOID_ROTATION_INVALID';return;
    end
end
radii=[double(Z.rL_m)+double(Z.d_margin_m)+double(Q.delta_payload_m); ...
       double(Z.rQ_m)+double(Z.d_margin_m)+double(Q.delta_uav_m); ...
       double(Z.rC_m)+double(Z.d_margin_m)+double(Q.delta_cable_m)];
if numel(radii)~=3||any(~isfinite(radii))||any(radii<0)|| ...
        ~(isscalar(Z.estimated_mass_kg)&&Z.estimated_mass_kg>0 && ...
        isfinite(Z.estimated_mass_kg))
    D.status='R106_INFLATION_OR_MASS_AUTHORITY_INVALID';return;
end
F=struct('time_s',double(eventTime), ...
    'reference_version',double(expectedVersion), ...
    'horizon_s',double(Z.horizon_s), ...
    'dt_s',double(Z.prediction_dt_s), ...
    'cable_length_m',L,'pL_EKF',x(13:15), ...
    'pQ_EKF',x(1:3),'q_EKF',x(19:21), ...
    'state_x24',x, ...
    'known_mask',mask,'C',C,'R',R,'Araw',axes, ...
    'ids',ids,'Dcomp',radii,'estimated_mass_kg',double(Z.estimated_mass_kg), ...
    'source','R106_R88_SAME_TICK_EKF_IMMUTABLE');
D.status='EXACT_R88_EKF_KNOWN_SET_EVENT_AUTHENTICATED';
if legacyMisclassified
    D.status='EXACT_RAW_R88_EKF_INDEPENDENTLY_REVALIDATED';
end
D.source='R106_R88_SAME_TICK_EKF_IMMUTABLE';
D.authenticated=true;
% A real same-event state is necessary but NOT a physical 4-s future.
end
