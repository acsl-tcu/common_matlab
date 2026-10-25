function B = AF7R58R83_BIND_R81_INPUTS(S)
% R83: Pure, source-bound adapter from ACTUAL R82 live replanner authority
% to the R81 HLC-payload-frame reference-only natural connector search.
% Never uses unseen obstacles or substitutes a reference for EKF state.
% Never certifies actuator, tension, tracking, tail or physical execution.
assert(isstruct(S)&&isscalar(S),'R83:ScalarSnapshotRequired');
required={'active_raw_jets','nominal_raw_jets_at','known_mask', ...
    'obstacle_packet','reference_version','reference_mode', ...
    'gravity_m_s2','cable_length_m','rQ_m','rC_m','rL_m', ...
    'd_margin_m','sensor_range_m','horizon_s','prediction_dt_s', ...
    'latest_EKF_mL_kg'};
for j=1:numel(required)
    assert(isfield(S,required{j}),'R83:MissingAuthority:%s',required{j});
end
assert(isnumeric(S.active_raw_jets)&&isequal(size(S.active_raw_jets),[3 8]) && ...
    all(isfinite(S.active_raw_jets(:))), 'R83:ActiveExactJetsRequired');
assert(isa(S.nominal_raw_jets_at,'function_handle'), 'R83:NominalJetsProviderRequired');
assert(isnumeric(S.reference_version)&&isscalar(S.reference_version)&& ...
    isfinite(S.reference_version)&&S.reference_version>=0 && ...
    S.reference_version==fix(S.reference_version),'R83:InvalidReferenceVersion');
mode=char(S.reference_mode);
assert(any(strcmp(mode,{'NOMINAL','REPAIRED'})),'R83:InvalidSourceMode');
scalarFields={'gravity_m_s2','cable_length_m','rQ_m','rC_m','rL_m', ...
   'd_margin_m','sensor_range_m','horizon_s','prediction_dt_s','latest_EKF_mL_kg'};
for j=1:numel(scalarFields)
    v=S.(scalarFields{j});
    assert(isnumeric(v)&&isscalar(v)&&isfinite(v),'R83:NonfiniteLiveAuthority:%s',scalarFields{j});
end
assert(all([S.gravity_m_s2,S.cable_length_m,S.sensor_range_m,S.horizon_s, ...
    S.prediction_dt_s,S.latest_EKF_mL_kg]>0) && ...
    all([S.rQ_m,S.rC_m,S.rL_m,S.d_margin_m]>=0), ...
    'R83:NonphysicalLiveAuthority');
P=S.obstacle_packet;
need={'C','R','Araw','ids','reference_version','reference_mode', ...
    'delta_payload_m','delta_uav_m','delta_cable_m'};
assert(isstruct(P)&&isscalar(P),'R83:PacketInvalid');
for j=1:numel(need)
    assert(isfield(P,need{j}),'R83:PacketMissingField:%s',need{j});
end
assert(isequal(double(P.reference_version),double(S.reference_version)) && ...
    strcmp(char(P.reference_mode),mode), 'R83:StaleObstaclePacket');
ids=double(P.ids(:));n=numel(ids);
assert(n>=1 && isequal(size(P.C),[3 n])&&isequal(size(P.R),[9 n])&& ...
    isequal(size(P.Araw),[3 n])&&all(isfinite([P.C(:);P.R(:);P.Araw(:);ids])) && ...
    all(P.Araw(:)>0)&&all(ids==fix(ids))&&all(ids>0)&& ...
    numel(unique(ids))==n,'R83:InvalidSourceObstacleGeometry');
mask=S.known_mask;
assert((islogical(mask)||isnumeric(mask))&&numel(mask)==n&& ...
    all(mask(:)==0 | mask(:)==1),'R83:InvalidKnownMask');
mask=logical(mask(:));ii=find(mask);
assert(~isempty(ii),'R83:NoSensorKnownObstacle');
% Packet rotation is 9-by-M column-major in the verified R82 source.
obs=repmat(struct('center',zeros(3,1),'R',eye(3),'axes',ones(3,1)),1,numel(ii));
for j=1:numel(ii)
    k=ii(j);
    R=reshape(double(P.R(:,k)),3,3);
    assert(norm(R'*R-eye(3),'fro')<1e-9 && abs(det(R)-1)<1e-9, ...
        'R83:PacketRotationNotSO3');
    obs(j).center=double(P.C(:,k));
    obs(j).R=R;
    obs(j).axes=double(P.Araw(:,k));
end
extra=double([P.delta_uav_m,P.delta_cable_m,P.delta_payload_m]);
assert(numel(extra)==3&&all(isfinite(extra))&&all(extra>=0),'R83:InvalidSweepAuthority');
% Existing physical radii and safety margin remain external authorities.
% Carry over PRE-EXISTING packet sweep inflation conservatively; the R80
% certifier separately covers its own continuous-time connector motion.
rQ=double(S.rQ_m+S.d_margin_m)+extra(1);
rC=double(S.rC_m+S.d_margin_m)+extra(2);
rL=double(S.rL_m+S.d_margin_m)+extra(3);
L=double(S.cable_length_m);g=double(S.gravity_m_s2);
% The finite-sensing limit is derived from CURRENT external parameters by
% exactly the production expression (R82 source line ~11817), not guessed.
rho=max([double(S.rQ_m),L+double(S.rL_m),L+double(S.rC_m)]);
vq=(double(S.sensor_range_m)-rho)/(double(S.horizon_s)+double(S.prediction_dt_s));
assert(isfinite(vq)&&vq>0,'R83:FiniteSensingAuthorityInfeasible');
% SUSPENDED_LOAD_REF_ADJUST in Flight: xd(3) -> xd(3)-L. Only the
% POSITION jet changes; velocities and orders 2..7 remain identical.
old=double(S.active_raw_jets);
old(3,1)=old(3,1)-L;
B=struct('status','R83_ACTUAL_LIVE_AUTHORITIES_BOUND__REFERENCE_ONLY', ...
   'source_version',double(S.reference_version),'source_mode',mode, ...
   'old_payload_jets',old, ...
   'nominal_payload_jets_at',@(t)r83_nominal_payload_jets(S.nominal_raw_jets_at,t,L), ...
   'obstacles',obs,'known_ids',ids(ii).', ...
   'live',struct('gravity_m_s2',g,'cable_length_m',L, ...
       'rQ_m',rQ,'rC_m',rC,'rL_m',rL, ...
       'finite_sensing_vQ_limit_m_s',vq, ...
       'latest_EKF_mL_kg',double(S.latest_EKF_mL_kg)), ...
   'raw_flight_to_HLC_payload_z_shift_m',-L, ...
   'uses_sensor_known_only',true,'physical_execution_certified',false, ...
   'production_commit_authorized',false);
end

function J=r83_nominal_payload_jets(fn,t,L)
J=double(fn(double(t)));
assert(isequal(size(J),[3 8])&&all(isfinite(J(:))), ...
    'R83:NonfiniteLiveNominalC7Jets');
J(3,1)=J(3,1)-L;
end
