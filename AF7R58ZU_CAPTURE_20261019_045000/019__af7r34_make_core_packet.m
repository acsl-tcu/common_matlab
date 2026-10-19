function P = af7r34_make_core_packet(replanner,CTX,timing,model)
% Numeric packet for the R14 native core.
%
% CTX/timing may be [] for AST-only evaluation.  In that case only the
% active-reference representation fields are meaningful.

assert(isa(replanner,'REPLANNING_SYSTEM'));

if nargin<4 || isempty(model)
    if strcmpi(char(replanner.active_reference_mode),'NOMINAL')
        model=af7r33_nominal_batch_model(replanner);
    elseif strcmpi(char(replanner.active_reference_mode),'REPAIRED')
        R=replanner.active_repaired_reference;
        assert(isstruct(R)&&isfield(R,'batch_flat_model')&& ...
            isstruct(R.batch_flat_model)&&logical(R.batch_flat_model.certified), ...
            '[AF7R34] repaired reference lacks AF7R33 batch_flat_model.');
        model=R.batch_flat_model;
    else
        error('[AF7R34] unsupported active reference mode.');
    end
end

P=struct();
P.nominal_signature=char(model.nominal_signature);
P.flight_t0=double(model.flight_reference_time_origin);

G=numel(model.corrections);
P.repair_t0=zeros(1,G);
P.repair_t1=zeros(1,G);
P.repair_offsets=zeros(1,G+1);
S=0;
for g=1:G
    P.repair_t0(g)=double(model.corrections(g).t0);
    P.repair_t1(g)=double(model.corrections(g).t1);
    P.repair_offsets(g)=S;
    S=S+numel(model.corrections(g).control_points);
end
P.repair_offsets(G+1)=S;
P.repair_seg0=zeros(1,S);
P.repair_seg1=zeros(1,S);
P.repair_cp=zeros(48,S);
q=0;
for g=1:G
    C=model.corrections(g);
    for k=1:numel(C.control_points)
        q=q+1;
        P.repair_seg0(q)=double(C.segment_t0(k));
        P.repair_seg1(q)=double(C.segment_t1(k));
        cp=double(C.control_points{k});
        assert(isequal(size(cp),[3 16])&&all(isfinite(cp),'all'));
        P.repair_cp(:,q)=cp(:);
    end
end

% Dummy finite values for AST-only op=2.
if isempty(CTX)
    P.t0=0; P.tm=1; P.t1=2;
    P.dt=1; P.gravity=1; P.cable_length=1;
    P.r_load=0; P.r_drone=0; P.r_cable=0; P.d_margin=0;
    P.sensor_range=1; P.horizon=1; P.tracking_inflation=0;
    P.base_payload_speed=0; P.base_payload_accel=0; P.base_payload_jerk=0;
    P.base_sigma=1; P.base_qdot=0; P.base_uav_speed=0;
    P.centers=zeros(3,0); P.rotations_flat=zeros(9,0);
    P.base_axes=zeros(3,0); P.ids=zeros(1,0);
    return;
end

RQ=CTX.request; LIV=CTX.live; O=CTX.obstacles;
P.t0=double(RQ.time);
P.tm=double(RQ.trigger_collision_time);
if nargin>=3 && isstruct(timing) && isfield(timing,'t1')
    P.t1=double(timing.t1);
else
    P.t1=double(replanner.resolve_reference_rejoin_search_end(P.tm));
end
assert(isfinite(P.t1)&&P.t1>P.tm,'[AF7R34] invalid rejoin search end.');

P.dt=double(LIV.dt);
P.gravity=double(LIV.gravity);
P.cable_length=double(LIV.cable_length);
P.r_load=double(LIV.r_load);
P.r_drone=double(LIV.r_drone);
P.r_cable=double(LIV.r_cable);
P.d_margin=double(LIV.d_margin);
P.sensor_range=double(LIV.sensor_range);
P.horizon=double(LIV.horizon);
P.tracking_inflation=0;

SC=replanner.active_speed_certificate;
replanner.validate_active_speed_certificate(SC);
P.base_payload_speed=double(SC.payload_speed_UB_m_s);
P.base_payload_accel=double(SC.payload_accel_UB_m_s2);
P.base_payload_jerk=double(SC.payload_jerk_UB_m_s3);
P.base_sigma=double(SC.specific_force_norm_LB_m_s2);
P.base_qdot=double(SC.qdot_UB_rad_s);
P.base_uav_speed=double(SC.uav_speed_UB_m_s);

P.centers=double(O.centers);
P.rotations_flat=double(O.rotations_flat);
P.base_axes=double(O.base_axes);
P.ids=double(O.ids(:).');
assert(size(P.centers,2)==double(O.count));
end
