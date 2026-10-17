function OUT = AF7R52A_NATIVE_PREPARED_COMMIT_SHADOW(replanner)
% AF7R52A_NATIVE_PREPARED_COMMIT_SHADOW
%
% Track A:
%   Prove that the post-callback commit data can be represented as one
%   contiguous numeric packet without changing the reference/certificate
%   mathematics.  This is SHADOW ONLY; no production authority is changed.
%
% The native helper prepares:
%   - correction segments/control polygons
%   - speed certificate numeric payload
%   - trigger sweep
%   - finite-sensing horizon packet
%   - obstacle inflation axes
% and provides a packed correction-jet evaluator.
%
% The shadow compares these values against the exact current MATLAB
% construction and compares 0..6 correction derivatives and HLC packets.

clc;
fprintf('\n====================================================================================================\n');
fprintf(' AF7R52A | NATIVE PREPARED-COMMIT PACKET SHADOW\n');
fprintf('====================================================================================================\n');
fprintf('[AUTHORITY] readOnly=1 | shadowOnly=1 | safetyMathUnchanged=1 | productionModified=0\n');

assert(isa(replanner,'REPLANNING_SYSTEM'));
assert(exist('af7r36_single_core_mex','file')==3,'[AF7R52A] AF7R36 MEX missing.');
assert(exist('af7r52_prepared_commit_mex','file')==3,'[AF7R52A] build AF7R52 MEX first.');

[E,M,~,authoritySource]=AF7R52_LOAD_TRUE_TRIGGER_AUTHORITY(replanner);
[CTX,pkt]=make_true_trigger_packet(replanner,E,M);
RAW=af7r36_single_core_mex(1,pkt);
assert(logical(RAW.success),'[AF7R52A] AF7R36 true-trigger replay failed.');

targetVersion=double(E.reference.version)+1;
P=make_compact_proposal(replanner,E,M,CTX,RAW,targetVersion);
SC=P.reference_speed_certificate;

% Exact current MATLAB target construction.
legacySweep=replanner.build_trigger_sweep_for_target(SC,targetVersion);
legacyPacket=replanner.prepare_obstacle_packet_fast_target( ...
    replanner.obstacle_packet,legacySweep,targetVersion);
legacySensor=replanner.build_sensor_horizon_for_target(SC,targetVersion);
legacyModel=append_model(M,P,targetVersion);

% Native prepared packet.
STATIC=make_static_packet(replanner,legacyPacket);
D=E.dependency_signature;
LIVE=struct('dt',double(D.dt),'gravity',double(D.gravity), ...
    'cable_length',double(D.cable_length), ...
    'r_load',double(D.r_load),'r_drone',double(D.r_drone), ...
    'r_cable',double(D.r_cable),'d_margin',double(D.d_margin), ...
    'sensor_range',double(D.sensor_range),'horizon',double(D.horizon));
Q=af7r52_prepared_commit_mex(1,RAW,STATIC,LIVE,targetVersion);

fprintf(' trigger t0 / collision / t1 [s]          = %.15g / %.15g / %.15g\n', ...
    double(RAW.t0),double(RAW.tm),double(RAW.t1));
fprintf(' true-trigger authority source            = %s\n',authoritySource);

% ---------------- exact/numeric packet comparisons -----------------------
legacySpeed=[SC.payload_speed_UB_m_s;SC.payload_accel_UB_m_s2; ...
    SC.payload_jerk_UB_m_s3;SC.specific_force_norm_LB_m_s2; ...
    SC.qdot_UB_rad_s;SC.uav_speed_UB_m_s];
legacySweepVec=[legacySweep.payload_m;legacySweep.uav_m;legacySweep.cable_m; ...
    legacySweep.sensor_one_period_m;legacySweep.dt];
legacySensorVec=[double(legacySensor.certified);legacySensor.VL;legacySensor.VQ; ...
    legacySensor.dt;legacySensor.sensor_range;legacySensor.horizon;legacySensor.rho_body; ...
    legacySensor.sensor_horizon_margin_m;legacySensor.first_detection_distance_LB_m; ...
    legacySensor.first_collision_lead_LB_s;legacySensor.before_horizon_entry_LB_s];
legacyGeom=[RAW.geometry_min_payload_margin;RAW.geometry_min_uav_margin; ...
    RAW.geometry_min_cable_margin;RAW.geometry_min_margin];

cpLegacy=flatten_cp(RAW.control_points);
dSpeed=maxabs(double(Q.speed)-double(legacySpeed));
dSweep=maxabs(double(Q.sweep)-double(legacySweepVec));
dSensor=maxabs(double(Q.sensor)-double(legacySensorVec));
dPay=maxabs(double(Q.Apay)-double(legacyPacket.Apay));
dUav=maxabs(double(Q.Auav)-double(legacyPacket.Auav));
dCable=maxabs(double(Q.Acable)-double(legacyPacket.Acable));
dGeom=maxabs(double(Q.geometry_margin)-double(legacyGeom));
dCP=maxabs(double(Q.control_points_matrix)-cpLegacy);
dSeg=max(maxabs(double(Q.segment_t0(:))-double(RAW.segment_t0(:))), ...
         maxabs(double(Q.segment_t1(:))-double(RAW.segment_t1(:))));

scale=max([1;abs(legacySpeed(:));abs(legacySweepVec(:)); ...
    abs(legacySensorVec(isfinite(legacySensorVec))); ...
    abs(legacyPacket.Apay(:));abs(legacyPacket.Auav(:));abs(legacyPacket.Acable(:)); ...
    abs(cpLegacy(:));abs(legacyGeom(:))]);
tol=4096*eps(max(scale));

fprintf('\n[A. PREPARED PACKET EQUIVALENCE]\n');
fprintf(' max |speed native-MATLAB|                = %.17g\n',dSpeed);
fprintf(' max |sweep native-MATLAB|                = %.17g\n',dSweep);
fprintf(' max |sensor native-MATLAB|               = %.17g\n',dSensor);
fprintf(' max |Apay native-MATLAB|                 = %.17g\n',dPay);
fprintf(' max |Auav native-MATLAB|                 = %.17g\n',dUav);
fprintf(' max |Acable native-MATLAB|               = %.17g\n',dCable);
fprintf(' max |geometry native-MATLAB|             = %.17g\n',dGeom);
fprintf(' max |control points native-MATLAB|       = %.17g\n',dCP);
fprintf(' max |segment times native-MATLAB|        = %.17g\n',dSeg);
fprintf(' diagnostic equivalence tolerance         = %.17g\n',tol);

% ---------------- derivative / HLC packet equivalence --------------------
tg=linspace(double(RAW.t0),double(RAW.t1),1915);
Yprep=double(af7r52_prepared_commit_mex(2,Q,tg));
Ylegacy=zeros(21,numel(tg));
Xlegacy=zeros(28,numel(tg));
Xprep=zeros(28,numel(tg));
Flegacy=zeros(28,numel(tg));
Fprep=zeros(28,numel(tg));

for k=1:numel(tg)
    Dk=replanner.evaluate_piecewise_bezier_correction_derivatives(tg(k),P,6);
    for rr=0:6
        Ylegacy(3*rr+(1:3),k)=Dk(:,rr+1);
    end

    xb=P.base_eval_fn(tg(k)); xb=double(xb(:));
    if numel(xb)<28, xb=[xb;zeros(28-numel(xb),1)]; end
    xf=P.base_flat_eval_fn(tg(k)); xf=double(xf(:));
    if numel(xf)<28, xf=[xf;zeros(28-numel(xf),1)]; end

    xp=xb; fp=xf;
    for rr=0:6
        ii=4*rr+(1:3);
        xp(ii)=xp(ii)+Yprep(3*rr+(1:3),k);
        fp(ii)=fp(ii)+Yprep(3*rr+(1:3),k);
    end
    Xprep(:,k)=xp(1:28);
    Fprep(:,k)=fp(1:28);
    Xlegacy(:,k)=replanner.evaluate_bezier_proposal_reference(tg(k),P);
    Flegacy(:,k)=replanner.evaluate_bezier_proposal_flat_reference(tg(k),P);
end

dCorr=maxabs(Yprep-Ylegacy);
dHLC=maxabs(Xprep-Xlegacy);
dFlat=maxabs(Fprep-Flegacy);

% Batch model equality: same mathematical correction represented from the
% packed 48xS matrix instead of nested candidate cells.
preparedModel=packed_model(M,Q,targetVersion);
[Ylm,vl]=af7r33_flat_jet_batch_model(replanner,tg,legacyModel);
[Ypm,vp]=af7r33_flat_jet_batch_model(replanner,tg,preparedModel);
dBatch=maxabs(double(Ylm)-double(Ypm));
validBatch=isequal(logical(vl),logical(vp))&&all(vl)&&all(vp);

fprintf('\n[B. REFERENCE EVALUATOR EQUIVALENCE]\n');
fprintf(' max |correction jet packed-legacy|       = %.17g\n',dCorr);
fprintf(' max |HLC 28-slot packed-legacy|          = %.17g\n',dHLC);
fprintf(' max |flat 28-slot packed-legacy|         = %.17g\n',dFlat);
fprintf(' max |AF7R33 batch model diff|            = %.17g\n',dBatch);
fprintf(' batch validity equal/all true            = %d\n',validBatch);

% ---------------- offline timing potential -------------------------------
for k=1:5
    legacy_prepare_once(replanner,E,M,P,SC,targetVersion);
    af7r52_prepared_commit_mex(1,RAW,STATIC,LIVE,targetVersion);
end

Nlegacy=80; Tlegacy=zeros(Nlegacy,1);
for k=1:Nlegacy
    q=tic;
    legacy_prepare_once(replanner,E,M,P,SC,targetVersion);
    Tlegacy(k)=1e3*toc(q);
end

Nnative=400; Tnative=zeros(Nnative,1);
for k=1:Nnative
    q=tic;
    QQ=af7r52_prepared_commit_mex(1,RAW,STATIC,LIVE,targetVersion);
    af7r52_prepared_commit_mex(2,QQ,[double(E.event_time),double(E.event_time)+double(D.horizon)]);
    Tnative(k)=1e3*toc(q);
end

fprintf('\n[C. OFFLINE SHADOW TIMING — NOT PRODUCTION WCET]\n');
fprintf(' MATLAB legacy bundle min/med/p95/max [ms] = %.6g / %.6g / %.6g / %.6g\n', ...
    min(Tlegacy),median(Tlegacy),pct95(Tlegacy),max(Tlegacy));
fprintf(' native prepared+2eval min/med/p95/max [ms]= %.6g / %.6g / %.6g / %.6g\n', ...
    min(Tnative),median(Tnative),pct95(Tnative),max(Tnative));
fprintf(' Windows/MATLAB hard WCET claimed          = 0\n');

closed = logical(Q.valid) && double(Q.target_version)==targetVersion && ...
    dSpeed<=tol && dSweep<=tol && dSensor<=tol && ...
    dPay<=tol && dUav<=tol && dCable<=tol && dGeom<=tol && ...
    dCP<=tol && dSeg<=tol && dCorr<=tol && dHLC<=tol && ...
    dFlat<=tol && dBatch<=tol && validBatch && logical(legacySensor.certified);

fprintf('\n[D. GATE]\n');
fprintf(' prepared packet equivalence closed       = %d\n',closed);
fprintf(' production source modified               = 0\n');
fprintf(' safety acceptance changed                = 0\n');
fprintf(' production install allowed by this file  = 0 (shadow first)\n');
fprintf('====================================================================================================\n');

OUT=struct();
OUT.schema='AF7R52A_NATIVE_PREPARED_COMMIT_SHADOW_V1';
OUT.closed=closed;
OUT.tolerance=tol;
OUT.max_speed_diff=dSpeed;
OUT.max_sweep_diff=dSweep;
OUT.max_sensor_diff=dSensor;
OUT.max_obstacle_axis_diff=max([dPay,dUav,dCable]);
OUT.max_control_point_diff=dCP;
OUT.max_correction_jet_diff=dCorr;
OUT.max_hlc_packet_diff=dHLC;
OUT.max_flat_packet_diff=dFlat;
OUT.max_batch_model_diff=dBatch;
OUT.legacy_timing_ms=Tlegacy;
OUT.prepared_timing_ms=Tnative;
OUT.prepared_packet=Q;
OUT.production_modified=false;
assignin('base','AF7R52A_PREPARED_COMMIT_SHADOW_OUT',OUT);
end


function [CTX,pkt]=make_true_trigger_packet(replanner,E,M)
CTX=af7r49_replay_context(replanner,E);
pkt=af7r34_make_core_packet(replanner,CTX,[],M);
SC0=E.speed_certificate;
assert(isstruct(SC0)&&isfield(SC0,'certified')&&logical(SC0.certified));
pkt.base_payload_speed=double(SC0.payload_speed_UB_m_s);
pkt.base_payload_accel=double(SC0.payload_accel_UB_m_s2);
pkt.base_payload_jerk=double(SC0.payload_jerk_UB_m_s3);
pkt.base_sigma=double(SC0.specific_force_norm_LB_m_s2);
pkt.base_qdot=double(SC0.qdot_UB_rad_s);
pkt.base_uav_speed=double(SC0.uav_speed_UB_m_s);
end


function P=make_compact_proposal(replanner,E,M,CTX,RAW,targetVersion)
P=struct();
P.success=logical(RAW.success); P.generated=P.success;
P.reason_code=double(RAW.reason_code); P.backend='BEZIER_D15_C7';
P.t0=double(RAW.t0);P.tm=double(RAW.tm);P.hold_end=double(RAW.hold_end);P.t1=double(RAW.t1);
P.c7_exact=true;P.representation_code=2;P.degree=15;
P.piecewise_segment_t0=double(RAW.segment_t0(:).');
P.piecewise_segment_t1=double(RAW.segment_t1(:).');
P.piecewise_control_points=RAW.control_points;
P.displacement=double(RAW.displacement(:));
P.direction=double(RAW.direction(:));
P.displacement_norm_m=double(RAW.max_control_norm_m);

SC=struct('certified',true,'continuous_time',true,'sampled_acceptance',false, ...
    'reference_mode','REPAIRED','reference_version',targetVersion, ...
    'payload_speed_UB_m_s',double(RAW.speed_payload), ...
    'payload_accel_UB_m_s2',double(RAW.speed_accel), ...
    'payload_jerk_UB_m_s3',double(RAW.speed_jerk), ...
    'specific_force_norm_LB_m_s2',double(RAW.speed_sigma), ...
    'qdot_UB_rad_s',double(RAW.speed_qdot), ...
    'uav_speed_UB_m_s',double(RAW.speed_uav), ...
    'base_reference_mode',char(E.reference.mode), ...
    'base_reference_version',double(E.reference.version));
if logical(RAW.speed_stageA)
    SC.source='TRIANGLE_BOUNDED_C7_REPAIRED_FLATNESS_KINEMATICS';
    SC.certificate_stage='R9_STAGE_A_GLOBAL_TRIANGLE';
else
    SC.source='R9_LOCAL_VECTOR_COUPLED_ADAPTIVE_BERNSTEIN_FLATNESS_SPEED';
    SC.certificate_stage='R9_STAGE_B_LOCAL_VECTOR';
    SC.local_leaf_count=double(RAW.speed_leaf_count);
    SC.local_split_count=double(RAW.speed_split_count);
    SC.vector_coupled_payload_velocity=true;
    SC.suspended_load_flatness=true;
    SC.fixed_depth_cap=false;
end
P.reference_speed_certificate=SC;

G=struct('mathematical_pass',logical(RAW.geometry_pass), ...
    'certified',logical(RAW.geometry_pass),'continuous_time',true, ...
    'whole_body',true,'all_detected_obstacles',true,'static_obstacles_only',true, ...
    'sampled_acceptance',false,'finite_cover_speed_tube',true, ...
    'support_function_separation',true,'corrected_flatness_cable_direction',true, ...
    'ieee754_outward_closed',false,'reason_code',double(RAW.geometry_reason), ...
    'min_payload_margin_m',double(RAW.geometry_min_payload_margin), ...
    'min_uav_margin_m',double(RAW.geometry_min_uav_margin), ...
    'min_cable_margin_m',double(RAW.geometry_min_cable_margin), ...
    'min_margin_m',double(RAW.geometry_min_margin), ...
    'leaf_count',double(RAW.geometry_leaf_count), ...
    'accepted_leaf_count',double(RAW.geometry_accepted_leaf_count), ...
    'split_count',double(RAW.geometry_split_count), ...
    'max_depth_used',double(RAW.geometry_max_depth), ...
    'reference_only_authority',true, ...
    'final_system_authority',false);
P.geometry_shadow_certificate=G;

fh=replanner.base_ref.func;
flatfh=replanner.nominal_flat_derivative_fn(1:7);
tf0=double(M.flight_reference_time_origin);
P.base_eval_fn=@(tg)replanner.evaluate_nominal_snapshot_global(fh,tf0,tg);
P.base_flat_eval_fn=@(tg)replanner.evaluate_nominal_flat_snapshot_global(fh,flatfh,tf0,tg);
end


function S=make_static_packet(replanner,legacyPacket)
O=replanner.static_obstacle_map;
M=numel(O);
S=struct();
S.type_code=zeros(1,M);
S.raw_param=nan(2,M);
S.Araw=double(legacyPacket.Araw);
assert(isequal(size(S.Araw),[3 M]));
for i=1:M
    typ=lower(char(O(i).type));
    if strcmp(typ,'cylinder')
        S.type_code(i)=1;
        rp=double(O(i).raw_param(:));
        assert(numel(rp)>=2);
        S.raw_param(:,i)=rp(1:2);
    elseif strcmp(typ,'sphere')
        S.type_code(i)=2;
        rp=double(O(i).raw_param(:));
        assert(~isempty(rp));
        S.raw_param(1,i)=rp(1);
        S.raw_param(2,i)=0;
    else
        S.type_code(i)=3;
        S.raw_param(:,i)=0;
    end
end
end


function model=append_model(M,P,targetVersion)
model=M;
C=struct('t0',double(P.t0),'t1',double(P.t1), ...
    'segment_t0',double(P.piecewise_segment_t0(:).'), ...
    'segment_t1',double(P.piecewise_segment_t1(:).'), ...
    'control_points',{P.piecewise_control_points});
model.corrections(end+1)=C;
model.reference_version=double(targetVersion);
model.certified=true;
model.schema='AF7R33_BATCH_FLAT_MODEL_V1';
end


function model=packed_model(M,Q,targetVersion)
model=M;
S=numel(Q.segment_t0);
CP=cell(1,S);
A=double(Q.control_points_matrix);
for k=1:S
    CP{k}=reshape(A(:,k),3,16);
end
C=struct('t0',double(Q.t0),'t1',double(Q.t1), ...
    'segment_t0',double(Q.segment_t0(:).'), ...
    'segment_t1',double(Q.segment_t1(:).'), ...
    'control_points',{CP});
model.corrections(end+1)=C;
model.reference_version=double(targetVersion);
model.certified=true;
model.schema='AF7R33_BATCH_FLAT_MODEL_V1';
end


function A=flatten_cp(C)
S=numel(C); A=zeros(48,S);
for k=1:S
    q=double(C{k});assert(isequal(size(q),[3 16]));
    A(:,k)=q(:);
end
end


function legacy_prepare_once(replanner,E,M,P,SC,targetVersion)
PP=P;
fh=replanner.base_ref.func;
flatfh=replanner.nominal_flat_derivative_fn(1:7);
tf0=double(M.flight_reference_time_origin);
PP.base_eval_fn=@(tg)replanner.evaluate_nominal_snapshot_global(fh,tf0,tg);
PP.base_flat_eval_fn=@(tg)replanner.evaluate_nominal_flat_snapshot_global(fh,flatfh,tf0,tg);
evalFn=@(tg)replanner.evaluate_bezier_proposal_reference(tg,PP);
flatFn=@(tg)replanner.evaluate_bezier_proposal_flat_reference(tg,PP);
tq=[double(E.event_time),double(E.event_time)+double(E.dependency_signature.horizon)];
for k=1:2
    a=evalFn(tq(k));b=flatFn(tq(k)); %#ok<NASGU>
end
SW=replanner.build_trigger_sweep_for_target(SC,targetVersion);
PK=replanner.prepare_obstacle_packet_fast_target(replanner.obstacle_packet,SW,targetVersion); %#ok<NASGU>
SH=replanner.build_sensor_horizon_for_target(SC,targetVersion); %#ok<NASGU>
BM=append_model(M,PP,targetVersion); %#ok<NASGU>
R=struct('valid',true,'backend','BEZIER_D15_C7', ...
    'eval_fn',evalFn,'flat_eval_fn',flatFn,'batch_flat_model',BM); %#ok<NASGU>
end


function y=maxabs(x)
x=double(x(:));x=x(isfinite(x));
if isempty(x),y=0;else,y=max(abs(x));end
end


function y=pct95(x)
x=sort(double(x(:))); n=numel(x);
if n==0,y=NaN;return;end
j=max(1,min(n,ceil(0.95*n)));y=x(j);
end
