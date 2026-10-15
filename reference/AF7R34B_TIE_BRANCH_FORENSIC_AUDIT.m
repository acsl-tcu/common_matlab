function OUT = AF7R34B_TIE_BRANCH_FORENSIC_AUDIT(replanner)
clc;
fprintf('\n====================================================================================================\n');
fprintf(' AF7R34B | DISJUNCTIVE GLOBAL-OPTIMUM TIE FORENSIC AUDIT\n');
fprintf('====================================================================================================\n');
fprintf('[AUTHORITY] readOnly=1 | noProductionModification=1 | safetyRecheck=1\n');

assert(isa(replanner,'REPLANNING_SYSTEM'));
assert(exist('af7r34_single_core_mex','file')==3,'[AF7R34B] core MEX missing.');

P0=AF7R34_PRIME_SINGLE_CORE_REFERENCE(replanner);
assert(P0.closed,'[AF7R34B] AST prime not closed.');

E=replanner.get_first_trigger_event();
assert(isstruct(E)&&isfield(E,'valid')&&logical(E.valid)&& ...
    logical(E.real_flight_request_edge),'[AF7R34B] no true trigger event.');
CTX=af7r32_replay_context(replanner,E);

PL=replanner.builtin_bezier_d15_c7_proposal(CTX);
[PN,RN]=af7r34_single_core_proposal(replanner,CTX);

assert(logical(PL.success)&&logical(PN.success), ...
    '[AF7R34B] both planners must return a certified proposal for this audit.');

sL=double(PL.piecewise_corridor_signs(:));
sN=double(PN.piecewise_corridor_signs(:));
sameSigns=isequal(size(sL),size(sN))&&all(sL==sN);
oppositeSigns=isequal(size(sL),size(sN))&&all(sL==-sN);

pL=double(PL.piecewise_component_position);
pN=double(PN.piecewise_component_position);
vL=double(PL.piecewise_component_velocity);
vN=double(PN.piecewise_component_velocity);

posSame=max(abs(pL(:)-pN(:)),[],'omitnan');
posMirror=max(abs(pL(:)+pN(:)),[],'omitnan');
velSame=max(abs(vL(:)-vN(:)),[],'omitnan');
velMirror=max(abs(vL(:)+vN(:)),[],'omitnan');

cpSame=0; cpMirror=0;
assert(numel(PL.piecewise_control_points)==numel(PN.piecewise_control_points));
for k=1:numel(PL.piecewise_control_points)
    A=double(PL.piecewise_control_points{k});
    B=double(PN.piecewise_control_points{k});
    cpSame=max(cpSame,max(abs(A-B),[],'all'));
    cpMirror=max(cpMirror,max(abs(A+B),[],'all'));
end

JL=double(PL.piecewise_qp_cost);
JN=double(PN.piecewise_qp_cost);
Jscale=max([1,abs(JL),abs(JN)]);
tieGuard=65536*eps(Jscale);
costTie=abs(JL-JN)<=tieGuard;

% Independent MATLAB theorem re-certification of the NATIVE-selected branch.
baseSC=replanner.active_speed_certificate;
LIV=CTX.live;
rhoBody=max([double(LIV.r_drone), ...
    double(LIV.cable_length)+double(LIV.r_load), ...
    double(LIV.cable_length)+double(LIV.r_cable)]);
VQlimit=(double(LIV.sensor_range)-rhoBody)/(double(LIV.horizon)+double(LIV.dt));
targetVersion=double(CTX.request.next_reference_version);

[dubTight,DB]=replanner.tighten_piecewise_derivative_bounds_for_speed( ...
    PN.piecewise_segment_t0,PN.piecewise_segment_t1,PN.piecewise_control_points, ...
    baseSC,VQlimit,6);

speedClosed=false;
SC=struct('certified',false);
LV=struct();
if isstruct(DB)&&isfield(DB,'certified')&&logical(DB.certified)
    PT=PN;
    PT.correction_derivative_ub=dubTight;
    SCtri=replanner.build_bezier_repaired_speed_certificate(PT,targetVersion);
    if isstruct(SCtri)&&isfield(SCtri,'certified')&&logical(SCtri.certified)&& ...
            double(SCtri.uav_speed_UB_m_s)<VQlimit
        SC=SCtri;
        speedClosed=true;
    end
end
if ~speedClosed
    [SClocal,LV]=replanner.build_piecewise_local_vector_speed_certificate( ...
        PN,PN.piecewise_segment_t0,PN.piecewise_segment_t1, ...
        PN.piecewise_control_points,CTX,baseSC,VQlimit,targetVersion);
    if isstruct(SClocal)&&isfield(SClocal,'certified')&&logical(SClocal.certified)&& ...
            double(SClocal.uav_speed_UB_m_s)<VQlimit
        SC=SClocal;
        speedClosed=true;
    end
end

PR=PN;
PR.reference_speed_certificate=SC;
rejoinPass=false;
geomPass=false;
Glegacy=struct();
if speedClosed
    rejoinPass=replanner.reference_rejoin_cell_continuously_safe(PR,PR.t1,CTX);
end
if rejoinPass
    Glegacy=replanner.build_static_continuous_wholebody_shadow_certificate(CTX,PR);
    geomPass=isstruct(Glegacy)&&isfield(Glegacy,'mathematical_pass')&& ...
        logical(Glegacy.mathematical_pass);
end

speedDiff=Inf;
if speedClosed
    a=SC; b=PN.reference_speed_certificate;
    speedDiff=max(abs([ ...
        double(a.payload_speed_UB_m_s)-double(b.payload_speed_UB_m_s), ...
        double(a.payload_accel_UB_m_s2)-double(b.payload_accel_UB_m_s2), ...
        double(a.payload_jerk_UB_m_s3)-double(b.payload_jerk_UB_m_s3), ...
        double(a.specific_force_norm_LB_m_s2)-double(b.specific_force_norm_LB_m_s2), ...
        double(a.qdot_UB_rad_s)-double(b.qdot_UB_rad_s), ...
        double(a.uav_speed_UB_m_s)-double(b.uav_speed_UB_m_s)]));
end

geomDiff=Inf;
geomTopology=false;
if geomPass
    a=Glegacy; b=PN.geometry_shadow_certificate;
    geomDiff=max(abs([ ...
        double(a.min_payload_margin_m)-double(b.min_payload_margin_m), ...
        double(a.min_uav_margin_m)-double(b.min_uav_margin_m), ...
        double(a.min_cable_margin_m)-double(b.min_cable_margin_m), ...
        double(a.min_margin_m)-double(b.min_margin_m)]));
    geomTopology=double(a.leaf_count)==double(b.leaf_count)&& ...
        double(a.split_count)==double(b.split_count)&& ...
        double(a.max_depth_used)==double(b.max_depth_used);
end

scale=max([1,posMirror,velMirror,cpMirror,finite0(speedDiff),finite0(geomDiff)]);
numGuard=65536*eps(scale);
mirrorClosed=oppositeSigns&&costTie&& ...
    posMirror<=numGuard&&velMirror<=numGuard&&cpMirror<=numGuard;
nativeLegacyRecertClosed=speedClosed&&rejoinPass&&geomPass&& ...
    finite0(speedDiff)<=numGuard&&finite0(geomDiff)<=numGuard&&geomTopology;

certifiedTieClosed=mirrorClosed&&nativeLegacyRecertClosed;

fprintf(' legacy corridor signs                    = [%s]\n',num2str(sL.'));
fprintf(' native corridor signs                    = [%s]\n',num2str(sN.'));
fprintf(' signs identical / exact opposite         = %d / %d\n',sameSigns,oppositeSigns);
fprintf(' component position same/mirror [max]     = %.17g / %.17g\n',posSame,posMirror);
fprintf(' component velocity same/mirror [max]     = %.17g / %.17g\n',velSame,velMirror);
fprintf(' control points same/mirror [max]         = %.17g / %.17g\n',cpSame,cpMirror);
fprintf(' legacy/native QP objective               = %.17g / %.17g\n',JL,JN);
fprintf(' QP objective |diff| / tie guard          = %.17g / %.17g\n',abs(JL-JN),tieGuard);
fprintf(' exact mirrored tied branch closed        = %d\n',mirrorClosed);
fprintf('\n[INDEPENDENT MATLAB RE-CERTIFICATION OF NATIVE BRANCH]\n');
fprintf(' speed / rejoin / whole-body pass         = %d / %d / %d\n',speedClosed,rejoinPass,geomPass);
fprintf(' speed certificate max |diff|             = %.17g\n',speedDiff);
fprintf(' geometry margin max |diff| [m]           = %.17g\n',geomDiff);
fprintf(' geometry topology equal                  = %d\n',geomTopology);
fprintf(' numeric comparison guard                 = %.17g\n',numGuard);
fprintf(' native branch legacy re-cert closed      = %d\n',nativeLegacyRecertClosed);
fprintf('\n certified disjunctive-tie equivalence     = %d\n',certifiedTieClosed);
fprintf(' production source modified               = 0\n');
fprintf('====================================================================================================\n');

OUT=struct('schema','AF7R34B_TIE_BRANCH_FORENSIC_AUDIT_V1', ...
    'closed',certifiedTieClosed,'mirror_closed',mirrorClosed, ...
    'legacy_recert_closed',nativeLegacyRecertClosed, ...
    'legacy_signs',sL,'native_signs',sN, ...
    'pos_same',posSame,'pos_mirror',posMirror, ...
    'vel_same',velSame,'vel_mirror',velMirror, ...
    'cp_same',cpSame,'cp_mirror',cpMirror, ...
    'cost_diff',abs(JL-JN),'tie_guard',tieGuard, ...
    'speed_diff',speedDiff,'geometry_diff',geomDiff, ...
    'native_raw',RN,'legacy_recert_geometry',Glegacy,'local_vector',LV);
assignin('base','AF7R34B_TIE_BRANCH_FORENSIC_AUDIT_OUT',OUT);
end

function x=finite0(x)
if isempty(x)||~isfinite(x), x=0; end
end
