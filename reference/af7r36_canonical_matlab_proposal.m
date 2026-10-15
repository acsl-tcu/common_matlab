function OUT = af7r36_canonical_matlab_proposal(replanner,CTX)
% Full MATLAB shadow specification for AF7R36.
% Current production theorems are reused after canonicalized synthesis.

OUT=replanner.empty_bezier_d15_c7_proposal();
RQ=CTX.request;
LIV=CTX.live;
O=CTX.obstacles;

OUT.t0=double(RQ.time);
OUT.tm=double(RQ.trigger_collision_time);
OUT.trigger_collision_time=OUT.tm;
OUT.obstacle_id=double(RQ.trigger_obstacle_id);
OUT.component_code=double(RQ.trigger_component_code);
OUT.base_eval_fn=replanner.snapshot_active_reference_eval_fn();
OUT.base_flat_eval_fn=replanner.snapshot_active_reference_flat_eval_fn();
OUT.execution_tracking_inflation_m=0;

searchEnd=replanner.resolve_reference_rejoin_search_end(OUT.tm);
if ~(isfinite(searchEnd)&&searchEnd>OUT.tm)
    OUT.reason_code=17;
    return;
end
OUT.t1=searchEnd;
OUT.timing_projected=false;

[EPI,okEpi]=af7r33_episode_batch_allobs(replanner,OUT,CTX,searchEnd);
OUT.unsafe_episode_certificate=EPI;
if ~okEpi || ~isstruct(EPI) || ~isfield(EPI,'component_count') || ...
        double(EPI.component_count)<1
    OUT.reason_code=22;
    return;
end

Cmp=EPI.components;
if double(Cmp(1).start_s)<=OUT.t0+8*eps(max(1,abs(OUT.t0)))
    OUT.reason_code=14;
    return;
end
if double(Cmp(end).end_s)>=OUT.t1-8*eps(max(1,abs(OUT.t1)))
    OUT.reason_code=17;
    return;
end
OUT.hold_end=double(Cmp(end).end_s);

[CANDS,SYN]=af7r36_build_canonical_piecewise_candidates( ...
    replanner,OUT,CTX,EPI,OUT.t1);
OUT.piecewise_synthesis_certificate=SYN;
OUT.corridor_certificate=SYN;
if isempty(CANDS)
    OUT.reason_code=23;
    return;
end

rhoBody=max([double(LIV.r_drone), ...
    double(LIV.cable_length)+double(LIV.r_load), ...
    double(LIV.cable_length)+double(LIV.r_cable)]);
VQlimit=(double(LIV.sensor_range)-rhoBody)/ ...
    (double(LIV.horizon)+double(LIV.dt));
if ~(isfinite(VQlimit)&&VQlimit>0)
    OUT.reason_code=16;
    return;
end

targetVersion=double(RQ.next_reference_version);
baseSC=replanner.active_speed_certificate;
speedPass=0;
rejoinPass=0;
geomTests=0;
lastSC=struct();
lastLV=struct();
lastG=struct();
selected=false;

for ic=1:numel(CANDS)
    Q=CANDS(ic);
    Ptry=OUT;
    Ptry.generated=true;
    Ptry.c7_exact=true;
    Ptry.representation_code=2;
    Ptry.degree=15;
    Ptry.piecewise_segment_t0=Q.segment_t0;
    Ptry.piecewise_segment_t1=Q.segment_t1;
    Ptry.piecewise_segment_kind=Q.segment_kind;
    Ptry.piecewise_control_points=Q.control_points;
    Ptry.piecewise_component_start=Q.component_start;
    Ptry.piecewise_component_end=Q.component_end;
    Ptry.piecewise_component_position=Q.component_position;
    Ptry.piecewise_component_velocity=Q.component_velocity;
    Ptry.piecewise_corridor_signs=Q.corridor_signs;
    Ptry.piecewise_qp_cost=Q.qp_cost;
    Ptry.correction_derivative_ub=Q.derivative_ub;
    Ptry.displacement_norm_m=Q.max_control_norm_m;
    Ptry.displacement=Q.first_component_position;
    if norm(Q.first_component_position)>0
        Ptry.direction=Q.first_component_position/norm(Q.first_component_position);
    else
        Ptry.direction=zeros(3,1);
    end
    Ptry.towing_structure_certificate=Q.towing_structure_certificate;

    [dubTight,DB]=replanner.tighten_piecewise_derivative_bounds_for_speed( ...
        Q.segment_t0,Q.segment_t1,Q.control_points,baseSC,VQlimit,6);
    Ptry.derivative_bound_certificate=DB;

    speedClosed=false;
    SCtry=struct('certified',false,'reason','UNINITIALIZED_R9_SPEED_CERT');
    LV=struct('certified',false,'not_required',false);

    if isstruct(DB)&&isfield(DB,'certified')&&logical(DB.certified)
        Ptry.correction_derivative_ub=dubTight;
        SCtri=replanner.build_bezier_repaired_speed_certificate(Ptry,targetVersion);
        if isstruct(SCtri)&&isfield(SCtri,'certified')&&logical(SCtri.certified)&& ...
                double(SCtri.uav_speed_UB_m_s)<VQlimit
            SCtry=SCtri;
            SCtry.certificate_stage='R9_STAGE_A_GLOBAL_TRIANGLE';
            speedClosed=true;
            LV=struct('certified',false,'not_required',true, ...
                'reason','STAGE_A_ALREADY_CERTIFIED');
        end
    end

    if ~speedClosed
        Ptry.correction_derivative_ub=double(Q.derivative_ub(:));
        [SClocal,LV]=replanner.build_piecewise_local_vector_speed_certificate( ...
            Ptry,Q.segment_t0,Q.segment_t1,Q.control_points, ...
            CTX,baseSC,VQlimit,targetVersion);
        lastLV=LV;
        if isstruct(SClocal)&&isfield(SClocal,'certified')&& ...
                logical(SClocal.certified)&& ...
                double(SClocal.uav_speed_UB_m_s)<VQlimit
            SCtry=SClocal;
            speedClosed=true;
        end
    end

    Ptry.local_vector_speed_certificate=LV;
    lastSC=SCtry;
    if ~speedClosed
        continue;
    end
    speedPass=speedPass+1;
    Ptry.reference_speed_certificate=SCtry;

    if ~replanner.reference_rejoin_cell_continuously_safe(Ptry,Ptry.t1,CTX)
        continue;
    end
    rejoinPass=rejoinPass+1;

    geomTests=geomTests+1;
    Gtry=replanner.build_static_continuous_wholebody_shadow_certificate(CTX,Ptry);
    lastG=Gtry;
    if isstruct(Gtry)&&isfield(Gtry,'mathematical_pass')&&logical(Gtry.mathematical_pass)
        Ptry.geometry_shadow_certificate=Gtry;
        OUT=Ptry;
        selected=true;
        break;
    end
end

OUT.piecewise_speed_pass_count=speedPass;
OUT.piecewise_rejoin_pass_count=rejoinPass;
OUT.piecewise_geometry_test_count=geomTests;

if ~selected
    OUT.reference_speed_certificate=lastSC;
    OUT.local_vector_speed_certificate=lastLV;
    OUT.geometry_shadow_certificate=lastG;
    if speedPass==0, OUT.reason_code=24; else, OUT.reason_code=25; end
    return;
end

OUT.success=true;
OUT.generated=true;
OUT.reason_code=0;
end
