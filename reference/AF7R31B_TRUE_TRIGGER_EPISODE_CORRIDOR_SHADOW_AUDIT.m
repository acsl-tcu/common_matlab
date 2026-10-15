function OUT = AF7R31B_TRUE_TRIGGER_EPISODE_CORRIDOR_SHADOW_AUDIT(replanner)
clc;
fprintf('\n====================================================================================================\n');
fprintf(' AF7R31B | TRUE-TRIGGER R13 BATCH FRONT-END SHADOW AUDIT\n');
fprintf('====================================================================================================\n');
fprintf('[AUTHORITY] readOnlyReplay=1 | productionModification=0 | sampledSafetyAcceptance=0\n');
assert(isa(replanner,'REPLANNING_SYSTEM'));
assert(exist('af7r31_geometry_batch_mex','file')==3,'[AF7R31B] geometry batch MEX missing.');
assert(exist('af7r31_nominal_flat_batch_mex','file')==3,'[AF7R31B] flat batch MEX missing.');
E=replanner.get_first_trigger_event();
assert(isstruct(E)&&isfield(E,'valid')&&logical(E.valid)&& ...
    isfield(E,'real_flight_request_edge')&&logical(E.real_flight_request_edge), ...
    '[AF7R31B] no real first trigger event captured.');
assert(strcmpi(char(E.reference.mode),'NOMINAL'), ...
    '[AF7R31B] this source-bound stage requires a NOMINAL first trigger.');
CTX=replay_context(replanner,E);
P=replanner.empty_bezier_d15_c7_proposal();
P.t0=double(E.event_time); P.tm=double(E.trigger.collision_time);
P.trigger_collision_time=P.tm;
P.obstacle_id=double(E.trigger.obstacle_id);
P.component_code=double(E.trigger.component_code);
P.base_eval_fn=replanner.snapshot_active_reference_eval_fn();
P.base_flat_eval_fn=replanner.snapshot_active_reference_flat_eval_fn();
searchEnd=replanner.resolve_reference_rejoin_search_end(P.tm);
assert(isfinite(searchEnd)&&searchEnd>P.tm,'[AF7R31B] search domain unavailable.');
P.t1=searchEnd;

replanner.build_bezier_nominal_unsafe_episode_components(P,CTX,searchEnd);
af7r31_episode_batch(replanner,P,CTX,searchEnd);
NREP=11; tLegacy=zeros(1,NREP); tBatch=zeros(1,NREP);
for k=1:NREP
    q=tic; [EL,okL]=replanner.build_bezier_nominal_unsafe_episode_components(P,CTX,searchEnd); tLegacy(k)=1e3*toc(q);
    q=tic; [EB,okB]=af7r31_episode_batch(replanner,P,CTX,searchEnd); tBatch(k)=1e3*toc(q);
end
[epiClosed,maxTimeDiff,diagEqual]=compare_episode(EL,EB,okL,okB);

corrClosed=false; maxA=NaN; maxb=NaN; tol=NaN;
corrLegacy=zeros(1,NREP); corrBatch=zeros(1,NREP);
if epiClosed && EL.component_count>=1
    C=double(EL.component_count); nvar=6*C;
    for k=1:NREP
        q=tic;
        for ie=1:numel(EL.episodes)
            n0=episode_normal(replanner,P,CTX,EL,ie);
            replanner.bezier_piecewise_episode_corridor_rows(P,CTX,EL,ie,n0,nvar);
            replanner.bezier_piecewise_episode_corridor_rows(P,CTX,EL,ie,-n0,nvar);
        end
        corrLegacy(k)=1e3*toc(q);
        q=tic;
        for ie=1:numel(EB.episodes)
            n0=episode_normal(replanner,P,CTX,EB,ie);
            af7r31_corridor_batch(replanner,P,CTX,EB,ie,n0,nvar);
            af7r31_corridor_batch(replanner,P,CTX,EB,ie,-n0,nvar);
        end
        corrBatch(k)=1e3*toc(q);
    end
    maxA=0; maxb=0; corrClosed=true;
    for ie=1:numel(EL.episodes)
        n0=episode_normal(replanner,P,CTX,EL,ie);
        for sgn=[1 -1]
            [AL,bL,oL]=replanner.bezier_piecewise_episode_corridor_rows(P,CTX,EL,ie,sgn*n0,nvar);
            [AB,bB,oB]=af7r31_corridor_batch(replanner,P,CTX,EB,ie,sgn*n0,nvar);
            if ~(oL&&oB&&isequal(size(AL),size(AB))&&isequal(size(bL),size(bB)))
                corrClosed=false; break;
            end
            maxA=max(maxA,max(abs(double(AL)-double(AB)),[],'all'));
            maxb=max(maxb,max(abs(double(bL)-double(bB)),[],'all'));
        end
        if ~corrClosed, break; end
    end
    scale=max([1,maxA,maxb]); tol=4096*eps(scale);
    corrClosed=corrClosed&&maxA<=tol&&maxb<=tol;
end

closed=epiClosed&&corrClosed;
fprintf(' trigger time [s]                         = %.9f\n',double(E.event_time));
fprintf(' detected obstacles                       = %d\n',double(E.detected_obstacles.count));
fprintf(' legacy/batch episode count               = %d / %d\n',EL.episode_count,EB.episode_count);
fprintf(' legacy/batch component count             = %d / %d\n',EL.component_count,EB.component_count);
fprintf(' episode max boundary |diff| [s]          = %.17g\n',maxTimeDiff);
fprintf(' episode diagnostic counts equal          = %d\n',diagEqual);
fprintf(' episode decision equivalence closed      = %d\n',epiClosed);
fprintf(' corridor max |A diff| / |b diff|         = %.17g / %.17g\n',maxA,maxb);
fprintf(' corridor tolerance                       = %.17g\n',tol);
fprintf(' corridor equivalence closed              = %d\n',corrClosed);
fprintf(' episode legacy min/med/max [ms]          = %.6g / %.6g / %.6g\n',min(tLegacy),median(tLegacy),max(tLegacy));
fprintf(' episode batch  min/med/max [ms]          = %.6g / %.6g / %.6g\n',min(tBatch),median(tBatch),max(tBatch));
fprintf(' corridor legacy min/med/max [ms]         = %.6g / %.6g / %.6g\n',min(corrLegacy),median(corrLegacy),max(corrLegacy));
fprintf(' corridor batch  min/med/max [ms]         = %.6g / %.6g / %.6g\n',min(corrBatch),median(corrBatch),max(corrBatch));
fprintf(' R13 batch front-end shadow closed        = %d\n',closed);
fprintf(' production source modified              = 0\n');
fprintf('====================================================================================================\n');

if closed
    src=which('REPLANNING_SYSTEM'); root=fileparts(src);
    token=struct('schema','AF7R31_FRONTEND_DUAL_TOKEN_V1','closed',true, ...
        'trigger_time',double(E.event_time),'episode_boundary_max_diff',maxTimeDiff, ...
        'corridor_A_max_diff',maxA,'corridor_b_max_diff',maxb, ...
        'episode_batch_median_ms',median(tBatch), ...
        'corridor_batch_median_ms',median(corrBatch));
    save(fullfile(root,'AF7R31_FRONTEND_DUAL_TOKEN.mat'),'token');
end
OUT=struct('schema','AF7R31B_TRUE_TRIGGER_EPISODE_CORRIDOR_SHADOW_AUDIT_V1', ...
    'closed',closed,'episode_closed',epiClosed,'corridor_closed',corrClosed, ...
    'episode_legacy_ms',tLegacy,'episode_batch_ms',tBatch, ...
    'corridor_legacy_ms',corrLegacy,'corridor_batch_ms',corrBatch, ...
    'max_episode_boundary_diff_s',maxTimeDiff,'max_A_diff',maxA,'max_b_diff',maxb);
assignin('base','AF7R31B_TRUE_TRIGGER_EPISODE_CORRIDOR_SHADOW_AUDIT_OUT',OUT);
end

function CTX=replay_context(replanner,E)
D=E.dependency_signature; SC=E.speed_certificate; O=E.detected_obstacles;
STATIC=struct('interface_version','REPLANNING_GENERATOR_CONTEXT_V1', ...
    'shadow_only',true,'backend_code',double(replanner.generator_backend_locked_code), ...
    'backend_name',char(replanner.generator_backend_locked_name), ...
    'backend_source_code',double(replanner.generator_backend_locked_source_code), ...
    'detected_only_obstacle_contract',true);
REQ=struct('time',double(E.event_time),'deadline_remaining_s',Inf, ...
    'elapsed_before_generator_s',0,'next_reference_version',double(E.reference.version)+1, ...
    'active_reference_mode',char(E.reference.mode),'active_reference_version',double(E.reference.version), ...
    'xd_nominal',E.state.xd_nominal(:),'xd_active',E.state.xd_active(:), ...
    'pQ_current',E.state.pQ_current(:),'pL_current',E.state.pL_current(:), ...
    'q_current',E.state.q_current(:),'cable_residual',double(E.state.cable_residual), ...
    'future_pL',zeros(3,0),'future_q',zeros(3,0), ...
    'future_reference_source',char(E.reference.source), ...
    'trigger_ttc',double(E.trigger.ttc),'trigger_collision_time',double(E.trigger.collision_time), ...
    'trigger_obstacle_id',double(E.trigger.obstacle_id),'trigger_component',char(E.trigger.component), ...
    'trigger_component_code',double(E.trigger.component_code), ...
    'trigger_pL_collision',E.trigger.pL_collision(:),'trigger_q_collision',E.trigger.q_collision(:), ...
    'trigger_pQ_collision',E.trigger.pQ_collision(:),'trigger_hit_index',double(E.trigger.hit_index));
LIVE=struct('dependency_refresh_count',double(replanner.dependency_refresh_count), ...
    'dt',double(D.dt),'sensor_range',double(D.sensor_range),'horizon',double(D.horizon), ...
    'r_drone',double(D.r_drone),'r_load',double(D.r_load),'r_cable',double(D.r_cable), ...
    'd_margin',double(D.d_margin),'cable_length',double(D.cable_length), ...
    'gravity',double(D.gravity),'mQ',double(D.mQ),'jx',double(D.jx),'jy',double(D.jy),'jz',double(D.jz), ...
    'mL',double(D.mL_snapshot),'active_speed_certified',logical(SC.certified), ...
    'active_payload_speed_ub_m_s',double(SC.payload_speed_UB_m_s), ...
    'active_uav_speed_ub_m_s',double(SC.uav_speed_UB_m_s), ...
    'active_qdot_ub_rad_s',double(SC.qdot_UB_rad_s), ...
    'active_constant_speed_required',false,'hard_modify_deadline_s',double(D.hard_modify_deadline_s), ...
    'command_limits',replanner.command_limit_certificate.limits);
OBS=O; if ~isfield(OBS,'raw'), OBS.raw=[]; end
CTX=struct('static',STATIC,'request',REQ,'obstacles',OBS,'live',LIVE);
end

function n0=episode_normal(replanner,P,CTX,EPI,ie)
ep=EPI.episodes(ie); tca=double(ep.start_s); tcb=double(ep.end_s);
xa=double(P.base_flat_eval_fn(tca)); xa=xa(:);
xb=double(P.base_flat_eval_fn(tcb)); xb=xb(:);
progress=xb(1:3)-xa(1:3);
if norm(progress)<=128*eps(max(1,norm(xa(1:3))))
    xm=double(P.base_flat_eval_fn(0.5*(tca+tcb))); xm=xm(:);
    progress=xm(5:7);
end
ii=double(ep.obstacle_index);
R=reshape(double(CTX.obstacles.rotations_flat(:,ii)),3,3);
a=double(CTX.obstacles.base_axes(:,ii));
n0=replanner.bezier_lateral_separator_normal(R,a,progress);
n0=n0/norm(n0);
end

function [closed,maxT,diagEqual]=compare_episode(A,B,okA,okB)
closed=logical(okA)==logical(okB) && A.episode_count==B.episode_count && ...
    A.component_count==B.component_count;
maxT=0;
if closed
    for k=1:A.episode_count
        if double(A.episodes(k).obstacle_index)~=double(B.episodes(k).obstacle_index) || ...
                double(A.episodes(k).obstacle_id)~=double(B.episodes(k).obstacle_id)
            closed=false; break;
        end
        maxT=max(maxT,abs(double(A.episodes(k).start_s)-double(B.episodes(k).start_s)));
        maxT=max(maxT,abs(double(A.episodes(k).end_s)-double(B.episodes(k).end_s)));
    end
end
scale=max([1,abs(double(A.t0)),abs(double(A.search_end_s))]);
closed=closed&&maxT<=512*eps(scale);
diagEqual=double(A.adaptive_leaf_count)==double(B.adaptive_leaf_count) && ...
    double(A.adaptive_safe_leaf_count)==double(B.adaptive_safe_leaf_count) && ...
    double(A.adaptive_unsafe_leaf_count)==double(B.adaptive_unsafe_leaf_count) && ...
    double(A.adaptive_split_count)==double(B.adaptive_split_count) && ...
    double(A.representability_fail_closed_count)==double(B.representability_fail_closed_count);
end
