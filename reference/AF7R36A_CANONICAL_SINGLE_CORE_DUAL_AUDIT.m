function OUT = AF7R36A_CANONICAL_SINGLE_CORE_DUAL_AUDIT(replanner)
clc;
fprintf('\n====================================================================================================\n');
fprintf(' AF7R36A | CANONICAL MATLAB <-> SINGLE-CALL C++ FULL DUAL AUDIT\n');
fprintf('====================================================================================================\n');
fprintf('[AUTHORITY] canonicalSynthesisSpec=1 | fullCertificateDual=1 | sampledSafetyAcceptance=0 | productionModification=0\n');

assert(isa(replanner,'REPLANNING_SYSTEM'));
assert(exist('af7r36_single_core_mex','file')==3,'[AF7R36A] canonical core MEX missing.');

P0=AF7R36_PRIME_CANONICAL_SINGLE_CORE_REFERENCE(replanner);
assert(P0.closed,'[AF7R36A] AST prime did not close.');

E=replanner.get_first_trigger_event();
assert(isstruct(E)&&isfield(E,'valid')&&logical(E.valid)&&logical(E.real_flight_request_edge), ...
    '[AF7R36A] no stored true Flight trigger.');
CTX=af7r32_replay_context(replanner,E);

q=tic;
PM=af7r36_canonical_matlab_proposal(replanner,CTX);
matlabCanonicalMs=1e3*toc(q);

q=tic;
[PN,RN]=af7r36_single_core_proposal(replanner,CTX);
nativeFirstMs=1e3*toc(q);

decisionSame=logical(PM.success)==logical(PN.success) && ...
    double(PM.reason_code)==double(PN.reason_code);

epiClosed=false; epiDiff=Inf;
EM=PM.unsafe_episode_certificate;
EN=PN.unsafe_episode_certificate;
if isstruct(EM)&&isstruct(EN)&& ...
        double(EM.episode_count)==double(EN.episode_count)&& ...
        double(EM.component_count)==double(EN.component_count)
    epiClosed=true; epiDiff=0;
    for k=1:double(EM.episode_count)
        epiClosed=epiClosed && ...
            double(EM.episodes(k).obstacle_index)==double(EN.episodes(k).obstacle_index) && ...
            double(EM.episodes(k).component_index)==double(EN.episodes(k).component_index);
        epiDiff=max(epiDiff,abs(double(EM.episodes(k).start_s)-double(EN.episodes(k).start_s)));
        epiDiff=max(epiDiff,abs(double(EM.episodes(k).end_s)-double(EN.episodes(k).end_s)));
    end
end

segDiff=0; cpDiff=0; compDiff=0; costDiff=0; speedDiff=0; geomDiff=0;
signEqual=true; geomCounts=true; structural=true;

if logical(PM.success)&&logical(PN.success)
    structural=numel(PM.piecewise_control_points)==numel(PN.piecewise_control_points) && ...
        numel(PM.piecewise_segment_t0)==numel(PN.piecewise_segment_t0);
    if structural
        segDiff=max([ ...
            max(abs(double(PM.piecewise_segment_t0(:))-double(PN.piecewise_segment_t0(:))),[],'omitnan'), ...
            max(abs(double(PM.piecewise_segment_t1(:))-double(PN.piecewise_segment_t1(:))),[],'omitnan')]);
        for k=1:numel(PM.piecewise_control_points)
            A=double(PM.piecewise_control_points{k});
            B=double(PN.piecewise_control_points{k});
            if ~isequal(size(A),size(B)), structural=false; break; end
            cpDiff=max(cpDiff,max(abs(A-B),[],'all'));
        end
    end

    compDiff=max([ ...
        max(abs(double(PM.piecewise_component_position(:))-double(PN.piecewise_component_position(:))),[],'omitnan'), ...
        max(abs(double(PM.piecewise_component_velocity(:))-double(PN.piecewise_component_velocity(:))),[],'omitnan')]);

    sM=double(PM.piecewise_corridor_signs(:));
    sN=double(PN.piecewise_corridor_signs(:));
    signEqual=isequal(sM,sN);
    costDiff=abs(double(PM.piecewise_qp_cost)-double(PN.piecewise_qp_cost));

    a=PM.reference_speed_certificate; b=PN.reference_speed_certificate;
    speedDiff=max(abs([ ...
        double(a.payload_speed_UB_m_s)-double(b.payload_speed_UB_m_s), ...
        double(a.payload_accel_UB_m_s2)-double(b.payload_accel_UB_m_s2), ...
        double(a.payload_jerk_UB_m_s3)-double(b.payload_jerk_UB_m_s3), ...
        double(a.specific_force_norm_LB_m_s2)-double(b.specific_force_norm_LB_m_s2), ...
        double(a.qdot_UB_rad_s)-double(b.qdot_UB_rad_s), ...
        double(a.uav_speed_UB_m_s)-double(b.uav_speed_UB_m_s)]));

    a=PM.geometry_shadow_certificate; b=PN.geometry_shadow_certificate;
    geomDiff=max(abs([ ...
        double(a.min_payload_margin_m)-double(b.min_payload_margin_m), ...
        double(a.min_uav_margin_m)-double(b.min_uav_margin_m), ...
        double(a.min_cable_margin_m)-double(b.min_cable_margin_m), ...
        double(a.min_margin_m)-double(b.min_margin_m)]));
    geomCounts=logical(a.mathematical_pass)==logical(b.mathematical_pass) && ...
        double(a.reason_code)==double(b.reason_code) && ...
        double(a.leaf_count)==double(b.leaf_count) && ...
        double(a.split_count)==double(b.split_count) && ...
        double(a.max_depth_used)==double(b.max_depth_used);
end

% Canonical separator current true-trigger diagnostic.
ep=EM.episodes(1);
xa=double(PM.base_flat_eval_fn(double(ep.start_s))); xa=xa(:);
xb=double(PM.base_flat_eval_fn(double(ep.end_s))); xb=xb(:);
progress=xb(1:3)-xa(1:3);
if norm(progress)<=128*eps(max(1,norm(xa(1:3))))
    xm=double(PM.base_flat_eval_fn(0.5*(double(ep.start_s)+double(ep.end_s)))); xm=xm(:);
    progress=xm(5:7);
end
ii=double(ep.obstacle_index);
R=reshape(double(CTX.obstacles.rotations_flat(:,ii)),3,3);
aa=double(CTX.obstacles.base_axes(:,ii));
[nCan,META]=af7r36_canonical_separator_normal(R,aa,progress);

% Repeated-replan evaluator equality.
repeatDiff=0; repeatClosed=true;
if logical(PM.success)
    model1=af7r33_make_batch_model_from_proposal( ...
        replanner,PM,double(E.reference.version)+1);
    pkt=af7r34_make_core_packet(replanner,[],[],model1);
    tg=unique([double(PM.t0),double(PM.t1),linspace(double(PM.t0),double(PM.t1),257)]);
    YC=af7r36_single_core_mex(2,pkt,tg);
    [YM,vm]=af7r33_flat_jet_batch_model(replanner,tg,model1);
    assert(all(vm));
    repeatDiff=max(abs(double(YC)-double(YM)),[],'all');
    repeatClosed=repeatDiff<=65536*eps(max(1,max(abs(YM),[],'all')));
end

scale=max([1,abs(double(PM.t0)),abs(double(PM.t1)), ...
    finite0(epiDiff),finite0(segDiff),finite0(cpDiff),finite0(compDiff), ...
    finite0(costDiff),finite0(speedDiff),finite0(geomDiff)]);
tol=65536*eps(scale);

closed=decisionSame&&epiClosed&&structural&&signEqual&&geomCounts&&repeatClosed&& ...
    finite0(epiDiff)<=tol&&finite0(segDiff)<=tol&&finite0(cpDiff)<=tol&& ...
    finite0(compDiff)<=tol&&finite0(costDiff)<=tol&& ...
    finite0(speedDiff)<=tol&&finite0(geomDiff)<=tol;

% Warm native timing.
for k=1:5
    af7r36_single_core_proposal(replanner,CTX);
end
N=31; tw=zeros(1,N); tc=zeros(1,N);
for k=1:N
    q=tic;
    [~,RR]=af7r36_single_core_proposal(replanner,CTX);
    tw(k)=1e3*toc(q);
    tc(k)=double(RR.total_ms);
end

fprintf(' trigger time [s]                         = %.9f\n',double(E.event_time));
fprintf(' canonical separator n                    = [%.17g %.17g %.17g]^T\n',nCan);
fprintf(' canonical separator mode                 = %s\n',META.mode);
fprintf(' degenerate top eigenspace                = %d\n',META.degenerate);
fprintf(' MATLAB/native success                    = %d / %d\n',PM.success,PN.success);
fprintf(' MATLAB/native reason                     = %d / %d\n',PM.reason_code,PN.reason_code);
fprintf(' episode boundary max |diff| [s]          = %.17g\n',epiDiff);
fprintf(' segment-time max |diff| [s]              = %.17g\n',segDiff);
fprintf(' corridor sign vectors equal              = %d\n',signEqual);
fprintf(' control-point max |diff|                 = %.17g\n',cpDiff);
fprintf(' component state max |diff|               = %.17g\n',compDiff);
fprintf(' QP objective |diff|                      = %.17g\n',costDiff);
fprintf(' speed certificate max |diff|             = %.17g\n',speedDiff);
fprintf(' geometry margin max |diff| [m]           = %.17g\n',geomDiff);
fprintf(' geometry topology/counts equal           = %d\n',geomCounts);
fprintf(' repeated-replan evaluator max |diff|     = %.17g\n',repeatDiff);
fprintf(' numeric equivalence tolerance            = %.17g\n',tol);
fprintf(' canonical full-core equivalence closed   = %d\n',closed);

fprintf('\n[TIMING -- replay only, NOT Windows hard WCET]\n');
fprintf(' canonical MATLAB full replay [ms]        = %.6g\n',matlabCanonicalMs);
fprintf(' native first wrapper [ms]                = %.6g\n',nativeFirstMs);
fprintf(' native wrapper min/med/max [ms]          = %.6g / %.6g / %.6g\n',min(tw),median(tw),max(tw));
fprintf(' native C++ core min/med/max [ms]         = %.6g / %.6g / %.6g\n',min(tc),median(tc),max(tc));
fprintf(' last core episode/hess/corr/QP/build [ms]= %.6g / %.6g / %.6g / %.6g / %.6g\n', ...
    RR.episode_ms,RR.hessian_ms,RR.corridor_ms,RR.qp_ms,RR.candidate_build_ms);
fprintf(' last core candidate / total [ms]         = %.6g / %.6g\n',RR.candidate_ms,RR.total_ms);
fprintf(' production source modified               = 0\n');
fprintf('====================================================================================================\n');

if closed
    root=fileparts(which('REPLANNING_SYSTEM'));
    token=struct('schema','AF7R36_CANONICAL_CORE_TOKEN_V1','closed',true, ...
        'tol',tol,'cp_diff',cpDiff,'speed_diff',speedDiff, ...
        'geom_diff',geomDiff,'wrapper_ms',tw,'core_ms',tc);
    save(fullfile(root,'AF7R36_CANONICAL_CORE_TOKEN.mat'),'token');
end

OUT=struct('schema','AF7R36A_CANONICAL_SINGLE_CORE_DUAL_AUDIT_V1', ...
    'closed',closed,'matlab',PM,'native',PN,'native_raw',RN, ...
    'wrapper_ms',tw,'core_ms',tc,'tolerance',tol, ...
    'canonical_separator',nCan,'canonical_meta',META);
assignin('base','AF7R36A_CANONICAL_SINGLE_CORE_DUAL_AUDIT_OUT',OUT);
end

function x=finite0(x)
if isempty(x)||~isfinite(x), x=0; end
end
