function OUT = AF7R34A_TRUE_TRIGGER_SINGLE_CORE_AUDIT(replanner)
clc;
fprintf('\n====================================================================================================\n');
fprintf(' AF7R34A | TRUE-TRIGGER R14 SINGLE-CALL CORE DUAL AUDIT\n');
fprintf('====================================================================================================\n');
fprintf('[AUTHORITY] shadowOnly=1 | fullHeavyCore=1 | sampledSafetyAcceptance=0 | productionModification=0\n');

assert(isa(replanner,'REPLANNING_SYSTEM'));
assert(exist('af7r34_single_core_mex','file')==3,'[AF7R34A] core MEX missing.');

A0=AF7R34_PRIME_SINGLE_CORE_REFERENCE(replanner);
assert(A0.closed,'[AF7R34A] nominal AST prime not closed.');

E=replanner.get_first_trigger_event();
assert(isstruct(E)&&isfield(E,'valid')&&logical(E.valid)&& ...
    isfield(E,'real_flight_request_edge')&&logical(E.real_flight_request_edge), ...
    '[AF7R34A] no real first Flight trigger stored.');
CTX=af7r32_replay_context(replanner,E);

% Legacy/current-production replay outside hard deadline.
q=tic;
PL=replanner.builtin_bezier_d15_c7_proposal(CTX);
legacyReplayMs=1e3*toc(q);

% Native single-call proposal.
q=tic;
[PN,RN]=af7r34_single_core_proposal(replanner,CTX);
nativeWrapperFirstMs=1e3*toc(q);

decisionSame=logical(PL.success)==logical(PN.success) && ...
    double(PL.reason_code)==double(PN.reason_code);

segClosed=false; cpDiff=NaN; segDiff=NaN; compDiff=NaN; speedDiff=NaN;
geomDiff=NaN; geomCounts=false; qpDiff=NaN;

if logical(PL.success)&&logical(PN.success)
    segClosed=numel(PL.piecewise_control_points)==numel(PN.piecewise_control_points) && ...
        numel(PL.piecewise_segment_t0)==numel(PN.piecewise_segment_t0);
    if segClosed
        segDiff=max([ ...
            max(abs(double(PL.piecewise_segment_t0(:))-double(PN.piecewise_segment_t0(:))),[],'omitnan'), ...
            max(abs(double(PL.piecewise_segment_t1(:))-double(PN.piecewise_segment_t1(:))),[],'omitnan')]);
        cpDiff=0;
        for k=1:numel(PL.piecewise_control_points)
            A=double(PL.piecewise_control_points{k});
            B=double(PN.piecewise_control_points{k});
            if ~isequal(size(A),size(B)), segClosed=false; break; end
            cpDiff=max(cpDiff,max(abs(A-B),[],'all'));
        end
    end
    compDiff=max([ ...
        max(abs(double(PL.piecewise_component_start(:))-double(PN.piecewise_component_start(:))),[],'omitnan'), ...
        max(abs(double(PL.piecewise_component_end(:))-double(PN.piecewise_component_end(:))),[],'omitnan'), ...
        max(abs(double(PL.piecewise_component_position(:))-double(PN.piecewise_component_position(:))),[],'omitnan'), ...
        max(abs(double(PL.piecewise_component_velocity(:))-double(PN.piecewise_component_velocity(:))),[],'omitnan')]);
    qpDiff=abs(double(PL.piecewise_qp_cost)-double(PN.piecewise_qp_cost));

    a=PL.reference_speed_certificate; b=PN.reference_speed_certificate;
    speedDiff=max(abs([ ...
        double(a.payload_speed_UB_m_s)-double(b.payload_speed_UB_m_s), ...
        double(a.payload_accel_UB_m_s2)-double(b.payload_accel_UB_m_s2), ...
        double(a.payload_jerk_UB_m_s3)-double(b.payload_jerk_UB_m_s3), ...
        double(a.specific_force_norm_LB_m_s2)-double(b.specific_force_norm_LB_m_s2), ...
        double(a.qdot_UB_rad_s)-double(b.qdot_UB_rad_s), ...
        double(a.uav_speed_UB_m_s)-double(b.uav_speed_UB_m_s)]));

    a=PL.geometry_shadow_certificate; b=PN.geometry_shadow_certificate;
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
else
    segClosed=decisionSame;
    cpDiff=0; segDiff=0; compDiff=0; speedDiff=0; geomDiff=0; qpDiff=0;
    geomCounts=true;
end

% Episode boundaries / components.
epiClosed=false; epiDiff=Inf;
EL=PL.unsafe_episode_certificate;
EN=PN.unsafe_episode_certificate;
if isstruct(EL)&&isstruct(EN)&& ...
        isfield(EL,'episode_count')&&isfield(EN,'episode_count')&& ...
        double(EL.episode_count)==double(EN.episode_count)&& ...
        double(EL.component_count)==double(EN.component_count)
    epiClosed=true; epiDiff=0;
    for k=1:double(EL.episode_count)
        epiClosed=epiClosed && ...
            double(EL.episodes(k).obstacle_index)==double(EN.episodes(k).obstacle_index);
        epiDiff=max(epiDiff,abs(double(EL.episodes(k).start_s)-double(EN.episodes(k).start_s)));
        epiDiff=max(epiDiff,abs(double(EL.episodes(k).end_s)-double(EN.episodes(k).end_s)));
    end
end

scale=max([1,abs(double(PL.t0)),abs(double(PL.t1)), ...
    finite0(cpDiff),finite0(compDiff),finite0(speedDiff),finite0(geomDiff),finite0(qpDiff)]);
tol=65536*eps(scale);
closed=decisionSame&&epiClosed&&segClosed&&geomCounts&& ...
    finite0(epiDiff)<=tol&&finite0(segDiff)<=tol&&finite0(cpDiff)<=tol&& ...
    finite0(compDiff)<=tol&&finite0(speedDiff)<=tol&&finite0(geomDiff)<=tol&& ...
    finite0(qpDiff)<=tol;

% Repeated-replan evaluator check using a real certified repair as the first
% correction model. This does not commit it.
repeatClosed=false; repeatDiff=NaN;
if logical(PL.success)
    model1=af7r33_make_batch_model_from_proposal( ...
        replanner,PL,double(E.reference.version)+1);
    pkt=af7r34_make_core_packet(replanner,[],[],model1);
    tg=unique([double(PL.t0),double(PL.t1), ...
        linspace(double(PL.t0),double(PL.t1),257)]);
    YC=af7r34_single_core_mex(2,pkt,tg);
    [YM,vm]=af7r33_flat_jet_batch_model(replanner,tg,model1);
    assert(all(vm));
    repeatDiff=max(abs(double(YC)-double(YM)),[],'all');
    rscale=max(1,max(abs(YM),[],'all'));
    repeatClosed=repeatDiff<=65536*eps(rscale);
else
    repeatClosed=true; repeatDiff=0;
end
closed=closed&&repeatClosed;

% Warm timing. This is not a Windows hard-WCET claim.
for k=1:5
    af7r34_single_core_proposal(replanner,CTX);
end
N=31;
tw=zeros(1,N); tc=zeros(1,N);
for k=1:N
    q=tic;
    [~,R]=af7r34_single_core_proposal(replanner,CTX);
    tw(k)=1e3*toc(q);
    tc(k)=double(R.total_ms);
end

fprintf(' trigger time [s]                         = %.9f\n',double(E.event_time));
fprintf(' legacy/native success                    = %d / %d\n',PL.success,PN.success);
fprintf(' legacy/native reason                     = %d / %d\n',PL.reason_code,PN.reason_code);
fprintf(' episode boundary max |diff| [s]          = %.17g\n',epiDiff);
fprintf(' segment-time max |diff| [s]              = %.17g\n',segDiff);
fprintf(' control-point max |diff|                 = %.17g\n',cpDiff);
fprintf(' component state max |diff|               = %.17g\n',compDiff);
fprintf(' QP objective |diff|                      = %.17g\n',qpDiff);
fprintf(' speed certificate max |diff|             = %.17g\n',speedDiff);
fprintf(' geometry margin max |diff| [m]           = %.17g\n',geomDiff);
fprintf(' geometry topology/counts equal           = %d\n',geomCounts);
fprintf(' repeated-replan evaluator max |diff|     = %.17g\n',repeatDiff);
fprintf(' equivalence tolerance                    = %.17g\n',tol);
fprintf(' single-core equivalence closed           = %d\n',closed);
fprintf('\n[TIMING -- replay only, not hard WCET]\n');
fprintf(' legacy full replay [ms]                  = %.6g\n',legacyReplayMs);
fprintf(' native first wrapper [ms]                = %.6g\n',nativeWrapperFirstMs);
fprintf(' native wrapper min/med/max [ms]          = %.6g / %.6g / %.6g\n',min(tw),median(tw),max(tw));
fprintf(' native C++ core min/med/max [ms]         = %.6g / %.6g / %.6g\n',min(tc),median(tc),max(tc));
fprintf(' last core episode/hess/corr/QP/build [ms]= %.6g / %.6g / %.6g / %.6g / %.6g\n', ...
    R.episode_ms,R.hessian_ms,R.corridor_ms,R.qp_ms,R.candidate_build_ms);
fprintf(' last core candidate / total [ms]         = %.6g / %.6g\n',R.candidate_ms,R.total_ms);
fprintf(' production source modified               = 0\n');
fprintf('====================================================================================================\n');

if closed
    root=fileparts(which('REPLANNING_SYSTEM'));
    token=struct('schema','AF7R34_SINGLE_CORE_TOKEN_V1', ...
        'closed',true,'trigger_time',double(E.event_time), ...
        'tolerance',tol,'episode_diff',epiDiff,'cp_diff',cpDiff, ...
        'speed_diff',speedDiff,'geometry_diff',geomDiff, ...
        'repeat_diff',repeatDiff,'wrapper_ms',tw,'core_ms',tc);
    save(fullfile(root,'AF7R34_SINGLE_CORE_TOKEN.mat'),'token');
end

OUT=struct('schema','AF7R34A_TRUE_TRIGGER_SINGLE_CORE_AUDIT_V1', ...
    'closed',closed,'legacy',PL,'native',PN,'raw',RN, ...
    'wrapper_ms',tw,'core_ms',tc,'tolerance',tol, ...
    'repeat_closed',repeatClosed,'repeat_diff',repeatDiff);
assignin('base','AF7R34A_TRUE_TRIGGER_SINGLE_CORE_AUDIT_OUT',OUT);
end

function x=finite0(x)
if isempty(x)||~isfinite(x), x=0; end
end
