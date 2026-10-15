function OUT = AF7R32B_TRUE_TRIGGER_WHOLEBODY_SHADOW_AUDIT(replanner)
clc;
fprintf('\n====================================================================================================\n');
fprintf(' AF7R32B | TRUE-TRIGGER WHOLE-BODY BATCH SHADOW AUDIT\n');
fprintf('====================================================================================================\n');
fprintf('[AUTHORITY] continuousTheoremSame=1 | breadthBatchOnly=1 | sampledSafetyAcceptance=0 | productionModification=0\n');

assert(isa(replanner,'REPLANNING_SYSTEM'));
assert(exist('af7r31_geometry_batch_mex','file')==3,'[AF7R32B] geometry batch MEX missing.');

E=replanner.get_first_trigger_event();
assert(isstruct(E)&&isfield(E,'valid')&&logical(E.valid)&&logical(E.real_flight_request_edge), ...
    '[AF7R32B] no real first trigger event.');
CTX=af7r32_replay_context(replanner,E);

% Recompute the production proposal outside the hard deadline.  This is a
% read-only replay of the real trigger and gives the exact candidate that the
% current theorem stack selects when timing is not allowed to discard it.
q=tic;
P=replanner.builtin_bezier_d15_c7_proposal(CTX);
proposalReplayMs=1e3*toc(q);
assert(isstruct(P)&&isfield(P,'success')&&logical(P.success), ...
    '[AF7R32B] legacy true-trigger replay did not produce a certified proposal.');

% Warm both whole-body implementations.
replanner.build_static_continuous_wholebody_shadow_certificate(CTX,P);
af7r32_wholebody_batch_shadow(replanner,CTX,P);

NREP=21; tl=zeros(1,NREP); tb=zeros(1,NREP);
for k=1:NREP
    q=tic; GL=replanner.build_static_continuous_wholebody_shadow_certificate(CTX,P); tl(k)=1e3*toc(q);
    q=tic; GB=af7r32_wholebody_batch_shadow(replanner,CTX,P); tb(k)=1e3*toc(q);
end

fieldsBool={'mathematical_pass','certified','static_separation_open','representability_open'};
sameBool=true;
for k=1:numel(fieldsBool)
    f=fieldsBool{k};
    sameBool=sameBool && isequal(logical(GL.(f)),logical(GB.(f)));
end
sameCounts=double(GL.leaf_count)==double(GB.leaf_count) && ...
    double(GL.accepted_leaf_count)==double(GB.accepted_leaf_count) && ...
    double(GL.split_count)==double(GB.split_count) && ...
    double(GL.max_depth_used)==double(GB.max_depth_used);

numFields={'min_margin_m','min_payload_margin_m','min_uav_margin_m','min_cable_margin_m', ...
    'min_static_payload_margin_m','min_static_uav_margin_m','min_static_cable_margin_m', ...
    'min_theorem_certifiable_radius_s'};
d=zeros(size(numFields));
for k=1:numel(numFields)
    a=double(GL.(numFields{k})); b=double(GB.(numFields{k}));
    if isinf(a)&&isinf(b)&&sign(a)==sign(b), d(k)=0; else, d(k)=abs(a-b); end
end
maxDiff=max(d);
scale=max([1,maxDiff,abs(double(GL.min_margin_m)),abs(double(GB.min_margin_m))]);
tol=8192*eps(scale);
closed=sameBool&&sameCounts&&double(GL.reason_code)==double(GB.reason_code)&&maxDiff<=tol;

% Candidate certificate substage forensic timing on the SAME selected P.
LIV=CTX.live;
rhoBody=max([double(LIV.r_drone), ...
    double(LIV.cable_length)+double(LIV.r_load), ...
    double(LIV.cable_length)+double(LIV.r_cable)]);
VQlimit=(double(LIV.sensor_range)-rhoBody)/(double(LIV.horizon)+double(LIV.dt));
baseSC=replanner.active_speed_certificate;
targetVersion=double(E.reference.version)+1;

Ns=11; tTight=zeros(1,Ns); tLocal=zeros(1,Ns); tRejoin=zeros(1,Ns);
for k=1:Ns
    q=tic;
    replanner.tighten_piecewise_derivative_bounds_for_speed( ...
        P.piecewise_segment_t0,P.piecewise_segment_t1,P.piecewise_control_points, ...
        baseSC,VQlimit,6);
    tTight(k)=1e3*toc(q);

    q=tic;
    replanner.build_piecewise_local_vector_speed_certificate( ...
        P,P.piecewise_segment_t0,P.piecewise_segment_t1,P.piecewise_control_points, ...
        CTX,baseSC,VQlimit,targetVersion);
    tLocal(k)=1e3*toc(q);

    q=tic;
    replanner.reference_rejoin_cell_continuously_safe(P,P.t1,CTX);
    tRejoin(k)=1e3*toc(q);
end

fprintf(' trigger time [s]                         = %.9f\n',double(E.event_time));
fprintf(' legacy proposal replay time [ms]         = %.6g\n',proposalReplayMs);
fprintf(' legacy/batch pass                        = %d / %d\n',GL.mathematical_pass,GB.mathematical_pass);
fprintf(' legacy/batch leaf count                  = %d / %d\n',GL.leaf_count,GB.leaf_count);
fprintf(' legacy/batch split count                 = %d / %d\n',GL.split_count,GB.split_count);
fprintf(' legacy/batch max depth                   = %d / %d\n',GL.max_depth_used,GB.max_depth_used);
fprintf(' max certificate numeric |diff|           = %.17g\n',maxDiff);
fprintf(' equivalence tolerance                    = %.17g\n',tol);
fprintf(' whole-body legacy min/med/max [ms]       = %.6g / %.6g / %.6g\n',min(tl),median(tl),max(tl));
fprintf(' whole-body batch  min/med/max [ms]       = %.6g / %.6g / %.6g\n',min(tb),median(tb),max(tb));
fprintf(' derivative-tighten med [ms]              = %.6g\n',median(tTight));
fprintf(' R9 local-vector med [ms]                 = %.6g\n',median(tLocal));
fprintf(' rejoin continuous med [ms]               = %.6g\n',median(tRejoin));
fprintf(' whole-body shadow equivalence closed     = %d\n',closed);
fprintf(' production source modified               = 0\n');
fprintf('====================================================================================================\n');

OUT=struct('schema','AF7R32B_TRUE_TRIGGER_WHOLEBODY_SHADOW_AUDIT_V1', ...
    'closed',closed,'legacy',GL,'batch',GB,'legacy_ms',tl,'batch_ms',tb, ...
    'tighten_ms',tTight,'local_vector_ms',tLocal,'rejoin_ms',tRejoin, ...
    'max_numeric_diff',maxDiff,'tolerance',tol,'proposal_replay_ms',proposalReplayMs);
assignin('base','AF7R32B_TRUE_TRIGGER_WHOLEBODY_SHADOW_AUDIT_OUT',OUT);
end
