function report = CBF_PREFLIGHT_WARMUP(hlc)
% CBF_PREFLIGHT_WARMUP  Flight-only physical/S3 CBF dry-run before takeoff.
%
% This function must be called AFTER an arming controller call has captured
% a nominal context, but BEFORE entering flight. It does NOT call the plant,
% advance reference time, touch the estimator/mass, or commit CBF supervisor
% and thrust dynamic-extension states. It does incur startup latency.
%
% This is JIT/solver/cache pre-execution, NOT a real-time guarantee.
% Existing CBF monitoring and active solve take longer than dt=0.025 s.
% A sample evaluation cannot establish continuous-time safety.

assert(isa(hlc,'HLC_SUSPENDED_CBF_PHYSICAL_FLIGHT'), ...
    'CBF_PREFLIGHT_WARMUP expects the flight-only HLC wrapper.');
assert(~hlc.cbfEnabled,'Preflight must not be run while CBF is enabled.');
assert(~isempty(hlc.cbfWarmupContext), ...
    'No HLC nominal context: run the arming controller once first.');
assert(any(strcmpi(hlc.cbfMode,{'physical','virtual'})) && strcmpi(hlc.cbfMethod,'S3'), ...
    'This preflight supports physical/S3 and virtual/S3 only.');
requestedMode=lower(char(hlc.cbfMode));
ctx=hlc.cbfWarmupContext;
assert(numel(ctx.x)==19 && numel(ctx.xd)==28 && numel(ctx.vf)==6 ...
    && numel(ctx.P)==9 && numel(ctx.uRaw)==4, 'Captured HLC data is malformed.');
assert(all(isfinite([ctx.x(:);ctx.xd(:);ctx.vf(:);ctx.P(:);ctx.uRaw(:)])), ...
    'Captured HLC data is not finite.');
T=tic;
% A known source tag makes an older MATLAB path/cache immediately visible.
% In a stale run this line will not print at all.
fprintf('[CBF-PREFLIGHT] SOURCE PREFLIGHT_20261015_NAV2 file=%s\n', ...
    which('CBF_PREFLIGHT_WARMUP'));
try
    src=Generate_CBF_ActuatorAware_6Methods('sourceversion');
catch ME
    error('CBF:StaleGenerator', ...
        ['The CBF generator does not implement sourceversion. ' ...
         'Overwrite the SAME Generate_CBF_ActuatorAware_6Methods.m and rerun ' ...
         'clear classes; clear functions; rehash. Actual error: %s'],ME.message);
end
if ~strcmp(src.tag,'CBF_GEN_20261015_NAV2')
    error('CBF:StaleGenerator','Expected CBF_GEN_20261015_NAV2; loaded %s from %s', ...
        src.tag,src.path);
end
fprintf('[CBF-PREFLIGHT] SOURCE %s file=%s\n',src.tag,src.path);
fprintf('[CBF-PREFLIGHT] before takeoff: numerical S3 kernel / monitor / filter / QP warmup.\n');

% Call ALL three S3 shapes for the selected input architecture on synthetic fixtures. This
% avoids the first invocation of the Taylor-jet evaluator during flight.
cfg=struct('modes',{{requestedMode}},'methods',{{'S3'}}, ...
    'writeFunctions',false,'smokeTest',true,'maxSeconds',180);
try
    smoke=Generate_CBF_ActuatorAware_6Methods('selftest',cfg);
    flags=string(smoke.status);
    if any(contains(flags,'FAILED')) || height(smoke)~=3
        error('CBF:PreflightSelftest', 'S3 evaluator smoke test did not pass all three components.');
    end
catch ME
    error('CBF:PreflightSelftest','Preflight S3 component check failed: %s',ME.message);
end

% Exercise the EXACT local function paths, not just independent quadprog:
% localPhysicalSampleBounds -> localQPWithSoftAnticipation -> quadprog,
% for both ordinary and soft-anticipatory QP. Synthetic data never reaches
% the plant, HLC command or supervisor state.
try
    qpWarm=Generate_CBF_ActuatorAware_6Methods('warmupphysicalqp',ctx.dt);
    if ~qpWarm.success
        error('CBF:PreflightQP','Synthetic physical QP exercise not successful.');
    end
catch ME
    error('CBF:PreflightQP','Cannot initialize sampled actuator QP paths: %s',ME.message);
end

opts=hlc.cbfOpts;
obs=ENVIRONMENT_OBSTACLE_ELLIPSE_MOVE(0); % obstacles must be stationary
opts.monitorOnly=true;
opts.emitEvents=false;
opts.experimentalContinueAfterPsiViolation=false;
opts.experimentalStopAtGeometricBoundary=false;
opts.preemptivePush=0;
opts.requireInitialChain=false;
% No CSV or first-loss snapshots from dry-run monitor.
try
    monitorDry=Generate_CBF_ActuatorAware_6Methods('supervise', ...
        'physical',hlc,ctx.x,ctx.xd,ctx.vf,ctx.F1,ctx.P,ctx.uRaw, ...
        obs,'S3',hlc.cbfShapes,hlc.cbfKappa,opts,struct(),ctx.dt);
    if ~monitorDry.success || numel(monitorDry.monitor.rows)~=3*numel(obs)
        error('CBF:PreflightMonitor', 'Dry-run monitor returned unexpected result.');
    end
catch ME
    error('CBF:PreflightMonitor','Physical/S3 monitor cannot initialize: %s',ME.message);
end

% Exercise the actual physical constrained map and localQP once on the
% arming state, even though it is deliberately NOT the flight state.
% Its feasibility at t=0 does NOT predict viability in flight.
opts2=hlc.cbfOpts;
opts2.requireInitialChain=false;
opts2.saveInitialFailure=false;
opts2.nominalActuatorU=localClip(ctx.uRaw,opts2.uLimits);
opts2.diagnosticOnly=false;
opts2.preemptivePush=0;
if strcmp(requestedMode,'virtual')
    aux=zeros(4,1);
else
    aux=[opts2.nominalActuatorU(1);0];
end
% Isolate a kernel/row assembly error from a map/QP assembly error.
% Diagnostic-only executes all nine physical/S3 derivatives without QP.
try
    optsAudit=opts2;
    optsAudit.diagnosticOnly=true;
    fprintf('[CBF-PREFLIGHT] phase=filter_derivatives, x=%s xd=%s vf=%s F1=%s P=%s aux=%s\n', ...
        mat2str(size(ctx.x)),mat2str(size(ctx.xd)),mat2str(size(ctx.vf)), ...
        mat2str(size(ctx.F1)),mat2str(size(ctx.P)),mat2str(size(aux)));
    diagDry=Generate_CBF_ActuatorAware_6Methods('filter', ...
        requestedMode,hlc,ctx.x,ctx.xd,reshape(ctx.vf,1,[]),ctx.F1,ctx.P,aux,obs, ...
        'S3',hlc.cbfShapes,hlc.cbfKappa,optsAudit);
    assert(numel(diagDry.rows)==3*numel(obs), ...
        'CBF:PreflightRowCount','Expected 3 rows per obstacle in the dry-run.');
    fprintf('[CBF-PREFLIGHT] phase=filter_qp mode=%s (dry map + QP)\n',requestedMode);
    filterDry=Generate_CBF_ActuatorAware_6Methods('filter', ...
        requestedMode,hlc,ctx.x,ctx.xd,reshape(ctx.vf,1,[]),ctx.F1,ctx.P,aux,obs, ...
        'S3',hlc.cbfShapes,hlc.cbfKappa,opts2);
catch ME
    fprintf(2,'[CBF-PREFLIGHT] ORIGINAL FAILURE (%s): %s\n',ME.identifier,ME.message);
    for jj=1:numel(ME.stack)
        fprintf(2,'  %s (line %d)\n',ME.stack(jj).name,ME.stack(jj).line);
    end
    % rethrow preserves the ORIGINAL source and line; don't obscure it.
    rethrow(ME);
end
if filterDry.success && isfield(opts2,'lockNominalYaw') && opts2.lockNominalYaw
    assert(abs(filterDry.uSafe(4)-opts2.nominalActuatorU(4))<=1e-6, ...
        'CBF:YawLockDryrun','Preflight QP did not preserve original HLC yaw torque.');
    fprintf('[CBF-PREFLIGHT] yaw lock verified: Mz(CBF)=Mz(HLC saturated)\n');
end
if ~filterDry.success
    fprintf(2,['[CBF-PREFLIGHT] Note: t=0 dry-run QP infeasible (or not solved): %s. ' ...
        'This is not an applied command and not evidence about flight QP feasibility.\n'],...
        filterDry.qp.reason);
end

report=struct('passed',true,'seconds',toc(T), ...
    'monitorRows',numel(monitorDry.monitor.rows), ...
    'monitorHmin',monitorDry.monitor.minH, ...
    'filterDryRunFeasible',logical(filterDry.success), ...
    'sampledQPWarmSeconds',qpWarm.seconds, ...
    'filterDryRunSeconds',filterDry.totalSeconds, ...
    'mode',requestedMode, ...
    'note','Preflight exercise only. Not a 25-ms real-time or safety certificate.');
fprintf(['[CBF-PREFLIGHT] COMPLETE before takeoff: %.3f s, ' ...
    '%d monitor rows, dryQPfeasible=%d. No CBF command applied.\n'],...
    report.seconds,report.monitorRows,report.filterDryRunFeasible);
end

function y=localClip(u,lim)
lim=lim(:);u=u(:);
y=[max(lim(1),min(lim(2),u(1))); ...
   max(-lim(3:5),min(lim(3:5),u(2:4)))];
end
