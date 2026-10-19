function AF7R58ZZY_R8_R2_KERNEL_C6_FORENSIC_AUDIT
% AF7R58ZZY-R8-R2 | EXACT KERNEL + C6 FORENSIC AUDIT
% -------------------------------------------------------------------------
% Diagnostic only. No production source is modified and no workload is
% moved into Arming, Takeoff, or Landing. The mission run preserves the
% current production equations. After the run has completed, exact observer
% snapshots and the committed batch model are replayed to distinguish
% cold/JIT/allocation cost from steady arithmetic cost.
%
% C6 audit is source-bound:
%   * current staged position must retain continuous C7 metadata (=> C6),
%   * current-run yaw must be proven C6 from the actual nominal source and
%     the canonical correction semantics,
%   * the generic staged REFERENCE_ONLY active gate is checked separately
%     for an explicit yaw-C6 revalidation contract.

clc;
fprintf('\n%s\n','====================================================================================================');
fprintf(' AF7R58ZZY-R8-R2 | EXACT KERNEL + C6 FORENSIC AUDIT\n');
fprintf('%s\n','====================================================================================================');
fprintf('[AUTHORITY] productionModified=0 | trajectoryMathChanged=0 | repairApplied=0 | postRunReplayOnly=1\n');
fprintf('[PHASE] newArmingWork=0 | newTakeoffWork=0 | newLandingWork=0 | addedWarmup=0\n');

EXPECTED_SOURCE_SHA='310cfdd01f9554a80b71e8d90464816a721859709ad7c4b9137cdc0f3a6eaf49';
EXPECTED_PARENT_SHA='2b874c2e92052de1c756c3b00f8f62f5cb08018a496b9c78b981df085f2efb18';
EXPECTED_R8_DIAG_SHA='5ca886499192ee309c0bfd7a39c2094de708e4a85104fb7a54e99b2b987e97df';
EXPECTED_RUNNER_SHA='d4d15d081ea8a6e3073b0a63ce36e47f983777ac2c81db194384de69f5e5fc37';

obsColumns={ ...
    't','total','stateFetch','lazyInit','eventGuardPhase','measurement','xhatPrev', ...
    'prevInput','oneStepTotal','oneStepParams','oneStepAttitude','oneStepLoad', ...
    'oneStepStruct','contractTotal','contractMeasured','contractVL','contractWL', ...
    'contractW','contractStruct','hiddenUpdate','noPrevMeasured','massError', ...
    'supportStruct','qPrevAssign','bridge','finalCommit','hasPrev','contractionUsed', ...
    'updateCountBefore','h','staticCacheValid','completed'};
refColumns={ ...
    't','total','assertTotal','assertDeadline','assertBasic','assertROFlatMeta', ...
    'assertSmooth','assertGeometry','assertSpeed','assertExec','setupAuthority', ...
    'cacheEligibility','shiftCalc','pathCode','shiftCount','canReuse','cacheValidBefore', ...
    'shiftAlloc','tailScalarEval','tailBatchEval','derive','validate','cacheStore', ...
    'refVersion','cacheVersionBefore','cacheT0Before','N','H','dt','safetyLevelCode', ...
    'tMinusCacheT0','completed'};
baseColumns={ ...
    't_s','do_total_ms','loadmass_ms','observer_ms','live_binding_ms','dependency_ms', ...
    'nominal_xd_ms','cert_guard_ms','active_ref_ms','geometry_ms','reference_packet_ms', ...
    'raw_sensor_ms','sensor_memory_ms','trigger_dispatch_ms','mex_kernel_ms', ...
    'trigger_decode_ms','request_block_ms','stage1_total_ms','stage1_future_ref_ms', ...
    'stage1_context_ms','stage1_proposal_ms','stage1_other_ms','advance_pipeline_ms', ...
    'pipeline_freshness_ms','stage2_refcert_ms','stage2_prepare_total_ms', ...
    'stage2_pregates_ms','stage2_batch_model_ms','stage2_sweep_ms','stage2_packet_ms', ...
    'stage2_sensor_cert_ms','stage2_handles_meta_ms','stage2_postcache_batch_ms', ...
    'stage2_postcache_derive_ms','stage2_bundle_store_ms','stage2_total_ms', ...
    'stage3_total_ms','stage3_precheck_ms','stage3_snapshot_ms','stage3_swap_guard_ms', ...
    'stage3_commit_tail_ms','output_state_ms','reserved','request_delta','commit_delta', ...
    'need_replan','known_obstacle_count','reference_version'};

scriptDir=fileparts(mfilename('fullpath'));
addpath(scriptDir,'-begin'); rehash;
srcPath=which('REPLANNING_SYSTEM');
parentPath=fullfile(scriptDir,'REPLANNING_SYSTEM_R5R4_DIAG.m');
r8Path=fullfile(scriptDir,'REPLANNING_SYSTEM_R8R2_DIAG.m');
runnerPath=which('run_flight_headless');
assert(~isempty(srcPath)&&isfile(srcPath),'[AF7R58ZZY-R8-R2] production source missing.');
assert(isfile(parentPath)&&isfile(r8Path),'[AF7R58ZZY-R8-R2] diagnostic files missing.');
assert(~isempty(runnerPath)&&isfile(runnerPath),'[AF7R58ZZY-R8-R2] runner missing.');
clear('REPLANNING_SYSTEM_R8R2_DIAG','REPLANNING_SYSTEM_R5R4_DIAG'); rehash;
assert(strcmpi(char(which('REPLANNING_SYSTEM_R8R2_DIAG')),char(r8Path)), ...
    '[AF7R58ZZY-R8-R2] script-local R8 class is not selected.');
assert(strcmpi(char(which('REPLANNING_SYSTEM_R5R4_DIAG')),char(parentPath)), ...
    '[AF7R58ZZY-R8-R2] script-local parent class is not selected.');

sourceSHA=sha256_file_r8(srcPath); parentSHA=sha256_file_r8(parentPath);
r8SHA=sha256_file_r8(r8Path); runnerSHA=sha256_file_r8(runnerPath);
assert(strcmpi(sourceSHA,EXPECTED_SOURCE_SHA),'[AF7R58ZZY-R8-R2] production SHA changed.');
assert(strcmpi(parentSHA,EXPECTED_PARENT_SHA),'[AF7R58ZZY-R8-R2] parent diagnostic SHA changed.');
assert(strcmpi(r8SHA,EXPECTED_R8_DIAG_SHA),'[AF7R58ZZY-R8-R2] R8 diagnostic SHA changed.');
assert(strcmpi(runnerSHA,EXPECTED_RUNNER_SHA),'[AF7R58ZZY-R8-R2] runner SHA changed.');
runnerText=fileread(runnerPath);
runnerSemantics=assert_runner_semantics_r8(runnerText);

% No phase-shift contract: R8 subclass may not override do()/allocation or
% call any setup/warmup routine. Snapshot assignments only observe already
% computed values inside the exact observer branch.
r8Text=fileread(r8Path);
noDoOverride=isempty(regexp(r8Text,'function\s+result\s*=\s*do\s*\(','once')) && ...
    isempty(regexp(r8Text,'function\s+\w*\s*=\s*do\s*\(','once'));
noPhaseAllocationEdit=~contains(r8Text,'cha_allocation');
noAddedWarmup=~contains(r8Text,'warmup_online_path(') && ...
    ~contains(r8Text,'observer_prepare_static_setup_cache(') && ...
    ~contains(r8Text,'prime_generator_integration_variants(');
assert(noDoOverride&&noPhaseAllocationEdit&&noAddedWarmup, ...
    '[AF7R58ZZY-R8-R2] phase-shift contract violated by diagnostic subclass.');

fprintf('\n[A. BINDING / PHASE CONTRACT]\n');
fprintf(' production SHA                 = %s\n',sourceSHA);
fprintf(' runner SHA                     = %s\n',runnerSHA);
fprintf(' runner semantics               = %d/%d\n',runnerSemantics.pass_count,runnerSemantics.total_count);
fprintf(' R8 diagnostic SHA              = %s\n',r8SHA);
fprintf(' no do() override               = %d\n',noDoOverride);
fprintf(' no phase-allocation edit       = %d\n',noPhaseAllocationEdit);
fprintf(' no added warmup/setup call     = %d\n',noAddedWarmup);

root=fileparts(srcPath);
outDir=tempname(root); mkdir(outDir);
reportPath=fullfile(outDir,'AF7R58ZZY_R8_R2_REPORT.txt');
outPath=fullfile(outDir,'AF7R58ZZY_R8_R2_OUT.mat');
rawPath=fullfile(outDir,'AF7R58ZZY_R8_R2_RAW_CAPTURE.mat');
tempRunnerPath=fullfile(root,'AF7R58ZZY_R8_R2_TEMP_RUNNER.m');
oldCtor='replanner_instance = REPLANNING_SYSTEM(agent,nominal_ref,replan_opts);';
newCtor='replanner_instance = REPLANNING_SYSTEM_R8R2_DIAG(agent,nominal_ref,replan_opts);';
assert(count_token_r8(runnerText,oldCtor)==1,'[AF7R58ZZY-R8-R2] runner constructor anchor count != 1.');
write_utf8_r8(tempRunnerPath,strrep(runnerText,oldCtor,newCtor));
cleanupRunner=onCleanup(@()delete_if_exists_r8(tempRunnerPath)); %#ok<NASGU>

runError=struct('identifier','','message','');
baseRing=nan(0,48); obsRing=nan(0,32); refRing=nan(0,32);
stagedStatus=struct(); runtime_profile=struct(); obj=[];

fprintf('\n[B. SINGLE UNMODIFIED-MATH FLIGHT RUN]\n');
try
    evalin('base','clear REPLANNING_SYSTEM_R8R2_DIAG REPLANNING_SYSTEM_R5R4_DIAG'); rehash;
    cmd=sprintf('run(''%s'')',strrep(tempRunnerPath,'''',''''''));
    evalin('base',cmd);
    obj=evalin('base','replanner_instance');
    assert(isa(obj,'REPLANNING_SYSTEM_R8R2_DIAG'),'[AF7R58ZZY-R8-R2] R8-R2 object not instantiated.');
    assert(isa(obj,'REPLANNING_SYSTEM'),'[AF7R58ZZY-R8-R2] production class contract lost.');
    nb=double(obj.af7r58zzy_r5_count); no=double(obj.af7r58zzy_r6_obs_count); nr=double(obj.af7r58zzy_r6_ref_count);
    if nb>0, baseRing=double(obj.af7r58zzy_r5_ring(1:nb,:)); end
    if no>0, obsRing=double(obj.af7r58zzy_r6_obs_ring(1:no,:)); end
    if nr>0, refRing=double(obj.af7r58zzy_r6_ref_ring(1:nr,:)); end
    stagedStatus=obj.get_staged_pipeline_status();
    if evalin('base','exist(''runtime_profile'',''var'')==1'), runtime_profile=evalin('base','runtime_profile'); end
catch ME
    runError.identifier=ME.identifier; runError.message=ME.message;
    fprintf(2,'%s%c',sprintf('[AF7R58ZZY-R8 RUN ERROR] %s | %s',ME.identifier,ME.message),10);
end

% ----------------------- C6 source/runtime audit ------------------------
C6=struct(); C6.current_position_c6_closed=false; C6.current_yaw_c6_closed=false;
C6.generic_staged_yaw_explicit_gate=false; C6.generic_staged_yaw_contract_closed=false;
C6.position_continuity_order=NaN; C6.yaw_policy=''; C6.nominal_yaw_constant_source=false;
C6.canonical_correction_leaves_yaw_untouched=false; C6.structural_residual=struct();
C6.direction_facing_yaw_intended=true; C6.direction_facing_yaw_implemented=false;
C6.nominal_horizontal_heading_defined=false; C6.vertical_nominal_requires_c6_heading_hold=false;

srcText=fileread(srcPath);
if ~isempty(obj) && isempty(runError.message)
    R=obj.active_repaired_reference; C=obj.active_repaired_certificates;
    if isstruct(C)&&isfield(C,'smoothness')&&isstruct(C.smoothness)
        if isfield(C.smoothness,'continuity_order'), C6.position_continuity_order=double(C.smoothness.continuity_order); end
        C6.current_position_c6_closed=isfield(C.smoothness,'certified')&&logical(C.smoothness.certified)&& ...
            isfield(C.smoothness,'continuous_time')&&logical(C.smoothness.continuous_time)&& ...
            isfield(C.smoothness,'sampled_acceptance')&&~logical(C.smoothness.sampled_acceptance)&& ...
            isfield(C.smoothness,'continuity_order')&&double(C.smoothness.continuity_order)>=6;
    end
    if isstruct(R)&&isfield(R,'yaw_policy'), C6.yaw_policy=char(R.yaw_policy); end

    genLinePath=which('gen_ref_line');
    canonPath=which('af7r58i_eval_canonical_reference');
    assert(~isempty(genLinePath)&&isfile(genLinePath),'[AF7R58ZZY-R8-R2] gen_ref_line.m missing.');
    assert(~isempty(canonPath)&&isfile(canonPath),'[AF7R58ZZY-R8-R2] canonical evaluator missing.');
    genText=fileread(genLinePath); canText=fileread(canonPath);
    C6.gen_ref_line_sha=sha256_file_r8(genLinePath); C6.canonical_eval_sha=sha256_file_r8(canonPath);
    C6.nominal_yaw_constant_source=contains(compact_ws_r8(genText),'yaw=atan2(dir(2),dir(1));') && ...
        contains(compact_ws_r8(genText),'ref=@(t)[pos(1);pos(2);pos(3);yaw];');
    cc=compact_ws_r8(canText);
    C6.canonical_correction_leaves_yaw_untouched=contains(cc,'xd(4*r+(1:3))=xd(4*r+(1:3))+D(:,r+1);') && ...
        ~contains(cc,'xd(4*r+4)=xd(4*r+4)+');
    C6.current_yaw_c6_closed=strcmpi(C6.yaw_policy,'C6_INDEPENDENT') && ...
        C6.nominal_yaw_constant_source && C6.canonical_correction_leaves_yaw_untouched;
    % Current runner nominal direction is +z, so horizontal heading is undefined.
    % A constant yaw is C-infinity and is the correct smooth hold for that purely
    % vertical nominal segment.  However, the staged canonical repair modifies only
    % XYZ derivative slots and contains no repaired-tangent yaw generator.  Therefore
    % when an avoidance correction creates horizontal motion, the active yaw remains
    % the nominal yaw instead of following atan2(v_y,v_x).
    rt=compact_ws_r8(runnerText);
    C6.nominal_horizontal_heading_defined=~contains(rt,'"direction",[0,0,1]');
    C6.vertical_nominal_requires_c6_heading_hold=~C6.nominal_horizontal_heading_defined;
    C6.direction_facing_yaw_implemented=~C6.canonical_correction_leaves_yaw_untouched;

    roStart=strfind(srcText,'if strcmpi(char(obj.active_repaired_safety_level),''REFERENCE_ONLY'')');
    execStart=strfind(srcText,'assert(strcmpi(char(obj.active_repaired_safety_level),''EXECUTION_AWARE'')');
    if ~isempty(roStart)&&~isempty(execStart)
        a=roStart(end); b=execStart(find(execStart>a,1,'first'));
        if ~isempty(b)
            roBranch=srcText(a:b-1);
            C6.generic_staged_yaw_explicit_gate=contains(roBranch,'yaw_c6') || contains(roBranch,'yaw_policy');
        end
    end
    C6.generic_staged_yaw_contract_closed=C6.generic_staged_yaw_explicit_gate;
    if isstruct(R)&&isfield(R,'batch_flat_model')
        C6.structural_residual=bezier_c7_structural_r8(R.batch_flat_model);
    end
end

% ------------------ Post-run exact kernel replay only -------------------
Replay=struct(); Replay.observer=struct(); Replay.batch=struct(); Replay.profile=struct();
replayError=struct('identifier','','message','');
try
    assert(~isempty(obj)&&isempty(runError.message),'run did not complete');
    nSnap=double(obj.af7r60_obs_snapshot_count);
    snaps=obj.af7r60_obs_snapshots(1:nSnap);
    Replay.observer.snapshot_count=nSnap;
    Replay.observer.events=cell(1,nSnap);
    for k=1:nSnap
        Replay.observer.events{k}=benchmark_observer_snapshot_r8(obj,snaps{k},40);
    end

    R=obj.active_repaired_reference;
    assert(isstruct(R)&&isfield(R,'batch_flat_model'),'active batch model unavailable');
    dt=double(obj.prediction_dt); H=double(obj.horizon_s); N=round(H/dt)+1;
    tc=double(obj.last_replan_commit_time)+dt;
    tq=tc+(0:N-1)*dt;
    Replay.batch=benchmark_batch_r8(obj,R,tq,30);

    % Profiler runs after the mission only. They are diagnostic and cannot
    % shift workload into any real mission phase.
    if nSnap>0
        [~,ii]=min(cellfun(@(x)abs(double(x.t)-5.10),snaps));
        Replay.profile.observer=profile_observer_r8(obj,snaps{ii},30);
    else
        Replay.profile.observer=struct('ok',false,'top',[]);
    end
    Replay.profile.batch=profile_batch_r8(obj,R,tq,10);
catch ME
    replayError.identifier=ME.identifier; replayError.message=ME.message;
    fprintf(2,'%s%c',sprintf('[AF7R58ZZY-R8 REPLAY ERROR] %s | %s',ME.identifier,ME.message),10);
end

% ------------------------- live timing roots ----------------------------
[obsDom,obsDomMs,obsDomT]=dominant_stage_r8(obsRing,[3 4 5 6 7 8 9 14 20 21 22 23 24 25 26], ...
    {'STATE_FETCH','LAZY_INIT','EVENT_GUARD_PHASE','MEASUREMENT_EVENT','XHAT_PREV','PREV_INPUT', ...
     'ONE_STEP_SUPPORT','TWO_EVENT_CONTRACTION','HIDDEN_UPDATE','NO_PREV_MEASURED','MASS_ERROR', ...
     'SUPPORT_STRUCT','Q_PREV_ASSIGN','HLC_MASS_BRIDGE','FINAL_STATE_COMMIT'});
[oneDom,oneDomMs,oneDomT]=dominant_stage_r8(obsRing,10:13, ...
    {'PARAMETER_ACCESS','ATTITUDE_EULER_BOUND','LOAD_CABLE_BOUND','ONE_STEP_STRUCT'});
[conDom,conDomMs,conDomT]=dominant_stage_r8(obsRing,15:19, ...
    {'MEASURED_BOX','VL_CONTRACTION','WL_CONTRACTION','W_CONTRACTION','CONTRACTION_STRUCT'});
stage2Rows=baseRing(isfinite(baseRing(:,36)),:);
[stage2Dom,stage2DomMs,stage2DomT]=dominant_stage_r8(stage2Rows,[25 27 28 29 30 31 32 33 34 35], ...
    {'REFERENCE_CERT','PREGATES','RESOURCE_BATCH_MODEL','SWEEP','OBSTACLE_PACKET','SENSOR_CERT', ...
     'HANDLES_META','POSTCACHE_BATCH','POSTCACHE_DERIVE','BUNDLE_STORE'});

obsComplete=~isempty(obsRing)&&any(abs(obsRing(:,1)-5.05)<1e-8)&&any(abs(obsRing(:,1)-5.10)<1e-8);
baseComplete=~isempty(baseRing)&&~isempty(stage2Rows);
replayComplete=isempty(replayError.message)&&isfield(Replay.batch,'valid')&&logical(Replay.batch.valid)&& ...
    isfield(Replay.observer,'snapshot_count')&&Replay.observer.snapshot_count>=1;
currentC6Closed=C6.current_position_c6_closed&&C6.current_yaw_c6_closed;
captureClosed=isempty(runError.message)&&obsComplete&&baseComplete&&replayComplete&&currentC6Closed;

% Cold-vs-steady classification.
Classification=struct('observer','OPEN','batch','OPEN');
if replayComplete
    liveOne=max(obsRing(:,9),[],'omitnan'); liveCon=max(obsRing(:,14),[],'omitnan');
    hotOne=[]; hotCon=[];
    for k=1:numel(Replay.observer.events)
        E=Replay.observer.events{k}; hotOne(end+1)=E.one_step_hot_median_ms; hotCon(end+1)=E.contract_hot_median_ms; %#ok<AGROW>
    end
    medOne=median(hotOne,'omitnan'); medCon=median(hotCon,'omitnan');
    Classification.observer=classify_ratio_r8(max(liveOne/live_nonzero_r8(medOne),liveCon/live_nonzero_r8(medCon)));
    liveBatch=max(stage2Rows(:,33),[],'omitnan');
    Classification.batch=classify_ratio_r8(liveBatch/live_nonzero_r8(Replay.batch.hot_median_ms));
end

RAW=struct(); RAW.schema='AF7R58ZZY_R8_R2_RAW_V1';
RAW.production_source=srcPath; RAW.production_sha=sourceSHA; RAW.runner=runnerPath; RAW.runner_sha=runnerSHA;
RAW.runner_semantics=runnerSemantics; RAW.base_columns=baseColumns; RAW.base_ring=baseRing;
RAW.observer_columns=obsColumns; RAW.observer_ring=obsRing; RAW.reference_columns=refColumns; RAW.reference_ring=refRing;
RAW.staged_status=stagedStatus; RAW.runtime_profile=runtime_profile; RAW.C6=C6; RAW.Replay=Replay;
RAW.run_error=runError; RAW.replay_error=replayError;
save(rawPath,'RAW','-v7.3');

OUT=struct(); OUT.schema='AF7R58ZZY_R8_R2_OUT_V1'; OUT.capture_closed=logical(captureClosed);
OUT.status=ternary_r8(captureClosed, ...
    ternary_r8(C6.generic_staged_yaw_contract_closed,'KERNEL_C6_FORENSIC_R8_R2_COMPLETE','KERNEL_C6_FORENSIC_R8_R2_COMPLETE__GENERIC_STAGED_YAW_GATE_OPEN'), ...
    'KERNEL_C6_FORENSIC_R8_R2_INCOMPLETE_FAIL_CLOSED');
OUT.production_modified=false; OUT.repair_applied=false; OUT.no_phase_shift=true;
OUT.C6=C6; OUT.Replay=Replay; OUT.Classification=Classification;
OUT.live_roots=struct('observer',obsDom,'observer_ms',obsDomMs,'observer_t',obsDomT, ...
    'one_step',oneDom,'one_step_ms',oneDomMs,'one_step_t',oneDomT, ...
    'contraction',conDom,'contraction_ms',conDomMs,'contraction_t',conDomT, ...
    'stage2',stage2Dom,'stage2_ms',stage2DomMs,'stage2_t',stage2DomT);
OUT.run_error=runError; OUT.replay_error=replayError; OUT.raw_capture_path=rawPath; OUT.report_path=reportPath;
save(outPath,'OUT','-v7.3');

fid=fopen(reportPath,'w'); assert(fid>=0,'[AF7R58ZZY-R8-R2] cannot open report.'); c=onCleanup(@()safe_fclose_r8(fid)); %#ok<NASGU>
say_r8(fid,'status=%s',OUT.status);
say_r8(fid,'captureClosed=%d',captureClosed);
say_r8(fid,'productionModified=0');
say_r8(fid,'repairApplied=0');
say_r8(fid,'noPhaseShift=1');
say_r8(fid,'sourceSHA=%s',sourceSHA);
say_r8(fid,'runnerSHA=%s',runnerSHA);
say_r8(fid,'r8DiagSHA=%s',r8SHA);
say_r8(fid,'runnerSemantics=%d/%d',runnerSemantics.pass_count,runnerSemantics.total_count);
say_r8(fid,'raw=%s',rawPath);
say_r8(fid,'out=%s',outPath);
say_r8(fid,'');

say_r8(fid,'[C6_CLOSURE]');
say_r8(fid,'currentPositionC6Closed=%d',C6.current_position_c6_closed);
say_r8(fid,'positionContinuityOrder=%.17g',C6.position_continuity_order);
say_r8(fid,'currentYawC6Closed=%d',C6.current_yaw_c6_closed);
say_r8(fid,'yawPolicy=%s',C6.yaw_policy);
say_r8(fid,'nominalYawConstantSource=%d',C6.nominal_yaw_constant_source);
say_r8(fid,'canonicalCorrectionLeavesYawUntouched=%d',C6.canonical_correction_leaves_yaw_untouched);
say_r8(fid,'directionFacingYawIntended=%d',C6.direction_facing_yaw_intended);
say_r8(fid,'directionFacingYawImplemented=%d',C6.direction_facing_yaw_implemented);
say_r8(fid,'nominalHorizontalHeadingDefined=%d',C6.nominal_horizontal_heading_defined);
say_r8(fid,'verticalNominalRequiresC6HeadingHold=%d',C6.vertical_nominal_requires_c6_heading_hold);
say_r8(fid,'genericStagedYawExplicitGate=%d',C6.generic_staged_yaw_explicit_gate);
say_r8(fid,'genericStagedYawContractClosed=%d',C6.generic_staged_yaw_contract_closed);
if isstruct(C6.structural_residual)&&isfield(C6.structural_residual,'groups')
    S=C6.structural_residual;
    say_r8(fid,'supplementaryBezierGroups=%d',S.groups);
    say_r8(fid,'supplementarySegments=%d',S.segments);
    say_r8(fid,'supplementaryMaxGlobalEndpointJetResidualD0D7=%.17g',S.max_global_endpoint_residual);
    say_r8(fid,'supplementaryMaxInternalJoinJetResidualD0D7=%.17g',S.max_internal_join_residual);
end
say_r8(fid,'');

say_r8(fid,'[LIVE_FLIGHT_ROOTS]');
say_r8(fid,'observerDominant=%s',obsDom);
say_r8(fid,'observerDominantMs=%.9g',obsDomMs);
say_r8(fid,'observerDominantT=%.12g',obsDomT);
say_r8(fid,'oneStepDominant=%s',oneDom);
say_r8(fid,'oneStepDominantMs=%.9g',oneDomMs);
say_r8(fid,'oneStepDominantT=%.12g',oneDomT);
say_r8(fid,'contractionDominant=%s',conDom);
say_r8(fid,'contractionDominantMs=%.9g',conDomMs);
say_r8(fid,'contractionDominantT=%.12g',conDomT);
say_r8(fid,'stage2Dominant=%s',stage2Dom);
say_r8(fid,'stage2DominantMs=%.9g',stage2DomMs);
say_r8(fid,'stage2DominantT=%.12g',stage2DomT);
say_r8(fid,'');

say_r8(fid,'[OBSERVER_POSTRUN_EXACT_REPLAY]');
say_r8(fid,'snapshotCount=%d',Replay.observer.snapshot_count);
say_r8(fid,'classification=%s',Classification.observer);
for k=1:numel(Replay.observer.events)
    E=Replay.observer.events{k};
    say_r8(fid,'event=%.12g oneFirstMs=%.9g oneHotMedianMs=%.9g contractFirstMs=%.9g contractHotMedianMs=%.9g oneExact=%d contractExact=%d', ...
        E.t,E.one_step_first_ms,E.one_step_hot_median_ms,E.contract_first_ms,E.contract_hot_median_ms,E.one_step_exact,E.contract_exact);
end
say_r8(fid,'');

say_r8(fid,'[BATCH_POSTRUN_EXACT_REPLAY]');
say_r8(fid,'classification=%s',Classification.batch);
say_r8(fid,'valid=%d',Replay.batch.valid);
say_r8(fid,'N=%d',Replay.batch.N);
say_r8(fid,'firstMs=%.9g',Replay.batch.first_ms);
say_r8(fid,'hotMedianMs=%.9g',Replay.batch.hot_median_ms);
say_r8(fid,'hotP95Ms=%.9g',Replay.batch.hot_p95_ms);
say_r8(fid,'tailScalarFirstMs=%.9g',Replay.batch.tail_scalar_first_ms);
say_r8(fid,'tailScalarHotMedianMs=%.9g',Replay.batch.tail_scalar_hot_median_ms);
say_r8(fid,'');

say_r8(fid,'[PROFILE_TOP_OBSERVER]'); print_profile_top_r8(fid,Replay.profile.observer);
say_r8(fid,''); say_r8(fid,'[PROFILE_TOP_BATCH]'); print_profile_top_r8(fid,Replay.profile.batch);
say_r8(fid,'');
say_r8(fid,'[NO_PHASE_SHIFT_PERFORMANCE_CONTRACT]');
say_r8(fid,'newArmingWork=0'); say_r8(fid,'newTakeoffWork=0'); say_r8(fid,'newLandingWork=0'); say_r8(fid,'addedWarmup=0');
say_r8(fid,'productionPerformanceChangedByR8=0'); say_r8(fid,'performanceRepairApplied=0');
say_r8(fid,'futureRepairMustNotRegressBaseline=1');
say_r8(fid,'');
say_r8(fid,'[INTERPRETATION]');
say_r8(fid,'timingIsEmpirical=1'); say_r8(fid,'postRunReplayOccursAfterMission=1'); say_r8(fid,'formalFutureWCETClaim=0');
say_r8(fid,'trajectoryMathChanged=0'); say_r8(fid,'certificateAcceptanceChanged=0'); say_r8(fid,'commitPolicyChanged=0');
if ~isempty(runError.message), say_r8(fid,'runError=%s | %s',runError.identifier,runError.message); end
if ~isempty(replayError.message), say_r8(fid,'replayError=%s | %s',replayError.identifier,replayError.message); end
safe_fclose_r8(fid); clear c;

fprintf('%s%c',sprintf('[C. RESULT]'),10);
fprintf('%s%c',sprintf(' current position C6 = %d (active order %.0f)',C6.current_position_c6_closed,C6.position_continuity_order),10);
fprintf('%s%c',sprintf(' current yaw C6      = %d | generic staged yaw gate = %d',C6.current_yaw_c6_closed,C6.generic_staged_yaw_explicit_gate),10);
fprintf('%s%c',sprintf(' direction-facing yaw = %d (0 means staged repair currently holds nominal yaw)',C6.direction_facing_yaw_implemented),10);
fprintf('%s%c',sprintf(' observer classification = %s',Classification.observer),10);
fprintf('%s%c',sprintf(' batch classification    = %s',Classification.batch),10);
fprintf('%s%c',sprintf(' status = %s',OUT.status),10);
fprintf('%s%c','RETURN THESE THREE FILES:',10);
fprintf('  %s%c  %s%c  %s%c',reportPath,10,outPath,10,rawPath,10);
if ~captureClosed
    error('[AF7R58ZZY-R8-R2] forensic audit incomplete; no repair decision accepted.');
end
end

function E=benchmark_observer_snapshot_r8(obj,S,nrep)
obj.af7r58zzy_r6_obs_capture_active=false;
T=tic; F=obj.observer_one_step_support(S.h,S.uPrev,S.I); oneFirst=1e3*toc(T);
T=tic; K=obj.observer_contract_two_events(S.C0,S.C1,S.F,S.xhat); conFirst=1e3*toc(T);
one=zeros(1,nrep); con=zeros(1,nrep);
for j=1:nrep
    T=tic; Fh=obj.observer_one_step_support(S.h,S.uPrev,S.I); one(j)=1e3*toc(T);
    T=tic; Kh=obj.observer_contract_two_events(S.C0,S.C1,S.F,S.xhat); con(j)=1e3*toc(T); %#ok<NASGU>
end
E=struct('t',double(S.t),'one_step_first_ms',oneFirst,'contract_first_ms',conFirst, ...
    'one_step_hot_median_ms',median(one),'one_step_hot_p95_ms',percentile_r8(one,95), ...
    'contract_hot_median_ms',median(con),'contract_hot_p95_ms',percentile_r8(con,95), ...
    'one_step_exact',isequaln(F,S.F)&&isequaln(Fh,S.F), ...
    'contract_exact',isequaln(K,S.K)&&isequaln(Kh,S.K));
end

function B=benchmark_batch_r8(obj,R,tq,nrep)
T=tic; [Y,V]=af7r33_flat_jet_batch_model(obj,tq,R.batch_flat_model); first=1e3*toc(T);
valid=isequal(size(Y),[21 numel(tq)])&&numel(V)==numel(tq)&&all(logical(V(:)))&&all(isfinite(double(Y)),'all');
tm=zeros(1,nrep);
for j=1:nrep
    T=tic; [Yh,Vh]=af7r33_flat_jet_batch_model(obj,tq,R.batch_flat_model); tm(j)=1e3*toc(T); %#ok<NASGU,ASGLU>
end
tTail=tq(end); T=tic; z=R.flat_eval_fn(tTail); sf=1e3*toc(T); %#ok<NASGU>
ts=zeros(1,max(20,nrep));
for j=1:numel(ts), T=tic; z=R.flat_eval_fn(tTail); ts(j)=1e3*toc(T); end %#ok<NASGU>
B=struct('valid',logical(valid),'N',numel(tq),'first_ms',first,'hot_median_ms',median(tm), ...
    'hot_p95_ms',percentile_r8(tm,95),'tail_scalar_first_ms',sf,'tail_scalar_hot_median_ms',median(ts), ...
    'same_last_replay',isequaln(Y,Yh)&&isequaln(V,Vh));
end

function P=profile_observer_r8(obj,S,nrep)
P=struct('ok',false,'top',[]);
try
    profile clear; profile on;
    for j=1:nrep
        obj.observer_one_step_support(S.h,S.uPrev,S.I);
        obj.observer_contract_two_events(S.C0,S.C1,S.F,S.xhat);
    end
    profile off; info=profile('info'); P.ok=true; P.top=top_profile_r8(info,20);
catch
    try, profile off; catch, end
end
end

function P=profile_batch_r8(obj,R,tq,nrep)
P=struct('ok',false,'top',[]);
try
    profile clear; profile on;
    for j=1:nrep, af7r33_flat_jet_batch_model(obj,tq,R.batch_flat_model); end
    profile off; info=profile('info'); P.ok=true; P.top=top_profile_r8(info,20);
catch
    try, profile off; catch, end
end
end

function T=top_profile_r8(info,n)
T=struct('name',{},'total_s',{},'calls',{});
if ~isstruct(info)||~isfield(info,'FunctionTable')||isempty(info.FunctionTable), return; end
F=info.FunctionTable; tt=[F.TotalTime]; [~,ix]=sort(tt,'descend'); ix=ix(1:min(n,numel(ix)));
for k=1:numel(ix)
    q=ix(k); T(k).name=char(F(q).FunctionName); T(k).total_s=double(F(q).TotalTime); T(k).calls=double(F(q).NumCalls); %#ok<AGROW>
end
end

function print_profile_top_r8(fid,P)
if ~isstruct(P)||~isfield(P,'ok')||~P.ok
    say_r8(fid,'profileAvailable=0');
    return;
end
say_r8(fid,'profileAvailable=1');
for k=1:numel(P.top)
    say_r8(fid,'rank=%d total_s=%.9g calls=%.0f name=%s',k,P.top(k).total_s,P.top(k).calls,P.top(k).name);
end
end

function S=bezier_c7_structural_r8(model)
S=struct('groups',0,'segments',0,'max_global_endpoint_residual',NaN,'max_internal_join_residual',NaN);
if ~isstruct(model)||~isfield(model,'corrections')||~isstruct(model.corrections), return; end
mg=0; mi=0; ng=numel(model.corrections); ns=0;
for g=1:ng
    C=model.corrections(g); ns=ns+numel(C.control_points); prevEnd=[];
    for j=1:numel(C.control_points)
        cp=double(C.control_points{j}); T=double(C.segment_t1(j)-C.segment_t0(j));
        [A,B]=endpoint_jets_r8(cp,T,7);
        if j==1, mg=max(mg,max(abs(A(:)))); end
        if j==numel(C.control_points), mg=max(mg,max(abs(B(:)))); end
        if ~isempty(prevEnd), mi=max(mi,max(abs(prevEnd(:)-A(:)))); end
        prevEnd=B;
    end
end
S.groups=ng; S.segments=ns; S.max_global_endpoint_residual=mg; S.max_internal_join_residual=mi;
end

function [A,B]=endpoint_jets_r8(cp,T,maxOrder)
A=zeros(3,maxOrder+1); B=A; Q=double(cp); deg=size(Q,2)-1; scale=1;
A(:,1)=Q(:,1); B(:,1)=Q(:,end);
for r=1:maxOrder
    Q=diff(Q,1,2); scale=scale*(deg-r+1)/T;
    A(:,r+1)=scale*Q(:,1); B(:,r+1)=scale*Q(:,end);
end
end

function say_r8(fid,fmt,varargin)
s=sprintf(fmt,varargin{:}); fprintf(fid,'%s%c',s,10);
end

function c=compact_ws_r8(s), c=regexprep(s,'\s+',''); end
function y=live_nonzero_r8(x)
if ~isfinite(x)||x<=0
    y=eps;
else
    y=x;
end
end
function s=classify_ratio_r8(r)
if ~isfinite(r), s='OPEN'; elseif r>=3, s='COLD_JIT_ALLOCATION_DOMINANT'; elseif r>=1.5, s='MIXED_COLD_AND_STEADY'; else, s='STEADY_KERNEL_DOMINANT'; end
end
function p=percentile_r8(x,q)
x=sort(double(x(isfinite(x)))); if isempty(x),p=NaN;return;end
k=1+(numel(x)-1)*q/100; a=floor(k); b=ceil(k); if a==b,p=x(a);else,p=x(a)+(k-a)*(x(b)-x(a));end
end

function S=assert_runner_semantics_r8(txt)
checks={ ...
    'dt_25ms','dt = 0.025;'; ...
    'terminal_50s','te = 50;'; ...
    'reference_only_commit','AVOIDANCE_COMMIT_POLICY = "REFERENCE_ONLY";'; ...
    'auto_commit_on','ENABLE_AUTO_REPLAN_COMMIT = true;'; ...
    'plant_true_mL_014','plant_loadmass = 0.14;'; ...
    'cable_length_2m','agent.parameter.set("cableL",2.0);'; ...
    'line_reference','{"gen_ref_line", {'; ...
    'line_speed_15','"velocity", 1.5,'; ...
    'line_direction_pos_z','"direction", [0, 0, 1]'; ...
    'sensor_range_15m','replan_opts.trigger_dist = 15.0;'; ...
    'payload_radius_05','replan_opts.r_load       = 0.5;'; ...
    'uav_radius_05','replan_opts.r_drone      = 0.5;'; ...
    'cable_radius_05','replan_opts.cable        = 0.5;'; ...
    'backend_1','replan_opts.generator_backend_flag = 1;'; ...
    'observer_on','replan_opts.ENABLE_OBSERVER_HARD_SUPPORT = true;'; ...
    'observer_sim_truth_mode','replan_opts.OBSERVER_HARD_SUPPORT_MODE = "SIM_NUMERIC_TRUTH";'; ...
    'observer_true_mass_binding','replan_opts.OBSERVER_SIM_TRUE_ML = plant_loadmass;'; ...
    'takeoff_end_5s','TAKEOFF_REFERENCE(agent,"zd",3.0,"te",5)'; ...
    'flight_start_5s','if time.t < 5.0'; ...
    'landing_start_47s','elseif time.t < 47.0'; ...
    'flight_ref_allocation','agent.cha_allocation.f.reference = ["timevarying","sload"];'; ...
    'flight_est_allocation','agent.cha_allocation.f.estimator = ["ekf","loadstate"];' ...
    };
pass=false(size(checks,1),1);
for k=1:size(checks,1), pass(k)=contains(txt,checks{k,2}); end
if ~pass(end)
    pass(end)=contains(txt,'agent.set_cha_allocation_for_all("estimator",["ekf","loadstate"]);');
end
w0=strfind(txt,'while time.t <= time.te'); w1=strfind(txt,'time.t = time.t + dt;'); orderOK=false;
if numel(w0)==1 && ~isempty(w1)
    q=w1(find(w1>w0,1,'first'));
    if ~isempty(q)
        blk=txt(w0:q);
        tok={ ...
            'do_prop_local(time,cha,cha,logger,env,agent,"sensor",1,true);', ...
            'do_prop_local(time,cha,cha,logger,env,agent,"estimator",1,true);', ...
            'do_prop_local(time,cha,cha,logger,env,agent,"reference",1,true);', ...
            'do_prop_local(time,cha,cha,logger,env,agent,"controller",1,true);', ...
            'do_prop_local(time,cha,cha,logger,env,agent,"input_transform",1,true);'};
        pos=nan(1,numel(tok));
        for j=1:numel(tok)
            z=strfind(blk,tok{j}); %#ok<STREMP>
            if numel(z)==1
                pos(j)=z;
            end
        end
        orderOK=all(isfinite(pos))&&all(diff(pos)>0);
    end
end
S=struct('closed',all([pass;orderOK]),'pass_count',nnz([pass;orderOK]), ...
    'total_count',numel([pass;orderOK]),'pass',[pass;orderOK]);
if ~S.closed, error('[AF7R58ZZY-R8-R2] current runner target semantics changed.'); end
end

function [name,val,t]=dominant_stage_r8(rows,cols,names)
name='NONE'; val=NaN; t=NaN; best=-Inf;
if isempty(rows), return; end
for i=1:size(rows,1)
    for j=1:numel(cols)
        x=rows(i,cols(j));
        if isfinite(x)&&x>best, best=x; name=names{j}; t=rows(i,1); end
    end
end
if isfinite(best), val=best; end
end

function S=reference_path_summary_r8(R)
S=struct('cache_hit',0,'shift1',0,'shiftm',0,'full_batch',0);
if isempty(R), return; end
p=R(:,14);
S.cache_hit=nnz(p==1); S.shift1=nnz(p==2); S.shiftm=nnz(p==3); S.full_batch=nnz(p==4);
end

function fprintf_numeric_row_r8(fid,r)
fprintf(fid,'row=');
for k=1:numel(r), fprintf(fid,' %.12g',r(k)); end
fprintf(fid,'\n');
end

function y=ternary_r8(c,a,b)
if c, y=a; else, y=b; end
end

function n=count_token_r8(txt,token)
n=numel(strfind(txt,token)); %#ok<STREMP>
end

function h=sha256_file_r8(path)
fid=fopen(path,'rb'); assert(fid>=0,'[AF7R58ZZY-R8-R2] cannot open file for SHA: %s',path);
c=onCleanup(@()fclose(fid)); %#ok<NASGU>
b=fread(fid,Inf,'*uint8');
md=javaMethod('getInstance','java.security.MessageDigest','SHA-256');
md.update(typecast(b,'int8')); d=md.digest(); u=mod(double(d),256);
h=lower(reshape(dec2hex(u,2).',1,[]));
end

function write_utf8_r8(path,txt)
fid=fopen(path,'w','n','UTF-8'); assert(fid>=0,'[AF7R58ZZY-R8-R2] cannot create temp runner.');
c=onCleanup(@()fclose(fid)); %#ok<NASGU>
fprintf(fid,'%s',txt);
end

function delete_if_exists_r8(path)
try, if isfile(path), delete(path); end, catch, end
end

function safe_fclose_r8(fid)
try, if fid>0, fclose(fid); end, catch, end
end
