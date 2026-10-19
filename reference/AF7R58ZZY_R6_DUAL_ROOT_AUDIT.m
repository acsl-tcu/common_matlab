function AF7R58ZZY_R6_DUAL_ROOT_AUDIT
% AF7R58ZZY-R6 | OBSERVER + POST-COMMIT REFERENCE ROOT-CAUSE AUDIT
% -------------------------------------------------------------------------
% One Flight run, diagnostic-only.
% Production REPLANNING_SYSTEM.m is never modified.
%
% Root questions:
%   A) Which exact observer sub-block causes the 5.05/5.10 s spike?
%   B) After Stage-2 prepares the post-commit packet, does 9.925 s actually
%      hit the prepared cache? If yes, what dominates before/inside the hit?
%   C) On later 1-dt rolling updates, is the cost the safety assertion,
%      tail scalar flat-evaluation, validation, or a hidden full batch?
%
% This audit does NOT change trajectory equations, continuous certificates,
% sensing geometry, activation time, stale rejection, or commit policy.

clc;
fprintf('\n%s\n','====================================================================================================');
fprintf(' AF7R58ZZY-R6 | DUAL ROOT-CAUSE AUDIT\n');
fprintf('%s\n','====================================================================================================');
fprintf('[AUTHORITY] productionModified=0 | diagnosticSubclassChain=1 | isaProduction=1 | oneFlightRun=1 | empiricalTimingOnly=1\n');
fprintf('[TARGETS] observer=4.95--5.20 s | repairedPacket=9.90--10.05 s | Stage2 context retained from R5-R4\n');

EXPECTED_SOURCE_SHA = '310cfdd01f9554a80b71e8d90464816a721859709ad7c4b9137cdc0f3a6eaf49';
EXPECTED_R4_DIAG_SHA = '2b874c2e92052de1c756c3b00f8f62f5cb08018a496b9c78b981df085f2efb18';
EXPECTED_R6_DIAG_SHA = '38e2c2a9a1cd994586ddbe7efd1ddb75e36d4673ceb464a80eb7ea216c587f8c';
EXPECTED_CURRENT_RUNNER_SHA = 'd4d15d081ea8a6e3073b0a63ce36e47f983777ac2c81db194384de69f5e5fc37';

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
addpath(scriptDir,'-begin');
rehash;
srcPath=which('REPLANNING_SYSTEM');
parentDiagPath=fullfile(scriptDir,'REPLANNING_SYSTEM_R5R4_DIAG.m');
r6DiagPath=fullfile(scriptDir,'REPLANNING_SYSTEM_R6_DIAG.m');
runnerPath=which('run_flight_headless');
assert(~isempty(srcPath)&&isfile(srcPath),'[AF7R58ZZY-R6] REPLANNING_SYSTEM.m not found.');
assert(isfile(parentDiagPath),'[AF7R58ZZY-R6] script-local R5-R4 diagnostic parent missing.');
assert(isfile(r6DiagPath),'[AF7R58ZZY-R6] script-local R6 diagnostic class missing.');
assert(~isempty(runnerPath)&&isfile(runnerPath),'[AF7R58ZZY-R6] run_flight_headless.m not found.');

clear('REPLANNING_SYSTEM_R6_DIAG','REPLANNING_SYSTEM_R5R4_DIAG');
rehash;
assert(strcmpi(char(which('REPLANNING_SYSTEM_R6_DIAG')),char(r6DiagPath)), ...
    '[AF7R58ZZY-R6] MATLAB did not resolve script-local R6 diagnostic class.');
assert(strcmpi(char(which('REPLANNING_SYSTEM_R5R4_DIAG')),char(parentDiagPath)), ...
    '[AF7R58ZZY-R6] MATLAB did not resolve script-local R5-R4 parent class.');

sourceSHA=sha256_file_r6(srcPath);
parentDiagSHA=sha256_file_r6(parentDiagPath);
r6DiagSHA=sha256_file_r6(r6DiagPath);
runnerSHA=sha256_file_r6(runnerPath);

fprintf('\n[A. EXACT SOURCE BINDING]\n');
fprintf(' production source    = %s\n',srcPath);
fprintf(' production SHA       = %s\n',sourceSHA);
fprintf(' parent diagnostic SHA= %s\n',parentDiagSHA);
fprintf(' R6 diagnostic SHA    = %s\n',r6DiagSHA);
fprintf(' runner SHA           = %s\n',runnerSHA);
assert(strcmpi(sourceSHA,EXPECTED_SOURCE_SHA),'[AF7R58ZZY-R6] production authority SHA changed.');
assert(strcmpi(parentDiagSHA,EXPECTED_R4_DIAG_SHA),'[AF7R58ZZY-R6] R5-R4 parent diagnostic SHA changed.');
assert(strcmpi(r6DiagSHA,EXPECTED_R6_DIAG_SHA),'[AF7R58ZZY-R6] R6 diagnostic SHA mismatch.');
assert(strcmpi(runnerSHA,EXPECTED_CURRENT_RUNNER_SHA),'[AF7R58ZZY-R6] current runner SHA changed.');

runnerText=fileread(runnerPath);
runnerSemantics=assert_runner_semantics_r6(runnerText);
fprintf(' runner target semantics closed = %d (%d/%d)\n', ...
    runnerSemantics.closed,runnerSemantics.pass_count,runnerSemantics.total_count);

D=fileread(r6DiagPath);
assert(contains(D,'classdef REPLANNING_SYSTEM_R6_DIAG < REPLANNING_SYSTEM_R5R4_DIAG'), ...
    '[AF7R58ZZY-R6] R6 class inheritance changed.');
assert(contains(D,EXPECTED_SOURCE_SHA)&&contains(D,EXPECTED_R4_DIAG_SHA), ...
    '[AF7R58ZZY-R6] R6 class authority comments not source-bound.');
% Equation/contract anchors copied from exact production source.
anchors={ ...
    'sqrtVend=sqrt(V0)+tauNorm*h/sqrt(2*Jmin);', ...
    'Aquad=(Pnorm^3)*m*L/den;', ...
    'rVL=0.5*ALmax*h+(norm(n0(ipL))+norm(n1(ipL)))/h;', ...
    'EyNorm2=obj.observer_nextfloat_up(', ...
    'obj.assert_hard_modify_deadline_limit();', ...
    'obj.validate_active_speed_certificate(obj.active_speed_certificate);', ...
    '[Y,valid]=af7r33_flat_jet_batch_model(obj,tq,R.batch_flat_model);', ...
    'xf=double(R.flat_eval_fn(t0+(N-1)*dt));'};
for k=1:numel(anchors)
    assert(contains(D,anchors{k}),'[AF7R58ZZY-R6] diagnostic equation/contract anchor missing: %s',anchors{k});
end
fprintf(' diagnostic equation/contract anchors = %d/%d\n',numel(anchors),numel(anchors));

root=fileparts(srcPath);
outDir=tempname(root); mkdir(outDir);
reportPath=fullfile(outDir,'AF7R58ZZY_R6_REPORT.txt');
outPath=fullfile(outDir,'AF7R58ZZY_R6_OUT.mat');
rawPath=fullfile(outDir,'AF7R58ZZY_R6_RAW_CAPTURE.mat');
tempRunnerPath=fullfile(root,'AF7R58ZZY_R6_TEMP_RUNNER.m');

oldCtor='replanner_instance = REPLANNING_SYSTEM(agent,nominal_ref,replan_opts);';
newCtor='replanner_instance = REPLANNING_SYSTEM_R6_DIAG(agent,nominal_ref,replan_opts);';
assert(count_token_r6(runnerText,oldCtor)==1,'[AF7R58ZZY-R6] runner constructor anchor count is not one.');
runnerDiag=strrep(runnerText,oldCtor,newCtor);
write_utf8_r6(tempRunnerPath,runnerDiag);
cleanupRunner=onCleanup(@()delete_if_exists_r6(tempRunnerPath)); %#ok<NASGU>

runError=struct('identifier','','message','');
baseRing=nan(0,48); obsRing=nan(0,32); refRing=nan(0,32);
stagedStatus=struct(); runtime_profile=struct();

fprintf('\n[B. SINGLE FLIGHT RUN]\n');
fprintf(' production source writes = 0\n');
fprintf(' temporary runner = %s\n',tempRunnerPath);
try
    evalin('base','clear REPLANNING_SYSTEM_R6_DIAG REPLANNING_SYSTEM_R5R4_DIAG');
    rehash;
    cmd=sprintf('run(''%s'')',strrep(tempRunnerPath,'''',''''''));
    evalin('base',cmd);
    obj=evalin('base','replanner_instance');
    assert(isa(obj,'REPLANNING_SYSTEM_R6_DIAG'),'[AF7R58ZZY-R6] R6 diagnostic object not instantiated.');
    assert(isa(obj,'REPLANNING_SYSTEM'),'[AF7R58ZZY-R6] production REPLANNING_SYSTEM type contract lost.');

    nb=double(obj.af7r58zzy_r5_count);
    no=double(obj.af7r58zzy_r6_obs_count);
    nr=double(obj.af7r58zzy_r6_ref_count);
    if nb>0, baseRing=double(obj.af7r58zzy_r5_ring(1:nb,:)); end
    if no>0, obsRing=double(obj.af7r58zzy_r6_obs_ring(1:no,:)); end
    if nr>0, refRing=double(obj.af7r58zzy_r6_ref_ring(1:nr,:)); end
    stagedStatus=obj.get_staged_pipeline_status();
    if evalin('base','exist(''runtime_profile'',''var'')==1')
        runtime_profile=evalin('base','runtime_profile');
    end
catch ME
    runError.identifier=ME.identifier; runError.message=ME.message;
    fprintf(2,'[AF7R58ZZY-R6 RUN ERROR] %s | %s\n',ME.identifier,ME.message);
    try
        if evalin('base','exist(''replanner_instance'',''var'')==1')
            obj=evalin('base','replanner_instance');
            nb=double(obj.af7r58zzy_r5_count); no=double(obj.af7r58zzy_r6_obs_count); nr=double(obj.af7r58zzy_r6_ref_count);
            if nb>0, baseRing=double(obj.af7r58zzy_r5_ring(1:nb,:)); end
            if no>0, obsRing=double(obj.af7r58zzy_r6_obs_ring(1:no,:)); end
            if nr>0, refRing=double(obj.af7r58zzy_r6_ref_ring(1:nr,:)); end
            stagedStatus=obj.get_staged_pipeline_status();
        end
    catch
    end
end

RAW=struct();
RAW.schema='AF7R58ZZY_R6_RAW_V1';
RAW.production_source=srcPath; RAW.production_sha=sourceSHA;
RAW.parent_diag_sha=parentDiagSHA; RAW.r6_diag_sha=r6DiagSHA;
RAW.runner=runnerPath; RAW.runner_sha=runnerSHA;
RAW.runner_semantics=runnerSemantics;
RAW.base_columns=baseColumns; RAW.base_ring=baseRing;
RAW.observer_columns=obsColumns; RAW.observer_ring=obsRing;
RAW.reference_columns=refColumns; RAW.reference_ring=refRing;
RAW.staged_status=stagedStatus; RAW.runtime_profile=runtime_profile;
RAW.run_error=runError;
save(rawPath,'RAW','-v7.3');

% --------------------------- analysis -----------------------------------
obsTopCols=[3 4 5 6 7 8 9 14 20 21 22 23 24 25 26];
obsTopNames={'STATE_FETCH','LAZY_INIT','EVENT_GUARD_PHASE','MEASUREMENT_EVENT','XHAT_PREV', ...
    'PREV_INPUT','ONE_STEP_SUPPORT','TWO_EVENT_CONTRACTION','HIDDEN_UPDATE','NO_PREV_MEASURED', ...
    'MASS_ERROR','SUPPORT_STRUCT','Q_PREV_ASSIGN','HLC_MASS_BRIDGE','FINAL_STATE_COMMIT'};
[obsDom,obsDomMs,obsDomT]=dominant_stage_r6(obsRing,obsTopCols,obsTopNames);
[oneDom,oneDomMs,oneDomT]=dominant_stage_r6(obsRing,10:13, ...
    {'PARAMETER_ACCESS','ATTITUDE_EULER_BOUND','LOAD_CABLE_BOUND','ONE_STEP_STRUCT'});
[conDom,conDomMs,conDomT]=dominant_stage_r6(obsRing,15:19, ...
    {'MEASURED_BOX','VL_CONTRACTION','WL_CONTRACTION','W_CONTRACTION','CONTRACTION_STRUCT'});

refTopCols=[3 11 12 13 18 19 20 21 22 23];
refTopNames={'ACTIVE_REFERENCE_ASSERT','AUTHORITY_SETUP','CACHE_ELIGIBILITY','SHIFT_CALC', ...
    'SHIFT_ALLOC','TAIL_SCALAR_EVAL','TAIL_BATCH_EVAL','DERIVE','VALIDATE','CACHE_STORE'};
[refDom,refDomMs,refDomT]=dominant_stage_r6(refRing,refTopCols,refTopNames);
[assertDom,assertDomMs,assertDomT]=dominant_stage_r6(refRing,4:10, ...
    {'DEADLINE_GUARD','BASIC_REFERENCE','RO_FLAT_META','SMOOTHNESS_META','GEOMETRY_META', ...
     'ACTIVE_SPEED_CERT','EXECUTION_AWARE_ASSERTS'});

stage2Rows=baseRing(isfinite(baseRing(:,36)),:);
stage3Rows=baseRing(isfinite(baseRing(:,37)),:);
stage1Rows=baseRing(isfinite(baseRing(:,18)),:);
[stage2Dom,stage2DomMs,stage2DomT]=dominant_stage_r6(stage2Rows, ...
    [25 27 28 29 30 31 32 33 34 35], ...
    {'REFERENCE_CERT','PREGATES','RESOURCE_BATCH_MODEL','SWEEP','OBSTACLE_PACKET', ...
     'SENSOR_CERT','HANDLES_META','POSTCACHE_BATCH','POSTCACHE_DERIVE','BUNDLE_STORE'});

obsComplete=~isempty(obsRing)&&all(obsRing(:,32)==1)&&any(abs(obsRing(:,1)-5.05)<1e-8)&&any(abs(obsRing(:,1)-5.10)<1e-8);
refComplete=~isempty(refRing)&&all(refRing(:,32)==1)&&any(refRing(:,1)>=9.925-1e-8)&&all(isfinite(refRing(:,14)));
baseComplete=~isempty(baseRing)&&~isempty(stage1Rows)&&~isempty(stage2Rows)&&~isempty(stage3Rows);
captureClosed=isempty(runError.message)&&obsComplete&&refComplete&&baseComplete&& ...
    ~strcmp(obsDom,'NONE')&&~strcmp(refDom,'NONE')&&~strcmp(stage2Dom,'NONE');

OUT=struct(); OUT.schema='AF7R58ZZY_R6_OUT_V1';
OUT.status=ternary_r6(captureClosed,'DUAL_ROOT_CAUSE_R6_COMPLETE','DUAL_ROOT_CAUSE_R6_INCOMPLETE_FAIL_CLOSED');
OUT.capture_closed=logical(captureClosed); OUT.production_modified=false;
OUT.source_sha=sourceSHA; OUT.parent_diag_sha=parentDiagSHA; OUT.r6_diag_sha=r6DiagSHA; OUT.runner_sha=runnerSHA;
OUT.observer=struct('dominant',obsDom,'dominant_ms',obsDomMs,'dominant_t',obsDomT, ...
    'one_step_dominant',oneDom,'one_step_ms',oneDomMs,'one_step_t',oneDomT, ...
    'contraction_dominant',conDom,'contraction_ms',conDomMs,'contraction_t',conDomT);
OUT.reference=struct('dominant',refDom,'dominant_ms',refDomMs,'dominant_t',refDomT, ...
    'assert_dominant',assertDom,'assert_ms',assertDomMs,'assert_t',assertDomT);
OUT.stage2=struct('dominant',stage2Dom,'dominant_ms',stage2DomMs,'dominant_t',stage2DomT);
OUT.path_summary=reference_path_summary_r6(refRing);
OUT.run_error=runError; OUT.raw_capture_path=rawPath; OUT.report_path=reportPath;
save(outPath,'OUT','-v7.3');

% --------------------------- report -------------------------------------
fid=fopen(reportPath,'w'); assert(fid>=0,'[AF7R58ZZY-R6] cannot open report.');
c=onCleanup(@()safe_fclose_r6(fid)); %#ok<NASGU>
fprintf(fid,'status=%s\n',OUT.status);
fprintf(fid,'captureClosed=%d\nproductionModified=0\nisaProductionContract=1\n',captureClosed);
fprintf(fid,'sourceSHA=%s\nparentDiagSHA=%s\nr6DiagSHA=%s\nrunnerSHA=%s\n',sourceSHA,parentDiagSHA,r6DiagSHA,runnerSHA);
fprintf(fid,'runnerSemantics=%d/%d\nraw=%s\nout=%s\n\n',runnerSemantics.pass_count,runnerSemantics.total_count,rawPath,outPath);

fprintf(fid,'[OBSERVER_ROOT]\n');
fprintf(fid,'dominant=%s\ndominantMs=%.9g\ndominantT=%.12g\n',obsDom,obsDomMs,obsDomT);
fprintf(fid,'oneStepDominant=%s\noneStepDominantMs=%.9g\noneStepDominantT=%.12g\n',oneDom,oneDomMs,oneDomT);
fprintf(fid,'contractionDominant=%s\ncontractionDominantMs=%.9g\ncontractionDominantT=%.12g\n',conDom,conDomMs,conDomT);
fprintf(fid,'columns=%s\n',strjoin(obsColumns,' '));
for i=1:size(obsRing,1), fprintf_numeric_row_r6(fid,obsRing(i,:)); end

fprintf(fid,'\n[REPAIRED_REFERENCE_ROOT]\n');
fprintf(fid,'dominant=%s\ndominantMs=%.9g\ndominantT=%.12g\n',refDom,refDomMs,refDomT);
fprintf(fid,'assertDominant=%s\nassertDominantMs=%.9g\nassertDominantT=%.12g\n',assertDom,assertDomMs,assertDomT);
PS=OUT.path_summary;
fprintf(fid,'pathCodes=1:CACHE_HIT,2:SHIFT1,3:SHIFTM,4:FULL_BATCH\n');
fprintf(fid,'cacheHitCount=%d\nshift1Count=%d\nshiftMCount=%d\nfullBatchCount=%d\n',PS.cache_hit,PS.shift1,PS.shiftm,PS.full_batch);
fprintf(fid,'columns=%s\n',strjoin(refColumns,' '));
for i=1:size(refRing,1), fprintf_numeric_row_r6(fid,refRing(i,:)); end

fprintf(fid,'\n[STAGE2_CONTEXT_FROM_PARENT_DIAGNOSTIC]\n');
fprintf(fid,'dominant=%s\ndominantMs=%.9g\ndominantT=%.12g\n',stage2Dom,stage2DomMs,stage2DomT);
fprintf(fid,'columns=t stage1 stage2 stage3 postcacheBatch postcacheDerive batchModel packet handlesMeta\n');
for i=1:size(baseRing,1)
    r=baseRing(i,:);
    if isfinite(r(18))||isfinite(r(36))||isfinite(r(37))
        fprintf(fid,'row= %.12g %.9g %.9g %.9g %.9g %.9g %.9g %.9g %.9g\n', ...
            r(1),r(18),r(36),r(37),r(33),r(34),r(28),r(30),r(32));
    end
end

fprintf(fid,'\n[INTERPRETATION_CONTRACT]\n');
fprintf(fid,'timingIsEmpirical=1\nformalFutureWCETClaim=0\n');
fprintf(fid,'trajectoryMathChanged=0\ncertificateAcceptanceChanged=0\nsensorGeometryChanged=0\n');
fprintf(fid,'activationTimeChanged=0\nstaleAtomicGatesChanged=0\ncommitPolicyChanged=0\n');
fprintf(fid,'productionSourceModified=0\ndiagnosticTicTocOverheadPresent=1\n');
if ~isempty(runError.message)
    fprintf(fid,'errorIdentifier=%s\nerrorMessage=%s\n',runError.identifier,runError.message);
end
safe_fclose_r6(fid); clear c;

fprintf('\n[C. RESULT]\n');
fprintf(' Observer root          = %s | %.6f ms @ %.3f s\n',obsDom,obsDomMs,obsDomT);
fprintf('   one-step subroot     = %s | %.6f ms @ %.3f s\n',oneDom,oneDomMs,oneDomT);
fprintf('   contraction subroot  = %s | %.6f ms @ %.3f s\n',conDom,conDomMs,conDomT);
fprintf(' Repaired packet root   = %s | %.6f ms @ %.3f s\n',refDom,refDomMs,refDomT);
fprintf('   assert subroot       = %s | %.6f ms @ %.3f s\n',assertDom,assertDomMs,assertDomT);
fprintf(' Stage-2 context root   = %s | %.6f ms @ %.3f s\n',stage2Dom,stage2DomMs,stage2DomT);
fprintf(' Reference paths        = hit:%d shift1:%d shiftM:%d full:%d\n',PS.cache_hit,PS.shift1,PS.shiftm,PS.full_batch);
fprintf('\nRETURN THESE THREE FILES:\n  %s\n  %s\n  %s\n',reportPath,outPath,rawPath);
fprintf('status = %s\n',OUT.status);

if ~captureClosed
    error('[AF7R58ZZY-R6] audit incomplete; no timing conclusion accepted. See report/run_error.');
end
end


function S=assert_runner_semantics_r6(txt)
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
if ~S.closed, error('[AF7R58ZZY-R6] current runner target semantics changed.'); end
end

function [name,val,t]=dominant_stage_r6(rows,cols,names)
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

function S=reference_path_summary_r6(R)
S=struct('cache_hit',0,'shift1',0,'shiftm',0,'full_batch',0);
if isempty(R), return; end
p=R(:,14);
S.cache_hit=nnz(p==1); S.shift1=nnz(p==2); S.shiftm=nnz(p==3); S.full_batch=nnz(p==4);
end

function fprintf_numeric_row_r6(fid,r)
fprintf(fid,'row=');
for k=1:numel(r), fprintf(fid,' %.12g',r(k)); end
fprintf(fid,'\n');
end

function y=ternary_r6(c,a,b)
if c, y=a; else, y=b; end
end

function n=count_token_r6(txt,token)
n=numel(strfind(txt,token)); %#ok<STREMP>
end

function h=sha256_file_r6(path)
fid=fopen(path,'rb'); assert(fid>=0,'[AF7R58ZZY-R6] cannot open file for SHA: %s',path);
c=onCleanup(@()fclose(fid)); %#ok<NASGU>
b=fread(fid,Inf,'*uint8');
md=javaMethod('getInstance','java.security.MessageDigest','SHA-256');
md.update(typecast(b,'int8')); d=md.digest(); u=mod(double(d),256);
h=lower(reshape(dec2hex(u,2).',1,[]));
end

function write_utf8_r6(path,txt)
fid=fopen(path,'w','n','UTF-8'); assert(fid>=0,'[AF7R58ZZY-R6] cannot create temp runner.');
c=onCleanup(@()fclose(fid)); %#ok<NASGU>
fprintf(fid,'%s',txt);
end

function delete_if_exists_r6(path)
try, if isfile(path), delete(path); end, catch, end
end

function safe_fclose_r6(fid)
try, if fid>0, fclose(fid); end, catch, end
end
