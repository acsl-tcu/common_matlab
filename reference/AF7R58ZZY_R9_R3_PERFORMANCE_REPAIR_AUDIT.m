function OUT = AF7R58ZZY_R9_R3_PERFORMANCE_REPAIR_AUDIT()
% AF7R58ZZY-R9-R3 | PERFORMANCE-ONLY SHADOW REPAIR AUDIT
%
% Purpose
%   1) Keep the exact production trajectory/certificate equations.
%   2) Remove observer cold/JIT spikes by routing the exact equations through
%      pre-mission-verified pure numeric kernels that are exercised by the
%      EXISTING observer preflight prime (no new mission-phase warmup).
%   3) Accelerate only the exact local decasteljau_batch recurrence inside
%      af7r33_flat_jet_batch_model using a runtime-built MEX shadow.
%   4) Require bitwise identity for the local recurrence and the full 21xN
%      batch helper before the optimized shadow Flight is allowed to run.
%
% Production REPLANNING_SYSTEM.m and the production AF7R33 helper are never
% modified. Arming/Takeoff/Landing allocation is not changed.

clc;
scriptDir=fileparts(mfilename('fullpath'));
addpath(scriptDir,'-begin'); rehash;

EXPECTED_SOURCE_SHA='310cfdd01f9554a80b71e8d90464816a721859709ad7c4b9137cdc0f3a6eaf49';
EXPECTED_RUNNER_SHA='d4d15d081ea8a6e3073b0a63ce36e47f983777ac2c81db194384de69f5e5fc37';
EXPECTED_R8_SHA='5ca886499192ee309c0bfd7a39c2094de708e4a85104fb7a54e99b2b987e97df';
EXPECTED_PARENT_SHA='2b874c2e92052de1c756c3b00f8f62f5cb08018a496b9c78b981df085f2efb18';

srcPath=which('REPLANNING_SYSTEM');
runnerPath=which('run_flight_headless');
r8Path=fullfile(scriptDir,'REPLANNING_SYSTEM_R8R2_DIAG.m');
parentPath=fullfile(scriptDir,'REPLANNING_SYSTEM_R5R4_DIAG.m');
r9Path=fullfile(scriptDir,'REPLANNING_SYSTEM_R9R3_DIAG.m');
cppPath=fullfile(scriptDir,'af7r62_decasteljau_batch_mex.cpp');
oneKernelPath=fullfile(scriptDir,'af7r62_observer_one_step_kernel.m');
conKernelPath=fullfile(scriptDir,'af7r62_observer_contract_kernel.m');
assert(~isempty(srcPath)&&isfile(srcPath),'[AF7R58ZZY-R9-R3] REPLANNING_SYSTEM.m missing.');
assert(~isempty(runnerPath)&&isfile(runnerPath),'[AF7R58ZZY-R9-R3] run_flight_headless.m missing.');
assert(all(cellfun(@isfile,{r8Path,parentPath,r9Path,cppPath,oneKernelPath,conKernelPath})), ...
    '[AF7R58ZZY-R9-R3] package is incomplete.');

sourceSHA=sha256_file_r9(srcPath);
runnerSHA=sha256_file_r9(runnerPath);
r8SHA=sha256_file_r9(r8Path);
parentSHA=sha256_file_r9(parentPath);
r9SHA=sha256_file_r9(r9Path);
assert(strcmpi(sourceSHA,EXPECTED_SOURCE_SHA),'[AF7R58ZZY-R9-R3] production source SHA mismatch.');
assert(strcmpi(runnerSHA,EXPECTED_RUNNER_SHA),'[AF7R58ZZY-R9-R3] runner SHA mismatch.');
assert(strcmpi(r8SHA,EXPECTED_R8_SHA),'[AF7R58ZZY-R9-R3] R8-R2 diagnostic SHA mismatch.');
assert(strcmpi(parentSHA,EXPECTED_PARENT_SHA),'[AF7R58ZZY-R9-R3] parent diagnostic SHA mismatch.');
runnerText=fileread(runnerPath);
runnerSemantics=assert_runner_semantics_r9(runnerText);

helperOriginalPath=which('af7r33_flat_jet_batch_model');
assert(~isempty(helperOriginalPath)&&isfile(helperOriginalPath), ...
    '[AF7R58ZZY-R9-R3] af7r33_flat_jet_batch_model.m missing.');
assert(strcmpi(fileparts(helperOriginalPath),fileparts(srcPath)), ...
    '[AF7R58ZZY-R9-R3] AF7R33 path authority is not the production reference directory.');
helperOriginalSHA=sha256_file_r9(helperOriginalPath);

r9Text=fileread(r9Path);
assert(~contains(r9Text,'function result_out = do(') && ~contains(r9Text,'function do('), ...
    '[AF7R58ZZY-R9-R3] R9 may not override do().');
assert(~contains(r9Text,'cha_allocation'), ...
    '[AF7R58ZZY-R9-R3] R9 may not edit phase allocation.');
assert(contains(r9Text,'af7r62_force_parent_compare = false'), ...
    '[AF7R58ZZY-R9-R3] superclass-dispatch diagnostic gate missing.');
assert(contains(r9Text,'S=observer_one_step_support@REPLANNING_SYSTEM_R8R2_DIAG') && ...
       contains(r9Text,'O=observer_contract_two_events@REPLANNING_SYSTEM_R8R2_DIAG'), ...
    '[AF7R58ZZY-R9-R3] same-name superclass dispatch anchors missing.');
compareStart=strfind(r9Text,'function E = af7r62_compare_observer_snapshot');
assert(numel(compareStart)==1,'[AF7R58ZZY-R9-R3] observer snapshot compare method anchor count != 1.');
compareTail=r9Text(compareStart(1):end);
compareEnd=strfind(compareTail,sprintf('\n        end'));
assert(~isempty(compareEnd),'[AF7R58ZZY-R9-R3] observer snapshot compare method end not found.');
compareBody=compareTail(1:compareEnd(1));
assert(~contains(compareBody,'@REPLANNING_SYSTEM_R8R2_DIAG'), ...
    '[AF7R58ZZY-R9-R3] illegal superclass dispatch remains in differently named compare method.');

fprintf('====================================================================================================\n');
fprintf(' AF7R58ZZY-R9-R3 | PERFORMANCE-ONLY SHADOW REPAIR AUDIT\n');
fprintf('====================================================================================================\n');
fprintf('[AUTHORITY] productionModified=0 | noPhaseShift=1 | mathChanged=0 | certificateChanged=0\n');
fprintf(' source SHA       = %s\n',sourceSHA);
fprintf(' runner SHA       = %s\n',runnerSHA);
fprintf(' runner semantics = %d/%d\n',runnerSemantics.pass_count,runnerSemantics.total_count);
fprintf(' AF7R33 SHA       = %s\n',helperOriginalSHA);
fprintf(' R9 SHA           = %s\n',r9SHA);

root=fileparts(srcPath);
outDir=tempname(root); mkdir(outDir);
reportPath=fullfile(outDir,'AF7R58ZZY_R9_R3_REPORT.txt');
outPath=fullfile(outDir,'AF7R58ZZY_R9_R3_OUT.mat');
rawPath=fullfile(outDir,'AF7R58ZZY_R9_R3_RAW_CAPTURE.mat');
shadowDir=tempname(root); mkdir(shadowDir);

Baseline=empty_run_r9(); Optimized=empty_run_r9();
Mex=struct('closed',false,'compile_ok',false,'capture_full_exact',false, ...
    'local_bitwise_exact',false,'full_helper_bitwise_exact',false, ...
    'capture_count',0,'original_helper',helperOriginalPath,'original_sha',helperOriginalSHA, ...
    'shadow_dir',shadowDir,'error',struct('identifier','','message',''));
ObserverPost=struct('closed',false,'count',0,'events',[], ...
    'premission_one_equiv_ms',NaN,'premission_contract_equiv_ms',NaN, ...
    'live_one_calls',0,'live_contract_calls',0);
runError=struct('identifier','','message','');

fprintf('\n[A. BASELINE FLIGHT -- CURRENT EXACT PRODUCTION MATH]\n');
try
    Baseline=run_flight_r9(root,runnerText,'REPLANNING_SYSTEM_R8R2_DIAG','',false);
    if ~Baseline.closed
        if ~isempty(Baseline.error.message)
            error('[AF7R58ZZY-R9-R3 BASELINE] %s | %s',Baseline.error.identifier,Baseline.error.message);
        end
        error('[AF7R58ZZY-R9-R3] baseline Flight did not close.');
    end
catch ME
    runError.identifier=ME.identifier; runError.message=ME.message;
end

if isempty(runError.message)
    fprintf('\n[B. BUILD EXACT DECASTELJAU SHADOW]\n');
    try
        objB=Baseline.obj;
        R=objB.active_repaired_reference;
        assert(isstruct(R)&&isfield(R,'batch_flat_model'), ...
            '[AF7R58ZZY-R9-R3] baseline active batch model unavailable.');
        dt=double(objB.prediction_dt); H=double(objB.horizon_s); N=round(H/dt)+1;
        t0=double(objB.last_replan_commit_time)+dt;
        tq=t0+(0:N-1)*dt;

        % Exact original full helper output before any path shadowing.
        clear af7r33_flat_jet_batch_model; rehash;
        assert(strcmpi(which('af7r33_flat_jet_batch_model'),helperOriginalPath), ...
            '[AF7R58ZZY-R9-R3] original AF7R33 helper is not path authority before capture.');
        [Yorig,Vorig]=af7r33_flat_jet_batch_model(objB,tq,R.batch_flat_model);
        Yorig=double(Yorig);

        src=fileread(helperOriginalPath);
        [decl,outVar,inVars,declStart,declEnd]=locate_local_decl_r9(src,'decasteljau_batch');
        assert(numel(inVars)==2 && ~isempty(outVar), ...
            '[AF7R58ZZY-R9-R3] unsupported decasteljau_batch signature: %s',decl);

        % R9-R3: never cut/delete a local-function block.  Rename only the
        % target declaration and insert a wrapper immediately before it.
        % This preserves every other local helper (including sha256_text)
        % byte-for-byte and avoids the R9 local-function-scope failure.
        originalLocalName='decasteljau_batch_r9r3_original';
        captureSrc=insert_wrapper_preserve_locals_r9( ...
            src,declStart,declEnd,decl,outVar,inVars,originalLocalName,'capture');
        assert(count_token_r9(captureSrc,'sha256_text')==count_token_r9(src,'sha256_text'), ...
            '[AF7R58ZZY-R9-R3] capture rewrite changed sha256_text references.');
        shadowHelper=fullfile(shadowDir,'af7r33_flat_jet_batch_model.m');
        write_utf8_r9(shadowHelper,captureSrc);

        addpath(shadowDir,'-begin'); rehash; clear af7r33_flat_jet_batch_model;
        assert(strcmpi(which('af7r33_flat_jet_batch_model'),shadowHelper), ...
            '[AF7R58ZZY-R9-R3] capture shadow helper did not become path authority.');
        global AF7R62_DECASTELJAU_CAPTURE;
        AF7R62_DECASTELJAU_CAPTURE={};
        [Ycap,Vcap]=af7r33_flat_jet_batch_model(objB,tq,R.batch_flat_model);
        Ycap=double(Ycap);
        Mex.capture_count=numel(AF7R62_DECASTELJAU_CAPTURE);
        Mex.capture_full_exact=bitwise_same_double_r9(Yorig,Ycap)&&isequaln(Vorig,Vcap);
        assert(Mex.capture_full_exact && Mex.capture_count>=1, ...
            '[AF7R58ZZY-R9-R3] source-extracted capture helper is not exact.');

        compile_decasteljau_mex_r9(cppPath,shadowDir);
        Mex.compile_ok=true;
        rehash; clear af7r62_decasteljau_batch_mex;
        mexPath=which('af7r62_decasteljau_batch_mex');
        assert(~isempty(mexPath),'[AF7R58ZZY-R9-R3] decasteljau MEX was not resolved.');
        Mex.mex_path=mexPath;

        localExact=true;
        for k=1:Mex.capture_count
            A=AF7R62_DECASTELJAU_CAPTURE{k};
            y0=A{3};
            ym=af7r62_decasteljau_batch_mex(A{1},A{2});
            localExact=localExact && bitwise_same_double_r9(double(y0),double(ym));
        end
        Mex.local_bitwise_exact=localExact;
        assert(localExact,'[AF7R58ZZY-R9-R3] MEX local recurrence is not bitwise exact.');

        fastSrc=insert_wrapper_preserve_locals_r9( ...
            src,declStart,declEnd,decl,outVar,inVars,originalLocalName,'mex');
        assert(count_token_r9(fastSrc,'sha256_text')==count_token_r9(src,'sha256_text'), ...
            '[AF7R58ZZY-R9-R3] MEX rewrite changed sha256_text references.');
        write_utf8_r9(shadowHelper,fastSrc);
        clear af7r33_flat_jet_batch_model; rehash;
        assert(strcmpi(which('af7r33_flat_jet_batch_model'),shadowHelper), ...
            '[AF7R58ZZY-R9-R3] fast shadow helper lost path authority.');
        [Yfast,Vfast]=af7r33_flat_jet_batch_model(objB,tq,R.batch_flat_model);
        Yfast=double(Yfast);
        Mex.full_helper_bitwise_exact=bitwise_same_double_r9(Yorig,Yfast)&&isequaln(Vorig,Vfast);
        assert(Mex.full_helper_bitwise_exact, ...
            '[AF7R58ZZY-R9-R3] full 21xN AF7R33 helper is not bitwise exact with MEX shadow.');

        % Hot benchmark after MEX is loaded and exactness has already closed.
        tm=zeros(1,30);
        for k=1:numel(tm)
            T=tic; af7r33_flat_jet_batch_model(objB,tq,R.batch_flat_model); tm(k)=1e3*toc(T);
        end
        Mex.hot_median_ms=median(tm);
        Mex.hot_p95_ms=percentile_r9(tm,95);
        Mex.closed=true;
    catch ME
        Mex.error.identifier=ME.identifier; Mex.error.message=ME.message;
        runError.identifier=ME.identifier; runError.message=ME.message;
        fprintf(2,'[AF7R58ZZY-R9-R3 MEX ERROR] %s | %s\n',ME.identifier,ME.message);
    end
end

if isempty(runError.message) && Mex.closed
    fprintf('\n[C. OPTIMIZED SHADOW FLIGHT -- EXACT OBSERVER KERNEL + EXACT MEX DECASTELJAU]\n');
    try
        % Baseline object is no longer needed. Clear base mission state, but
        % keep the already-built MEX and the shadow helper on path.
        try, evalin('base','clearvars'); catch, end
        objB=[]; %#ok<NASGU>
        clear af7r33_flat_jet_batch_model; rehash;
        Optimized=run_flight_r9(root,runnerText,'REPLANNING_SYSTEM_R9R3_DIAG',shadowDir,true);
        if ~Optimized.closed
            if ~isempty(Optimized.error.message)
                error('[AF7R58ZZY-R9-R3 OPTIMIZED] %s | %s',Optimized.error.identifier,Optimized.error.message);
            end
            error('[AF7R58ZZY-R9-R3] optimized shadow Flight did not close.');
        end
        assert(strcmpi(which('af7r33_flat_jet_batch_model'),fullfile(shadowDir,'af7r33_flat_jet_batch_model.m')), ...
            '[AF7R58ZZY-R9-R3] optimized Flight did not use exact fast shadow helper.');

        objO=Optimized.obj;
        assert(logical(objO.af7r62_observer_one_equiv_closed) && ...
            logical(objO.af7r62_observer_contract_equiv_closed), ...
            '[AF7R58ZZY-R9-R3] observer kernel equivalence did not close before Flight.');

        nSnap=double(objO.af7r60_obs_snapshot_count);
        ev=repmat(struct('one_step_equal',false,'one_step_matches_stored',false, ...
            'contract_equal',false,'contract_matches_stored',false),1,nSnap);
        allObs=true;
        for k=1:nSnap
            ev(k)=objO.af7r62_compare_observer_snapshot(objO.af7r60_obs_snapshots{k});
            allObs=allObs && ev(k).one_step_equal && ev(k).one_step_matches_stored && ...
                ev(k).contract_equal && ev(k).contract_matches_stored;
        end
        ObserverPost=struct('closed',allObs&&nSnap>=1,'count',nSnap,'events',ev, ...
            'premission_one_equiv_ms',double(objO.af7r62_observer_one_equiv_ms), ...
            'premission_contract_equiv_ms',double(objO.af7r62_observer_contract_equiv_ms), ...
            'live_one_calls',double(objO.af7r62_observer_kernel_live_one_calls), ...
            'live_contract_calls',double(objO.af7r62_observer_kernel_live_contract_calls));
        assert(ObserverPost.closed,'[AF7R58ZZY-R9-R3] post-mission observer equivalence is open.');
    catch ME
        runError.identifier=ME.identifier; runError.message=ME.message;
        fprintf(2,'[AF7R58ZZY-R9-R3 OPT ERROR] %s | %s\n',ME.identifier,ME.message);
    end
end

Perf=compare_performance_r9(Baseline,Optimized);
noRegression=Perf.closed && Perf.observer_peak_ratio<=1.05 && ...
    Perf.stage2_postcache_ratio<=1.05 && Perf.stage2_total_ratio<=1.05 && ...
    Perf.postcommit_ref_ratio<=1.05 && Perf.flight_entry_do_ratio<=1.05 && Perf.replan_do_ratio<=1.05;
meaningfulImprovement=Perf.closed && ...
    (Perf.observer_peak_ratio<=0.50 || Perf.observer_peak_ms<=1.0) && ...
    Perf.stage2_postcache_ratio<=0.80;

captureClosed=isempty(runError.message) && Baseline.closed && Optimized.closed && ...
    Mex.closed && ObserverPost.closed && noRegression && meaningfulImprovement;

RAW=struct(); RAW.schema='AF7R58ZZY_R9_R3_RAW_V1';
RAW.production_source=srcPath; RAW.production_sha=sourceSHA; RAW.runner=runnerPath; RAW.runner_sha=runnerSHA;
RAW.runner_semantics=runnerSemantics; RAW.original_helper=helperOriginalPath; RAW.original_helper_sha=helperOriginalSHA;
RAW.baseline=strip_run_obj_r9(Baseline); RAW.optimized=strip_run_obj_r9(Optimized);
RAW.Mex=Mex; RAW.ObserverPost=ObserverPost; RAW.Perf=Perf; RAW.run_error=runError;
save(rawPath,'RAW','-v7.3');

OUT=struct(); OUT.schema='AF7R58ZZY_R9_R3_OUT_V1'; OUT.capture_closed=logical(captureClosed);
OUT.status=ternary_r9(captureClosed,'PERFORMANCE_SHADOW_REPAIR_R9_R3_COMPLETE', ...
    'PERFORMANCE_SHADOW_REPAIR_R9_R3_INCOMPLETE_FAIL_CLOSED');
OUT.production_modified=false; OUT.shadow_only=true; OUT.no_phase_shift=true;
OUT.arming_work_added=false; OUT.takeoff_work_added=false; OUT.landing_work_added=false;
OUT.trajectory_math_changed=false; OUT.certificate_acceptance_changed=false; OUT.commit_policy_changed=false;
OUT.Mex=Mex; OUT.ObserverPost=ObserverPost; OUT.Perf=Perf; OUT.run_error=runError;
OUT.report_path=reportPath; OUT.raw_capture_path=rawPath;
save(outPath,'OUT','-v7.3');

fid=fopen(reportPath,'w'); assert(fid>=0,'[AF7R58ZZY-R9-R3] cannot open report.');
c=onCleanup(@()safe_fclose_r9(fid)); %#ok<NASGU>
say_r9(fid,'status=%s',OUT.status);
say_r9(fid,'captureClosed=%d',captureClosed);
say_r9(fid,'productionModified=0');
say_r9(fid,'shadowOnly=1');
say_r9(fid,'noPhaseShift=1');
say_r9(fid,'newArmingWork=0');
say_r9(fid,'newTakeoffWork=0');
say_r9(fid,'newLandingWork=0');
say_r9(fid,'sourceSHA=%s',sourceSHA);
say_r9(fid,'runnerSHA=%s',runnerSHA);
say_r9(fid,'originalAF7R33SHA=%s',helperOriginalSHA);
say_r9(fid,'r9DiagnosticSHA=%s',r9SHA);
say_r9(fid,'runnerSemantics=%d/%d',runnerSemantics.pass_count,runnerSemantics.total_count);
say_r9(fid,'raw=%s',rawPath); say_r9(fid,'out=%s',outPath); say_r9(fid,'');

say_r9(fid,'[EXACTNESS_GATE]');
say_r9(fid,'captureShadowFullExact=%d',Mex.capture_full_exact);
say_r9(fid,'localDecasteljauMexBitwiseExact=%d',Mex.local_bitwise_exact);
say_r9(fid,'fullAF7R33HelperBitwiseExact=%d',Mex.full_helper_bitwise_exact);
say_r9(fid,'decasteljauCapturedCalls=%d',Mex.capture_count);
if isfield(Mex,'hot_median_ms'), say_r9(fid,'mexShadowHotMedianMs=%.9g',Mex.hot_median_ms); end
if isfield(Mex,'hot_p95_ms'), say_r9(fid,'mexShadowHotP95Ms=%.9g',Mex.hot_p95_ms); end
say_r9(fid,'observerPremissionOneStepExact=%d',Optimized.observer_one_equiv_closed);
say_r9(fid,'observerPremissionContractionExact=%d',Optimized.observer_contract_equiv_closed);
say_r9(fid,'observerPostMissionAllExact=%d',ObserverPost.closed);
say_r9(fid,'');

say_r9(fid,'[PERFORMANCE_COMPARISON]');
say_r9(fid,'columns=metric baseline_ms optimized_ms ratio');
say_r9(fid,'observerPeak %.9g %.9g %.9g',Perf.observer_peak_baseline_ms,Perf.observer_peak_ms,Perf.observer_peak_ratio);
say_r9(fid,'flightEntryDoPeak %.9g %.9g %.9g',Perf.flight_entry_do_baseline_ms,Perf.flight_entry_do_ms,Perf.flight_entry_do_ratio);
say_r9(fid,'replanDoPeak %.9g %.9g %.9g',Perf.replan_do_baseline_ms,Perf.replan_do_ms,Perf.replan_do_ratio);
say_r9(fid,'stage2Total %.9g %.9g %.9g',Perf.stage2_total_baseline_ms,Perf.stage2_total_ms,Perf.stage2_total_ratio);
say_r9(fid,'stage2Postcache %.9g %.9g %.9g',Perf.stage2_postcache_baseline_ms,Perf.stage2_postcache_ms,Perf.stage2_postcache_ratio);
say_r9(fid,'postcommitRefPacket %.9g %.9g %.9g',Perf.postcommit_ref_baseline_ms,Perf.postcommit_ref_ms,Perf.postcommit_ref_ratio);
say_r9(fid,'noRegression=%d',noRegression);
say_r9(fid,'meaningfulImprovement=%d',meaningfulImprovement);
say_r9(fid,'');

say_r9(fid,'[OBSERVER_KERNEL]');
say_r9(fid,'premissionOneEquivMs=%.9g',ObserverPost.premission_one_equiv_ms);
say_r9(fid,'premissionContractEquivMs=%.9g',ObserverPost.premission_contract_equiv_ms);
say_r9(fid,'liveOneCalls=%d',ObserverPost.live_one_calls);
say_r9(fid,'liveContractCalls=%d',ObserverPost.live_contract_calls);
say_r9(fid,'postMissionSnapshotCount=%d',ObserverPost.count);
say_r9(fid,'');

say_r9(fid,'[CONTRACT]');
say_r9(fid,'trajectoryMathChanged=0');
say_r9(fid,'derivativeOrderReduced=0');
say_r9(fid,'certificateAcceptanceChanged=0');
say_r9(fid,'commitPolicyChanged=0');
say_r9(fid,'observerTheoremChanged=0');
say_r9(fid,'workShiftedToArmingTakeoffLanding=0');
say_r9(fid,'diagnosticPremissionEquivalenceWorkAdded=1');
say_r9(fid,'productionInstallPerformed=0');
if ~isempty(runError.message), say_r9(fid,'runError=%s | %s',runError.identifier,runError.message); end
safe_fclose_r9(fid); clear c;

fprintf('\n[D. RESULT]\n');
fprintf(' status                 = %s\n',OUT.status);
fprintf(' observer peak ratio    = %.4f\n',Perf.observer_peak_ratio);
fprintf(' postcache ratio        = %.4f\n',Perf.stage2_postcache_ratio);
fprintf(' full helper bitwise    = %d\n',Mex.full_helper_bitwise_exact);
fprintf(' observer exact         = %d\n',ObserverPost.closed);
fprintf('RETURN THESE THREE FILES:\n  %s\n  %s\n  %s\n',reportPath,outPath,rawPath);
if ~captureClosed
    error('[AF7R58ZZY-R9-R3] shadow performance repair did not close; production install is forbidden.');
end
end

function R=run_flight_r9(root,runnerText,className,shadowDir,useR9)
R=empty_run_r9();
try, evalin('base','clearvars'); catch, end
try, evalin('base','clear REPLANNING_SYSTEM_R9R3_DIAG REPLANNING_SYSTEM_R8R2_DIAG REPLANNING_SYSTEM_R5R4_DIAG'); catch, end
rehash;
oldCtor='replanner_instance = REPLANNING_SYSTEM(agent,nominal_ref,replan_opts);';
newCtor=sprintf('replanner_instance = %s(agent,nominal_ref,replan_opts);',className);
assert(count_token_r9(runnerText,oldCtor)==1,'[AF7R58ZZY-R9-R3] runner constructor anchor count != 1.');
txt=strrep(runnerText,oldCtor,newCtor);
if ~isempty(shadowDir)
    p=strrep(shadowDir,'''','''''');
    pre=sprintf('addpath(''%s'',''-begin''); clear af7r33_flat_jet_batch_model; rehash;%c',p,10);
    txt=[pre txt];
end
tmp=fullfile(root,sprintf('AF7R58ZZY_R9_R3_TEMP_%s.m',className));
write_utf8_r9(tmp,txt); c=onCleanup(@()delete_if_exists_r9(tmp)); %#ok<NASGU>
try
    cmd=sprintf('run(''%s'')',strrep(tmp,'''',''''''));
    evalin('base',cmd);
    obj=evalin('base','replanner_instance');
    assert(isa(obj,className),'[AF7R58ZZY-R9-R3] requested diagnostic class not instantiated.');
    assert(isa(obj,'REPLANNING_SYSTEM'),'[AF7R58ZZY-R9-R3] production class contract lost.');
    nb=double(obj.af7r58zzy_r5_count); no=double(obj.af7r58zzy_r6_obs_count); nr=double(obj.af7r58zzy_r6_ref_count);
    if nb>0, R.base_ring=double(obj.af7r58zzy_r5_ring(1:nb,:)); end
    if no>0, R.obs_ring=double(obj.af7r58zzy_r6_obs_ring(1:no,:)); end
    if nr>0, R.ref_ring=double(obj.af7r58zzy_r6_ref_ring(1:nr,:)); end
    R.obj=obj; R.closed=~isempty(R.base_ring)&&~isempty(R.obs_ring);
    if useR9
        R.observer_one_equiv_closed=logical(obj.af7r62_observer_one_equiv_closed);
        R.observer_contract_equiv_closed=logical(obj.af7r62_observer_contract_equiv_closed);
    end
catch ME
    R.error=struct('identifier',ME.identifier,'message',ME.message);
end
end

function R=empty_run_r9()
R=struct('closed',false,'base_ring',nan(0,48),'obs_ring',nan(0,32),'ref_ring',nan(0,32), ...
    'obj',[],'error',struct('identifier','','message',''), ...
    'observer_one_equiv_closed',false,'observer_contract_equiv_closed',false);
end

function S=strip_run_obj_r9(R)
S=R; S.obj=[];
end

function P=compare_performance_r9(B,O)
P=struct('closed',false,'observer_peak_baseline_ms',NaN,'observer_peak_ms',NaN,'observer_peak_ratio',NaN, ...
    'flight_entry_do_baseline_ms',NaN,'flight_entry_do_ms',NaN,'flight_entry_do_ratio',NaN, ...
    'replan_do_baseline_ms',NaN,'replan_do_ms',NaN,'replan_do_ratio',NaN, ...
    'stage2_total_baseline_ms',NaN,'stage2_total_ms',NaN,'stage2_total_ratio',NaN, ...
    'stage2_postcache_baseline_ms',NaN,'stage2_postcache_ms',NaN,'stage2_postcache_ratio',NaN, ...
    'postcommit_ref_baseline_ms',NaN,'postcommit_ref_ms',NaN,'postcommit_ref_ratio',NaN);
if ~(B.closed&&O.closed), return; end
P.observer_peak_baseline_ms=max(B.obs_ring(:,2),[],'omitnan');
P.observer_peak_ms=max(O.obs_ring(:,2),[],'omitnan');
P.observer_peak_ratio=P.observer_peak_ms/nonzero_r9(P.observer_peak_baseline_ms);
mbf=B.base_ring(:,1)>=4.95-1e-10 & B.base_ring(:,1)<=5.20+1e-10;
mof=O.base_ring(:,1)>=4.95-1e-10 & O.base_ring(:,1)<=5.20+1e-10;
P.flight_entry_do_baseline_ms=max(B.base_ring(mbf,3),[],'omitnan');
P.flight_entry_do_ms=max(O.base_ring(mof,3),[],'omitnan');
P.flight_entry_do_ratio=P.flight_entry_do_ms/nonzero_r9(P.flight_entry_do_baseline_ms);
mbr=B.base_ring(:,1)>=9.80-1e-10 & B.base_ring(:,1)<=10.10+1e-10;
mor=O.base_ring(:,1)>=9.80-1e-10 & O.base_ring(:,1)<=10.10+1e-10;
P.replan_do_baseline_ms=max(B.base_ring(mbr,3),[],'omitnan');
P.replan_do_ms=max(O.base_ring(mor,3),[],'omitnan');
P.replan_do_ratio=P.replan_do_ms/nonzero_r9(P.replan_do_baseline_ms);
b2=B.base_ring(isfinite(B.base_ring(:,36)),:); o2=O.base_ring(isfinite(O.base_ring(:,36)),:);
if isempty(b2)||isempty(o2), return; end
P.stage2_total_baseline_ms=max(b2(:,36),[],'omitnan'); P.stage2_total_ms=max(o2(:,36),[],'omitnan');
P.stage2_total_ratio=P.stage2_total_ms/nonzero_r9(P.stage2_total_baseline_ms);
P.stage2_postcache_baseline_ms=max(b2(:,33),[],'omitnan'); P.stage2_postcache_ms=max(o2(:,33),[],'omitnan');
P.stage2_postcache_ratio=P.stage2_postcache_ms/nonzero_r9(P.stage2_postcache_baseline_ms);
mb=B.base_ring(:,1)>=9.925-1e-10 & B.base_ring(:,1)<=10.05+1e-10;
mo=O.base_ring(:,1)>=9.925-1e-10 & O.base_ring(:,1)<=10.05+1e-10;
P.postcommit_ref_baseline_ms=max(B.base_ring(mb,11),[],'omitnan');
P.postcommit_ref_ms=max(O.base_ring(mo,11),[],'omitnan');
P.postcommit_ref_ratio=P.postcommit_ref_ms/nonzero_r9(P.postcommit_ref_baseline_ms);
P.closed=all(isfinite([P.observer_peak_baseline_ms P.observer_peak_ms ...
    P.flight_entry_do_baseline_ms P.flight_entry_do_ms P.replan_do_baseline_ms P.replan_do_ms ...
    P.stage2_total_baseline_ms P.stage2_total_ms P.stage2_postcache_baseline_ms P.stage2_postcache_ms ...
    P.postcommit_ref_baseline_ms P.postcommit_ref_ms]));
end

function [decl,outVar,inVars,s,e]=locate_local_decl_r9(src,name)
pat=['(?m)^[ \t]*function[^\n]*\<' regexptranslate('escape',name) '\>\s*\([^\n]*$'];
[s,e]=regexp(src,pat,'start','end','once');
assert(~isempty(s),'[AF7R58ZZY-R9-R3] local function %s not found.',name);
decl=src(s:e);
expr=['function\s+(?:\[?\s*)?([A-Za-z]\w*)\s*(?:\]?)\s*=\s*' ...
    regexptranslate('escape',name) '\s*\(([^)]*)\)'];
tok=regexp(decl,expr,'tokens','once');
assert(~isempty(tok),'[AF7R58ZZY-R9-R3] unsupported local function declaration: %s',decl);
outVar=strtrim(tok{1}); args=strtrim(tok{2});
if isempty(args), inVars={}; else, inVars=strtrim(strsplit(args,',')); end
end

function out=insert_wrapper_preserve_locals_r9(src,s,e,decl,outVar,inVars,originalName,mode)
assert(numel(inVars)==2,'[AF7R58ZZY-R9-R3] decasteljau wrapper requires two inputs.');
a1=inVars{1}; a2=inVars{2};
renamedDecl=regexprep(decl,'(?<![A-Za-z0-9_])decasteljau_batch(?=\s*\()',originalName,'once');
assert(~strcmp(renamedDecl,decl),'[AF7R58ZZY-R9-R3] failed to rename local declaration.');
switch lower(mode)
    case 'capture'
        wrapper=sprintf(['function %s=decasteljau_batch(%s,%s)%c' ...
            'global AF7R62_DECASTELJAU_CAPTURE;%c' ...
            '%s=%s(%s,%s);%c' ...
            'if isempty(AF7R62_DECASTELJAU_CAPTURE), AF7R62_DECASTELJAU_CAPTURE={}; end%c' ...
            'AF7R62_DECASTELJAU_CAPTURE{end+1}={%s,%s,%s};%c' ...
            'end%c%c'], ...
            outVar,a1,a2,10,10,outVar,originalName,a1,a2,10,10,a1,a2,outVar,10,10,10);
    case 'mex'
        wrapper=sprintf(['function %s=decasteljau_batch(%s,%s)%c' ...
            '%s=af7r62_decasteljau_batch_mex(%s,%s);%c' ...
            'end%c%c'],outVar,a1,a2,10,outVar,a1,a2,10,10,10);
    otherwise
        error('[AF7R58ZZY-R9-R3] unknown wrapper mode.');
end
% Only the declaration token is changed.  The original body and every
% following local function stay byte-identical.
out=[src(1:s-1) wrapper renamedDecl src(e+1:end)];
end

function compile_decasteljau_mex_r9(cppPath,outDir)
old=pwd; c=onCleanup(@()cd(old)); %#ok<NASGU>
cd(outDir);
err=[];
if ispc
    try
        mex('-R2018a','COMPFLAGS=$COMPFLAGS /fp:strict',cppPath,'-outdir',outDir);
        return;
    catch ME
        err=ME;
    end
else
    try
        mex('-R2018a','CXXFLAGS=$CXXFLAGS -ffp-contract=off -fno-fast-math',cppPath,'-outdir',outDir);
        return;
    catch ME
        err=ME;
    end
end
try
    mex('-R2018a',cppPath,'-outdir',outDir);
catch ME2
    if isempty(err), rethrow(ME2); end
    error('[AF7R58ZZY-R9-R3] MEX compile failed. strict=%s | default=%s',err.message,ME2.message);
end
end

function tf=bitwise_same_double_r9(a,b)
a=double(a); b=double(b);
tf=isequal(size(a),size(b));
if ~tf, return; end
ua=typecast(a(:),'uint64'); ub=typecast(b(:),'uint64'); tf=isequal(ua,ub);
end

function S=assert_runner_semantics_r9(txt)
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
if ~pass(end), pass(end)=contains(txt,'agent.set_cha_allocation_for_all("estimator",["ekf","loadstate"]);'); end
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
        for j=1:numel(tok), z=strfind(blk,tok{j}); if numel(z)==1,pos(j)=z;end, end %#ok<STREMP>
        orderOK=all(isfinite(pos))&&all(diff(pos)>0);
    end
end
S=struct('closed',all([pass;orderOK]),'pass_count',nnz([pass;orderOK]), ...
    'total_count',numel([pass;orderOK]),'pass',[pass;orderOK]);
assert(S.closed,'[AF7R58ZZY-R9-R3] current runner target semantics changed.');
end

function p=percentile_r9(x,q)
x=sort(double(x(isfinite(x)))); if isempty(x),p=NaN;return;end
k=1+(numel(x)-1)*q/100; a=floor(k); b=ceil(k);
if a==b,p=x(a);else,p=x(a)+(k-a)*(x(b)-x(a));end
end
function y=nonzero_r9(x)
if ~isfinite(x)||x<=0
    y=eps;
else
    y=x;
end
end

function y=ternary_r9(c,a,b)
if c
    y=a;
else
    y=b;
end
end

function n=count_token_r9(txt,token)
n=numel(strfind(txt,token)); %#ok<STREMP>
end

function h=sha256_file_r9(path)
fid=fopen(path,'rb'); assert(fid>=0,'[AF7R58ZZY-R9-R3] cannot open file for SHA: %s',path);
c=onCleanup(@()fclose(fid)); %#ok<NASGU>
b=fread(fid,Inf,'*uint8'); md=javaMethod('getInstance','java.security.MessageDigest','SHA-256');
md.update(typecast(b,'int8')); d=md.digest(); u=mod(double(d),256); h=lower(reshape(dec2hex(u,2).',1,[]));
end
function write_utf8_r9(path,txt)
fid=fopen(path,'w','n','UTF-8'); assert(fid>=0,'[AF7R58ZZY-R9-R3] cannot create file: %s',path);
c=onCleanup(@()fclose(fid)); %#ok<NASGU>
fprintf(fid,'%s',txt);
end
function delete_if_exists_r9(path)
try
    if isfile(path), delete(path); end
catch
end
end

function say_r9(fid,fmt,varargin)
fprintf(fid,'%s%c',sprintf(fmt,varargin{:}),10);
end

function safe_fclose_r9(fid)
try
    if fid>0, fclose(fid); end
catch
end
end
