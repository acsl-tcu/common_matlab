function OUT = AF7R58ZZY_R11_PREMISSION_FAST_SPECIALIZATION_AUDIT()
% AF7R58ZZY-R11 | PRE-MISSION SAME-CLASS FAST-HELPER SPECIALIZATION AUDIT
%
% Performance-only shadow experiment.
% - Production REPLANNING_SYSTEM.m is read-only.
% - Observer path is exactly the R8-R2 diagnostic/production-equation path.
% - R11 reuses the EXISTING pre-motion post-callback full-packet prime; no new prime calls.
% - Only the REAL Stage2 post-commit 161-point cache batch call is redirected
%   to a runtime-generated helper derived from the exact current AF7R33 source.
% - The helper is enabled only after local MEX and full 21xN helper bitwise
%   equivalence close.
% - No new Arming/Takeoff/Landing work and no new mission-phase warmup.

clc;
scriptDir=fileparts(mfilename('fullpath'));
addpath(scriptDir,'-begin'); rehash;

EXPECTED_SOURCE_SHA='310cfdd01f9554a80b71e8d90464816a721859709ad7c4b9137cdc0f3a6eaf49';
EXPECTED_RUNNER_SHA='d4d15d081ea8a6e3073b0a63ce36e47f983777ac2c81db194384de69f5e5fc37';
EXPECTED_AF7R33_SHA='1f57a0066acb75859dfe565b3c38a99d5c272b8afae4698131d7aeeb70093889';
EXPECTED_R8_SHA='5ca886499192ee309c0bfd7a39c2094de708e4a85104fb7a54e99b2b987e97df';
EXPECTED_R5R4_SHA='2b874c2e92052de1c756c3b00f8f62f5cb08018a496b9c78b981df085f2efb18';
EXPECTED_R10_SHA='83d16646d7e78b25777bb02c919df50a31245866b08ba5e2911804b0412fffed';
EXPECTED_R11_SHA='3abb9596c940451278385f7629f26860ee5bd77d6784d11920c777bd464c70c2';

srcPath=which('REPLANNING_SYSTEM');
runnerPath=which('run_flight_headless');
helperPath=which('af7r33_flat_jet_batch_model');
r8Path=fullfile(scriptDir,'REPLANNING_SYSTEM_R8R2_DIAG.m');
r5Path=fullfile(scriptDir,'REPLANNING_SYSTEM_R5R4_DIAG.m');
r10Path=fullfile(scriptDir,'REPLANNING_SYSTEM_R10_DIAG.m');
r11Path=fullfile(scriptDir,'REPLANNING_SYSTEM_R11_DIAG.m');
cppPath=fullfile(scriptDir,'af7r63_decasteljau_batch_mex.cpp');
assert(all(cellfun(@isfile,{srcPath,runnerPath,helperPath,r8Path,r5Path,r10Path,r11Path,cppPath})), ...
    '[AF7R58ZZY-R10] required source/package file missing.');

sourceSHA=sha256_file_r10(srcPath);
runnerSHA=sha256_file_r10(runnerPath);
helperSHA=sha256_file_r10(helperPath);
r8SHA=sha256_file_r10(r8Path);
r5SHA=sha256_file_r10(r5Path);
r10SHA=sha256_file_r10(r10Path);
r11SHA=sha256_file_r10(r11Path);
assert(strcmpi(sourceSHA,EXPECTED_SOURCE_SHA),'[AF7R58ZZY-R10] production source SHA mismatch.');
assert(strcmpi(runnerSHA,EXPECTED_RUNNER_SHA),'[AF7R58ZZY-R10] runner SHA mismatch.');
assert(strcmpi(helperSHA,EXPECTED_AF7R33_SHA),'[AF7R58ZZY-R10] AF7R33 helper SHA mismatch.');
assert(strcmpi(r8SHA,EXPECTED_R8_SHA),'[AF7R58ZZY-R10] R8-R2 diagnostic SHA mismatch.');
assert(strcmpi(r5SHA,EXPECTED_R5R4_SHA),'[AF7R58ZZY-R10] R5-R4 diagnostic SHA mismatch.');
assert(strcmpi(r10SHA,EXPECTED_R10_SHA),'[AF7R58ZZY-R11] R10 parent SHA mismatch.');
assert(strcmpi(r11SHA,EXPECTED_R11_SHA),'[AF7R58ZZY-R11] R11 diagnostic SHA mismatch.');
assert(strcmpi(fileparts(helperPath),fileparts(srcPath)), ...
    '[AF7R58ZZY-R10] AF7R33 helper is not bound to the production source folder.');

runnerText=fileread(runnerPath);
runnerSemantics=assert_runner_semantics_r10(runnerText);
r10Text=fileread(r10Path);
assert(~contains(r10Text,'function result_out = do(') && ~contains(r10Text,'function do('), ...
    '[AF7R58ZZY-R10] R10 may not override do().');
assert(~contains(r10Text,'cha_allocation'), ...
    '[AF7R58ZZY-R10] R10 may not edit phase allocation.');
assert(~contains(r10Text,'function S = observer_one_step_support') && ...
       ~contains(r10Text,'function O = observer_contract_two_events'), ...
    '[AF7R58ZZY-R10] observer path must remain inherited R8-R2.');
assert(count_token_r10(r10Text,'af7r33_flat_jet_batch_model_R10FAST(obj,tqCache,batchModel)')==1, ...
    '[AF7R58ZZY-R10] direct Stage2 R10FAST connection count != 1.');
r11Text=fileread(r11Path);
assert(~contains(r11Text,'function result_out = do(') && ~contains(r11Text,'function do('), ...
    '[AF7R58ZZY-R11] R11 may not override do().');
assert(~contains(r11Text,'warmup_online_path') && ~contains(r11Text,'cha_allocation'), ...
    '[AF7R58ZZY-R11] R11 may not add/shift phase warmup work.');
assert(count_token_r10(r11Text,'function [pL_ref,q_ref,source] = repaired_reference_packet(obj,t_now)')==1, ...
    '[AF7R58ZZY-R11] repaired packet override missing/duplicated.');
assert(count_token_r10(r11Text,'af7r33_flat_jet_batch_model_R10FAST(obj,tq,R.batch_flat_model)')==2, ...
    '[AF7R58ZZY-R11] packet fast-helper connection count != 2.');

fprintf('====================================================================================================\n');
fprintf(' AF7R58ZZY-R11 | PRE-MISSION SAME-CLASS FAST-HELPER SPECIALIZATION AUDIT\n');
fprintf('====================================================================================================\n');
fprintf('[AUTHORITY] productionModified=0 | observerChanged=0 | noPhaseShift=1 | mathChanged=0\n');
fprintf(' source SHA       = %s\n',sourceSHA);
fprintf(' runner SHA       = %s\n',runnerSHA);
fprintf(' AF7R33 SHA       = %s\n',helperSHA);
fprintf(' R10 parent SHA   = %s\n',r10SHA);
fprintf(' R11 SHA          = %s\n',r11SHA);
fprintf(' runner semantics = %d/%d\n',runnerSemantics.pass_count,runnerSemantics.total_count);

root=fileparts(srcPath);
outDir=tempname(root); mkdir(outDir);
shadowDir=tempname(root); mkdir(shadowDir);
reportPath=fullfile(outDir,'AF7R58ZZY_R11_REPORT.txt');
outPath=fullfile(outDir,'AF7R58ZZY_R11_OUT.mat');
rawPath=fullfile(outDir,'AF7R58ZZY_R11_RAW_CAPTURE.mat');

Baseline=empty_run_r10(); Optimized=empty_run_r10();
Mex=struct('closed',false,'capture_full_exact',false,'local_bitwise_exact',false, ...
    'full_helper_bitwise_exact',false,'capture_count',0,'hot_median_ms',NaN, ...
    'hot_p95_ms',NaN,'error',struct('identifier','','message',''));
runError=struct('identifier','','message','');

fprintf('\n[A. BASELINE FLIGHT -- R8-R2 EXACT PRODUCTION-EQUATION PATH]\n');
try
    Baseline=run_flight_r10(root,scriptDir,runnerText,'REPLANNING_SYSTEM_R8R2_DIAG','');
    assert(Baseline.closed,'[AF7R58ZZY-R10] baseline Flight did not close.');
catch ME
    runError.identifier=ME.identifier; runError.message=ME.message;
end

if isempty(runError.message)
    fprintf('\n[B. BUILD + VERIFY EXPLICIT R10FAST HELPER]\n');
    try
        objB=Baseline.obj;
        R=objB.active_repaired_reference;
        assert(isstruct(R)&&isfield(R,'batch_flat_model'), ...
            '[AF7R58ZZY-R10] baseline active batch model unavailable.');
        dt=double(objB.prediction_dt); H=double(objB.horizon_s); N=round(H/dt)+1;
        t0=double(objB.last_replan_commit_time)+dt;
        tq=t0+(0:N-1)*dt;

        [Yorig,Vorig]=af7r33_flat_jet_batch_model(objB,tq,R.batch_flat_model);
        Yorig=double(Yorig);

        src=fileread(helperPath);
        captureName='af7r33_flat_jet_batch_model_R10CAPTURE';
        fastName='af7r33_flat_jet_batch_model_R10FAST';
        captureSrc=build_named_shadow_helper_r10(src,captureName,'capture');
        fastSrc=build_named_shadow_helper_r10(src,fastName,'mex');
        capturePath=fullfile(shadowDir,[captureName '.m']);
        fastPath=fullfile(shadowDir,[fastName '.m']);
        write_utf8_r10(capturePath,captureSrc);
        write_utf8_r10(fastPath,fastSrc);

        addpath(shadowDir,'-begin'); rehash;
        clear af7r33_flat_jet_batch_model_R10CAPTURE af7r33_flat_jet_batch_model_R10FAST;
        global AF7R63_DECASTELJAU_CAPTURE;
        AF7R63_DECASTELJAU_CAPTURE={};
        [Ycap,Vcap]=af7r33_flat_jet_batch_model_R10CAPTURE(objB,tq,R.batch_flat_model);
        Ycap=double(Ycap);
        Mex.capture_count=numel(AF7R63_DECASTELJAU_CAPTURE);
        Mex.capture_full_exact=bitwise_same_double_r10(Yorig,Ycap)&&isequaln(Vorig,Vcap);
        assert(Mex.capture_full_exact&&Mex.capture_count>=1, ...
            '[AF7R58ZZY-R10] capture helper is not full-output exact.');

        compile_decasteljau_mex_r10(cppPath,shadowDir);
        rehash; clear af7r63_decasteljau_batch_mex;
        assert(~isempty(which('af7r63_decasteljau_batch_mex')), ...
            '[AF7R58ZZY-R10] MEX not resolved.');
        localExact=true;
        for k=1:Mex.capture_count
            A=AF7R63_DECASTELJAU_CAPTURE{k};
            ym=af7r63_decasteljau_batch_mex(A{1},A{2});
            localExact=localExact && bitwise_same_double_r10(double(A{3}),double(ym));
        end
        Mex.local_bitwise_exact=localExact;
        assert(localExact,'[AF7R58ZZY-R10] local de Casteljau MEX is not bitwise exact.');

        clear af7r33_flat_jet_batch_model_R10FAST; rehash;
        [Yfast,Vfast]=af7r33_flat_jet_batch_model_R10FAST(objB,tq,R.batch_flat_model);
        Yfast=double(Yfast);
        Mex.full_helper_bitwise_exact=bitwise_same_double_r10(Yorig,Yfast)&&isequaln(Vorig,Vfast);
        assert(Mex.full_helper_bitwise_exact, ...
            '[AF7R58ZZY-R10] R10FAST 21xN helper is not bitwise exact.');

        tm=zeros(1,30);
        for k=1:numel(tm)
            T=tic; af7r33_flat_jet_batch_model_R10FAST(objB,tq,R.batch_flat_model); tm(k)=1e3*toc(T);
        end
        Mex.hot_median_ms=median(tm);
        Mex.hot_p95_ms=percentile_r10(tm,95);
        Mex.closed=true;
        fprintf(' R10FAST exact=1 | hot median=%.4f ms | p95=%.4f ms\n',Mex.hot_median_ms,Mex.hot_p95_ms);
    catch ME
        Mex.error.identifier=ME.identifier; Mex.error.message=ME.message;
        runError.identifier=ME.identifier; runError.message=ME.message;
        fprintf(2,'[AF7R58ZZY-R10 FAST ERROR] %s | %s\n',ME.identifier,ME.message);
    end
end

if isempty(runError.message)&&Mex.closed
    fprintf('\n[C. OPTIMIZED SHADOW FLIGHT -- EXISTING PRE-MISSION PRIME + DIRECT STAGE2 FAST PATH]\n');
    try
        try, evalin('base','clearvars'); catch, end
        objB=[]; %#ok<NASGU>
        Optimized=run_flight_r10(root,scriptDir,runnerText,'REPLANNING_SYSTEM_R11_DIAG',shadowDir);
        assert(Optimized.closed,'[AF7R58ZZY-R10] optimized Flight did not close.');
        assert(Optimized.stage2_fast_real_calls>=1, ...
            '[AF7R58ZZY-R10] optimized Flight never entered the direct Stage2 R10FAST call.');
    catch ME
        runError.identifier=ME.identifier; runError.message=ME.message;
        fprintf(2,'[AF7R58ZZY-R10 OPT ERROR] %s | %s\n',ME.identifier,ME.message);
    end
end

Perf=compare_performance_r10(Baseline,Optimized);
fastConnected=Optimized.stage2_fast_real_calls>=1;
premissionFastClosed=Optimized.packet_fast_premission_full_calls>=1;
baselineStage2Rows=count_stage2_rows_r11(Baseline.base_ring);
optimizedStage2Rows=count_stage2_rows_r11(Optimized.base_ring);
noStage2Retry=(optimizedStage2Rows==1);
meaningfulImprovement=Perf.closed && Perf.stage2_postcache_ratio<=0.75 && Perf.stage2_total_ratio<=0.80;
changedPathNoRegression=Perf.closed && Perf.stage2_postcache_ratio<=1.00 && Perf.stage2_total_ratio<=1.00;
captureClosed=isempty(runError.message)&&Baseline.closed&&Optimized.closed&&Mex.closed&& ...
    fastConnected&&premissionFastClosed&&noStage2Retry&& ...
    changedPathNoRegression&&meaningfulImprovement;

RAW=struct(); RAW.schema='AF7R58ZZY_R11_RAW_V1';
RAW.production_source=srcPath; RAW.production_sha=sourceSHA; RAW.runner=runnerPath; RAW.runner_sha=runnerSHA;
RAW.original_helper=helperPath; RAW.original_helper_sha=helperSHA; RAW.runner_semantics=runnerSemantics;
RAW.baseline=strip_run_obj_r10(Baseline); RAW.optimized=strip_run_obj_r10(Optimized);
RAW.Mex=Mex; RAW.Perf=Perf; RAW.fast_connected=fastConnected; RAW.premission_fast_closed=premissionFastClosed;
RAW.baseline_stage2_rows=baselineStage2Rows; RAW.optimized_stage2_rows=optimizedStage2Rows; RAW.no_stage2_retry=noStage2Retry; RAW.run_error=runError;
save(rawPath,'RAW','-v7.3');

OUT=struct(); OUT.schema='AF7R58ZZY_R11_OUT_V1'; OUT.capture_closed=logical(captureClosed);
OUT.status=ternary_r10(captureClosed,'PREMISSION_FAST_SPECIALIZATION_R11_COMPLETE','PREMISSION_FAST_SPECIALIZATION_R11_INCOMPLETE_FAIL_CLOSED');
OUT.production_modified=false; OUT.shadow_only=true; OUT.no_phase_shift=true;
OUT.arming_work_added=false; OUT.takeoff_work_added=false; OUT.landing_work_added=false;
OUT.observer_optimization_applied=false; OUT.trajectory_math_changed=false;
OUT.certificate_acceptance_changed=false; OUT.commit_policy_changed=false;
OUT.Mex=Mex; OUT.Perf=Perf; OUT.fast_connected=fastConnected; OUT.premission_fast_closed=premissionFastClosed;
OUT.baseline_stage2_rows=baselineStage2Rows; OUT.optimized_stage2_rows=optimizedStage2Rows; OUT.no_stage2_retry=noStage2Retry; OUT.run_error=runError;
OUT.report_path=reportPath; OUT.raw_capture_path=rawPath;
save(outPath,'OUT','-v7.3');

fid=fopen(reportPath,'w'); assert(fid>=0,'[AF7R58ZZY-R10] cannot open report.');
c=onCleanup(@()safe_fclose_r10(fid)); %#ok<NASGU>
say_r10(fid,'status=%s',OUT.status);
say_r10(fid,'captureClosed=%d',captureClosed);
say_r10(fid,'productionModified=0');
say_r10(fid,'shadowOnly=1');
say_r10(fid,'noPhaseShift=1');
say_r10(fid,'newArmingWork=0'); say_r10(fid,'newTakeoffWork=0'); say_r10(fid,'newLandingWork=0');
say_r10(fid,'observerOptimizationApplied=0');
say_r10(fid,'sourceSHA=%s',sourceSHA); say_r10(fid,'runnerSHA=%s',runnerSHA);
say_r10(fid,'originalAF7R33SHA=%s',helperSHA); say_r10(fid,'r10ParentSHA=%s',r10SHA); say_r10(fid,'r11DiagnosticSHA=%s',r11SHA);
say_r10(fid,'runnerSemantics=%d/%d',runnerSemantics.pass_count,runnerSemantics.total_count);
say_r10(fid,'raw=%s',rawPath); say_r10(fid,'out=%s',outPath); say_r10(fid,'');

say_r10(fid,'[EXACTNESS_AND_CONNECTION]');
say_r10(fid,'captureShadowFullExact=%d',Mex.capture_full_exact);
say_r10(fid,'localDecasteljauMexBitwiseExact=%d',Mex.local_bitwise_exact);
say_r10(fid,'fullR10FastHelperBitwiseExact=%d',Mex.full_helper_bitwise_exact);
say_r10(fid,'decasteljauCapturedCalls=%d',Mex.capture_count);
say_r10(fid,'r10FastHotMedianMs=%.9g',Mex.hot_median_ms);
say_r10(fid,'r10FastHotP95Ms=%.9g',Mex.hot_p95_ms);
say_r10(fid,'stage2FastTotalCalls=%d',Optimized.stage2_fast_calls);
say_r10(fid,'stage2FastRealCalls=%d',Optimized.stage2_fast_real_calls);
say_r10(fid,'fastConnected=%d',fastConnected);
say_r10(fid,'packetFastFullCalls=%d',Optimized.packet_fast_full_calls);
say_r10(fid,'packetFastPremissionFullCalls=%d',Optimized.packet_fast_premission_full_calls);
say_r10(fid,'packetFastRealFullCalls=%d',Optimized.packet_fast_real_full_calls);
say_r10(fid,'premissionFastClosed=%d',premissionFastClosed);
say_r10(fid,'baselineStage2Rows=%d',baselineStage2Rows);
say_r10(fid,'optimizedStage2Rows=%d',optimizedStage2Rows);
say_r10(fid,'noStage2Retry=%d',noStage2Retry); say_r10(fid,'');

say_r10(fid,'[PERFORMANCE_COMPARISON]');
say_r10(fid,'columns=metric baseline_ms optimized_ms ratio');
say_r10(fid,'observerPeak %.9g %.9g %.9g',Perf.observer_peak_baseline_ms,Perf.observer_peak_ms,Perf.observer_peak_ratio);
say_r10(fid,'flightEntryDoPeak %.9g %.9g %.9g',Perf.flight_entry_do_baseline_ms,Perf.flight_entry_do_ms,Perf.flight_entry_do_ratio);
say_r10(fid,'replanDoPeak %.9g %.9g %.9g',Perf.replan_do_baseline_ms,Perf.replan_do_ms,Perf.replan_do_ratio);
say_r10(fid,'stage2Total %.9g %.9g %.9g',Perf.stage2_total_baseline_ms,Perf.stage2_total_ms,Perf.stage2_total_ratio);
say_r10(fid,'stage2Postcache %.9g %.9g %.9g',Perf.stage2_postcache_baseline_ms,Perf.stage2_postcache_ms,Perf.stage2_postcache_ratio);
say_r10(fid,'postcommitRefPacket %.9g %.9g %.9g',Perf.postcommit_ref_baseline_ms,Perf.postcommit_ref_ms,Perf.postcommit_ref_ratio);
say_r10(fid,'changedPathNoRegression=%d',changedPathNoRegression);
say_r10(fid,'meaningfulImprovement=%d',meaningfulImprovement); say_r10(fid,'');

say_r10(fid,'[CONTRACT]');
say_r10(fid,'trajectoryMathChanged=0'); say_r10(fid,'derivativeOrderReduced=0');
say_r10(fid,'certificateAcceptanceChanged=0'); say_r10(fid,'commitPolicyChanged=0');
say_r10(fid,'observerPathChanged=0'); say_r10(fid,'workShiftedToArmingTakeoffLanding=0');
say_r10(fid,'addedPremissionPrimeCallCount=0'); say_r10(fid,'existingPremissionFullBatchPrimeSubstituted=1');
say_r10(fid,'productionInstallPerformed=0');
if ~isempty(runError.message), say_r10(fid,'runError=%s | %s',runError.identifier,runError.message); end
safe_fclose_r10(fid); clear c;

fprintf('\n[D. RESULT]\n');
fprintf(' status                 = %s\n',OUT.status);
fprintf(' fast real calls        = %d\n',Optimized.stage2_fast_real_calls);
fprintf(' premission fast full   = %d\n',Optimized.packet_fast_premission_full_calls);
fprintf(' optimized Stage2 rows = %d\n',optimizedStage2Rows);
fprintf(' postcache ratio        = %.4f\n',Perf.stage2_postcache_ratio);
fprintf(' stage2 total ratio     = %.4f\n',Perf.stage2_total_ratio);
fprintf(' full helper bitwise    = %d\n',Mex.full_helper_bitwise_exact);
fprintf('RETURN THESE THREE FILES:\n  %s\n  %s\n  %s\n',reportPath,outPath,rawPath);
if ~captureClosed
    error('[AF7R58ZZY-R11] pre-mission same-class fast specialization did not close; production install is forbidden.');
end
end

function R=run_flight_r10(root,scriptDir,runnerText,className,shadowDir)
R=empty_run_r10();
try, evalin('base','clearvars'); catch, end
try, evalin('base','clear REPLANNING_SYSTEM_R11_DIAG REPLANNING_SYSTEM_R10_DIAG REPLANNING_SYSTEM_R8R2_DIAG REPLANNING_SYSTEM_R5R4_DIAG'); catch, end
rehash;
oldCtor='replanner_instance = REPLANNING_SYSTEM(agent,nominal_ref,replan_opts);';
newCtor=sprintf('replanner_instance = %s(agent,nominal_ref,replan_opts);',className);
assert(count_token_r10(runnerText,oldCtor)==1,'[AF7R58ZZY-R10] runner constructor anchor count != 1.');
txt=strrep(runnerText,oldCtor,newCtor);
pre=sprintf('addpath(''%s'',''-begin'');',strrep(scriptDir,'''',''''''));
if ~isempty(shadowDir), pre=[pre sprintf('addpath(''%s'',''-begin'');',strrep(shadowDir,'''',''''''))]; end %#ok<AGROW>
pre=[pre sprintf('rehash;%c',10)];
txt=[pre txt];
tmp=fullfile(root,sprintf('AF7R58ZZY_R10_TEMP_%s.m',className));
write_utf8_r10(tmp,txt); c=onCleanup(@()delete_if_exists_r10(tmp)); %#ok<NASGU>
try
    cmd=sprintf('run(''%s'')',strrep(tmp,'''',''''''));
    evalin('base',cmd);
    obj=evalin('base','replanner_instance');
    assert(isa(obj,className),'[AF7R58ZZY-R10] requested diagnostic class not instantiated.');
    assert(isa(obj,'REPLANNING_SYSTEM'),'[AF7R58ZZY-R10] production class contract lost.');
    nb=double(obj.af7r58zzy_r5_count); no=double(obj.af7r58zzy_r6_obs_count); nr=double(obj.af7r58zzy_r6_ref_count);
    if nb>0, R.base_ring=double(obj.af7r58zzy_r5_ring(1:nb,:)); end
    if no>0, R.obs_ring=double(obj.af7r58zzy_r6_obs_ring(1:no,:)); end
    if nr>0, R.ref_ring=double(obj.af7r58zzy_r6_ref_ring(1:nr,:)); end
    R.obj=obj; R.closed=~isempty(R.base_ring)&&~isempty(R.obs_ring);
    if isa(obj,'REPLANNING_SYSTEM_R10_DIAG')
        R.stage2_fast_calls=double(obj.af7r63_stage2_fast_calls);
        R.stage2_fast_real_calls=double(obj.af7r63_stage2_fast_real_calls);
    end
    if isa(obj,'REPLANNING_SYSTEM_R11_DIAG')
        R.packet_fast_full_calls=double(obj.af7r64_packet_fast_full_calls);
        R.packet_fast_premission_full_calls=double(obj.af7r64_packet_fast_premission_full_calls);
        R.packet_fast_real_full_calls=double(obj.af7r64_packet_fast_real_full_calls);
    end
catch ME
    R.error=struct('identifier',ME.identifier,'message',ME.message);
end
end

function R=empty_run_r10()
R=struct('closed',false,'base_ring',nan(0,48),'obs_ring',nan(0,32),'ref_ring',nan(0,32), ...
    'obj',[],'stage2_fast_calls',0,'stage2_fast_real_calls',0, ...
    'packet_fast_full_calls',0,'packet_fast_premission_full_calls',0,'packet_fast_real_full_calls',0, ...
    'error',struct('identifier','','message',''));
end
function S=strip_run_obj_r10(R)
S=R;
S.obj=[];
end

function P=compare_performance_r10(B,O)
P=struct('closed',false,'observer_peak_baseline_ms',NaN,'observer_peak_ms',NaN,'observer_peak_ratio',NaN, ...
    'flight_entry_do_baseline_ms',NaN,'flight_entry_do_ms',NaN,'flight_entry_do_ratio',NaN, ...
    'replan_do_baseline_ms',NaN,'replan_do_ms',NaN,'replan_do_ratio',NaN, ...
    'stage2_total_baseline_ms',NaN,'stage2_total_ms',NaN,'stage2_total_ratio',NaN, ...
    'stage2_postcache_baseline_ms',NaN,'stage2_postcache_ms',NaN,'stage2_postcache_ratio',NaN, ...
    'postcommit_ref_baseline_ms',NaN,'postcommit_ref_ms',NaN,'postcommit_ref_ratio',NaN);
if ~(B.closed&&O.closed), return; end
P.observer_peak_baseline_ms=max(B.obs_ring(:,2),[],'omitnan'); P.observer_peak_ms=max(O.obs_ring(:,2),[],'omitnan');
P.observer_peak_ratio=P.observer_peak_ms/nonzero_r10(P.observer_peak_baseline_ms);
mbf=B.base_ring(:,1)>=4.95-1e-10 & B.base_ring(:,1)<=5.20+1e-10;
mof=O.base_ring(:,1)>=4.95-1e-10 & O.base_ring(:,1)<=5.20+1e-10;
P.flight_entry_do_baseline_ms=max(B.base_ring(mbf,3),[],'omitnan'); P.flight_entry_do_ms=max(O.base_ring(mof,3),[],'omitnan');
P.flight_entry_do_ratio=P.flight_entry_do_ms/nonzero_r10(P.flight_entry_do_baseline_ms);
mbr=B.base_ring(:,1)>=9.80-1e-10 & B.base_ring(:,1)<=10.10+1e-10;
mor=O.base_ring(:,1)>=9.80-1e-10 & O.base_ring(:,1)<=10.10+1e-10;
P.replan_do_baseline_ms=max(B.base_ring(mbr,3),[],'omitnan'); P.replan_do_ms=max(O.base_ring(mor,3),[],'omitnan');
P.replan_do_ratio=P.replan_do_ms/nonzero_r10(P.replan_do_baseline_ms);
b2=B.base_ring(isfinite(B.base_ring(:,36)),:); o2=O.base_ring(isfinite(O.base_ring(:,36)),:);
if isempty(b2)||isempty(o2), return; end
P.stage2_total_baseline_ms=max(b2(:,36),[],'omitnan'); P.stage2_total_ms=max(o2(:,36),[],'omitnan');
P.stage2_total_ratio=P.stage2_total_ms/nonzero_r10(P.stage2_total_baseline_ms);
P.stage2_postcache_baseline_ms=max(b2(:,33),[],'omitnan'); P.stage2_postcache_ms=max(o2(:,33),[],'omitnan');
P.stage2_postcache_ratio=P.stage2_postcache_ms/nonzero_r10(P.stage2_postcache_baseline_ms);
mb=B.base_ring(:,1)>=9.925-1e-10 & B.base_ring(:,1)<=10.05+1e-10;
mo=O.base_ring(:,1)>=9.925-1e-10 & O.base_ring(:,1)<=10.05+1e-10;
P.postcommit_ref_baseline_ms=max(B.base_ring(mb,11),[],'omitnan'); P.postcommit_ref_ms=max(O.base_ring(mo,11),[],'omitnan');
P.postcommit_ref_ratio=P.postcommit_ref_ms/nonzero_r10(P.postcommit_ref_baseline_ms);
P.closed=all(isfinite([P.observer_peak_baseline_ms P.observer_peak_ms P.flight_entry_do_baseline_ms P.flight_entry_do_ms ...
    P.replan_do_baseline_ms P.replan_do_ms P.stage2_total_baseline_ms P.stage2_total_ms ...
    P.stage2_postcache_baseline_ms P.stage2_postcache_ms P.postcommit_ref_baseline_ms P.postcommit_ref_ms]));
end

function out=build_named_shadow_helper_r10(src,newTopName,mode)
% Rename only the top-level function and local de Casteljau declaration; keep
% every other local helper byte-preserved.
pat='(?m)^[ \t]*function[^\n]*\<af7r33_flat_jet_batch_model\>\s*\([^\n]*$';
[s0,e0]=regexp(src,pat,'start','end','once');
assert(~isempty(s0),'[AF7R58ZZY-R10] AF7R33 top-level declaration not found.');
top=src(s0:e0);
top2=regexprep(top,'(?<![A-Za-z0-9_])af7r33_flat_jet_batch_model(?=\s*\()',newTopName,'once');
src2=[src(1:s0-1) top2 src(e0+1:end)];
[decl,outVar,inVars,s,e]=locate_local_decl_r10(src2,'decasteljau_batch');
orig='decasteljau_batch_r10_original';
renamed=regexprep(decl,'(?<![A-Za-z0-9_])decasteljau_batch(?=\s*\()',orig,'once');
a1=inVars{1}; a2=inVars{2};
switch lower(mode)
    case 'capture'
        wrapper=sprintf(['function %s=decasteljau_batch(%s,%s)%c' ...
            'global AF7R63_DECASTELJAU_CAPTURE;%c' ...
            '%s=%s(%s,%s);%c' ...
            'if isempty(AF7R63_DECASTELJAU_CAPTURE), AF7R63_DECASTELJAU_CAPTURE={}; end%c' ...
            'AF7R63_DECASTELJAU_CAPTURE{end+1}={%s,%s,%s};%c' ...
            'end%c%c'],outVar,a1,a2,10,10,outVar,orig,a1,a2,10,10,a1,a2,outVar,10,10,10);
    case 'mex'
        wrapper=sprintf(['function %s=decasteljau_batch(%s,%s)%c' ...
            '%s=af7r63_decasteljau_batch_mex(%s,%s);%c' ...
            'end%c%c'],outVar,a1,a2,10,outVar,a1,a2,10,10,10);
    otherwise
        error('[AF7R58ZZY-R10] unknown shadow mode.');
end
out=[src2(1:s-1) wrapper renamed src2(e+1:end)];
end

function [decl,outVar,inVars,s,e]=locate_local_decl_r10(src,name)
pat=['(?m)^[ \t]*function[^\n]*\<' regexptranslate('escape',name) '\>\s*\([^\n]*$'];
[s,e]=regexp(src,pat,'start','end','once'); assert(~isempty(s),'[AF7R58ZZY-R10] local %s not found.',name);
decl=src(s:e);
expr=['function\s+(?:\[?\s*)?([A-Za-z]\w*)\s*(?:\]?)\s*=\s*' regexptranslate('escape',name) '\s*\(([^)]*)\)'];
tok=regexp(decl,expr,'tokens','once'); assert(~isempty(tok),'[AF7R58ZZY-R10] unsupported local declaration.');
outVar=strtrim(tok{1}); args=strtrim(tok{2}); if isempty(args), inVars={}; else, inVars=strtrim(strsplit(args,',')); end
assert(numel(inVars)==2,'[AF7R58ZZY-R10] de Casteljau signature changed.');
end

function compile_decasteljau_mex_r10(cppPath,outDir)
old=pwd; c=onCleanup(@()cd(old)); %#ok<NASGU> 
cd(outDir); err=[];
if ispc
    try, mex('-R2018a','COMPFLAGS=$COMPFLAGS /fp:strict',cppPath,'-outdir',outDir); return; catch ME, err=ME; end
else
    try, mex('-R2018a','CXXFLAGS=$CXXFLAGS -ffp-contract=off -fno-fast-math',cppPath,'-outdir',outDir); return; catch ME, err=ME; end
end
try, mex('-R2018a',cppPath,'-outdir',outDir); catch ME2
    if isempty(err), rethrow(ME2); end
    error('[AF7R58ZZY-R10] MEX compile failed. strict=%s | default=%s',err.message,ME2.message);
end
end

function tf=bitwise_same_double_r10(a,b)
a=double(a); b=double(b); tf=isequal(size(a),size(b)); if ~tf, return; end
tf=isequal(typecast(a(:),'uint64'),typecast(b(:),'uint64'));
end

function S=assert_runner_semantics_r10(txt)
checks={ ...
    'dt_25ms','dt = 0.025;'; 'terminal_50s','te = 50;'; ...
    'reference_only_commit','AVOIDANCE_COMMIT_POLICY = "REFERENCE_ONLY";'; ...
    'auto_commit_on','ENABLE_AUTO_REPLAN_COMMIT = true;'; 'plant_true_mL_014','plant_loadmass = 0.14;'; ...
    'cable_length_2m','agent.parameter.set("cableL",2.0);'; 'line_reference','{"gen_ref_line", {'; ...
    'line_speed_15','"velocity", 1.5,'; 'line_direction_pos_z','"direction", [0, 0, 1]'; ...
    'sensor_range_15m','replan_opts.trigger_dist = 15.0;'; 'payload_radius_05','replan_opts.r_load       = 0.5;'; ...
    'uav_radius_05','replan_opts.r_drone      = 0.5;'; 'cable_radius_05','replan_opts.cable        = 0.5;'; ...
    'backend_1','replan_opts.generator_backend_flag = 1;'; ...
    'observer_on','replan_opts.ENABLE_OBSERVER_HARD_SUPPORT = true;'; ...
    'observer_sim_truth_mode','replan_opts.OBSERVER_HARD_SUPPORT_MODE = "SIM_NUMERIC_TRUTH";'; ...
    'observer_true_mass_binding','replan_opts.OBSERVER_SIM_TRUE_ML = plant_loadmass;'; ...
    'takeoff_end_5s','TAKEOFF_REFERENCE(agent,"zd",3.0,"te",5)'; 'flight_start_5s','if time.t < 5.0'; ...
    'landing_start_47s','elseif time.t < 47.0'; ...
    'flight_ref_allocation','agent.cha_allocation.f.reference = ["timevarying","sload"];'; ...
    'flight_est_allocation','agent.cha_allocation.f.estimator = ["ekf","loadstate"];'};
pass=false(size(checks,1),1); for k=1:size(checks,1), pass(k)=contains(txt,checks{k,2}); end
if ~pass(end), pass(end)=contains(txt,'agent.set_cha_allocation_for_all("estimator",["ekf","loadstate"]);'); end
w0=strfind(txt,'while time.t <= time.te'); w1=strfind(txt,'time.t = time.t + dt;'); orderOK=false;
if numel(w0)==1 && ~isempty(w1)
    q=w1(find(w1>w0,1,'first')); if ~isempty(q)
        blk=txt(w0:q); tok={'do_prop_local(time,cha,cha,logger,env,agent,"sensor",1,true);', ...
            'do_prop_local(time,cha,cha,logger,env,agent,"estimator",1,true);', ...
            'do_prop_local(time,cha,cha,logger,env,agent,"reference",1,true);', ...
            'do_prop_local(time,cha,cha,logger,env,agent,"controller",1,true);', ...
            'do_prop_local(time,cha,cha,logger,env,agent,"input_transform",1,true);'};
        pos=nan(1,numel(tok)); for j=1:numel(tok), z=strfind(blk,tok{j}); if numel(z)==1,pos(j)=z;end,end %#ok<STREMP>
        orderOK=all(isfinite(pos))&&all(diff(pos)>0);
    end
end
S=struct('closed',all([pass;orderOK]),'pass_count',nnz([pass;orderOK]),'total_count',numel([pass;orderOK]));
assert(S.closed,'[AF7R58ZZY-R10] current runner target semantics changed.');
end


function n=count_stage2_rows_r11(ring)
if isempty(ring), n=0; return; end
n=nnz(isfinite(ring(:,36)));
end

function p=percentile_r10(x,q)
x=sort(double(x(isfinite(x)))); if isempty(x),p=NaN;return;end
k=1+(numel(x)-1)*q/100; a=floor(k); b=ceil(k); if a==b,p=x(a);else,p=x(a)+(k-a)*(x(b)-x(a));end
end
function y=nonzero_r10(x)
if ~isfinite(x)||x<=0
    y=eps;
else
    y=x;
end
end
function y=ternary_r10(c,a,b)
if c
    y=a;
else
    y=b;
end
end
function n=count_token_r10(txt,token)
n=numel(strfind(txt,token)); %#ok<STREMP>
end
function h=sha256_file_r10(path)
fid=fopen(path,'rb'); assert(fid>=0,'[AF7R58ZZY-R10] cannot open file for SHA: %s',path); c=onCleanup(@()fclose(fid)); %#ok<NASGU>
b=fread(fid,Inf,'*uint8'); md=javaMethod('getInstance','java.security.MessageDigest','SHA-256'); md.update(typecast(b,'int8'));
d=md.digest(); u=mod(double(d),256); h=lower(reshape(dec2hex(u,2).',1,[]));
end
function write_utf8_r10(path,txt)
fid=fopen(path,'w','n','UTF-8');
assert(fid>=0,'[AF7R58ZZY-R10] cannot create file: %s',path);
c=onCleanup(@()fclose(fid)); %#ok<NASGU>
fprintf(fid,'%s',txt);
end
function delete_if_exists_r10(path)
try
    if isfile(path), delete(path); end
catch
end
end
function say_r10(fid,fmt,varargin)
fprintf(fid,'%s%c',sprintf(fmt,varargin{:}),10);
end
function safe_fclose_r10(fid)
try
    if fid>0, fclose(fid); end
catch
end
end
