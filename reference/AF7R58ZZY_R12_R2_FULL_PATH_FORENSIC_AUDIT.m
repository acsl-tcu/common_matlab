function OUT = AF7R58ZZY_R12_FULL_PATH_FORENSIC_AUDIT()
% AF7R58ZZY-R12 | FULL-PATH TIMING FORENSIC AUDIT
%
% Diagnostic only.  No production installation and no new optimization.
% R12 corrects the R11 performance authority error: R5 timing-ring column 2
% is do() total; column 3 is get_load_mass(), NOT do().
%
% The audit runs the exact R8-R2 baseline and the exact R11 shadow, then
% closes the following on one time axis:
%   - full do() wall time and non-overlapping top-level subtotal residual,
%   - observer live vs exact post-mission hot replay,
%   - Stage 1/2/3 detailed timing and Stage2 unmeasured residuals,
%   - R10FAST live Stage2 vs post-mission same-object replay/profile,
%   - post-commit repaired-reference packet substage timing.
% A locally improved bucket can never mark performance closed by itself.

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
    '[AF7R58ZZY-R12] required source/package file missing.');
sourceSHA=sha256_file_r10(srcPath); runnerSHA=sha256_file_r10(runnerPath);
helperSHA=sha256_file_r10(helperPath); r8SHA=sha256_file_r10(r8Path);
r5SHA=sha256_file_r10(r5Path); r10SHA=sha256_file_r10(r10Path); r11SHA=sha256_file_r10(r11Path);
assert(strcmpi(sourceSHA,EXPECTED_SOURCE_SHA),'[R12] production source SHA mismatch.');
assert(strcmpi(runnerSHA,EXPECTED_RUNNER_SHA),'[R12] runner SHA mismatch.');
assert(strcmpi(helperSHA,EXPECTED_AF7R33_SHA),'[R12] AF7R33 SHA mismatch.');
assert(strcmpi(r8SHA,EXPECTED_R8_SHA),'[R12] R8-R2 SHA mismatch.');
assert(strcmpi(r5SHA,EXPECTED_R5R4_SHA),'[R12] R5-R4 SHA mismatch.');
assert(strcmpi(r10SHA,EXPECTED_R10_SHA),'[R12] R10 SHA mismatch.');
assert(strcmpi(r11SHA,EXPECTED_R11_SHA),'[R12] R11 SHA mismatch.');
assert(strcmpi(fileparts(helperPath),fileparts(srcPath)), '[R12] AF7R33 not source-folder bound.');

runnerText=fileread(runnerPath);
runnerSemantics=assert_runner_semantics_r10(runnerText);
assert(count_token_r10(fileread(r10Path),'af7r33_flat_jet_batch_model_R10FAST(obj,tqCache,batchModel)')==1, ...
    '[R12] direct Stage2 fast connection is not unique.');
assert(count_token_r10(fileread(r11Path),'af7r33_flat_jet_batch_model_R10FAST(obj,tq,R.batch_flat_model)')==2, ...
    '[R12] R11 packet fast connections changed.');

fprintf('====================================================================================================\n');
fprintf(' AF7R58ZZY-R12 | FULL-PATH TIMING FORENSIC AUDIT\n');
fprintf('====================================================================================================\n');
fprintf('[AUTHORITY] diagnosticOnly=1 | productionModified=0 | noPhaseShift=1 | doAuthorityColumn=2\n');
fprintf(' source SHA       = %s\n',sourceSHA);
fprintf(' runner SHA       = %s\n',runnerSHA);
fprintf(' AF7R33 SHA       = %s\n',helperSHA);
fprintf(' R10/R11 SHA      = %s / %s\n',r10SHA,r11SHA);
fprintf(' runner semantics = %d/%d\n',runnerSemantics.pass_count,runnerSemantics.total_count);

root=fileparts(srcPath);
outDir=tempname(root); mkdir(outDir);
shadowDir=tempname(root); mkdir(shadowDir);
reportPath=fullfile(outDir,'AF7R58ZZY_R12_R2_REPORT.txt');
outPath=fullfile(outDir,'AF7R58ZZY_R12_R2_OUT.mat');
rawPath=fullfile(outDir,'AF7R58ZZY_R12_R2_RAW_CAPTURE.mat');
Baseline=empty_run_r10(); Optimized=empty_run_r10();
Mex=struct('closed',false,'capture_full_exact',false,'local_bitwise_exact',false, ...
    'full_helper_bitwise_exact',false,'capture_count',0,'hot_median_ms',NaN,'hot_p95_ms',NaN, ...
    'error',struct('identifier','','message',''));
runError=struct('identifier','','message','');

fprintf('\n[A. BASELINE FLIGHT -- R8-R2]\n');
try
    Baseline=run_flight_r10(root,scriptDir,runnerText,'REPLANNING_SYSTEM_R8R2_DIAG','');
    assert(Baseline.closed,'[R12] baseline Flight did not close.');
catch ME
    runError.identifier=ME.identifier; runError.message=ME.message;
end

if isempty(runError.message)
    fprintf('\n[B. REBUILD + EXACTLY VERIFY EXISTING R10FAST SHADOW]\n');
    try
        objB=Baseline.obj; Rb=objB.active_repaired_reference;
        assert(isstruct(Rb)&&isfield(Rb,'batch_flat_model'),'[R12] baseline batch model unavailable.');
        dt=double(objB.prediction_dt); H=double(objB.horizon_s); N=round(H/dt)+1;
        t0=double(objB.last_replan_commit_time)+dt; tq=t0+(0:N-1)*dt;
        [Yorig,Vorig]=af7r33_flat_jet_batch_model(objB,tq,Rb.batch_flat_model); Yorig=double(Yorig);
        src=fileread(helperPath);
        captureName='af7r33_flat_jet_batch_model_R10CAPTURE';
        fastName='af7r33_flat_jet_batch_model_R10FAST';
        write_utf8_r10(fullfile(shadowDir,[captureName '.m']),build_named_shadow_helper_r10(src,captureName,'capture'));
        write_utf8_r10(fullfile(shadowDir,[fastName '.m']),build_named_shadow_helper_r10(src,fastName,'mex'));
        addpath(shadowDir,'-begin'); rehash;
        clear af7r33_flat_jet_batch_model_R10CAPTURE af7r33_flat_jet_batch_model_R10FAST;
        global AF7R63_DECASTELJAU_CAPTURE; AF7R63_DECASTELJAU_CAPTURE={};
        [Ycap,Vcap]=af7r33_flat_jet_batch_model_R10CAPTURE(objB,tq,Rb.batch_flat_model); Ycap=double(Ycap);
        Mex.capture_count=numel(AF7R63_DECASTELJAU_CAPTURE);
        Mex.capture_full_exact=bitwise_same_double_r10(Yorig,Ycap)&&isequaln(Vorig,Vcap);
        assert(Mex.capture_full_exact&&Mex.capture_count>=1,'[R12] capture helper not exact.');
        compile_decasteljau_mex_r10(cppPath,shadowDir); rehash; clear af7r63_decasteljau_batch_mex;
        localExact=true;
        for k=1:Mex.capture_count
            A=AF7R63_DECASTELJAU_CAPTURE{k}; ym=af7r63_decasteljau_batch_mex(A{1},A{2});
            localExact=localExact&&bitwise_same_double_r10(double(A{3}),double(ym));
        end
        Mex.local_bitwise_exact=localExact; assert(localExact,'[R12] local MEX not bitwise exact.');
        clear af7r33_flat_jet_batch_model_R10FAST; rehash;
        [Yfast,Vfast]=af7r33_flat_jet_batch_model_R10FAST(objB,tq,Rb.batch_flat_model); Yfast=double(Yfast);
        Mex.full_helper_bitwise_exact=bitwise_same_double_r10(Yorig,Yfast)&&isequaln(Vorig,Vfast);
        assert(Mex.full_helper_bitwise_exact,'[R12] full R10FAST helper not exact.');
        tm=zeros(1,30); for k=1:numel(tm), T=tic; af7r33_flat_jet_batch_model_R10FAST(objB,tq,Rb.batch_flat_model); tm(k)=1e3*toc(T); end
        Mex.hot_median_ms=median(tm); Mex.hot_p95_ms=percentile_r10(tm,95); Mex.closed=true;
    catch ME
        Mex.error.identifier=ME.identifier; Mex.error.message=ME.message;
        runError.identifier=ME.identifier; runError.message=ME.message;
    end
end

if isempty(runError.message)&&Mex.closed
    fprintf('\n[C. R11 SHADOW FLIGHT -- SAME EXACT FAST CONNECTION]\n');
    try
        Optimized=run_flight_r10(root,scriptDir,runnerText,'REPLANNING_SYSTEM_R11_DIAG',shadowDir);
        assert(Optimized.closed,'[R12] R11 Flight did not close.');
        assert(Optimized.stage2_fast_real_calls>=1,'[R12] real Stage2 did not use fast helper.');
    catch ME
        runError.identifier=ME.identifier; runError.message=ME.message;
    end
end

Forensic=struct(); diagError=struct('identifier','','message','');
try
    assert(isempty(runError.message)&&Baseline.closed&&Optimized.closed,'[R12] flight pair incomplete.');
    Forensic.performance=compare_performance_r12(Baseline,Optimized);
    Forensic.baseline_windows=window_forensic_r12(Baseline.base_ring);
    Forensic.optimized_windows=window_forensic_r12(Optimized.base_ring);
    Forensic.baseline_pipeline=pipeline_forensic_r12(Baseline.base_ring);
    Forensic.optimized_pipeline=pipeline_forensic_r12(Optimized.base_ring);
    Forensic.observer=observer_forensic_r12(Optimized.obj,Optimized.obs_ring,40);
    Forensic.fast=fast_forensic_r12(Optimized.obj,Optimized.base_ring,30);
    Forensic.reference=reference_forensic_r12(Optimized.ref_ring);
catch ME
    diagError.identifier=ME.identifier; diagError.message=ME.message;
end

metricsClosed=isempty(diagError.message)&&isfield(Forensic,'performance')&&Forensic.performance.closed;
exactClosed=Mex.closed&&Mex.capture_full_exact&&Mex.local_bitwise_exact&&Mex.full_helper_bitwise_exact;
connectionClosed=Optimized.stage2_fast_real_calls>=1;
residualClosed=false; replayClosed=false;
if metricsClosed
    residualClosed=Forensic.optimized_windows.residual_finite && Forensic.optimized_pipeline.residual_finite;
    replayClosed=Forensic.observer.closed && Forensic.fast.closed && Forensic.reference.closed;
end
diagnosticClosed=isempty(runError.message)&&metricsClosed&&exactClosed&&connectionClosed&&residualClosed&&replayClosed;
performanceClosed=false; control25Closed=false; desired4Closed=false; desired3Closed=false;
if metricsClosed
    control25Closed=Forensic.performance.target_do_peak_ms<=25.0;
    desired4Closed=Forensic.performance.target_do_peak_ms<=4.0;
    desired3Closed=Forensic.performance.target_do_peak_ms<=3.0;
    performanceClosed=control25Closed && Forensic.performance.no_regression_changed_path;
end
if diagnosticClosed && performanceClosed
    status='FULL_PATH_FORENSIC_R12_COMPLETE__PERFORMANCE_25MS_CLOSED';
elseif diagnosticClosed
    status='FULL_PATH_FORENSIC_R12_COMPLETE__PERFORMANCE_OPEN';
else
    status='FULL_PATH_FORENSIC_R12_INCOMPLETE_FAIL_CLOSED';
end

RAW=struct(); RAW.status=status; RAW.baseline=strip_run_obj_r10(Baseline); RAW.optimized=strip_run_obj_r10(Optimized);
RAW.mex=Mex; RAW.forensic=Forensic; RAW.run_error=runError; RAW.diag_error=diagError;
RAW.authority=struct('source_sha',sourceSHA,'runner_sha',runnerSHA,'af7r33_sha',helperSHA, ...
    'r8_sha',r8SHA,'r5_sha',r5SHA,'r10_sha',r10SHA,'r11_sha',r11SHA,'runner_semantics',runnerSemantics);
save(rawPath,'RAW','-v7.3');
OUT=struct(); OUT.status=status; OUT.diagnostic_closed=diagnosticClosed; OUT.performance_closed=performanceClosed;
OUT.control_period_25ms_closed=control25Closed; OUT.historical_4ms_closed=desired4Closed; OUT.desired_3ms_closed=desired3Closed;
OUT.production_modified=false; OUT.shadow_only=true; OUT.no_phase_shift=true; OUT.new_arming_work=0; OUT.new_takeoff_work=0; OUT.new_landing_work=0;
OUT.forensic=Forensic; OUT.mex=Mex; OUT.run_error=runError; OUT.diag_error=diagError; OUT.raw_capture_path=rawPath; OUT.report_path=reportPath;
save(outPath,'OUT','-v7.3');

fid=fopen(reportPath,'w'); assert(fid>=0); c=onCleanup(@()safe_fclose_r10(fid)); %#ok<NASGU>
say_r10(fid,'status=%s',status); say_r10(fid,'diagnosticClosed=%d',diagnosticClosed); say_r10(fid,'performanceClosed=%d',performanceClosed);
say_r10(fid,'controlPeriod25msClosed=%d',control25Closed); say_r10(fid,'historical4msClosed=%d',desired4Closed); say_r10(fid,'desired3msClosed=%d',desired3Closed);
say_r10(fid,'productionModified=0'); say_r10(fid,'shadowOnly=1'); say_r10(fid,'noPhaseShift=1');
say_r10(fid,'newArmingWork=0'); say_r10(fid,'newTakeoffWork=0'); say_r10(fid,'newLandingWork=0');
say_r10(fid,'doAuthorityColumn=2'); say_r10(fid,'column3Meaning=GET_LOAD_MASS_NOT_DO_TOTAL');
say_r10(fid,'sourceSHA=%s',sourceSHA); say_r10(fid,'runnerSHA=%s',runnerSHA); say_r10(fid,'originalAF7R33SHA=%s',helperSHA);
say_r10(fid,'runnerSemantics=%d/%d',runnerSemantics.pass_count,runnerSemantics.total_count);
say_r10(fid,'raw=%s',rawPath); say_r10(fid,'out=%s',outPath);

say_r10(fid,''); say_r10(fid,'[CORRECT_FULL_PATH_PERFORMANCE]');
if metricsClosed
    P=Forensic.performance;
    say_r10(fid,'columns=metric baseline_ms optimized_ms ratio');
    say_r10(fid,'flightEntryDoTotalPeak %.9g %.9g %.9g',P.flight_entry_do_baseline_ms,P.flight_entry_do_ms,P.flight_entry_do_ratio);
    say_r10(fid,'replanDoTotalPeak %.9g %.9g %.9g',P.replan_do_baseline_ms,P.replan_do_ms,P.replan_do_ratio);
    say_r10(fid,'targetDoTotalPeak %.9g %.9g %.9g',P.target_do_baseline_peak_ms,P.target_do_peak_ms,P.target_do_ratio);
    say_r10(fid,'observerPeak %.9g %.9g %.9g',P.observer_peak_baseline_ms,P.observer_peak_ms,P.observer_peak_ratio);
    say_r10(fid,'stage1Total %.9g %.9g %.9g',P.stage1_baseline_ms,P.stage1_ms,P.stage1_ratio);
    say_r10(fid,'stage2Total %.9g %.9g %.9g',P.stage2_total_baseline_ms,P.stage2_total_ms,P.stage2_total_ratio);
    say_r10(fid,'stage2Postcache %.9g %.9g %.9g',P.stage2_postcache_baseline_ms,P.stage2_postcache_ms,P.stage2_postcache_ratio);
    say_r10(fid,'stage3Total %.9g %.9g %.9g',P.stage3_baseline_ms,P.stage3_ms,P.stage3_ratio);
    say_r10(fid,'postcommitRefPacket %.9g %.9g %.9g',P.postcommit_ref_baseline_ms,P.postcommit_ref_ms,P.postcommit_ref_ratio);
end

if metricsClosed
    say_r10(fid,''); say_r10(fid,'[BASELINE_FULL_DO_ROWS]'); print_window_rows_r12(fid,Forensic.baseline_windows);
    say_r10(fid,''); say_r10(fid,'[R11_FULL_DO_ROWS]'); print_window_rows_r12(fid,Forensic.optimized_windows);
    say_r10(fid,''); say_r10(fid,'[BASELINE_PIPELINE_DETAIL]'); print_pipeline_r12(fid,Forensic.baseline_pipeline);
    say_r10(fid,''); say_r10(fid,'[R11_PIPELINE_DETAIL]'); print_pipeline_r12(fid,Forensic.optimized_pipeline);
end

say_r10(fid,''); say_r10(fid,'[OBSERVER_LIVE_VS_EXACT_REPLAY]');
if isfield(Forensic,'observer')&&Forensic.observer.closed
    say_r10(fid,'snapshotCount=%d',Forensic.observer.snapshot_count);
    say_r10(fid,'preflightPrimeDone=%d',Forensic.observer.preflight_prime_done);
    say_r10(fid,'preflightPrimeMs=%.9g',Forensic.observer.preflight_prime_ms);
    say_r10(fid,'columns=t liveTotal oneHot contractHot hotSum liveToHotRatio exact');
    for k=1:numel(Forensic.observer.events)
        if iscell(Forensic.observer.events), E=Forensic.observer.events{k}; else, E=Forensic.observer.events(k); end
        say_r10(fid,'row=%.6g %.9g %.9g %.9g %.9g %.9g %d',E.t,E.live_total_ms,E.one_hot_median_ms,E.contract_hot_median_ms,E.hot_sum_ms,E.live_to_hot_ratio,E.exact);
    end
    say_r10(fid,'classification=%s',Forensic.observer.classification);
end

say_r10(fid,''); say_r10(fid,'[FAST_HELPER_LIVE_VS_SAME_OBJECT_REPLAY]');
if isfield(Forensic,'fast')&&Forensic.fast.closed
    F=Forensic.fast;
    say_r10(fid,'liveStage2PostcacheMs=%.9g',F.live_stage2_postcache_ms);
    say_r10(fid,'sameObjectFirstMs=%.9g',F.first_ms); say_r10(fid,'sameObjectHotMedianMs=%.9g',F.hot_median_ms); say_r10(fid,'sameObjectHotP95Ms=%.9g',F.hot_p95_ms);
    say_r10(fid,'liveMinusHotMedianMs=%.9g',F.live_minus_hot_median_ms);
    say_r10(fid,'originalHelperHotMedianMs=%.9g',F.original_hot_median_ms);
    say_r10(fid,'bitwiseExact=%d',F.bitwise_exact);
    print_profile_top_r12(fid,F.profile);
end

say_r10(fid,''); say_r10(fid,'[POSTCOMMIT_REFERENCE_PACKET_DETAIL]'); if isfield(Forensic,'reference'), print_reference_r12(fid,Forensic.reference); else, say_r10(fid,'closed=0'); end
say_r10(fid,''); say_r10(fid,'[INTERPRETATION_CONTRACT]');
say_r10(fid,'timingIsEmpirical=1'); say_r10(fid,'formalWCETClaim=0'); say_r10(fid,'localBucketImprovementCannotClosePerformance=1');
say_r10(fid,'trajectoryMathChanged=0'); say_r10(fid,'certificateAcceptanceChanged=0'); say_r10(fid,'commitPolicyChanged=0');
say_r10(fid,'productionInstallPerformed=0');
if ~isempty(runError.message), say_r10(fid,'runError=%s | %s',runError.identifier,runError.message); end
if ~isempty(diagError.message), say_r10(fid,'diagError=%s | %s',diagError.identifier,diagError.message); end

fprintf('\n[D. RESULT]\n status                 = %s\n',status);
if metricsClosed
    fprintf(' full do peak R11       = %.4f ms\n',Forensic.performance.target_do_peak_ms);
    fprintf(' observer peak R11      = %.4f ms\n',Forensic.performance.observer_peak_ms);
    fprintf(' Stage2 total / cache   = %.4f / %.4f ms\n',Forensic.performance.stage2_total_ms,Forensic.performance.stage2_postcache_ms);
end
fprintf(' diagnostic closed      = %d\n performance closed     = %d\n',diagnosticClosed,performanceClosed);
fprintf('RETURN THESE THREE FILES:\n %s\n %s\n %s\n',reportPath,outPath,rawPath);
end

function P=compare_performance_r12(B,O)
P=struct('closed',false);
if ~(B.closed&&O.closed), return; end
P.observer_peak_baseline_ms=max(B.obs_ring(:,2),[],'omitnan'); P.observer_peak_ms=max(O.obs_ring(:,2),[],'omitnan'); P.observer_peak_ratio=P.observer_peak_ms/nonzero_r10(P.observer_peak_baseline_ms);
mbf=winmask_r12(B.base_ring(:,1),4.95,5.20); mof=winmask_r12(O.base_ring(:,1),4.95,5.20);
P.flight_entry_do_baseline_ms=max(B.base_ring(mbf,2),[],'omitnan'); P.flight_entry_do_ms=max(O.base_ring(mof,2),[],'omitnan'); P.flight_entry_do_ratio=P.flight_entry_do_ms/nonzero_r10(P.flight_entry_do_baseline_ms);
mbr=winmask_r12(B.base_ring(:,1),9.80,10.10); mor=winmask_r12(O.base_ring(:,1),9.80,10.10);
P.replan_do_baseline_ms=max(B.base_ring(mbr,2),[],'omitnan'); P.replan_do_ms=max(O.base_ring(mor,2),[],'omitnan'); P.replan_do_ratio=P.replan_do_ms/nonzero_r10(P.replan_do_baseline_ms);
P.target_do_baseline_peak_ms=max(P.flight_entry_do_baseline_ms,P.replan_do_baseline_ms); P.target_do_peak_ms=max(P.flight_entry_do_ms,P.replan_do_ms); P.target_do_ratio=P.target_do_peak_ms/nonzero_r10(P.target_do_baseline_peak_ms);
b1=B.base_ring(isfinite(B.base_ring(:,18)),:); o1=O.base_ring(isfinite(O.base_ring(:,18)),:); P.stage1_baseline_ms=max(b1(:,18),[],'omitnan'); P.stage1_ms=max(o1(:,18),[],'omitnan'); P.stage1_ratio=P.stage1_ms/nonzero_r10(P.stage1_baseline_ms);
b2=B.base_ring(isfinite(B.base_ring(:,36)),:); o2=O.base_ring(isfinite(O.base_ring(:,36)),:); P.stage2_total_baseline_ms=max(b2(:,36),[],'omitnan'); P.stage2_total_ms=max(o2(:,36),[],'omitnan'); P.stage2_total_ratio=P.stage2_total_ms/nonzero_r10(P.stage2_total_baseline_ms); P.stage2_postcache_baseline_ms=max(b2(:,33),[],'omitnan'); P.stage2_postcache_ms=max(o2(:,33),[],'omitnan'); P.stage2_postcache_ratio=P.stage2_postcache_ms/nonzero_r10(P.stage2_postcache_baseline_ms);
b3=B.base_ring(isfinite(B.base_ring(:,37)),:); o3=O.base_ring(isfinite(O.base_ring(:,37)),:); P.stage3_baseline_ms=max(b3(:,37),[],'omitnan'); P.stage3_ms=max(o3(:,37),[],'omitnan'); P.stage3_ratio=P.stage3_ms/nonzero_r10(P.stage3_baseline_ms);
mb=winmask_r12(B.base_ring(:,1),9.925,10.05); mo=winmask_r12(O.base_ring(:,1),9.925,10.05); P.postcommit_ref_baseline_ms=max(B.base_ring(mb,11),[],'omitnan'); P.postcommit_ref_ms=max(O.base_ring(mo,11),[],'omitnan'); P.postcommit_ref_ratio=P.postcommit_ref_ms/nonzero_r10(P.postcommit_ref_baseline_ms);
P.no_regression_changed_path=P.stage2_total_ms<=P.stage2_total_baseline_ms && P.stage2_postcache_ms<=P.stage2_postcache_baseline_ms;
v=[P.observer_peak_baseline_ms P.observer_peak_ms P.flight_entry_do_baseline_ms P.flight_entry_do_ms P.replan_do_baseline_ms P.replan_do_ms P.target_do_baseline_peak_ms P.target_do_peak_ms P.stage1_baseline_ms P.stage1_ms P.stage2_total_baseline_ms P.stage2_total_ms P.stage2_postcache_baseline_ms P.stage2_postcache_ms P.stage3_baseline_ms P.stage3_ms P.postcommit_ref_baseline_ms P.postcommit_ref_ms]; P.closed=all(isfinite(v));
end

function W=window_forensic_r12(ring)
W=struct(); W.flight=rows_for_window_r12(ring,4.95,5.20); W.replan=rows_for_window_r12(ring,9.80,10.10);
allr=[W.flight;W.replan]; W.residual_finite=~isempty(allr)&&all(isfinite(allr(:,end)));
end
function X=rows_for_window_r12(ring,a,b)
m=winmask_r12(ring(:,1),a,b); R=ring(m,:); X=nan(size(R,1),20);
% t do load observer live dependency base cert active geom ref sensor memory trigger mex decode request advance output residual
for i=1:size(R,1)
    top=R(i,[3 4 5 6 7 8 9 10 11 12 13 14 17 23 42]); top(~isfinite(top))=0;
    X(i,:)=[R(i,1),R(i,2),R(i,3),R(i,4),R(i,5),R(i,6),R(i,7),R(i,8),R(i,9),R(i,10),R(i,11),R(i,12),R(i,13),R(i,14),R(i,15),R(i,16),R(i,17),R(i,23),R(i,42),R(i,2)-sum(top)];
end
end
function print_window_rows_r12(fid,W)
say_r10(fid,'columns=t doTotal loadmass observer liveBind dependency baseRef certGuard activeRef geometry refPacket rawSensor sensorMemory trigger mexNested decodeNested request advance output residual');
A=[W.flight;W.replan];
for i=1:size(A,1), say_r10(fid,['row=' repmat(' %.9g',1,20)],A(i,:)); end
end

function P=pipeline_forensic_r12(ring)
P=struct(); P.stage1=ring(isfinite(ring(:,18)),[1 18 19 20 21 22]);
S2=ring(isfinite(ring(:,36)),:); P.stage2=nan(size(S2,1),15);
for i=1:size(S2,1)
    prep=S2(i,26); parts=S2(i,27:35); q=parts; q(~isfinite(q))=0;
    P.stage2(i,:)=[S2(i,1),S2(i,36),S2(i,25),prep,parts, S2(i,36)-sum_finite_r12(S2(i,[25 26])), prep-sum(q)];
end
S3=ring(isfinite(ring(:,37)),:); P.stage3=S3(:,[1 37 38 39 40 41]);
P.residual_finite=~isempty(P.stage2)&&all(isfinite(P.stage2(:,end-1:end)));
end
function print_pipeline_r12(fid,P)
say_r10(fid,'STAGE1 columns=t total futureRef context proposal other'); for i=1:size(P.stage1,1), say_r10(fid,'stage1= %.9g %.9g %.9g %.9g %.9g %.9g',P.stage1(i,:)); end
say_r10(fid,'STAGE2 columns=t total refCert prepare pregates batchModel sweep packet sensorCert handlesMeta postcache derive bundle stageResidual prepareResidual'); for i=1:size(P.stage2,1), say_r10(fid,['stage2=' repmat(' %.9g',1,15)],P.stage2(i,:)); end
say_r10(fid,'STAGE3 columns=t total precheck snapshot swapGuard commitTail'); for i=1:size(P.stage3,1), say_r10(fid,'stage3= %.9g %.9g %.9g %.9g %.9g %.9g',P.stage3(i,:)); end
end

function O=observer_forensic_r12(obj,obsRing,nrep)
O=struct('closed',false,'snapshot_count',0,'events',[],'classification','UNRESOLVED','preflight_prime_done',false,'preflight_prime_ms',NaN);
O.preflight_prime_done=logical(obj.observer_preflight_prime_done); O.preflight_prime_ms=double(obj.observer_preflight_prime_ms);
n=double(obj.af7r60_obs_snapshot_count); O.snapshot_count=n; if n<1, return; end
E=repmat(struct('t',NaN,'live_total_ms',NaN,'one_hot_median_ms',NaN,'contract_hot_median_ms',NaN,'hot_sum_ms',NaN,'live_to_hot_ratio',NaN,'exact',false),1,n);
for k=1:n
    S=obj.af7r60_obs_snapshots{k}; one=zeros(1,nrep); con=zeros(1,nrep); F=[]; K=[];
    for j=1:nrep
        T=tic; F=obj.observer_one_step_support(S.h,S.uPrev,S.I); one(j)=1e3*toc(T);
        T=tic; K=obj.observer_contract_two_events(S.C0,S.C1,S.F,S.xhat); con(j)=1e3*toc(T);
    end
    [~,ii]=min(abs(obsRing(:,1)-double(S.t))); live=obsRing(ii,2);
    hs=median(one)+median(con); E(k)=struct('t',double(S.t),'live_total_ms',live,'one_hot_median_ms',median(one),'contract_hot_median_ms',median(con),'hot_sum_ms',hs,'live_to_hot_ratio',live/nonzero_r10(hs),'exact',isequaln(F,S.F)&&isequaln(K,S.K));
end
O.events=E; ex=[E.exact]; ratios=[E.live_to_hot_ratio];
if all(ex)&&max(ratios)>20, O.classification='LIVE_COLD_OR_ALLOCATION_DOMINANT_DESPITE_PREFLIGHT_PRIME'; elseif all(ex), O.classification='MIXED_OR_STEADY'; else, O.classification='REPLAY_EQUIVALENCE_OPEN'; end
O.closed=all(ex)&&all(isfinite(ratios));
end

function F=fast_forensic_r12(obj,baseRing,nrep)
F=struct('closed',false,'live_stage2_postcache_ms',NaN,'first_ms',NaN,'hot_median_ms',NaN,'hot_p95_ms',NaN,'live_minus_hot_median_ms',NaN,'original_hot_median_ms',NaN,'bitwise_exact',false,'profile',struct('ok',false,'top',[]));
R=obj.active_repaired_reference; assert(isstruct(R)&&isfield(R,'batch_flat_model'),'[R12] optimized active batch model unavailable.'); dt=double(obj.prediction_dt); H=double(obj.horizon_s); N=round(H/dt)+1; tc=double(obj.last_replan_commit_time)+dt; tq=tc+(0:N-1)*dt;
S2=baseRing(isfinite(baseRing(:,36)),:); assert(~isempty(S2)); F.live_stage2_postcache_ms=max(S2(:,33),[],'omitnan');
T=tic; [Yf,Vf]=af7r33_flat_jet_batch_model_R10FAST(obj,tq,R.batch_flat_model); F.first_ms=1e3*toc(T); tf=zeros(1,nrep); for k=1:nrep, T=tic; [Yh,Vh]=af7r33_flat_jet_batch_model_R10FAST(obj,tq,R.batch_flat_model); tf(k)=1e3*toc(T); end
F.hot_median_ms=median(tf); F.hot_p95_ms=percentile_r10(tf,95); F.live_minus_hot_median_ms=F.live_stage2_postcache_ms-F.hot_median_ms;
to=zeros(1,nrep); for k=1:nrep, T=tic; [Yo,Vo]=af7r33_flat_jet_batch_model(obj,tq,R.batch_flat_model); to(k)=1e3*toc(T); end; F.original_hot_median_ms=median(to);
F.bitwise_exact=bitwise_same_double_r10(double(Yf),double(Yo))&&isequaln(Vf,Vo)&&bitwise_same_double_r10(double(Yh),double(Yo))&&isequaln(Vh,Vo);
F.profile=profile_fast_r12(obj,R,tq,15); F.closed=F.bitwise_exact&&F.profile.ok&&all(isfinite([F.live_stage2_postcache_ms F.hot_median_ms F.original_hot_median_ms]));
end
function P=profile_fast_r12(obj,R,tq,nrep)
P=struct('ok',false,'top',[]); try, profile clear; profile on; for k=1:nrep, af7r33_flat_jet_batch_model_R10FAST(obj,tq,R.batch_flat_model); end; profile off; info=profile('info'); P.ok=true; P.top=top_profile_r12(info,20); catch, try, profile off; catch, end; end
end
function T=top_profile_r12(info,n)
T=struct('name',{},'total_s',{},'calls',{}); if ~isstruct(info)||~isfield(info,'FunctionTable')||isempty(info.FunctionTable), return; end; A=info.FunctionTable; tt=[A.TotalTime]; [~,ix]=sort(tt,'descend'); ix=ix(1:min(n,numel(ix))); for k=1:numel(ix), q=ix(k); T(k).name=char(A(q).FunctionName); T(k).total_s=double(A(q).TotalTime); T(k).calls=double(A(q).NumCalls); end
end
function print_profile_top_r12(fid,P)
if ~P.ok, say_r10(fid,'profileAvailable=0'); return; end; say_r10(fid,'profileAvailable=1'); for k=1:numel(P.top), say_r10(fid,'rank=%d total_s=%.9g calls=%.0f name=%s',k,P.top(k).total_s,P.top(k).calls,P.top(k).name); end
end

function R=reference_forensic_r12(refRing)
R=struct('closed',false,'rows',[],'peak_total_ms',NaN,'peak_t',NaN,'peak_dominant',''); if isempty(refRing), return; end; X=refRing(isfinite(refRing(:,2)),:); if isempty(X), return; end
% output: t,total,assert,setup,eligibility,shift,alloc,tailScalar,tailBatch,derive,validate,store,pathCode
R.rows=X(:,[1 2 3 11 12 13 18 19 20 21 22 23 14]); [R.peak_total_ms,ii]=max(X(:,2)); R.peak_t=X(ii,1); vals=X(ii,[3 11 12 13 18 19 20 21 22 23]); names={'ASSERT','SETUP_AUTHORITY','CACHE_ELIGIBILITY','SHIFT_CALC','SHIFT_ALLOC','TAIL_SCALAR','TAIL_BATCH','DERIVE','VALIDATE','CACHE_STORE'}; [~,jj]=max(vals); R.peak_dominant=names{jj}; R.closed=true;
end
function print_reference_r12(fid,R)
if ~R.closed, say_r10(fid,'closed=0'); return; end; say_r10(fid,'peakTotalMs=%.9g',R.peak_total_ms); say_r10(fid,'peakT=%.9g',R.peak_t); say_r10(fid,'peakDominant=%s',R.peak_dominant); say_r10(fid,'columns=t total assert setup eligibility shiftCalc shiftAlloc tailScalar tailBatch derive validate store pathCode'); for i=1:size(R.rows,1), say_r10(fid,['row=' repmat(' %.9g',1,13)],R.rows(i,:)); end
end
function m=winmask_r12(t,a,b), m=t>=a-1e-10 & t<=b+1e-10; end
function s=sum_finite_r12(x), y=x; y(~isfinite(y))=0; s=sum(y); end

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
