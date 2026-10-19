function OUT = AF7R58ZZY_R14_R2_OBSERVER_AOT_RUNTIME_AUDIT()
% AF7R58ZZY-R14-R2 | AOT/MEX OBSERVER SHADOW + FULL RUNTIME AUDIT
% Shadow only. No production install. No added observer warmup call.
% Primary control deadline authority: runtime_profile.online_before_plant_s.
% full_iteration_s is simulation+plant+logger context only and is reported
% separately; it is not substituted for the onboard/control deadline.

clc;
scriptDir=fileparts(mfilename('fullpath'));
addpath(scriptDir,'-begin'); rehash;

EXPECTED_SOURCE_SHA='310cfdd01f9554a80b71e8d90464816a721859709ad7c4b9137cdc0f3a6eaf49';
EXPECTED_RUNNER_SHA='d4d15d081ea8a6e3073b0a63ce36e47f983777ac2c81db194384de69f5e5fc37';
EXPECTED_R8_SHA='5ca886499192ee309c0bfd7a39c2094de708e4a85104fb7a54e99b2b987e97df';
EXPECTED_R5_SHA='2b874c2e92052de1c756c3b00f8f62f5cb08018a496b9c78b981df085f2efb18';
EXPECTED_R14_SHA='9b499dad043254509e2d9fe8e50640695f59c7a407980ff24cac017e496d1532';
EXPECTED_CPP_SHA='8db6028f20b726b800fdb8b12b9b5e78d2606d07164cfd29b59c99af0336ca32';

srcPath=which('REPLANNING_SYSTEM'); runnerPath=which('run_flight_headless');
r8Path=fullfile(scriptDir,'REPLANNING_SYSTEM_R8R2_DIAG.m');
r5Path=fullfile(scriptDir,'REPLANNING_SYSTEM_R5R4_DIAG.m');
r14Path=fullfile(scriptDir,'REPLANNING_SYSTEM_R14R2_DIAG.m');
cppPath=fullfile(scriptDir,'af7r66_observer_mex.cpp');
assert(all(cellfun(@isfile,{srcPath,runnerPath,r8Path,r5Path,r14Path,cppPath})), ...
    '[R14] required authority/package file missing.');
sourceSHA=sha256_file_r14(srcPath); runnerSHA=sha256_file_r14(runnerPath);
r8SHA=sha256_file_r14(r8Path); r5SHA=sha256_file_r14(r5Path);
r14SHA=sha256_file_r14(r14Path); cppSHA=sha256_file_r14(cppPath);
assert(strcmpi(sourceSHA,EXPECTED_SOURCE_SHA),'[R14] production source SHA mismatch.');
assert(strcmpi(runnerSHA,EXPECTED_RUNNER_SHA),'[R14] runner SHA mismatch.');
assert(strcmpi(r8SHA,EXPECTED_R8_SHA),'[R14] R8-R2 SHA mismatch.');
assert(strcmpi(r5SHA,EXPECTED_R5_SHA),'[R14] R5-R4 SHA mismatch.');
assert(strcmpi(r14SHA,EXPECTED_R14_SHA),'[R14] R14 diagnostic class SHA mismatch.');
assert(strcmpi(cppSHA,EXPECTED_CPP_SHA),'[R14] observer MEX source SHA mismatch.');
runnerText=fileread(runnerPath); sem=assert_runner_semantics_r14(runnerText);

fprintf('====================================================================================================\n');
fprintf(' AF7R58ZZY-R14-R2 | AOT/MEX OBSERVER SHADOW + FULL RUNTIME AUDIT\n');
fprintf('====================================================================================================\n');
fprintf('[AUTHORITY] shadowOnly=1 | productionModified=0 | addedWarmup=0 | phaseShift=0\n');
fprintf(' source SHA = %s\n runner SHA = %s\n R14 SHA    = %s\n MEX src    = %s\n', ...
    sourceSHA,runnerSHA,r14SHA,cppSHA);

root=fileparts(srcPath); outDir=tempname(root); mkdir(outDir);
reportPath=fullfile(outDir,'AF7R58ZZY_R14_R2_REPORT.txt');
outPath=fullfile(outDir,'AF7R58ZZY_R14_R2_OUT.mat');
rawPath=fullfile(outDir,'AF7R58ZZY_R14_R2_RAW_CAPTURE.mat');
mexDir=fullfile(outDir,'mex'); mkdir(mexDir);

Base=empty_run_r14(); Opt=empty_run_r14();
Build=struct('closed',false,'strict',false,'message','','mex_dir',mexDir);
Exact=struct('closed',false,'selected_mode',NaN,'mode0',empty_mode_r14(0),'mode1',empty_mode_r14(1));
Post=struct('closed',false,'all_exact',false,'count',0);
BM=struct(); OM=struct(); BR=struct(); OR=struct();
runError=struct('identifier','','message',''); diagError=struct('identifier','','message','');

try
    fprintf('\n[A. BASELINE R8-R2 FLIGHT]\n');
    Base=run_flight_r14(root,scriptDir,runnerText,'REPLANNING_SYSTEM_R8R2_DIAG','',NaN);
    assert(Base.closed,'[R14] baseline flight did not close.');

    fprintf('\n[B. BUILD AOT/MEX OBSERVER KERNEL]\n');
    Build=compile_observer_mex_r14(cppPath,mexDir);
    assert(Build.closed,'[R14] observer MEX compilation failed: %s',Build.message);
    addpath(mexDir,'-begin'); rehash;

    fprintf('\n[C. PRE-FLIGHT BITWISE GATE ON BASELINE LIVE SNAPSHOTS]\n');
    Exact.mode0=safe_test_mex_mode_r14(Base.obj,0);
    Exact.mode1=safe_test_mex_mode_r14(Base.obj,1);
    if Exact.mode0.all_exact
        Exact.selected_mode=0;
    elseif Exact.mode1.all_exact
        Exact.selected_mode=1;
    else
        Exact.selected_mode=NaN;
    end
    Exact.closed=isfinite(Exact.selected_mode);
    fprintf(' mode0 exact=%d | oneMedian=%.6f ms | contractMedian=%.6f ms\n', ...
        Exact.mode0.all_exact,Exact.mode0.one_hot_median_ms,Exact.mode0.contract_hot_median_ms);
    fprintf(' mode1 exact=%d | oneMedian=%.6f ms | contractMedian=%.6f ms\n', ...
        Exact.mode1.all_exact,Exact.mode1.one_hot_median_ms,Exact.mode1.contract_hot_median_ms);
    assert(Exact.closed,'[R14] neither compiled observer mode is bitwise exact.');

    % Exactness testing above must not act as an extra runtime warmup. Unload
    % the MEX before the optimized mission. The next load/call therefore occurs
    % only through the EXISTING observer preflight event sequence.
    clear af7r66_observer_mex;
    rehash;

    fprintf('\n[D. OPTIMIZED R14 SHADOW FLIGHT | MEX mode=%d]\n',round(Exact.selected_mode));
    Opt=run_flight_r14(root,scriptDir,runnerText,'REPLANNING_SYSTEM_R14R2_DIAG',mexDir,Exact.selected_mode);
    assert(Opt.closed,'[R14] optimized flight did not close.');

    fprintf('\n[E. POST-RUN BITWISE REPLAY + FULL RUNTIME]\n');
    Post=post_replay_r14(Base.obj,Opt.obj);
    assert(Post.all_exact,'[R14] optimized live observer output is not bitwise exact to parent replay.');
    BM=observer_metrics_r14(Base.obj); OM=observer_metrics_r14(Opt.obj);
    BR=runtime_metrics_r14(Base.runtime_profile); OR=runtime_metrics_r14(Opt.runtime_profile);
catch ME
    if isempty(runError.message) && (~Base.closed || ~Opt.closed)
        runError.identifier=ME.identifier; runError.message=ME.message;
    else
        diagError.identifier=ME.identifier; diagError.message=ME.message;
    end
end

connected=Opt.closed && isa(Opt.obj,'REPLANNING_SYSTEM_R14R2_DIAG') && ...
    double(Opt.obj.af7r66_one_mex_calls)>0 && double(Opt.obj.af7r66_contract_mex_calls)>0 && ...
    double(Opt.obj.af7r66_fallback_calls)==0;
observerRepairClosed=false; observerPeakRatio=NaN; earlyOnline25Closed=false;
controlPeriod25msClosed=false; performanceClosed=false;
if Base.closed && Opt.closed && isfield(BM,'peak_total_ms') && isfield(OM,'peak_total_ms')
    observerPeakRatio=OM.peak_total_ms/max(realmin,BM.peak_total_ms);
    observerRepairClosed=Exact.closed && Post.all_exact && connected && ...
        OM.peak_total_ms<=5.0 && observerPeakRatio<=0.35;
    earlyOnline25Closed=OR.early_online_peak_ms<=25.0;
    controlPeriod25msClosed=OR.flight_online_peak_ms<=25.0;
    performanceClosed=controlPeriod25msClosed;
end

diagnosticClosed=Base.closed && Build.closed && Exact.closed && Opt.closed && ...
    Post.all_exact && connected && isempty(runError.message) && isempty(diagError.message);
if diagnosticClosed && observerRepairClosed && performanceClosed
    status='OBSERVER_AOT_R14_R2_COMPLETE__GLOBAL_25MS_CLOSED';
elseif diagnosticClosed && observerRepairClosed
    status='OBSERVER_AOT_R14_R2_COMPLETE__OBSERVER_CLOSED__GLOBAL_PERFORMANCE_OPEN';
elseif diagnosticClosed
    status='OBSERVER_AOT_R14_R2_COMPLETE__OBSERVER_REPAIR_OPEN';
else
    status='OBSERVER_AOT_R14_R2_INCOMPLETE_FAIL_CLOSED';
end

Authority=struct('source_sha',sourceSHA,'runner_sha',runnerSHA,'r8_sha',r8SHA, ...
    'r5_sha',r5SHA,'r14_sha',r14SHA,'cpp_sha',cppSHA,'runner_semantics',sem);
Contract=struct('production_modified',false,'shadow_only',true,'added_warmup',0, ...
    'new_arming_work',0,'new_takeoff_work',0,'new_landing_work',0, ...
    'trajectory_math_changed',false,'certificate_acceptance_changed',false, ...
    'observer_theorem_changed',false,'commit_policy_changed',false, ...
    'full_iteration_is_simulation_context_only',true);
RAW=struct('status',status,'authority',Authority,'contract',Contract,'build',Build, ...
    'exactness',Exact,'post_replay',Post,'baseline_observer',BM,'optimized_observer',OM, ...
    'baseline_runtime',BR,'optimized_runtime',OR,'baseline_profile',Base.runtime_profile, ...
    'optimized_profile',Opt.runtime_profile,'observer_repair_closed',observerRepairClosed, ...
    'early_online_25ms_closed',earlyOnline25Closed,'control_period_25ms_closed',controlPeriod25msClosed, ...
    'performance_closed',performanceClosed,'run_error',runError,'diag_error',diagError);
save(rawPath,'RAW','-v7.3');
OUT=struct('status',status,'diagnostic_closed',diagnosticClosed,'observer_repair_closed',observerRepairClosed, ...
    'performance_closed',performanceClosed,'control_period_25ms_closed',controlPeriod25msClosed, ...
    'early_online_25ms_closed',earlyOnline25Closed,'observer_peak_ratio',observerPeakRatio, ...
    'selected_mex_mode',Exact.selected_mode,'baseline_observer',BM,'optimized_observer',OM, ...
    'baseline_runtime',BR,'optimized_runtime',OR,'production_modified',false, ...
    'added_warmup',0,'new_arming_work',0,'new_takeoff_work',0,'new_landing_work',0, ...
    'raw_capture_path',rawPath,'report_path',reportPath,'run_error',runError,'diag_error',diagError);
save(outPath,'OUT','-v7.3');
write_report_r14(reportPath,OUT,RAW,Base,Opt,Exact,Post,Authority,Contract);

fprintf('\n[F. RESULT]\n');
fprintf(' status                    = %s\n',status);
fprintf(' diagnostic closed         = %d\n',diagnosticClosed);
fprintf(' observer repair closed    = %d\n',observerRepairClosed);
fprintf(' early online <=25 ms      = %d\n',earlyOnline25Closed);
fprintf(' global online <=25 ms     = %d\n',controlPeriod25msClosed);
if isfield(BM,'peak_total_ms') && isfield(OM,'peak_total_ms')
    fprintf(' observer peak [ms]        = %.4f -> %.4f\n',BM.peak_total_ms,OM.peak_total_ms);
end
if isfield(BR,'early_online_peak_ms') && isfield(OR,'early_online_peak_ms')
    fprintf(' early online peak [ms]    = %.4f -> %.4f\n',BR.early_online_peak_ms,OR.early_online_peak_ms);
    fprintf(' replan online peak [ms]   = %.4f -> %.4f\n',BR.replan_online_peak_ms,OR.replan_online_peak_ms);
end
fprintf('RETURN THESE THREE FILES:\n %s\n %s\n %s\n',reportPath,outPath,rawPath);
if ~diagnosticClosed
    error('[AF7R58ZZY-R14-R2] AOT observer shadow audit did not close; production install is forbidden.');
end
end

function R=empty_run_r14()
R=struct('closed',false,'obj',[],'runtime_profile',struct(), ...
    'error',struct('identifier','','message',''));
end

function M=empty_mode_r14(mode)
M=struct('mode',mode,'tested',0,'one_exact',false,'contract_exact',false, ...
    'all_exact',false,'one_hot_median_ms',NaN,'contract_hot_median_ms',NaN, ...
    'pair_hot_median_ms',NaN,'message','');
end

function R=run_flight_r14(root,scriptDir,runnerText,className,mexDir,mexMode)
R=empty_run_r14();
try
    evalin('base','clearvars');
catch
end
rehash;
oldCtor='replanner_instance = REPLANNING_SYSTEM(agent,nominal_ref,replan_opts);';
assert(count_token_r14(runnerText,oldCtor)==1,'[R14] runner constructor anchor !=1.');
newCtor=sprintf('replanner_instance = %s(agent,nominal_ref,replan_opts);',className);
if strcmp(className,'REPLANNING_SYSTEM_R14R2_DIAG')
    assert(isfinite(mexMode)&&ismember(round(mexMode),[0 1]),'[R14] invalid selected MEX mode.');
    newCtor=sprintf('%s%c%s',newCtor,10, ...
        sprintf('replanner_instance.af7r66_mex_mode = %d; replanner_instance.af7r66_use_mex = true;',round(mexMode)));
end
txt=strrep(runnerText,oldCtor,newCtor);
pre=sprintf('addpath(''%s'',''-begin'');',strrep(scriptDir,'''',''''''));
if ~isempty(mexDir)
    pre=sprintf('%saddpath(''%s'',''-begin'');',pre,strrep(mexDir,'''',''''''));
end
txt=sprintf('%srehash;%c%s',pre,10,txt);
tmp=fullfile(root,sprintf('AF7R58ZZY_R14_R2_%s_TEMP_RUNNER.m',className));
write_utf8_r14(tmp,txt); c=onCleanup(@()delete_if_exists_r14(tmp)); %#ok<NASGU>
try
    cmd=sprintf('run(''%s'')',strrep(tmp,'''','''''')); evalin('base',cmd);
    obj=evalin('base','replanner_instance'); P=evalin('base','runtime_profile');
    assert(isa(obj,className)&&isa(obj,'REPLANNING_SYSTEM'),'[R14] diagnostic class contract lost.');
    assert(isstruct(P)&&isfield(P,'k')&&P.k>0,'[R14] runtime_profile unavailable.');
    R.obj=obj; R.runtime_profile=P; R.closed=true;
catch ME
    R.error.identifier=ME.identifier; R.error.message=ME.message;
end
end

function B=compile_observer_mex_r14(cppPath,mexDir)
B=struct('closed',false,'strict',false,'message','','mex_dir',mexDir);
try
    if ispc
        try
            mex('-R2018a','COMPFLAGS=$COMPFLAGS /fp:strict',cppPath,'-outdir',mexDir);
            B.strict=true;
        catch MEstrict
            mex('-R2018a',cppPath,'-outdir',mexDir);
            B.message=['strict compile fallback: ' MEstrict.message];
        end
    else
        try
            mex('-R2018a','CXXFLAGS=$CXXFLAGS -ffp-contract=off -fno-fast-math',cppPath,'-outdir',mexDir);
            B.strict=true;
        catch MEstrict
            mex('-R2018a',cppPath,'-outdir',mexDir);
            B.message=['strict compile fallback: ' MEstrict.message];
        end
    end
    addpath(mexDir,'-begin'); rehash;
    B.closed=(exist('af7r66_observer_mex','file')==3);
    if ~B.closed, B.message='compiled MEX not visible on MATLAB path'; end
catch ME
    B.message=[ME.identifier ' | ' ME.message];
end
end

function M=safe_test_mex_mode_r14(obj,mode)
M=empty_mode_r14(mode);
try
    M=test_mex_mode_r14(obj,mode);
catch ME
    M.message=[ME.identifier ' | ' ME.message];
end
end

function M=test_mex_mode_r14(obj,mode)
M=empty_mode_r14(mode);
n=double(obj.af7r60_obs_snapshot_count); assert(n>=1,'[R14] baseline observer snapshots unavailable.');
K0=obj.observer_static_setup_cache; assert(obj.observer_static_setup_cache_valid,'[R14] baseline observer static cache invalid.');
oneOK=true; conOK=true;
for k=1:n
    S=obj.af7r60_obs_snapshots{k};
    Fm=af7r66_observer_mex(1,double(mode),S.h,S.uPrev,S.I,K0);
    Km=af7r66_observer_mex(2,double(mode),S.C0,S.C1,Fm,S.xhat);
    oneOK=oneOK&&bitwise_equal_r14(Fm,S.F);
    conOK=conOK&&bitwise_equal_r14(Km,S.K);
end
M.tested=n; M.one_exact=oneOK; M.contract_exact=conOK; M.all_exact=oneOK&&conOK;
% Hot benchmark is diagnostic only. MEX is explicitly unloaded before the
% optimized flight, so these calls cannot act as runtime warmup.
S=obj.af7r60_obs_snapshots{1}; nrep=40; a=zeros(1,nrep); b=zeros(1,nrep); pair=zeros(1,nrep);
for q=1:nrep
    Tpair=tic; T=tic; Fm=af7r66_observer_mex(1,double(mode),S.h,S.uPrev,S.I,K0); a(q)=1e3*toc(T);
    T=tic; af7r66_observer_mex(2,double(mode),S.C0,S.C1,Fm,S.xhat); b(q)=1e3*toc(T); pair(q)=1e3*toc(Tpair);
end
M.one_hot_median_ms=median(a); M.contract_hot_median_ms=median(b); M.pair_hot_median_ms=median(pair);
end

function P=post_replay_r14(parentObj,optObj)
P=struct('closed',false,'all_exact',false,'count',0);
n=double(optObj.af7r60_obs_snapshot_count); assert(n>=1,'[R14] optimized live snapshots unavailable.');
ok=true;
for k=1:n
    S=optObj.af7r60_obs_snapshots{k};
    Fp=parentObj.observer_one_step_support(S.h,S.uPrev,S.I);
    Kp=parentObj.observer_contract_two_events(S.C0,S.C1,Fp,S.xhat);
    ok=ok&&bitwise_equal_r14(Fp,S.F)&&bitwise_equal_r14(Kp,S.K);
end
P.count=n; P.all_exact=ok; P.closed=ok;
end

function M=observer_metrics_r14(obj)
n=double(obj.af7r58zzy_r6_obs_count); R=obj.af7r58zzy_r6_obs_ring(1:n,:);
R=R(isfinite(R(:,1))&R(:,1)>=4.95-1e-10&R(:,1)<=5.20+1e-10,:);
assert(~isempty(R),'[R14] live observer timing rows unavailable.');
M=struct(); M.row_count=size(R,1); M.peak_total_ms=max(R(:,2));
z=R(:,9); z=z(isfinite(z)); M.peak_one_ms=max(z);
z=R(:,14); z=z(isfinite(z)); M.peak_contract_ms=max(z);
[~,i]=max(R(:,2)); M.peak_t=R(i,1); M.rows=R;
end

function M=runtime_metrics_r14(P)
K=double(P.k); t=double(P.t(1:K)); ph=P.phase(1:K);
on=1e3*double(P.online_before_plant_s(1:K)); full=1e3*double(P.full_iteration_s(1:K));
f=(ph=='f'); early=f&t>=4.95-1e-10&t<=5.20+1e-10; repl=f&t>=9.80-1e-10&t<=10.10+1e-10;
assert(any(f)&&any(early)&&any(repl),'[R14] runtime_profile target windows unavailable.');
M=struct(); M.flight_online_peak_ms=max(on(f)); M.flight_full_peak_ms=max(full(f));
M.early_online_peak_ms=max(on(early)); M.early_full_peak_ms=max(full(early));
M.replan_online_peak_ms=max(on(repl)); M.replan_full_peak_ms=max(full(repl));
M.flight_online_over25_count=nnz(on(f)>25.0); M.flight_full_over25_count=nnz(full(f)>25.0);
M.early_online_over25_count=nnz(on(early)>25.0); M.replan_online_over25_count=nnz(on(repl)>25.0);
M.flight_online_median_ms=median(on(f)); M.flight_full_median_ms=median(full(f));
[~,i]=max(on(f)); tf=t(f); M.flight_online_peak_t=tf(i);
[~,i]=max(on(early)); te=t(early); M.early_online_peak_t=te(i);
[~,i]=max(on(repl)); tr=t(repl); M.replan_online_peak_t=tr(i);
M.top_online=top_rows_r14(t(f),on(f),full(f),8);
end

function T=top_rows_r14(t,on,full,n)
[~,ix]=sort(on,'descend'); ix=ix(1:min(n,numel(ix))); T=[t(ix(:)).' ; on(ix(:)).' ; full(ix(:)).'].';
end

function tf=bitwise_equal_r14(a,b)
if ~strcmp(class(a),class(b)) || ~isequal(size(a),size(b)), tf=false; return; end
if isa(a,'double')
    tf=isequal(typecast(a(:),'uint64'),typecast(b(:),'uint64')); return;
elseif islogical(a) || ischar(a) || isstring(a) || isnumeric(a)
    tf=isequaln(a,b); return;
elseif isstruct(a)
    fa=fieldnames(a); fb=fieldnames(b); if ~isequal(fa,fb), tf=false; return; end
    tf=true;
    for j=1:numel(a)
        for k=1:numel(fa)
            if ~bitwise_equal_r14(a(j).(fa{k}),b(j).(fa{k})), tf=false; return; end
        end
    end
    return;
elseif iscell(a)
    tf=true; for k=1:numel(a), if ~bitwise_equal_r14(a{k},b{k}), tf=false; return; end, end; return;
else
    tf=isequaln(a,b);
end
end

function write_report_r14(path,OUT,RAW,Base,Opt,Exact,Post,A,C)
fid=fopen(path,'w'); assert(fid>=0,'[R14] cannot create report.'); c=onCleanup(@()safe_fclose_r14(fid)); %#ok<NASGU>
say_r14(fid,'status=%s',OUT.status); say_r14(fid,'diagnosticClosed=%d',OUT.diagnostic_closed);
say_r14(fid,'observerRepairClosed=%d',OUT.observer_repair_closed); say_r14(fid,'performanceClosed=%d',OUT.performance_closed);
say_r14(fid,'controlPeriod25msClosed=%d',OUT.control_period_25ms_closed); say_r14(fid,'earlyOnline25msClosed=%d',OUT.early_online_25ms_closed);
say_r14(fid,'productionModified=0'); say_r14(fid,'shadowOnly=1'); say_r14(fid,'addedWarmup=0');
say_r14(fid,'newArmingWork=0'); say_r14(fid,'newTakeoffWork=0'); say_r14(fid,'newLandingWork=0');
say_r14(fid,'sourceSHA=%s',A.source_sha); say_r14(fid,'runnerSHA=%s',A.runner_sha); say_r14(fid,'r14SHA=%s',A.r14_sha); say_r14(fid,'cppSHA=%s',A.cpp_sha);
say_r14(fid,'runnerSemantics=%d/%d',A.runner_semantics.pass_count,A.runner_semantics.total_count);
say_r14(fid,'raw=%s',OUT.raw_capture_path); say_r14(fid,'out=%s',strrep(OUT.raw_capture_path,'RAW_CAPTURE','OUT'));

say_r14(fid,''); say_r14(fid,'[MEX_BUILD_AND_BITWISE_GATE]');
say_r14(fid,'buildClosed=%d',RAW.build.closed); say_r14(fid,'strictCompile=%d',RAW.build.strict); say_r14(fid,'buildMessage=%s',RAW.build.message);
say_r14(fid,'selectedMode=%.0f',Exact.selected_mode);
say_r14(fid,'mode0Exact=%d',Exact.mode0.all_exact); say_r14(fid,'mode0Tested=%d',Exact.mode0.tested); say_r14(fid,'mode0Message=%s',Exact.mode0.message); say_r14(fid,'mode0OneHotMedianMs=%.9g',Exact.mode0.one_hot_median_ms); say_r14(fid,'mode0ContractHotMedianMs=%.9g',Exact.mode0.contract_hot_median_ms);
say_r14(fid,'mode1Exact=%d',Exact.mode1.all_exact); say_r14(fid,'mode1Tested=%d',Exact.mode1.tested); say_r14(fid,'mode1Message=%s',Exact.mode1.message); say_r14(fid,'mode1OneHotMedianMs=%.9g',Exact.mode1.one_hot_median_ms); say_r14(fid,'mode1ContractHotMedianMs=%.9g',Exact.mode1.contract_hot_median_ms);
say_r14(fid,'postRunBitwiseExact=%d',Post.all_exact); say_r14(fid,'postRunReplayCount=%d',Post.count);

if Base.closed && Opt.closed && isfield(OUT.baseline_observer,'peak_total_ms')
    B=OUT.baseline_observer; O=OUT.optimized_observer;
    say_r14(fid,''); say_r14(fid,'[OBSERVER_LIVE_PERFORMANCE]');
    say_r14(fid,'columns=metric baseline_ms optimized_ms ratio');
    say_r14(fid,'observerPeak %.9g %.9g %.9g',B.peak_total_ms,O.peak_total_ms,O.peak_total_ms/max(realmin,B.peak_total_ms));
    say_r14(fid,'oneStepPeak %.9g %.9g %.9g',B.peak_one_ms,O.peak_one_ms,O.peak_one_ms/max(realmin,B.peak_one_ms));
    say_r14(fid,'contractionPeak %.9g %.9g %.9g',B.peak_contract_ms,O.peak_contract_ms,O.peak_contract_ms/max(realmin,B.peak_contract_ms));
    say_r14(fid,'baselinePeakT=%.9g',B.peak_t); say_r14(fid,'optimizedPeakT=%.9g',O.peak_t);
    say_r14(fid,'mexOneCalls=%d',round(Opt.obj.af7r66_one_mex_calls)); say_r14(fid,'mexContractCalls=%d',round(Opt.obj.af7r66_contract_mex_calls)); say_r14(fid,'fallbackCalls=%d',round(Opt.obj.af7r66_fallback_calls));
end

if Base.closed && Opt.closed && isfield(OUT.baseline_runtime,'flight_online_peak_ms')
    B=OUT.baseline_runtime; O=OUT.optimized_runtime;
    say_r14(fid,''); say_r14(fid,'[FULL_RUNTIME_PROFILE]');
    say_r14(fid,'deadlineAuthority=ONLINE_BEFORE_PLANT');
    say_r14(fid,'fullIterationRole=SIMULATION_PLANT_PLUS_LOGGER_CONTEXT_ONLY');
    say_r14(fid,'columns=metric baseline_ms optimized_ms ratio');
    emit_metric_r14(fid,'flightOnlinePeak',B.flight_online_peak_ms,O.flight_online_peak_ms);
    emit_metric_r14(fid,'earlyOnlinePeak',B.early_online_peak_ms,O.early_online_peak_ms);
    emit_metric_r14(fid,'replanOnlinePeak',B.replan_online_peak_ms,O.replan_online_peak_ms);
    emit_metric_r14(fid,'flightFullIterationPeak',B.flight_full_peak_ms,O.flight_full_peak_ms);
    emit_metric_r14(fid,'earlyFullIterationPeak',B.early_full_peak_ms,O.early_full_peak_ms);
    emit_metric_r14(fid,'replanFullIterationPeak',B.replan_full_peak_ms,O.replan_full_peak_ms);
    say_r14(fid,'baselineOnlineOver25Count=%d',B.flight_online_over25_count); say_r14(fid,'optimizedOnlineOver25Count=%d',O.flight_online_over25_count);
    say_r14(fid,'baselineFullIterationOver25Count=%d',B.flight_full_over25_count); say_r14(fid,'optimizedFullIterationOver25Count=%d',O.flight_full_over25_count);
    say_r14(fid,'optimizedEarlyOnlineOver25Count=%d',O.early_online_over25_count); say_r14(fid,'optimizedReplanOnlineOver25Count=%d',O.replan_online_over25_count);
    say_r14(fid,''); say_r14(fid,'[OPTIMIZED_TOP_ONLINE_SPIKES]'); say_r14(fid,'columns=t onlineBeforePlantMs fullIterationMs');
    for k=1:size(O.top_online,1), say_r14(fid,'row=%.9g %.9g %.9g',O.top_online(k,1),O.top_online(k,2),O.top_online(k,3)); end
end

say_r14(fid,''); say_r14(fid,'[CONTRACT]');
say_r14(fid,'trajectoryMathChanged=%d',C.trajectory_math_changed); say_r14(fid,'certificateAcceptanceChanged=%d',C.certificate_acceptance_changed); say_r14(fid,'observerTheoremChanged=%d',C.observer_theorem_changed); say_r14(fid,'commitPolicyChanged=%d',C.commit_policy_changed);
say_r14(fid,'workShiftedToArmingTakeoffLanding=0'); say_r14(fid,'productionInstallPerformed=0'); say_r14(fid,'sameSessionSequentialComparison=1'); say_r14(fid,'fullIterationIsNotControlDeadline=1');
if ~isempty(OUT.run_error.message), say_r14(fid,'runError=%s | %s',OUT.run_error.identifier,OUT.run_error.message); end
if ~isempty(OUT.diag_error.message), say_r14(fid,'diagError=%s | %s',OUT.diag_error.identifier,OUT.diag_error.message); end
fclose(fid); clear c;
end

function emit_metric_r14(fid,name,a,b), say_r14(fid,'%s %.9g %.9g %.9g',name,a,b,b/max(realmin,a)); end

function S=assert_runner_semantics_r14(txt)
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
    'flight_est_allocation','agent.cha_allocation.f.estimator = ["ekf","loadstate"];'; ...
    'runtime_online','runtime_profile.online_before_plant_s'; ...
    'runtime_full','runtime_profile.full_iteration_s'};
pass=false(size(checks,1),1); for k=1:size(checks,1), pass(k)=contains(txt,checks{k,2}); end
if ~pass(22), pass(22)=contains(txt,'agent.set_cha_allocation_for_all("estimator",["ekf","loadstate"]);'); end
w0=strfind(txt,'while time.t <= time.te'); w1=strfind(txt,'time.t = time.t + dt;'); orderOK=false;
if numel(w0)==1 && ~isempty(w1)
    q=w1(find(w1>w0,1,'first')); if ~isempty(q)
        blk=txt(w0:q); tok={ ...
            'do_prop_local(time,cha,cha,logger,env,agent,"sensor",1,true);', ...
            'do_prop_local(time,cha,cha,logger,env,agent,"estimator",1,true);', ...
            'do_prop_local(time,cha,cha,logger,env,agent,"reference",1,true);', ...
            'do_prop_local(time,cha,cha,logger,env,agent,"controller",1,true);', ...
            'do_prop_local(time,cha,cha,logger,env,agent,"input_transform",1,true);'};
        pos=nan(1,numel(tok)); for j=1:numel(tok), z=strfind(blk,tok{j}); if numel(z)==1,pos(j)=z;end,end %#ok<STREMP>
        orderOK=all(isfinite(pos))&&all(diff(pos)>0);
    end
end
S=struct('closed',all([pass;orderOK]),'pass_count',nnz([pass;orderOK]),'total_count',numel([pass;orderOK]));
assert(S.closed,'[R14] current runner target semantics changed.');
end

function n=count_token_r14(txt,token), n=numel(strfind(txt,token)); end %#ok<STREMP>
function h=sha256_file_r14(path)
fid=fopen(path,'rb'); assert(fid>=0,'[R14] cannot open file for SHA: %s',path); c=onCleanup(@()fclose(fid)); %#ok<NASGU>
b=fread(fid,Inf,'*uint8'); md=javaMethod('getInstance','java.security.MessageDigest','SHA-256'); md.update(typecast(b,'int8')); d=md.digest(); u=mod(double(d),256); h=lower(reshape(dec2hex(u,2).',1,[]));
end
function write_utf8_r14(path,txt), fid=fopen(path,'w','n','UTF-8'); assert(fid>=0); c=onCleanup(@()fclose(fid)); fprintf(fid,'%s',txt); end %#ok<NASGU>
function delete_if_exists_r14(path), try, if isfile(path), delete(path); end, catch, end, end
function say_r14(fid,fmt,varargin), fprintf(fid,'%s%c',sprintf(fmt,varargin{:}),10); end
function safe_fclose_r14(fid), try, if fid>0, fclose(fid); end, catch, end, end
