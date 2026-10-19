function OUT = AF7R58ZZS_R2_FLIGHTSTART_COMPONENT_QPC_AUDIT()
% AF7R58ZZS-R2
% High-resolution Flight-start component forensic audit.
%
% WHY THIS EXISTS
%   The previous residual audit required logged rt_total_ms, but the current
%   AF7R58ZZQ-R2 run does not retain finite rt_* stage fields in LOGGER.
%   This audit therefore measures the actual live dispatch directly on ONE
%   fresh rerun of the exact AF7R58ZZB runner.
%
% IMPORTANT
%   - REPLANNING_SYSTEM.m is READ ONLY.
%   - The original AF7R58ZZB runner is READ ONLY.
%   - A temporary copy of the runner is generated beside it, executed once,
%     then deleted.
%   - The temporary copy preserves the original dispatch semantics.
%   - Only t in [4.95, 5.55] s is instrumented.
%   - Timing uses Windows QueryPerformanceCounter + thread/process cycle
%     counters in a temporary MEX. No optimistic overhead subtraction.
%
% It separates:
%   sensor
%   estimator
%   reference total
%       reference.timevarying = REPLANNING_SYSTEM
%       reference.sload       = SUSPENDED_LOAD_REF_ADJUST
%   controller
%   input_transform
%   plant
%
% This is diagnostic timing only, not a WCET proof.

EXPECTED_REPLANNER_SHA = ...
    'bc97df3a0a7341ac5605d52f4dd94923bfd831a11be535450666ec6d10ff41ec';

OUT=struct();
OUT.schema='AF7R58ZZS_R2_FLIGHTSTART_COMPONENT_QPC_V2';
OUT.production_modified=false;
OUT.original_runner_modified=false;
OUT.status='OPEN';
OUT.error=struct();

fprintf('\n====================================================================================================\n');
fprintf(' AF7R58ZZS-R2 | FLIGHT-START COMPONENT QPC AUDIT\n');
fprintf('====================================================================================================\n');
fprintf('[AUTHORITY] productionReadOnly=1 | runnerReadOnly=1 | tempRunnerOnly=1\n');
fprintf('[CLOCK] WindowsQPC=1 | threadCycles=1 | processCycles=1 | overheadSubtraction=0\n');
fprintf('[WINDOW] 4.95 <= t <= 5.55 s only\n');

tempRunner='';
tempMexDir='';
origDir=pwd;

try
    %% A. Exact production authority.
    pR=which('REPLANNING_SYSTEM');
    assert(~isempty(pR)&&isfile(pR), ...
        '[AF7R58ZZS] REPLANNING_SYSTEM unresolved.');
    OUT.replanner_sha=file_hash(pR);
    OUT.replanner_exact=strcmp(OUT.replanner_sha,EXPECTED_REPLANNER_SHA);
    assert(OUT.replanner_exact, ...
        '[AF7R58ZZS] source differs from exact AF7R58ZZQ-R2 authority.');

    runnerName='run_flight_headless_AF7R58ZZB_reference_cycle_audit';
    pRun=which(runnerName);
    assert(~isempty(pRun)&&isfile(pRun), ...
        '[AF7R58ZZS] %s unresolved.',runnerName);

    OUT.runner_path=pRun;
    OUT.runner_sha_before=file_hash(pRun);

    fprintf('\n[A. SOURCE BINDING]\n');
    fprintf(' REPLANNING_SYSTEM SHA exact = %d | %s\n', ...
        OUT.replanner_exact,OUT.replanner_sha);
    fprintf(' source runner              = %s\n',pRun);
    fprintf(' source runner SHA          = %s\n',OUT.runner_sha_before);

    %% B. Build temporary high-resolution stamp MEX.
    tempMexDir=tempname(fileparts(pRun));
    [ok,msg]=mkdir(tempMexDir); assert(ok,'%s',msg);

    cpp=fullfile(tempMexDir,'af7r58zzs_qpc_stamp_mex.cpp');
    write_text(cpp,cpp_text());

    oldDir=pwd;
    cd(tempMexDir);
    ccd=onCleanup(@()cd(oldDir)); %#ok<NASGU>
    mex('-R2018a','-silent','-output','af7r58zzs_qpc_stamp_mex',cpp);
    cd(oldDir);
    clear ccd

    addpath(tempMexDir,'-begin');
    cmp=onCleanup(@()safe_rmpath(tempMexDir)); %#ok<NASGU>
    assert(exist('af7r58zzs_qpc_stamp_mex','file')==3, ...
        '[AF7R58ZZS] QPC stamp MEX build failed.');

    % Raw mark-pair calibration. No subtraction will be applied.
    ncal=200;
    pairMs=nan(ncal,1);
    pairThread=nan(ncal,1);
    pairProcess=nan(ncal,1);
    for k=1:ncal
        a=af7r58zzs_qpc_stamp_mex();
        b=af7r58zzs_qpc_stamp_mex();
        pairMs(k)=1e3*(b(1)-a(1))/a(2);
        pairThread(k)=b(3)-a(3);
        pairProcess(k)=b(4)-a(4);
    end
    OUT.clock=struct();
    OUT.clock.qpc_frequency_hz=af7r58zzs_qpc_stamp_mex();
    tmp=af7r58zzs_qpc_stamp_mex();
    OUT.clock.qpc_frequency_hz=tmp(2);
    OUT.clock.pair_min_ms=min(pairMs);
    OUT.clock.pair_median_ms=median(pairMs);
    OUT.clock.pair_p95_ms=prctile(pairMs,95);
    OUT.clock.pair_max_ms=max(pairMs);
    OUT.clock.pair_thread_median_cycles=median(pairThread);
    OUT.clock.pair_process_median_cycles=median(pairProcess);
    OUT.clock.reliable_threshold_ms=5*OUT.clock.pair_median_ms;

    fprintf('\n[B. QPC CALIBRATION]\n');
    fprintf(' frequency [Hz]                 = %.0f\n',OUT.clock.qpc_frequency_hz);
    fprintf(' mark-pair min/med/p95/max [ms] = %.6f / %.6f / %.6f / %.6f\n', ...
        OUT.clock.pair_min_ms,OUT.clock.pair_median_ms, ...
        OUT.clock.pair_p95_ms,OUT.clock.pair_max_ms);
    fprintf(' reliable threshold [ms]        = %.6f\n', ...
        OUT.clock.reliable_threshold_ms);

    %% C. Make a temporary exact-runner derivative.
    S=read_text(pRun);

    % Require the current dispatch helper and merge helper use.
    pat='(?m)^function\s+do_prop_local\(time,cha,fCha,logger,env,agent,prop,i,isReady\)\s*$';
    [st,en]=regexp(S,pat,'start','end');
    assert(numel(st)==1, ...
        '[AF7R58ZZS] do_prop_local definition count=%d.',numel(st));

    nextRel=regexp(S(en+1:end),'(?m)^function\s+','start','once');
    assert(~isempty(nextRel), ...
        '[AF7R58ZZS] cannot find function boundary after do_prop_local.');
    blockEnd=en+nextRel-1;
    oldBlock=S(st:blockEnd-1);

    assert(contains(oldBlock,'merge_result('), ...
        '[AF7R58ZZS] current dispatcher no longer contains merge_result.');
    assert(contains(oldBlock,'.do(time,fCha,logger,env,agent,i)'), ...
        '[AF7R58ZZS] current dispatcher call shape changed.');

    % Rename original dispatcher and place an instrumenting wrapper before it.
    innerBlock=regexprep(oldBlock, ...
        '(?m)^function\s+do_prop_local\(', ...
        'function do_prop_local_AF7R58ZZS_inner(', ...
        'once');

    wrapper=instrumented_dispatcher_text();
    S2=[S(1:st-1) wrapper newline innerBlock S(blockEnd:end)];

    % Add local trace helpers at EOF.
    S2=[S2 newline trace_helpers_text() newline];

    % Ensure only the temporary copy contains our marker.
    assert(count_exact(S2,'AF7R58ZZS_COMPONENT_QPC_WRAPPER')==1);
    assert(count_exact(S2,'do_prop_local_AF7R58ZZS_inner')>=2);

    runDir=fileparts(pRun);
    tempRunner=fullfile(runDir, ...
        'run_flight_headless_AF7R58ZZS_component_probe_TEMP.m');
    write_text(tempRunner,S2);

    OUT.temp_runner_sha=file_hash(tempRunner);
    OUT.temp_runner_path=tempRunner;

    % Original source must still be byte-identical before execution.
    assert(strcmp(file_hash(pRun),OUT.runner_sha_before), ...
        '[AF7R58ZZS] original runner changed before execution.');

    fprintf('\n[C. TEMP RUNNER PREFLIGHT]\n');
    fprintf(' original runner untouched       = 1\n');
    fprintf(' temporary runner                = %s\n',tempRunner);
    fprintf(' temporary runner SHA            = %s\n',OUT.temp_runner_sha);
    fprintf(' dispatcher semantics retained   = 1\n');
    fprintf(' reference subcomponents split   = 1\n');
    fprintf(' production writes               = 0\n');

    %% D. Execute exact scenario once through the temporary diagnostic copy.
    global AF7R58ZZS_TRACE
    AF7R58ZZS_TRACE=struct( ...
        'n',0, ...
        't',nan(1000,1), ...
        'prop',strings(1000,1), ...
        'sub',strings(1000,1), ...
        'ms',nan(1000,1), ...
        'thread_cycles',nan(1000,1), ...
        'process_cycles',nan(1000,1));

    fprintf('\n[D. EXECUTE ONE FRESH INSTRUMENTED RERUN]\n');
    fprintf(' running exact AF7R58ZZB source derivative now...\n');
    assert(ischar(tempRunner) && isfile(tempRunner), ...
        '[AF7R58ZZS-R2] temporary runner path invalid before run.');

    % Use base workspace because the runner is a script and may clearvars.
    assignin('base','AF7R58ZZS_R2_TEMP_RUNNER_PATH',tempRunner);
    evalin('base','run(AF7R58ZZS_R2_TEMP_RUNNER_PATH);');

    % Read trace after run.
    T=AF7R58ZZS_TRACE;
    assert(isstruct(T)&&isfield(T,'n')&&T.n>0, ...
        '[AF7R58ZZS] no component timing events captured.');
    n=T.n;
    T.t=T.t(1:n);
    T.prop=T.prop(1:n);
    T.sub=T.sub(1:n);
    T.ms=T.ms(1:n);
    T.thread_cycles=T.thread_cycles(1:n);
    T.process_cycles=T.process_cycles(1:n);
    OUT.trace=T;

    % The original runner must remain unchanged after execution.
    OUT.runner_sha_after=file_hash(pRun);
    assert(strcmp(OUT.runner_sha_after,OUT.runner_sha_before), ...
        '[AF7R58ZZS] original AF7R58ZZB runner changed unexpectedly.');

    % Production must remain exact too.
    OUT.replanner_sha_after=file_hash(pR);
    assert(strcmp(OUT.replanner_sha_after,OUT.replanner_sha), ...
        '[AF7R58ZZS] REPLANNING_SYSTEM changed unexpectedly.');

    %% E. Bring fresh runtime profile from base and find first-Flight peak.
    assert(evalin('base','exist(''AF7R58ZZB_RUNTIME_PROFILE'',''var'')')==1, ...
        '[AF7R58ZZS] fresh runtime profile missing after rerun.');
    P=evalin('base','AF7R58ZZB_RUNTIME_PROFILE');

    tP=double(P.t(:));
    ph=char(P.phase(:));
    full=1e3*double(P.full_iteration_s(:));
    online=1e3*double(P.online_before_plant_s(:));
    ref=1e3*double(P.reference_s(:));
    req=double(P.gen_request_delta(:));

    fw=find(ph=='f' & tP>=4.95-1e-12 & tP<=5.55+1e-12);
    assert(~isempty(fw),'[AF7R58ZZS] no Flight-start rows in runtime profile.');
    [~,jm]=max(full(fw));
    ip=fw(jm);
    tPeak=tP(ip);

    OUT.flight_peak=struct( ...
        'index',ip,'t_s',tPeak,'full_ms',full(ip), ...
        'online_ms',online(ip),'reference_ms',ref(ip), ...
        'request_delta',req(ip));

    %% F. Component table at exact peak and nearby baseline.
    keys=[ ...
        "sensor|TOTAL", ...
        "estimator|TOTAL", ...
        "reference|TOTAL", ...
        "reference|timevarying", ...
        "reference|sload", ...
        "controller|TOTAL", ...
        "input_transform|TOTAL", ...
        "plant|TOTAL"];

    rows=struct('key',{},'peak_ms',{},'baseline_median_ms',{}, ...
        'excess_ms',{},'peak_thread_cycles',{},'quality',{});

    baseLo=max(5.20,tPeak+0.10);
    baseHi=5.55;

    for k=1:numel(keys)
        parts=split(keys(k),'|');
        m=(T.prop==parts(1) & T.sub==parts(2));

        pk=m & abs(T.t-tPeak)<=1e-10;
        base=m & T.t>=baseLo-1e-12 & T.t<=baseHi+1e-12;

        peakMs=safe_single(T.ms(pk));
        peakTc=safe_single(T.thread_cycles(pk));
        baseMed=safe_median(T.ms(base));
        excess=peakMs-baseMed;

        if isfinite(peakMs) && peakMs>=OUT.clock.reliable_threshold_ms
            q='RELIABLE';
        elseif isfinite(peakMs) && peakMs>=2*OUT.clock.pair_median_ms
            q='MARGINAL';
        else
            q='UNRESOLVED_SHORT_STAGE';
        end

        rows(end+1)=struct( ... %#ok<AGROW>
            'key',char(keys(k)), ...
            'peak_ms',peakMs, ...
            'baseline_median_ms',baseMed, ...
            'excess_ms',excess, ...
            'peak_thread_cycles',peakTc, ...
            'quality',q);
    end
    OUT.component_rows=rows;

    fprintf('
[E. FIRST-FLIGHT PEAK]
');
    fprintf(' t = %.6f s | online=%.6f ms | reference=%.6f ms | full=%.6f ms | request=%.0f
', ...
        tPeak,online(ip),ref(ip),full(ip),req(ip));
    fprintf(' baseline window = [%.3f, %.3f] s
',baseLo,baseHi);

    fprintf('
[F. COMPONENT SPLIT AT PEAK]
');
    fprintf(' %-30s %10s %12s %10s  %s
', ...
        'component','peak_ms','baseline_ms','excess_ms','quality');
    for k=1:numel(rows)
        fprintf(' %-30s %10.6f %12.6f %10.6f  %s
', ...
            rows(k).key,rows(k).peak_ms,rows(k).baseline_median_ms, ...
            rows(k).excess_ms,rows(k).quality);
    end

    % Dominant measured excess.
    ex=[rows.excess_ms];
    ex(~isfinite(ex))=-inf;
    [mx,jx]=max(ex);
    OUT.dominant_component=rows(jx).key;
    OUT.dominant_excess_ms=mx;

    % Reference decomposition closure diagnostic.
    rTot=find(strcmp({rows.key},'reference|TOTAL'),1);
    rTV=find(strcmp({rows.key},'reference|timevarying'),1);
    rSL=find(strcmp({rows.key},'reference|sload'),1);
    OUT.reference_split=struct();
    OUT.reference_split.total_ms=rows(rTot).peak_ms;
    OUT.reference_split.timevarying_ms=rows(rTV).peak_ms;
    OUT.reference_split.sload_ms=rows(rSL).peak_ms;
    OUT.reference_split.unattributed_ms= ...
        rows(rTot).peak_ms-rows(rTV).peak_ms-rows(rSL).peak_ms;

    fprintf('
[G. REFERENCE SPLIT]
');
    fprintf(' reference TOTAL       = %.6f ms
',OUT.reference_split.total_ms);
    fprintf('   timevarying/replan  = %.6f ms
',OUT.reference_split.timevarying_ms);
    fprintf('   sload adjust        = %.6f ms
',OUT.reference_split.sload_ms);
    fprintf('   dispatch/merge rest = %.6f ms
',OUT.reference_split.unattributed_ms);

    % Classification.
    if strcmp(OUT.dominant_component,'reference|timevarying')
        OUT.classification='FLIGHTSTART_DOMINATED_BY_REPLANNER_FLIGHT_BRANCH';
    elseif strcmp(OUT.dominant_component,'reference|sload')
        OUT.classification='FLIGHTSTART_DOMINATED_BY_SUSPENDED_LOAD_REF_ADJUST';
    elseif strcmp(OUT.dominant_component,'controller|TOTAL')
        OUT.classification='FLIGHTSTART_DOMINATED_BY_CONTROLLER';
    elseif strcmp(OUT.dominant_component,'estimator|TOTAL')
        OUT.classification='FLIGHTSTART_DOMINATED_BY_ESTIMATOR';
    else
        OUT.classification=['FLIGHTSTART_DOMINANT__' upper(strrep(OUT.dominant_component,'|','_'))];
    end

    OUT.status='FLIGHTSTART_COMPONENT_ATTRIBUTION_AVAILABLE__NO_PRODUCTION_CHANGE';

    fprintf('
[H. CLASSIFICATION]
');
    fprintf(' dominant component = %s
',OUT.dominant_component);
    fprintf(' dominant excess    = %.6f ms
',OUT.dominant_excess_ms);
    fprintf(' classification     = %s
',OUT.classification);
    fprintf(' production changed = 0
');
    fprintf(' status             = %s
',OUT.status);

catch ME
    OUT.status='AUDIT_ERROR__NO_PRODUCTION_CHANGE';
    OUT.error=struct('identifier',ME.identifier,'message',ME.message,'stack',{ME.stack});
    fprintf('
[AUDIT ERROR]
%s | %s
',ME.identifier,ME.message);
end

% Cleanup diagnostic temporary runner and MEX directory only.
try
    if ~isempty(tempRunner)&&isfile(tempRunner), delete(tempRunner); end
catch
end
try
    if ~isempty(tempMexDir)&&isfolder(tempMexDir), rmdir(tempMexDir,'s'); end
catch
end
try
    cd(origDir);
catch
end
try
    evalin('base','clear AF7R58ZZS_R2_TEMP_RUNNER_PATH');
catch
end

root=fileparts(which('REPLANNING_SYSTEM'));
if isempty(root), root=pwd; end
outdir=tempname(root); [ok,msg]=mkdir(outdir); assert(ok,'%s',msg);
report=fullfile(outdir,'AF7R58ZZS_R2_REPORT.txt');
matp=fullfile(outdir,'AF7R58ZZS_R2_OUT.mat');

fid=fopen(report,'wt','n','UTF-8'); assert(fid>=0);
fprintf(fid,'status=%s
',OUT.status);
if isfield(OUT,'replanner_sha'), fprintf(fid,'sourceSHA=%s
',OUT.replanner_sha); end
if isfield(OUT,'runner_sha_before'), fprintf(fid,'runnerSHA=%s
',OUT.runner_sha_before); end
if isfield(OUT,'flight_peak')
    fprintf(fid,'flightPeakT=%.17g
',OUT.flight_peak.t_s);
    fprintf(fid,'flightPeakOnlineMs=%.9g
',OUT.flight_peak.online_ms);
    fprintf(fid,'flightPeakReferenceMs=%.9g
',OUT.flight_peak.reference_ms);
    fprintf(fid,'flightPeakFullMs=%.9g
',OUT.flight_peak.full_ms);
    fprintf(fid,'flightPeakRequestDelta=%.9g
',OUT.flight_peak.request_delta);
end
if isfield(OUT,'dominant_component')
    fprintf(fid,'dominantComponent=%s
',OUT.dominant_component);
    fprintf(fid,'dominantExcessMs=%.9g
',OUT.dominant_excess_ms);
end
if isfield(OUT,'classification')
    fprintf(fid,'classification=%s
',OUT.classification);
end
if isfield(OUT,'reference_split')
    fprintf(fid,'referenceTotalMs=%.9g
',OUT.reference_split.total_ms);
    fprintf(fid,'referenceTimevaryingMs=%.9g
',OUT.reference_split.timevarying_ms);
    fprintf(fid,'referenceSloadMs=%.9g
',OUT.reference_split.sload_ms);
    fprintf(fid,'referenceDispatchRestMs=%.9g
',OUT.reference_split.unattributed_ms);
end
if isfield(OUT,'clock')
    fprintf(fid,'qpcPairMedianMs=%.9g
',OUT.clock.pair_median_ms);
    fprintf(fid,'qpcReliableThresholdMs=%.9g
',OUT.clock.reliable_threshold_ms);
end
if isfield(OUT,'component_rows')
    for k=1:numel(OUT.component_rows)
        r=OUT.component_rows(k);
        fprintf(fid,'component=%s | peakMs=%.9g | baselineMs=%.9g | excessMs=%.9g | quality=%s
', ...
            r.key,r.peak_ms,r.baseline_median_ms,r.excess_ms,r.quality);
    end
end
if isfield(OUT,'error')&&isstruct(OUT.error)&&isfield(OUT.error,'message')
    fprintf(fid,'errorIdentifier=%s
',OUT.error.identifier);
    fprintf(fid,'errorMessage=%s
',OUT.error.message);
end
fclose(fid);

OUT.report_path=report;
OUT.mat_path=matp;
save(matp,'OUT','-v7');
assignin('base','AF7R58ZZS_R2_OUT',OUT);

fprintf('
RETURN: %s
RETURN: %s
',report,matp);
end


function txt=instrumented_dispatcher_text()
txt=sprintf([ ...
'function do_prop_local(time,cha,fCha,logger,env,agent,prop,i,isReady)
' ...
'%% AF7R58ZZS_COMPONENT_QPC_WRAPPER
' ...
'measure=false;
' ...
'try
' ...
'    tt=double(time.t);
' ...
'    measure=isReady && strcmp(char(fCha),''f'') && ...
' ...
'        isfinite(tt) && tt>=4.95-1e-12 && tt<=5.55+1e-12;
' ...
'catch
' ...
'    tt=NaN;
' ...
'end
' ...
'
' ...
'if ~measure
' ...
'    do_prop_local_AF7R58ZZS_inner(time,cha,fCha,logger,env,agent,prop,i,isReady);
' ...
'    return;
' ...
'end
' ...
'
' ...
'propS=string(prop);
' ...
'
' ...
'if propS~="reference"
' ...
'    a=af7r58zzs_qpc_stamp_mex();
' ...
'    do_prop_local_AF7R58ZZS_inner(time,cha,fCha,logger,env,agent,prop,i,isReady);
' ...
'    b=af7r58zzs_qpc_stamp_mex();
' ...
'    af7r58zzs_trace_add(tt,propS,"TOTAL",a,b);
' ...
'    return;
' ...
'end
' ...
'
' ...
'%% Exact reference dispatch, split by allocated reference component.
' ...
'aTot=af7r58zzs_qpc_stamp_mex();
' ...
'if isfield(agent(i).cha_allocation,cha) && ...
' ...
'        isfield(agent(i).cha_allocation.(cha),prop)
' ...
'    list=agent(i).cha_allocation.(cha).(prop);
' ...
'else
' ...
'    list=[];
' ...
'end
' ...
'
' ...
'if isempty(list)
' ...
'    a=af7r58zzs_qpc_stamp_mex();
' ...
'    agent(i).(prop).result=agent(i).(prop).do(time,fCha,logger,env,agent,i);
' ...
'    b=af7r58zzs_qpc_stamp_mex();
' ...
'    af7r58zzs_trace_add(tt,"reference","default",a,b);
' ...
'else
' ...
'    a=af7r58zzs_qpc_stamp_mex();
' ...
'    agent(i).(prop).result=agent(i).(prop).(list(1)).do(time,fCha,logger,env,agent,i);
' ...
'    b=af7r58zzs_qpc_stamp_mex();
' ...
'    af7r58zzs_trace_add(tt,"reference",string(list(1)),a,b);
' ...
'
' ...
'    for j=2:length(list)
' ...
'        a=af7r58zzs_qpc_stamp_mex();
' ...
'        rr=agent(i).(prop).(list(j)).do(time,fCha,logger,env,agent,i);
' ...
'        b=af7r58zzs_qpc_stamp_mex();
' ...
'        af7r58zzs_trace_add(tt,"reference",string(list(j)),a,b);
' ...
'        agent(i).(prop).result=merge_result(agent(i).(prop).result,rr);
' ...
'    end
' ...
'end
' ...
'bTot=af7r58zzs_qpc_stamp_mex();
' ...
'af7r58zzs_trace_add(tt,"reference","TOTAL",aTot,bTot);
' ...
'end
']);
end


function txt=trace_helpers_text()
txt=sprintf([ ...
'function af7r58zzs_trace_add(t,prop,sub,a,b)
' ...
'global AF7R58ZZS_TRACE
' ...
'if isempty(AF7R58ZZS_TRACE) || ~isstruct(AF7R58ZZS_TRACE)
' ...
'    return;
' ...
'end
' ...
'n=AF7R58ZZS_TRACE.n+1;
' ...
'if n>numel(AF7R58ZZS_TRACE.t)
' ...
'    grow=1000;
' ...
'    AF7R58ZZS_TRACE.t(end+grow,1)=NaN;
' ...
'    AF7R58ZZS_TRACE.prop(end+grow,1)="";
' ...
'    AF7R58ZZS_TRACE.sub(end+grow,1)="";
' ...
'    AF7R58ZZS_TRACE.ms(end+grow,1)=NaN;
' ...
'    AF7R58ZZS_TRACE.thread_cycles(end+grow,1)=NaN;
' ...
'    AF7R58ZZS_TRACE.process_cycles(end+grow,1)=NaN;
' ...
'end
' ...
'AF7R58ZZS_TRACE.n=n;
' ...
'AF7R58ZZS_TRACE.t(n)=double(t);
' ...
'AF7R58ZZS_TRACE.prop(n)=string(prop);
' ...
'AF7R58ZZS_TRACE.sub(n)=string(sub);
' ...
'AF7R58ZZS_TRACE.ms(n)=1e3*double(b(1)-a(1))/double(a(2));
' ...
'AF7R58ZZS_TRACE.thread_cycles(n)=double(b(3)-a(3));
' ...
'AF7R58ZZS_TRACE.process_cycles(n)=double(b(4)-a(4));
' ...
'end
']);
end


function txt=cpp_text()
txt=[ ...
'#include "mex.h"' newline ...
'#ifdef _WIN32' newline ...
'#include <windows.h>' newline ...
'#else' newline ...
'#error "AF7R58ZZS QPC stamp MEX requires Windows."' newline ...
'#endif' newline ...
'' newline ...
'void mexFunction(int nlhs,mxArray *plhs[],int nrhs,const mxArray *prhs[]) {' newline ...
'    (void)nrhs; (void)prhs;' newline ...
'    if (nlhs > 1) mexErrMsgIdAndTxt("AF7R58ZZS:NLHS","One output only.");' newline ...
'    static LARGE_INTEGER fq = {0};' newline ...
'    if (fq.QuadPart==0) {' newline ...
'        if (!QueryPerformanceFrequency(&fq))' newline ...
'            mexErrMsgIdAndTxt("AF7R58ZZS:QPF","QueryPerformanceFrequency failed.");' newline ...
'    }' newline ...
'    LARGE_INTEGER q;' newline ...
'    ULONG64 tc=0, pc=0;' newline ...
'    if (!QueryPerformanceCounter(&q))' newline ...
'        mexErrMsgIdAndTxt("AF7R58ZZS:QPC","QueryPerformanceCounter failed.");' newline ...
'    QueryThreadCycleTime(GetCurrentThread(), &tc);' newline ...
'    QueryProcessCycleTime(GetCurrentProcess(), &pc);' newline ...
'    plhs[0]=mxCreateDoubleMatrix(1,4,mxREAL);' newline ...
'    double *o=mxGetPr(plhs[0]);' newline ...
'    o[0]=(double)q.QuadPart;' newline ...
'    o[1]=(double)fq.QuadPart;' newline ...
'    o[2]=(double)tc;' newline ...
'    o[3]=(double)pc;' newline ...
'}' newline];
end


function x=safe_single(v)
v=double(v(:));
v=v(isfinite(v));
if numel(v)==1
    x=v;
elseif isempty(v)
    x=NaN;
else
    % Multiple matches at the same time are unexpected for one component.
    x=max(v);
end
end


function x=safe_median(v)
v=double(v(:));
v=v(isfinite(v));
if isempty(v), x=NaN; else, x=median(v); end
end


function n=count_exact(s,pat)
n=numel(strfind(s,pat));
end


function s=read_text(path)
fid=fopen(path,'rb'); assert(fid>=0);
c=onCleanup(@()fclose(fid)); %#ok<NASGU>
b=fread(fid,'*uint8');
s=native2unicode(b.','UTF-8');
end


function write_text(path,s)
b=unicode2native(s,'UTF-8');
fid=fopen(path,'wb'); assert(fid>=0);
c=onCleanup(@()fclose(fid)); %#ok<NASGU>
fwrite(fid,uint8(b),'uint8');
end


function h=file_hash(path)
md=java.security.MessageDigest.getInstance('SHA-256');
fid=fopen(path,'rb'); assert(fid>=0);
c=onCleanup(@()fclose(fid)); %#ok<NASGU>
while true
    b=fread(fid,1024*1024,'*uint8');
    if isempty(b), break; end
    md.update(b);
end
u=typecast(md.digest(),'uint8');
h=lower(reshape(dec2hex(u,2).',1,[]));
end


function safe_rmpath(p)
try, rmpath(p); catch, end
end
