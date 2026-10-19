function OUT = AF7R58ZZY_TWO_TARGET_DEEP_SPLIT()
% AF7R58ZZY
% =========================================================================
% Robust one-run diagnosis for ONLY the two user-requested timing problems:
%
%   A) Flight entry : 4.95 -- 5.20 s
%   B) Replan       : 9.80 -- 10.10 s
%
% It intentionally ignores unrelated sporadic spikes.
%
% IMPORTANT DESIGN
%   * REPLANNING_SYSTEM.m is NEVER modified.
%   * The original AF7R58ZZB runner is temporarily instrumented IN PLACE,
%     under its original filename/path, then restored byte-for-byte.
%     This avoids the failed "copied runner in a temp folder" execution.
%   * The runner's own clearvars cannot destroy this function's local backup
%     or onCleanup object.
%   * Per-reference component timing is measured directly:
%         timevarying = REPLANNING_SYSTEM
%         sload       = SUSPENDED_LOAD_REF_ADJUST
%         merge_result
%   * Existing REPLANNING_SYSTEM light audit is enabled only through opts.
%   * Immediately after Flight, fixed numeric raw data are saved BEFORE
%     any report/summary construction.  If later analysis fails, no Flight
%     rerun is needed.
%
% Diagnostic only.  Do not use these instrumented values as final WCET.
% =========================================================================

EXPECTED_REPLANNER_SHA = ...
    '310cfdd01f9554a80b71e8d90464816a721859709ad7c4b9137cdc0f3a6eaf49';
EXPECTED_RUNNER_SHA = ...
    '91fa19d72b13e6909d8920659422e3d0cfe169dc9c106b8c78f6d984c60cf507';

OUT=struct();
OUT.schema='AF7R58ZZY_TWO_TARGET_DEEP_SPLIT_V1';
OUT.production_modified=false;
OUT.original_runner_modified=false;
OUT.flight_runs=0;
OUT.raw_capture_path='';
OUT.status='OPEN';
OUT.error=struct();

fprintf('\n====================================================================================================\n');
fprintf(' AF7R58ZZY | TWO-TARGET DEEP SPLIT\n');
fprintf('====================================================================================================\n');
fprintf('[ONLY] FlightEntry 4.95--5.20 s | Replan 9.80--10.10 s\n');
fprintf('[IGNORE] unrelated sporadic spikes = 1\n');
fprintf('[WORKER] process=0 | thread=0 | parpool=0 | parfeval=0\n');
fprintf('[ROBUST] originalRunnerNamePreserved=1 | exactByteRestore=1 | rawBeforeAnalysis=1\n');

pR='';
pRun='';
origRunBytes=uint8([]);
helperPath='';

try
    %% A. Bind exact current sources.
    pR=which('REPLANNING_SYSTEM');
    assert(~isempty(pR)&&isfile(pR), ...
        '[AF7R58ZZY] REPLANNING_SYSTEM unresolved.');
    sourceSHA=file_hash(pR);
    assert(strcmp(sourceSHA,EXPECTED_REPLANNER_SHA), ...
        '[AF7R58ZZY] REPLANNING_SYSTEM is not exact current ZZV-R2 authority.');

    runnerName='run_flight_headless_AF7R58ZZB_reference_cycle_audit';
    pRun=which(runnerName);
    assert(~isempty(pRun)&&isfile(pRun), ...
        '[AF7R58ZZY] AF7R58ZZB runner unresolved.');
    runnerSHA=file_hash(pRun);
    assert(strcmp(runnerSHA,EXPECTED_RUNNER_SHA), ...
        '[AF7R58ZZY] AF7R58ZZB runner differs from exact bound source.');

    OUT.source_sha=sourceSHA;
    OUT.runner_sha=runnerSHA;

    fprintf('\n[A. EXACT SOURCE BINDING]\n');
    fprintf(' REPLANNING_SYSTEM = %s\n',pR);
    fprintf(' SHA-256           = %s\n',sourceSHA);
    fprintf(' runner            = %s\n',pRun);
    fprintf(' SHA-256           = %s\n',runnerSHA);

    %% B. Build persistent reference-component timing tap.
    root=fileparts(pRun);
    helperPath=fullfile(root,'AF7R58ZZY_REF_TAP.m');
    assert(~isfile(helperPath), ...
        '[AF7R58ZZY] helper name already exists; refusing to overwrite.');

    helperText = [ ...
"function OUT=AF7R58ZZY_REF_TAP(action,varargin)" newline ...
"persistent S" newline ...
"if isempty(S)" newline ...
"    S=struct('n',0,'t',nan(1,10000),'phase',repmat(' ',1,10000), ..." newline ...
"        'ordinal',nan(1,10000),'name',{cell(1,10000)}, ..." newline ...
"        'call_ms',nan(1,10000),'merge_ms',nan(1,10000));" newline ...
"end" newline ...
"switch lower(char(string(action)))" newline ...
"    case 'reset'" newline ...
"        S.n=0;" newline ...
"        OUT=[];" newline ...
"    case 'record'" newline ...
"        S.n=S.n+1;" newline ...
"        if S.n>numel(S.t)" newline ...
"            g=10000;" newline ...
"            S.t(end+1:end+g)=nan;" newline ...
"            S.phase(end+1:end+g)=' ';" newline ...
"            S.ordinal(end+1:end+g)=nan;" newline ...
"            S.name(end+1:end+g)={[]};" newline ...
"            S.call_ms(end+1:end+g)=nan;" newline ...
"            S.merge_ms(end+1:end+g)=nan;" newline ...
"        end" newline ...
"        k=S.n;" newline ...
"        S.t(k)=double(varargin{1});" newline ...
"        ph=char(string(varargin{2}));" newline ...
"        if isempty(ph), ph='?'; end" newline ...
"        S.phase(k)=ph(1);" newline ...
"        S.ordinal(k)=double(varargin{3});" newline ...
"        S.name{k}=char(string(varargin{4}));" newline ...
"        S.call_ms(k)=double(varargin{5});" newline ...
"        S.merge_ms(k)=double(varargin{6});" newline ...
"        OUT=[];" newline ...
"    case 'get'" newline ...
"        n=S.n;" newline ...
"        OUT=struct('n',n,'t',S.t(1:n),'phase',S.phase(1:n), ..." newline ...
"            'ordinal',S.ordinal(1:n),'name',{S.name(1:n)}, ..." newline ...
"            'call_ms',S.call_ms(1:n),'merge_ms',S.merge_ms(1:n));" newline ...
"    otherwise" newline ...
"        error('[AF7R58ZZY_REF_TAP] unknown action.');" newline ...
"end" newline ...
"end" newline ...
    ];
    write_utf8(helperPath,char(helperText));
    rehash;
    AF7R58ZZY_REF_TAP('reset');

    %% C. Patch ORIGINAL runner temporarily, under the SAME path/name.
    origRunBytes=read_bytes(pRun);
    R=native2unicode(origRunBytes.','UTF-8');

    % C1. Enable existing internal light timings via constructor opts only.
    ctor='replanner_instance = REPLANNING_SYSTEM(agent,nominal_ref,replan_opts);';
    assert(count_exact(R,ctor)==1, ...
        '[AF7R58ZZY] replanner constructor anchor unresolved.');
    optPatch=sprintf([ ...
        '%% AF7R58ZZY TEMP DIAGNOSTIC OPTIONS ONLY\\n' ...
        'replan_opts.ENABLE_LIGHT_RUNTIME_AUDIT = true;\\n' ...
        'replan_opts.SPIKE_CAPTURE_THRESHOLD_MS = 4.0;\\n\\n']);
    R=strrep(R,ctor,[optPatch ctor]);

    % C2. Insert a reference-only timing branch at the TOP of do_prop_local.
    % It preserves the exact original order:
    %   list(1).do -> merge_result(list(2).do) -> ...
    % Other properties still use the untouched original code.
    sig='function do_prop_local(time,cha,fCha,logger,env,agent,prop,i,isReady)';
    ps=strfind(R,sig);
    assert(numel(ps)==1,'[AF7R58ZZY] do_prop_local signature unresolved.');

    tail=R(ps(1):end);
    plantPat=['if ~isReady && strcmp(prop,"plant")' newline ...
              '    return;' newline ...
              'end'];
    pp=strfind(tail,plantPat);
    assert(numel(pp)==1, ...
        '[AF7R58ZZY] do_prop_local plant-guard anchor unresolved.');

    insertAt=ps(1)+pp(1)-1+length(plantPat);
    branch = sprintf([ ...
        '\\n\\n%% AF7R58ZZY TEMP: exact per-reference-component timing\\n' ...
        'if strcmp(prop,"reference")\\n' ...
        '    if isfield(agent(i).cha_allocation,cha) && ...\\n' ...
        '            isfield(agent(i).cha_allocation.(cha),prop)\\n' ...
        '        zlist = agent(i).cha_allocation.(cha).(prop);\\n' ...
        '    else\\n' ...
        '        zlist = [];\\n' ...
        '    end\\n' ...
        '    if isempty(zlist)\\n' ...
        '        zt=tic;\\n' ...
        '        agent(i).(prop).result = ...\\n' ...
        '            agent(i).(prop).do(time,fCha,logger,env,agent,i);\\n' ...
        '        AF7R58ZZY_REF_TAP(''record'',time.t,cha,1,''reference.default'',1e3*toc(zt),0);\\n' ...
        '    else\\n' ...
        '        zt=tic;\\n' ...
        '        zres=agent(i).(prop).(zlist(1)).do(time,fCha,logger,env,agent,i);\\n' ...
        '        zcall=1e3*toc(zt);\\n' ...
        '        agent(i).(prop).result=zres;\\n' ...
        '        AF7R58ZZY_REF_TAP(''record'',time.t,cha,1,char(string(zlist(1))),zcall,0);\\n' ...
        '        for zj=2:length(zlist)\\n' ...
        '            zt=tic;\\n' ...
        '            zr=agent(i).(prop).(zlist(zj)).do(time,fCha,logger,env,agent,i);\\n' ...
        '            zcall=1e3*toc(zt);\\n' ...
        '            zm=tic;\\n' ...
        '            agent(i).(prop).result=merge_result(agent(i).(prop).result,zr);\\n' ...
        '            zmerge=1e3*toc(zm);\\n' ...
        '            AF7R58ZZY_REF_TAP(''record'',time.t,cha,zj,char(string(zlist(zj))),zcall,zmerge);\\n' ...
        '        end\\n' ...
        '    end\\n' ...
        '    return;\\n' ...
        'end\\n']);
    R=[R(1:insertAt) branch R(insertAt+1:end)];

    % C3. Parse the entire temporarily patched runner before touching disk.
    parseDir=tempname(root);
    [ok,msg]=mkdir(parseDir); assert(ok,'%s',msg);
    parseName='AF7R58ZZY_PARSE_RUNNER';
    % The runner is a script with local functions; checkcode is the safest
    % no-execution syntax scan available for this script form.
    parsePath=fullfile(parseDir,[parseName '.m']);
    write_utf8(parsePath,R);
    issues=checkcode(parsePath,'-id');
    syntaxBad=false;
    for kk=1:numel(issues)
        id=char(string(issues(kk).id));
        if contains(id,'SYNER') || contains(id,'PARSE') || ...
                contains(lower(char(string(issues(kk).message))),'syntax')
            syntaxBad=true;
        end
    end
    assert(~syntaxBad,'[AF7R58ZZY] patched runner syntax scan failed.');

    % Hold exact restoration in THIS function workspace.  The runner's
    % clearvars affects base workspace only and cannot clear this guard.
    restoreGuard=onCleanup(@()restore_runner_exact(pRun,origRunBytes)); %#ok<NASGU>

    write_utf8(pRun,R);
    assert(~strcmp(file_hash(pRun),runnerSHA), ...
        '[AF7R58ZZY] temporary runner patch was not written.');

    fprintf('\n[B. TEMPORARY IN-PLACE INSTRUMENTATION]\n');
    fprintf(' REPLANNING_SYSTEM source modified = 0\n');
    fprintf(' original runner path/name retained = 1\n');
    fprintf(' reference components timed individually = 1\n');
    fprintf(' existing replanner light audit enabled = 1\n');

    %% D. Run EXACT standard runner name once.
    fprintf('\n[C. ONE FLIGHT RUN]\n');
    evalin('base',[runnerName ';']);
    OUT.flight_runs=1;

    % Capture data immediately.
    if evalin('base','exist(''AF7R58ZZB_RUNTIME_PROFILE'',''var'')')==1
        P=evalin('base','AF7R58ZZB_RUNTIME_PROFILE');
    elseif evalin('base','exist(''runtime_profile'',''var'')')==1
        P=evalin('base','runtime_profile');
    else
        error('[AF7R58ZZY] runtime profile missing immediately after Flight.');
    end
    assert(evalin('base','exist(''logger'',''var'')')==1, ...
        '[AF7R58ZZY] logger missing immediately after Flight.');
    L=evalin('base','logger');

    TAP=AF7R58ZZY_REF_TAP('get');

    Sstage=struct();
    if evalin('base','exist(''replanner_instance'',''var'')')==1
        ro=evalin('base','replanner_instance');
        if isa(ro,'REPLANNING_SYSTEM') && ismethod(ro,'get_staged_pipeline_status')
            Sstage=ro.get_staged_pipeline_status();
        end
    end

    [tl,X,names]=extract_logger_timing(L);

    % SAVE RAW BEFORE ANY SUMMARY/STRUCT-ARRAY LOGIC.
    rawDir=tempname(fileparts(pR));
    [ok,msg]=mkdir(rawDir); assert(ok,'%s',msg);
    rawPath=fullfile(rawDir,'AF7R58ZZY_RAW_CAPTURE.mat');

    RAW=struct();
    RAW.schema='AF7R58ZZY_RAW_CAPTURE_V1';
    RAW.source_sha=sourceSHA;
    RAW.runner_sha=runnerSHA;
    RAW.runtime_profile=P;
    RAW.reference_tap=TAP;
    RAW.logger_time=tl;
    RAW.timing_names=names;
    RAW.timing_matrix=X;
    RAW.staged_pipeline_status=Sstage;
    save(rawPath,'RAW','-v7.3');
    OUT.raw_capture_path=rawPath;

    fprintf('\n[D. RAW SAVED BEFORE ANALYSIS]\n');
    fprintf(' %s\n',rawPath);

    % Restore runner BEFORE summary analysis.
    restore_runner_exact(pRun,origRunBytes);
    assert(strcmp(file_hash(pRun),runnerSHA), ...
        '[AF7R58ZZY] runner exact-byte restore failed.');

    %% E. Two windows only. Numeric matrices, no heterogeneous struct arrays.
    F=analyze_target(P,TAP,tl,X,names,4.95,5.20);
    G=analyze_target(P,TAP,tl,X,names,9.80,10.10);

    OUT.flight_entry=F;
    OUT.replan_window=G;
    OUT.staged_pipeline_status=Sstage;

    fprintf('\n====================================================================================================\n');
    fprintf(' E1. FLIGHT ENTRY 4.95--5.20 s\n');
    fprintf('====================================================================================================\n');
    print_target(F);

    fprintf('\n====================================================================================================\n');
    fprintf(' E2. REPLAN 9.80--10.10 s\n');
    fprintf('====================================================================================================\n');
    print_target(G);

    fprintf('\n====================================================================================================\n');
    fprintf(' F. STAGED PIPELINE\n');
    fprintf('====================================================================================================\n');
    print_stage(Sstage);

    OUT.status='TWO_TARGET_DEEP_SPLIT_COMPLETE';

    report=fullfile(rawDir,'AF7R58ZZY_REPORT.txt');
    matp=fullfile(rawDir,'AF7R58ZZY_OUT.mat');
    write_report(report,OUT);
    OUT.report_path=report;
    OUT.mat_path=matp;
    save(matp,'OUT','-v7.3');
    assignin('base','AF7R58ZZY_OUT',OUT);

    fprintf('\n====================================================================================================\n');
    fprintf(' G. FINISH\n');
    fprintf('====================================================================================================\n');
    fprintf(' status = %s\n',OUT.status);
    fprintf(' source modified = 0\n');
    fprintf(' runner restored = 1\n');
    fprintf(' raw    = %s\n',rawPath);
    fprintf(' report = %s\n',report);
    fprintf(' mat    = %s\n',matp);

catch ME
    OUT.status='DIAGNOSTIC_ERROR__EXACT_RESTORE_ATTEMPTED';
    OUT.error=struct('identifier',ME.identifier,'message',ME.message,'stack',{ME.stack});

    fprintf('\n[AF7R58ZZY ERROR]\n');
    fprintf(' identifier = %s\n',ME.identifier);
    fprintf(' message    = %s\n',ME.message);
    if ~isempty(ME.stack)
        fprintf(' line       = %d\n',ME.stack(1).line);
    end
    if isfield(OUT,'raw_capture_path') && ~isempty(OUT.raw_capture_path) && ...
            isfile(OUT.raw_capture_path)
        fprintf(' RAW ALREADY SAVED: %s\n',OUT.raw_capture_path);
        fprintf(' No Flight rerun is required for follow-up extraction.\n');
    end
end

%% H. Cleanup / proof.
try
    if ~isempty(pRun)&&~isempty(origRunBytes)
        restore_runner_exact(pRun,origRunBytes);
    end
catch
end
try
    if ~isempty(helperPath)&&isfile(helperPath)
        clear AF7R58ZZY_REF_TAP
        delete(helperPath);
        rehash;
    end
catch
end

try
    OUT.production_modified= ...
        ~strcmp(file_hash(pR),EXPECTED_REPLANNER_SHA);
catch
end
try
    OUT.original_runner_modified= ...
        ~strcmp(file_hash(pRun),EXPECTED_RUNNER_SHA);
catch
end

fprintf('\n[RESTORE PROOF]\n');
fprintf(' REPLANNING_SYSTEM modified = %d\n',OUT.production_modified);
fprintf(' original runner modified  = %d\n',OUT.original_runner_modified);
fprintf(' status = %s\n',OUT.status);
end


function A=analyze_target(P,TAP,tl,X,names,t0,t1)
t=double(P.t(:));
ph=char(P.phase(:));

% Runtime-profile reference total if available.
refProfile=get_profile_vector(P,'reference_s',numel(t));

idx=find(ph=='f' & t>=t0-1e-12 & t<=t1+1e-12);
assert(~isempty(idx),'[AF7R58ZZY] target window has no Flight samples.');

% Fixed numeric columns.
%  1 t
%  2 online
%  3 full
%  4 reference total
%  5 timevarying (= REPLANNING_SYSTEM component wrapper wall)
%  6 sload
%  7 merge_result
%  8 component sum
%  9 replanner internal light e2e
% 10 wrapper timevarying - internal replanner
% 11 baseCertGeom
% 12 sensorTrigger
% 13 outputState
% 14 baseRef
% 15 flightGuard
% 16 certGuard
% 17 activeRef
% 18 geometry
% 19 genCritical
% 20 genCallback
% 21 requestDelta
% 22 commitDelta
M=nan(numel(idx),22);

for k=1:numel(idx)
    i=idx(k);
    tt=t(i);
    M(k,1)=tt;
    M(k,2)=1e3*get_profile(P,'online_before_plant_s',i);
    M(k,3)=1e3*get_profile(P,'full_iteration_s',i);
    if isfinite(refProfile(i))
        M(k,4)=1e3*refProfile(i);
    end

    % Exact per-reference component tap.
    hit=find(TAP.phase=='f' & abs(double(TAP.t)-tt)<=1e-10*max(1,abs(tt)));
    tv=NaN; sl=NaN; mg=0;
    for h=hit(:).'
        nm=char(TAP.name{h});
        if strcmp(nm,'timevarying')
            tv=double(TAP.call_ms(h));
        elseif strcmp(nm,'sload')
            sl=double(TAP.call_ms(h));
        end
        if isfinite(double(TAP.merge_ms(h)))
            mg=mg+double(TAP.merge_ms(h));
        end
    end
    M(k,5)=tv;
    M(k,6)=sl;
    M(k,7)=mg;
    if isfinite(tv)||isfinite(sl)
        M(k,8)=sum([tv sl mg],'omitnan');
    end

    % Internal replanner light timings from logger.
    [dmin,j]=min(abs(tl-tt));
    if ~isempty(j)&&isfinite(dmin)&&dmin<=1e-8*max(1,abs(tt))
        M(k,9)=get_timing(X,j,names,'rt_light_e2e_ms');
        if isfinite(M(k,5))&&isfinite(M(k,9))
            M(k,10)=M(k,5)-M(k,9);
        end
        M(k,11)=get_timing(X,j,names,'rt_spike_stage_base_cert_geom_ms');
        M(k,12)=get_timing(X,j,names,'rt_spike_stage_sensor_trigger_ms');
        M(k,13)=get_timing(X,j,names,'rt_spike_stage_output_state_ms');
        M(k,14)=get_timing(X,j,names,'rt_spike_base_ref_ms');
        M(k,15)=get_timing(X,j,names,'rt_spike_flight_guard_ms');
        M(k,16)=get_timing(X,j,names,'rt_spike_cert_guard_ms');
        M(k,17)=get_timing(X,j,names,'rt_spike_active_ref_ms');
        M(k,18)=get_timing(X,j,names,'rt_spike_geometry_ms');
    end

    M(k,19)=get_profile(P,'gen_last_cycle_critical_ms',i);
    M(k,20)=get_profile(P,'gen_last_callback_ms',i);
    M(k,21)=get_profile(P,'gen_request_delta',i);
    M(k,22)=get_profile(P,'gen_commit_delta',i);
end

% If reference_s is unavailable, exact component wrapper sum is the fallback.
bad=~isfinite(M(:,4)) & isfinite(M(:,8));
M(bad,4)=M(bad,8);

[~,kp]=max_finite(M(:,4));

A=struct();
A.t0=t0;
A.t1=t1;
A.samples=M;
A.peak_row=kp;
A.peak_t_s=M(kp,1);
A.peak_reference_ms=M(kp,4);
A.peak_timevarying_ms=M(kp,5);
A.peak_sload_ms=M(kp,6);
A.peak_merge_ms=M(kp,7);
A.peak_replanner_internal_ms=M(kp,9);
A.peak_timevarying_residual_ms=M(kp,10);
A.peak_base_cert_geom_ms=M(kp,11);
A.peak_sensor_trigger_ms=M(kp,12);
A.peak_output_state_ms=M(kp,13);
A.peak_gen_critical_ms=M(kp,19);
A.peak_request_delta=M(kp,21);
A.peak_commit_delta=M(kp,22);

vals=[M(kp,11) M(kp,12) M(kp,13)];
nms={'BASE_CERT_GEOM','SENSOR_TRIGGER','OUTPUT_STATE'};
vv=vals; vv(~isfinite(vv))=-inf;
[mx,jx]=max(vv);
if isfinite(mx)
    A.dominant_internal=nms{jx};
    A.dominant_internal_ms=mx;
else
    A.dominant_internal='NO_FINE_CAPTURE';
    A.dominant_internal_ms=NaN;
end
end


function print_target(A)
fprintf(' Columns:\n');
fprintf(' t[s] | reference | timevarying | sload | merge | replannerE2E | baseCertGeom | sensorTrigger | outputState | genCritical | req | commit [ms except flags]\n');

M=A.samples;
for k=1:size(M,1)
    fprintf('%6.3f | %9.3f | %11.3f | %6.3f | %5.3f | %12.3f | %12.3f | %13.3f | %11.3f | %11.3f | %3.0f | %6.0f\n', ...
        M(k,1),M(k,4),M(k,5),M(k,6),M(k,7),M(k,9), ...
        M(k,11),M(k,12),M(k,13),M(k,19),M(k,21),M(k,22));
end

fprintf('\n PEAK REFERENCE ROW\n');
fprintf(' t                         = %.3f s\n',A.peak_t_s);
fprintf(' reference total           = %.6f ms\n',A.peak_reference_ms);
fprintf(' timevarying / replanner   = %.6f ms\n',A.peak_timevarying_ms);
fprintf(' sload                     = %.6f ms\n',A.peak_sload_ms);
fprintf(' merge_result              = %.6f ms\n',A.peak_merge_ms);
fprintf(' replanner internal e2e    = %.6f ms\n',A.peak_replanner_internal_ms);
fprintf(' wrapper - internal        = %.6f ms\n',A.peak_timevarying_residual_ms);
fprintf(' BASE_CERT_GEOM            = %.6f ms\n',A.peak_base_cert_geom_ms);
fprintf(' SENSOR_TRIGGER            = %.6f ms\n',A.peak_sensor_trigger_ms);
fprintf(' OUTPUT_STATE              = %.6f ms\n',A.peak_output_state_ms);
fprintf(' dominant replanner stage  = %s | %.6f ms\n', ...
    A.dominant_internal,A.dominant_internal_ms);
fprintf(' generator critical        = %.6f ms\n',A.peak_gen_critical_ms);
fprintf(' request / commit          = %.0f / %.0f\n', ...
    A.peak_request_delta,A.peak_commit_delta);

r=A.samples(A.peak_row,:);
fprintf(' BASE_CERT_GEOM fine split:\n');
fprintf('   baseRef / flightGuard / certGuard / activeRef / geometry\n');
fprintf('   %.6f / %.6f / %.6f / %.6f / %.6f ms\n', ...
    r(14),r(15),r(16),r(17),r(18));
end


function print_stage(S)
if ~isstruct(S)||isempty(fieldnames(S))
    fprintf(' staged status unavailable\n');
    return;
end
p={'stage1_count','stage2_count','commit_count','fail_count', ...
   'last_stage1_ms','last_stage2_ms','last_stage3_ms', ...
   'trigger_kernel_skip_count','postcommit_cache_prepare_count','last_reason'};
for k=1:numel(p)
    if isfield(S,p{k})
        v=S.(p{k});
        if isnumeric(v)&&isscalar(v)
            fprintf(' %-36s = %.9g\n',p{k},double(v));
        else
            fprintf(' %-36s = %s\n',p{k},char(string(v)));
        end
    end
end
end


function [tl,X,names]=extract_logger_timing(L)
D=member(L,'Data');
A=member(D,'agent');
if numel(A)>1, A=A(1); end
RR=member(member(A,'reference'),'result');
if ~iscell(RR), RR=num2cell(RR); end
RR=RR(:);

tl=member(D,'t');
assert(~isempty(tl),'[AF7R58ZZY] logger.Data.t unavailable.');
tl=double(tl(:));
N=min(numel(RR),numel(tl));
RR=RR(1:N);
tl=tl(1:N);

names={ ...
    'rt_light_e2e_ms', ...
    'rt_spike_capture_time', ...
    'rt_spike_total_ms', ...
    'rt_spike_stage_front_ms', ...
    'rt_spike_stage_loadmass_ms', ...
    'rt_spike_stage_dependency_ms', ...
    'rt_spike_stage_base_cert_geom_ms', ...
    'rt_spike_stage_sensor_trigger_ms', ...
    'rt_spike_stage_output_state_ms', ...
    'rt_spike_base_ref_ms', ...
    'rt_spike_flight_guard_ms', ...
    'rt_spike_cert_guard_ms', ...
    'rt_spike_active_ref_ms', ...
    'rt_spike_geometry_ms'};

X=nan(N,numel(names));
for i=1:N
    st=member(RR{i},'state');
    if isempty(st), continue; end
    for j=1:numel(names)
        [v,ok]=scalar(member(st,names{j}));
        if ok, X(i,j)=v; end
    end
end
end


function v=get_profile_vector(P,nm,N)
v=nan(N,1);
if isstruct(P)&&isfield(P,nm)
    a=double(P.(nm)(:));
elseif istable(P)&&ismember(nm,P.Properties.VariableNames)
    a=double(P.(nm)(:));
else
    return;
end
m=min(N,numel(a));
v(1:m)=a(1:m);
end


function x=get_profile(P,nm,i)
x=NaN;
if isstruct(P)&&isfield(P,nm)
    v=P.(nm);
elseif istable(P)&&ismember(nm,P.Properties.VariableNames)
    v=P.(nm);
else
    return;
end
if numel(v)>=i && isfinite(double(v(i)))
    x=double(v(i));
end
end


function x=get_timing(X,i,names,nm)
x=NaN;
j=find(strcmp(names,nm),1);
if ~isempty(j)&&i<=size(X,1)&&isfinite(X(i,j))
    x=X(i,j);
end
end


function [m,k]=max_finite(v)
vv=double(v(:));
good=isfinite(vv);
assert(any(good),'[AF7R58ZZY] no finite values in target window.');
ix=find(good);
[m,j]=max(vv(good)); %#ok<ASGLU>
k=ix(j);
end


function write_report(path,OUT)
fid=fopen(path,'wt','n','UTF-8'); assert(fid>=0);
c=onCleanup(@()fclose(fid)); %#ok<NASGU>

fprintf(fid,'status=%s\n',OUT.status);
fprintf(fid,'sourceSHA=%s\n',OUT.source_sha);
fprintf(fid,'runnerSHA=%s\n',OUT.runner_sha);
fprintf(fid,'flightRuns=%d\n',OUT.flight_runs);
fprintf(fid,'raw=%s\n',OUT.raw_capture_path);

write_one(fid,'FLIGHT_ENTRY',OUT.flight_entry);
write_one(fid,'REPLAN_WINDOW',OUT.replan_window);
end


function write_one(fid,label,A)
fprintf(fid,'\n[%s]\n',label);
fprintf(fid,'peakT=%.9g\n',A.peak_t_s);
fprintf(fid,'referenceMs=%.9g\n',A.peak_reference_ms);
fprintf(fid,'timevaryingMs=%.9g\n',A.peak_timevarying_ms);
fprintf(fid,'sloadMs=%.9g\n',A.peak_sload_ms);
fprintf(fid,'mergeMs=%.9g\n',A.peak_merge_ms);
fprintf(fid,'replannerE2EMs=%.9g\n',A.peak_replanner_internal_ms);
fprintf(fid,'baseCertGeomMs=%.9g\n',A.peak_base_cert_geom_ms);
fprintf(fid,'sensorTriggerMs=%.9g\n',A.peak_sensor_trigger_ms);
fprintf(fid,'outputStateMs=%.9g\n',A.peak_output_state_ms);
fprintf(fid,'dominant=%s\n',A.dominant_internal);
fprintf(fid,'dominantMs=%.9g\n',A.dominant_internal_ms);
fprintf(fid,'genCriticalMs=%.9g\n',A.peak_gen_critical_ms);
fprintf(fid,'request=%.0f\n',A.peak_request_delta);
fprintf(fid,'commit=%.0f\n',A.peak_commit_delta);
for k=1:size(A.samples,1)
    fprintf(fid,'row=');
    fprintf(fid,' %.9g',A.samples(k,:));
    fprintf(fid,'\n');
end
end


function [x,ok]=scalar(v)
x=NaN; ok=false;
for k=1:3
    if iscell(v)&&numel(v)==1
        v=v{1};
    else
        break;
    end
end
if (isnumeric(v)||islogical(v))&&isscalar(v)&&isfinite(double(v))
    x=double(v);
    ok=true;
end
end


function v=member(x,n)
v=[];
try
    for k=1:3
        if iscell(x)&&numel(x)==1
            x=x{1};
        else
            break;
        end
    end
    if isstruct(x)&&isscalar(x)&&isfield(x,n)
        v=x.(n);
    elseif isobject(x)&&isscalar(x)&&isprop(x,n)
        v=x.(n);
    end
catch
    v=[];
end
end


function n=count_exact(s,pat)
n=numel(strfind(s,pat));
end


function b=read_bytes(path)
fid=fopen(path,'rb'); assert(fid>=0);
c=onCleanup(@()fclose(fid)); %#ok<NASGU>
b=fread(fid,'*uint8');
end


function write_utf8(path,s)
b=unicode2native(s,'UTF-8');
fid=fopen(path,'wb'); assert(fid>=0);
c=onCleanup(@()fclose(fid)); %#ok<NASGU>
fwrite(fid,uint8(b),'uint8');
end


function restore_runner_exact(path,b)
try
    fid=fopen(path,'wb');
    if fid>=0
        fwrite(fid,uint8(b),'uint8');
        fclose(fid);
    end
catch
end
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
