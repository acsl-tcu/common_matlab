function OUT = AF7R58ZZY_R4_TWO_TARGET_DEEP_SPLIT()
% AF7R58ZZY-R4
% =========================================================================
% One-run deep timing split for ONLY the two requested problems:
%
%   A) Flight entry : 4.95 -- 5.20 s
%   B) Replan       : 9.80 -- 10.10 s
%
% Unrelated sporadic spikes are intentionally ignored.
%
% This R2 removes the previous generated helper-file path completely.
% The earlier failure at unicode2native came from serializing a generated
% string-array helper.  R2 uses ONLY a numeric MATLAB global timing buffer
% inserted into the original runner while it remains at its ORIGINAL path
% and ORIGINAL filename.
%
% SAFETY / SOURCE POLICY
%   - REPLANNING_SYSTEM.m is never modified.
%   - Original ZZB runner is backed up in memory, temporarily instrumented
%     in place, executed ONCE, then restored byte-for-byte.
%   - No worker / parpool / parfeval / backgroundPool.
%   - H=4 s / finite sensing / continuous certificate are untouched.
%   - Raw numeric capture is saved BEFORE summary/report construction.
%     Therefore if post-analysis fails, the Flight does NOT need rerunning.
%
% Diagnostic timing only; instrumented values are not final WCET evidence.
% =========================================================================

EXPECTED_REPLANNER_SHA = ...
    '310cfdd01f9554a80b71e8d90464816a721859709ad7c4b9137cdc0f3a6eaf49';
EXPECTED_RUNNER_SHA = ...
    '91fa19d72b13e6909d8920659422e3d0cfe169dc9c106b8c78f6d984c60cf507';

OUT=struct();
OUT.schema='AF7R58ZZY_R4_TWO_TARGET_DEEP_SPLIT_V1';
OUT.production_modified=false;
OUT.original_runner_modified=false;
OUT.flight_runs=0;
OUT.raw_capture_path='';
OUT.status='OPEN';
OUT.error=struct();

fprintf('\n====================================================================================================\n');
fprintf(' AF7R58ZZY-R4 | TWO-TARGET DEEP SPLIT\n');
fprintf('====================================================================================================\n');
fprintf('[ONLY] FlightEntry=4.95--5.20 s | Replan=9.80--10.10 s\n');
fprintf('[IGNORE] unrelated sporadic spikes = 1\n');
fprintf('[WORKER] process=0 | thread=0 | parpool=0 | parfeval=0\n');
fprintf('[R4] generatedHelperFile=0 | numericGlobalTap=1 | rawBeforeAnalysis=1 | falseCountAsserts=0\n');

pR='';
pRun='';
origRunBytes=uint8([]);
restoreGuard=[];

% Shared numeric timing buffer.
% Columns:
%   1 t
%   2 phase ASCII
%   3 reference ordinal
%   4 component call ms
%   5 merge_result ms
%   6 list length
global AF7R58ZZY_R2_REF_TAP_DATA
AF7R58ZZY_R2_REF_TAP_DATA=nan(12000,6);
AF7R58ZZY_R2_REF_TAP_DATA(1,1)=0; % row 1 col 1 stores count

try
    %% A. Exact source binding.
    pR=which('REPLANNING_SYSTEM');
    assert(~isempty(pR)&&isfile(pR), ...
        '[AF7R58ZZY-R4] REPLANNING_SYSTEM unresolved.');
    sourceSHA=file_hash(pR);
    assert(strcmp(sourceSHA,EXPECTED_REPLANNER_SHA), ...
        '[AF7R58ZZY-R4] REPLANNING_SYSTEM is not exact current ZZV-R2 source.');

    runnerName='run_flight_headless_AF7R58ZZB_reference_cycle_audit';
    pRun=which(runnerName);
    assert(~isempty(pRun)&&isfile(pRun), ...
        '[AF7R58ZZY-R4] exact ZZB runner unresolved.');
    runnerSHA=file_hash(pRun);
    assert(strcmp(runnerSHA,EXPECTED_RUNNER_SHA), ...
        '[AF7R58ZZY-R4] ZZB runner differs from exact bound source.');

    OUT.source_sha=sourceSHA;
    OUT.runner_sha=runnerSHA;

    fprintf('\n[A. EXACT SOURCE BINDING]\n');
    fprintf(' REPLANNING_SYSTEM = %s\n',pR);
    fprintf(' SHA-256           = %s\n',sourceSHA);
    fprintf(' runner            = %s\n',pRun);
    fprintf(' SHA-256           = %s\n',runnerSHA);

    %% B. Read and audit runner before patching.
    origRunBytes=read_bytes(pRun);
    R=native2unicode(origRunBytes.','UTF-8');

    ctor='replanner_instance = REPLANNING_SYSTEM(agent,nominal_ref,replan_opts);';
    assert(count_exact(R,ctor)==1, ...
        '[AF7R58ZZY-R4] replanner constructor anchor unresolved.');

    sig='function do_prop_local(time,cha,fCha,logger,env,agent,prop,i,isReady)';
    assert(count_exact(R,sig)==1, ...
        '[AF7R58ZZY-R4] do_prop_local signature unresolved.');

    flightAlloc='agent.cha_allocation.f.reference = ["timevarying","sload"];';
    assert(count_exact(compact_ws(R),compact_ws(flightAlloc))==1, ...
        ['[AF7R58ZZY-R4] Flight reference allocation is not exactly ' ...
         '[timevarying,sload].']);

    assert(~contains(R,'AF7R58ZZY_R2_REF_TAP_DATA'), ...
        '[AF7R58ZZY-R4] runner already contains R2 instrumentation.');

    %% C. Patch only diagnostic options + reference wrapper timing.
    % C1. Enable existing REPLANNING_SYSTEM light timings via opts.
    optPatch=sprintf([ ...
        '%% AF7R58ZZY-R4 TEMP DIAGNOSTIC OPTIONS ONLY\n' ...
        'replan_opts.ENABLE_LIGHT_RUNTIME_AUDIT = true;\n' ...
        'replan_opts.SPIKE_CAPTURE_THRESHOLD_MS = 4.0;\n\n']);
    R=strrep(R,ctor,[optPatch ctor]);

    % C2. Add global declaration immediately after runner clearvars line.
    clearAnchor='clearvars; close all; clc;';
    assert(count_exact(R,clearAnchor)==1, ...
        '[AF7R58ZZY-R4] runner clearvars anchor unresolved.');

    globalPatch=sprintf([ ...
        'clearvars; close all; clc;\n' ...
        '%% AF7R58ZZY-R4 TEMP numeric reference timing tap\n' ...
        'global AF7R58ZZY_R2_REF_TAP_DATA\n' ...
        'if isempty(AF7R58ZZY_R2_REF_TAP_DATA) || size(AF7R58ZZY_R2_REF_TAP_DATA,2)~=6\n' ...
        '    AF7R58ZZY_R2_REF_TAP_DATA=nan(12000,6);\n' ...
        'end\n' ...
        'AF7R58ZZY_R2_REF_TAP_DATA(:)=NaN;\n' ...
        'AF7R58ZZY_R2_REF_TAP_DATA(1,1)=0;']);
    R=strrep(R,clearAnchor,globalPatch);

    % C3. Add a reference-only branch inside do_prop_local.
    %
    % This branch preserves the exact reference order and merge semantics:
    %   list(1).do -> list(2).do -> merge_result
    % For Flight, ordinal 1=timevarying, ordinal 2=sload (audited above).
    plantPat=sprintf([ ...
        'if ~isReady && strcmp(prop,"plant")\n' ...
        '    return;\n' ...
        'end']);
    assert(count_exact(R,plantPat)==1, ...
        '[AF7R58ZZY-R4] do_prop_local plant guard unresolved.');

    branch=sprintf([ ...
        '%s\n\n' ...
        '%% AF7R58ZZY-R4 TEMP exact reference-component timing\n' ...
        'if strcmp(prop,"reference")\n' ...
        '    global AF7R58ZZY_R2_REF_TAP_DATA\n' ...
        '    if isfield(agent(i).cha_allocation,cha) && ...\n' ...
        '            isfield(agent(i).cha_allocation.(cha),prop)\n' ...
        '        zlist=agent(i).cha_allocation.(cha).(prop);\n' ...
        '    else\n' ...
        '        zlist=[];\n' ...
        '    end\n' ...
        '    if isempty(zlist)\n' ...
        '        zt=tic;\n' ...
        '        agent(i).(prop).result=agent(i).(prop).do(time,fCha,logger,env,agent,i);\n' ...
        '        zcall=1e3*toc(zt);\n' ...
        '        if strcmp(cha,''f'')\n' ...
        '            zn=AF7R58ZZY_R2_REF_TAP_DATA(1,1)+1;\n' ...
        '            assert(zn+1<=size(AF7R58ZZY_R2_REF_TAP_DATA,1),''[AF7R58ZZY-R4] timing tap overflow.'');\n' ...
        '            AF7R58ZZY_R2_REF_TAP_DATA(1,1)=zn;\n' ...
        '            AF7R58ZZY_R2_REF_TAP_DATA(zn+1,:)=[double(time.t) double(''f'') 1 zcall 0 0];\n' ...
        '        end\n' ...
        '    else\n' ...
        '        zt=tic;\n' ...
        '        zres=agent(i).(prop).(zlist(1)).do(time,fCha,logger,env,agent,i);\n' ...
        '        zcall=1e3*toc(zt);\n' ...
        '        agent(i).(prop).result=zres;\n' ...
        '        if strcmp(cha,''f'')\n' ...
        '            zn=AF7R58ZZY_R2_REF_TAP_DATA(1,1)+1;\n' ...
        '            assert(zn+1<=size(AF7R58ZZY_R2_REF_TAP_DATA,1),''[AF7R58ZZY-R4] timing tap overflow.'');\n' ...
        '            AF7R58ZZY_R2_REF_TAP_DATA(1,1)=zn;\n' ...
        '            AF7R58ZZY_R2_REF_TAP_DATA(zn+1,:)=[double(time.t) double(''f'') 1 zcall 0 double(length(zlist))];\n' ...
        '        end\n' ...
        '        for zj=2:length(zlist)\n' ...
        '            zt=tic;\n' ...
        '            zr=agent(i).(prop).(zlist(zj)).do(time,fCha,logger,env,agent,i);\n' ...
        '            zcall=1e3*toc(zt);\n' ...
        '            zm=tic;\n' ...
        '            agent(i).(prop).result=merge_result(agent(i).(prop).result,zr);\n' ...
        '            zmerge=1e3*toc(zm);\n' ...
        '            if strcmp(cha,''f'')\n' ...
        '                zn=AF7R58ZZY_R2_REF_TAP_DATA(1,1)+1;\n' ...
        '                assert(zn+1<=size(AF7R58ZZY_R2_REF_TAP_DATA,1),''[AF7R58ZZY-R4] timing tap overflow.'');\n' ...
        '                AF7R58ZZY_R2_REF_TAP_DATA(1,1)=zn;\n' ...
        '                AF7R58ZZY_R2_REF_TAP_DATA(zn+1,:)=[double(time.t) double(''f'') double(zj) zcall zmerge double(length(zlist))];\n' ...
        '            end\n' ...
        '        end\n' ...
        '    end\n' ...
        '    return;\n' ...
        'end'],plantPat);

    R=strrep(R,plantPat,branch);

    % C4. Minimal preflight after patch construction.
    %
    % Exact source SHA + unique replacement anchors above are the authority.
    % Do NOT add diagnostic-internal count assertions that can reject a valid
    % generated branch.  We check only that the intended executable inserts
    % exist and that the original constructor remains unique.
    assert(count_exact(R,'replan_opts.ENABLE_LIGHT_RUNTIME_AUDIT = true;')==1, ...
        '[AF7R58ZZY-R4] light-audit option patch missing.');
    assert(count_exact(R,'replan_opts.SPIKE_CAPTURE_THRESHOLD_MS = 4.0;')==1, ...
        '[AF7R58ZZY-R4] spike-threshold patch missing.');
    assert(count_exact(R,'if strcmp(prop,"reference")')==1, ...
        '[AF7R58ZZY-R4] reference timing branch missing.');
    assert(count_exact(R,'zmerge=1e3*toc(zm);')==1, ...
        '[AF7R58ZZY-R4] reference merge timing insert missing.');
    assert(count_exact(R,ctor)==1, ...
        '[AF7R58ZZY-R4] replanner constructor count changed after patch.');

    % Exact backup/restore guard exists in THIS function workspace, so the
    % runner's base-workspace clearvars cannot delete it.
    restoreGuard=onCleanup(@()restore_runner_exact(pRun,origRunBytes)); %#ok<NASGU>

    write_utf8_char(pRun,R);
    rehash;
    tempRunnerSHA=file_hash(pRun);
    assert(~strcmp(tempRunnerSHA,runnerSHA), ...
        '[AF7R58ZZY-R4] temporary runner patch was not written.');

    % REPLANNING_SYSTEM must remain byte-identical.
    assert(strcmp(file_hash(pR),sourceSHA), ...
        '[AF7R58ZZY-R4] REPLANNING_SYSTEM changed unexpectedly.');

    fprintf('\n[B. TEMPORARY DIAGNOSTIC PATCH]\n');
    fprintf(' production REPLANNING_SYSTEM modified = 0\n');
    fprintf(' runner kept at original path/name      = 1\n');
    fprintf(' generated helper file                  = 0\n');
    fprintf(' exact reference components             = timevarying / sload / merge_result\n');
    fprintf(' temporary runner SHA                   = %s\n',tempRunnerSHA);

    %% D. Run exactly once.
    fprintf('\n[C. ONE FLIGHT RUN]\n');
    evalin('base',[runnerName ';']);
    OUT.flight_runs=1;

    % Capture runtime profile immediately.
    if evalin('base','exist(''AF7R58ZZB_RUNTIME_PROFILE'',''var'')')==1
        P=evalin('base','AF7R58ZZB_RUNTIME_PROFILE');
    elseif evalin('base','exist(''runtime_profile'',''var'')')==1
        P=evalin('base','runtime_profile');
    else
        error('[AF7R58ZZY-R4] runtime profile missing immediately after Flight.');
    end

    assert(evalin('base','exist(''logger'',''var'')')==1, ...
        '[AF7R58ZZY-R4] logger missing immediately after Flight.');
    L=evalin('base','logger');

    % Numeric global tap survives runner clearvars.
    global AF7R58ZZY_R2_REF_TAP_DATA
    nTap=round(AF7R58ZZY_R2_REF_TAP_DATA(1,1));
    assert(isfinite(nTap)&&nTap>0, ...
        '[AF7R58ZZY-R4] reference timing tap contains no Flight rows.');
    TAP=double(AF7R58ZZY_R2_REF_TAP_DATA(2:nTap+1,:));

    Sstage=struct();
    if evalin('base','exist(''replanner_instance'',''var'')')==1
        ro=evalin('base','replanner_instance');
        if isa(ro,'REPLANNING_SYSTEM') && ismethod(ro,'get_staged_pipeline_status')
            Sstage=ro.get_staged_pipeline_status();
        end
    end

    [tl,X,names]=extract_logger_timing(L);

    %% E. Save fixed numeric RAW before ANY summary building.
    rawDir=tempname(fileparts(pR));
    [ok,msg]=mkdir(rawDir); assert(ok,'%s',msg);
    rawPath=fullfile(rawDir,'AF7R58ZZY_R4_RAW_CAPTURE.mat');

    RAW=struct();
    RAW.schema='AF7R58ZZY_R4_RAW_CAPTURE_V1';
    RAW.source_sha=sourceSHA;
    RAW.runner_sha=runnerSHA;
    RAW.runtime_profile=P;
    RAW.reference_tap_columns={ ...
        't_s','phase_ascii','ordinal','call_ms','merge_ms','list_length'};
    RAW.reference_tap=TAP;
    RAW.logger_time=tl;
    RAW.timing_names=names;
    RAW.timing_matrix=X;
    RAW.staged_pipeline_status=Sstage;
    save(rawPath,'RAW','-v7.3');
    OUT.raw_capture_path=rawPath;

    fprintf('\n[D. RAW CAPTURE SAVED BEFORE ANALYSIS]\n');
    fprintf(' %s\n',rawPath);

    % Restore original runner immediately after raw capture.
    restore_runner_exact(pRun,origRunBytes);
    rehash;
    assert(strcmp(file_hash(pRun),runnerSHA), ...
        '[AF7R58ZZY-R4] exact runner restore failed.');
    assert(strcmp(file_hash(pR),sourceSHA), ...
        '[AF7R58ZZY-R4] production source changed unexpectedly.');

    %% F. Analyze ONLY the two target windows.
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
    fprintf(' F. REPLAN STAGE ALIGNMENT\n');
    fprintf('====================================================================================================\n');
    print_replan_alignment(G,Sstage);

    fprintf('\n====================================================================================================\n');
    fprintf(' G. STAGED PIPELINE STATUS\n');
    fprintf('====================================================================================================\n');
    print_stage(Sstage);

    OUT.status='TWO_TARGET_DEEP_SPLIT_R4_COMPLETE';

    report=fullfile(rawDir,'AF7R58ZZY_R4_REPORT.txt');
    matp=fullfile(rawDir,'AF7R58ZZY_R4_OUT.mat');
    write_report(report,OUT);
    OUT.report_path=report;
    OUT.mat_path=matp;
    save(matp,'OUT','-v7.3');
    assignin('base','AF7R58ZZY_R4_OUT',OUT);

    fprintf('\n====================================================================================================\n');
    fprintf(' H. FINISH\n');
    fprintf('====================================================================================================\n');
    fprintf(' status = %s\n',OUT.status);
    fprintf(' raw    = %s\n',rawPath);
    fprintf(' report = %s\n',report);
    fprintf(' mat    = %s\n',matp);

catch ME
    OUT.status='DIAGNOSTIC_ERROR__EXACT_RESTORE_ATTEMPTED';
    OUT.error=struct('identifier',ME.identifier,'message',ME.message,'stack',{ME.stack});

    fprintf('\n[AF7R58ZZY-R4 ERROR]\n');
    fprintf(' identifier = %s\n',ME.identifier);
    fprintf(' message    = %s\n',ME.message);
    if ~isempty(ME.stack)
        fprintf(' line       = %d\n',ME.stack(1).line);
    end
    if isfield(OUT,'raw_capture_path') && ~isempty(OUT.raw_capture_path) && ...
            isfile(OUT.raw_capture_path)
        fprintf(' RAW ALREADY SAVED: %s\n',OUT.raw_capture_path);
        fprintf(' No Flight rerun is needed for follow-up extraction.\n');
    end
end

%% I. Guaranteed cleanup / proof.
try
    if ~isempty(pRun)&&~isempty(origRunBytes)
        restore_runner_exact(pRun,origRunBytes);
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
fprintf(' flight runs               = %d\n',OUT.flight_runs);
fprintf(' status                    = %s\n',OUT.status);

% Do not clear classes or user workspace.
clear global AF7R58ZZY_R2_REF_TAP_DATA
end


function A=analyze_target(P,TAP,tl,X,names,t0,t1)
t=double(P.t(:));
ph=char(P.phase(:));

refProfile=get_profile_vector(P,'reference_s',numel(t));

idx=find(ph=='f' & t>=t0-1e-12 & t<=t1+1e-12);
assert(~isempty(idx),'[AF7R58ZZY-R4] target window has no Flight samples.');

% Numeric matrix columns:
%  1 t
%  2 online ms
%  3 full ms
%  4 reference total ms
%  5 timevarying wrapper ms
%  6 sload ms
%  7 merge_result ms
%  8 exact component sum ms
%  9 REPLANNING_SYSTEM internal light e2e ms
% 10 timevarying wrapper - internal replanner ms
% 11 BASE_CERT_GEOM ms
% 12 SENSOR_TRIGGER ms
% 13 OUTPUT_STATE ms
% 14 baseRef ms
% 15 flightGuard ms
% 16 certGuard ms
% 17 activeRef ms
% 18 geometry ms
% 19 genCritical ms
% 20 genCallback ms
% 21 requestDelta
% 22 commitDelta
% 23 deadlineDelta
M=nan(numel(idx),23);

for k=1:numel(idx)
    i=idx(k);
    tt=t(i);

    M(k,1)=tt;
    M(k,2)=1e3*get_profile(P,'online_before_plant_s',i);
    M(k,3)=1e3*get_profile(P,'full_iteration_s',i);

    if isfinite(refProfile(i))
        M(k,4)=1e3*refProfile(i);
    end

    % TAP ordinal 1=timevarying, ordinal 2=sload for Flight.
    hit=find(abs(TAP(:,1)-tt)<=1e-10*max(1,abs(tt)) & ...
             TAP(:,2)==double('f'));

    tv=NaN;
    sl=NaN;
    mg=0;
    for h=hit(:).'
        ord=round(TAP(h,3));
        if ord==1
            tv=TAP(h,4);
        elseif ord==2
            sl=TAP(h,4);
        end
        if isfinite(TAP(h,5))
            mg=mg+TAP(h,5);
        end
    end

    M(k,5)=tv;
    M(k,6)=sl;
    M(k,7)=mg;
    if isfinite(tv)||isfinite(sl)
        M(k,8)=sum([tv sl mg],'omitnan');
    end

    % Existing REPLANNING_SYSTEM internal timing mirrors.
    [dmin,j]=min(abs(tl-tt));
    if ~isempty(j)&&isfinite(dmin)&&dmin<=1e-8*max(1,abs(tt))
        % rt_light_e2e_ms is the current-cycle value.
        M(k,9)=get_timing(X,j,names,'rt_light_e2e_ms');

        if isfinite(M(k,5))&&isfinite(M(k,9))
            M(k,10)=M(k,5)-M(k,9);
        end

        % rt_spike_* fine fields are a latched snapshot. Use them only
        % when capture_time is this exact cycle; otherwise leave NaN.
        cap=get_timing(X,j,names,'rt_spike_capture_time');
        if isfinite(cap) && abs(cap-tt)<=1e-8*max(1,abs(tt))
            M(k,11)=get_timing(X,j,names,'rt_spike_stage_base_cert_geom_ms');
            M(k,12)=get_timing(X,j,names,'rt_spike_stage_sensor_trigger_ms');
            M(k,13)=get_timing(X,j,names,'rt_spike_stage_output_state_ms');

            M(k,14)=get_timing(X,j,names,'rt_spike_base_ref_ms');
            M(k,15)=get_timing(X,j,names,'rt_spike_flight_guard_ms');
            M(k,16)=get_timing(X,j,names,'rt_spike_cert_guard_ms');
            M(k,17)=get_timing(X,j,names,'rt_spike_active_ref_ms');
            M(k,18)=get_timing(X,j,names,'rt_spike_geometry_ms');
        end
    end

    M(k,19)=get_profile(P,'gen_last_cycle_critical_ms',i);
    M(k,20)=get_profile(P,'gen_last_callback_ms',i);
    M(k,21)=get_profile(P,'gen_request_delta',i);
    M(k,22)=get_profile(P,'gen_commit_delta',i);
    M(k,23)=get_profile(P,'gen_deadline_delta',i);
end

% Fallback when reference_s field is absent.
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
A.peak_component_sum_ms=M(kp,8);
A.peak_replanner_internal_ms=M(kp,9);
A.peak_timevarying_minus_internal_ms=M(kp,10);
A.peak_base_cert_geom_ms=M(kp,11);
A.peak_sensor_trigger_ms=M(kp,12);
A.peak_output_state_ms=M(kp,13);
A.peak_gen_critical_ms=M(kp,19);
A.peak_gen_callback_ms=M(kp,20);
A.peak_request_delta=M(kp,21);
A.peak_commit_delta=M(kp,22);
A.peak_deadline_delta=M(kp,23);

vals=[M(kp,11) M(kp,12) M(kp,13)];
nms={'BASE_CERT_GEOM','SENSOR_TRIGGER','OUTPUT_STATE'};
vv=vals;
vv(~isfinite(vv))=-inf;
[mx,jx]=max(vv);
if isfinite(mx)
    A.dominant_internal=nms{jx};
    A.dominant_internal_ms=mx;
else
    A.dominant_internal='NO_FINE_CAPTURE';
    A.dominant_internal_ms=NaN;
end

% Replan stage alignment from first request inside target window.
jr=find(M(:,21)>0,1,'first');
if isempty(jr)
    A.request_t_s=NaN;
    A.stage1_t_s=NaN;
    A.stage2_t_s=NaN;
    A.stage3_t_s=NaN;
else
    A.request_t_s=M(jr,1);
    A.stage1_t_s=A.request_t_s;
    A.stage2_t_s=A.request_t_s+0.025;
    A.stage3_t_s=A.request_t_s+0.050;
end
end


function print_target(A)
fprintf(' t[s] | refTot | timevarying | sload | merge | replE2E | baseCertGeom | sensorTrigger | outState | genCrit | req | commit\n');
M=A.samples;
for k=1:size(M,1)
    fprintf('%6.3f | %7.3f | %11.3f | %6.3f | %5.3f | %7.3f | %12.3f | %13.3f | %8.3f | %7.3f | %3.0f | %6.0f\n', ...
        M(k,1),M(k,4),M(k,5),M(k,6),M(k,7),M(k,9), ...
        M(k,11),M(k,12),M(k,13),M(k,19),M(k,21),M(k,22));
end

fprintf('\n PEAK REFERENCE ROW\n');
fprintf(' t                          = %.3f s\n',A.peak_t_s);
fprintf(' reference total            = %.6f ms\n',A.peak_reference_ms);
fprintf(' timevarying wrapper        = %.6f ms\n',A.peak_timevarying_ms);
fprintf(' sload                      = %.6f ms\n',A.peak_sload_ms);
fprintf(' merge_result               = %.6f ms\n',A.peak_merge_ms);
fprintf(' component sum              = %.6f ms\n',A.peak_component_sum_ms);
fprintf(' replanner internal e2e     = %.6f ms\n',A.peak_replanner_internal_ms);
fprintf(' timevarying-internal gap   = %.6f ms\n',A.peak_timevarying_minus_internal_ms);
fprintf(' BASE_CERT_GEOM             = %.6f ms\n',A.peak_base_cert_geom_ms);
fprintf(' SENSOR_TRIGGER             = %.6f ms\n',A.peak_sensor_trigger_ms);
fprintf(' OUTPUT_STATE               = %.6f ms\n',A.peak_output_state_ms);
fprintf(' dominant replanner stage   = %s | %.6f ms\n', ...
    A.dominant_internal,A.dominant_internal_ms);
fprintf(' generator critical/callback= %.6f / %.6f ms\n', ...
    A.peak_gen_critical_ms,A.peak_gen_callback_ms);
fprintf(' request/commit/deadline    = %.0f / %.0f / %.0f\n', ...
    A.peak_request_delta,A.peak_commit_delta,A.peak_deadline_delta);

r=A.samples(A.peak_row,:);
fprintf(' BASE_CERT_GEOM fine split:\n');
fprintf('   baseRef / flightGuard / certGuard / activeRef / geometry\n');
fprintf('   %.6f / %.6f / %.6f / %.6f / %.6f ms\n', ...
    r(14),r(15),r(16),r(17),r(18));
end


function print_replan_alignment(A,S)
if ~isfinite(A.request_t_s)
    fprintf(' no request found in 9.80--10.10 s\n');
    return;
end

fprintf(' request t       = %.3f s\n',A.request_t_s);
fprintf(' Stage1 expected = %.3f s\n',A.stage1_t_s);
fprintf(' Stage2 expected = %.3f s\n',A.stage2_t_s);
fprintf(' Stage3 expected = %.3f s\n',A.stage3_t_s);

M=A.samples;
times=[A.stage1_t_s A.stage2_t_s A.stage3_t_s];
labels={'STAGE1','STAGE2','STAGE3'};

for k=1:3
    [d,j]=min(abs(M(:,1)-times(k)));
    if isfinite(d)&&d<=1e-8*max(1,abs(times(k)))
        fprintf(' %-6s row: t=%.3f | ref=%.4f | timevarying=%.4f | sensorTrigger=%.4f | genCrit=%.4f | req=%.0f | commit=%.0f\n', ...
            labels{k},M(j,1),M(j,4),M(j,5),M(j,12),M(j,19),M(j,21),M(j,22));
    end
end

if isstruct(S)
    if isfield(S,'last_stage1_ms'), fprintf(' measured last_stage1_ms = %.6f\n',double(S.last_stage1_ms)); end
    if isfield(S,'last_stage2_ms'), fprintf(' measured last_stage2_ms = %.6f\n',double(S.last_stage2_ms)); end
    if isfield(S,'last_stage3_ms'), fprintf(' measured last_stage3_ms = %.6f\n',double(S.last_stage3_ms)); end
end
end


function print_stage(S)
if ~isstruct(S)||isempty(fieldnames(S))
    fprintf(' staged status unavailable\n');
    return;
end
p={'stage1_count','stage2_count','commit_count','stale_reject_count','fail_count', ...
   'last_stage1_ms','last_stage2_ms','last_stage3_ms', ...
   'trigger_kernel_skip_count','postcommit_cache_prepare_count','worker_used','last_reason'};
for k=1:numel(p)
    if isfield(S,p{k})
        v=S.(p{k});
        if isnumeric(v)&&isscalar(v)
            fprintf(' %-38s = %.9g\n',p{k},double(v));
        elseif islogical(v)&&isscalar(v)
            fprintf(' %-38s = %d\n',p{k},logical(v));
        else
            fprintf(' %-38s = %s\n',p{k},char(string(v)));
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
assert(~isempty(tl),'[AF7R58ZZY-R4] logger.Data.t unavailable.');
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
assert(any(good),'[AF7R58ZZY-R4] no finite target-window values.');
ix=find(good);
[m,j]=max(vv(good));
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
fprintf(fid,'replannerInternalMs=%.9g\n',A.peak_replanner_internal_ms);
fprintf(fid,'baseCertGeomMs=%.9g\n',A.peak_base_cert_geom_ms);
fprintf(fid,'sensorTriggerMs=%.9g\n',A.peak_sensor_trigger_ms);
fprintf(fid,'outputStateMs=%.9g\n',A.peak_output_state_ms);
fprintf(fid,'dominant=%s\n',A.dominant_internal);
fprintf(fid,'dominantMs=%.9g\n',A.dominant_internal_ms);
fprintf(fid,'genCriticalMs=%.9g\n',A.peak_gen_critical_ms);
fprintf(fid,'genCallbackMs=%.9g\n',A.peak_gen_callback_ms);
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


function s=compact_ws(s)
s=regexprep(s,'\s+','');
end


function b=read_bytes(path)
fid=fopen(path,'rb'); assert(fid>=0);
c=onCleanup(@()fclose(fid)); %#ok<NASGU>
b=fread(fid,'*uint8');
end


function write_utf8_char(path,s)
% R2: no string arrays are accepted here.
% Flatten an ordinary char matrix deterministically if ever presented.
if isstring(s)
    s=char(join(s,""));
end
assert(ischar(s),'[AF7R58ZZY-R4] write source must be char.');
if ~isrow(s)
    s=reshape(s.',1,[]);
end
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
