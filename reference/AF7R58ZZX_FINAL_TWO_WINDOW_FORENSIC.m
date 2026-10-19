function OUT = AF7R58ZZX_FINAL_TWO_WINDOW_FORENSIC()
% AF7R58ZZX
% Final self-contained forensic for ONLY:
%   A) Flight-entry reference spike  : 4.95--5.20 s
%   B) Replan reference spike        : 9.80--10.10 s
%
% IMPORTANT
%   - Runs the flight ONCE.
%   - Does NOT modify REPLANNING_SYSTEM.m.
%   - Does NOT modify the original AF7R58ZZB runner.
%   - Creates one temporary runner copy that only enables the existing
%     built-in light runtime audit via replan_opts.
%   - Immediately saves a RAW_CAPTURE.mat BEFORE any summary analysis.
%     Therefore, if summary/report generation fails, NO rerun is needed.
%   - Ignores all unrelated irregular spikes by design.
%
% Diagnostic only. Not WCET evidence.

EXPECTED_REPLANNER_SHA = ...
    '310cfdd01f9554a80b71e8d90464816a721859709ad7c4b9137cdc0f3a6eaf49';
EXPECTED_RUNNER_SHA = ...
    '91fa19d72b13e6909d8920659422e3d0cfe169dc9c106b8c78f6d984c60cf507';

OUT=struct();
OUT.schema='AF7R58ZZX_FINAL_TWO_WINDOW_FORENSIC_V1';
OUT.production_modified=false;
OUT.original_runner_modified=false;
OUT.flight_runs=0;
OUT.status='OPEN';
OUT.error=struct();

fprintf('\n====================================================================================================\n');
fprintf(' AF7R58ZZX | FINAL TWO-WINDOW FORENSIC\n');
fprintf('====================================================================================================\n');
fprintf('[SCOPE] FlightEntry 4.95--5.20 s | Replan 9.80--10.10 s | all other spikes ignored\n');
fprintf('[AUTHORITY] productionReadOnly=1 | originalRunnerReadOnly=1 | worker=0\n');
fprintf('[ROBUSTNESS] rawCaptureSavedBeforeAnalysis=1 | oneFlightRunOnly=1\n');

tempRunner='';
tempDir='';

try
    %% A. Exact source binding.
    pR=which('REPLANNING_SYSTEM');
    assert(~isempty(pR)&&isfile(pR), ...
        '[AF7R58ZZX] REPLANNING_SYSTEM unresolved.');
    srcSHA=file_hash(pR);
    assert(strcmp(srcSHA,EXPECTED_REPLANNER_SHA), ...
        '[AF7R58ZZX] REPLANNING_SYSTEM is not exact current ZZV-R2 source.');

    runnerName='run_flight_headless_AF7R58ZZB_reference_cycle_audit';
    pRun=which(runnerName);
    assert(~isempty(pRun)&&isfile(pRun), ...
        '[AF7R58ZZX] exact AF7R58ZZB runner unresolved.');
    runSHA=file_hash(pRun);
    assert(strcmp(runSHA,EXPECTED_RUNNER_SHA), ...
        '[AF7R58ZZX] AF7R58ZZB runner differs from the current bound runner.');

    OUT.source_sha=srcSHA;
    OUT.runner_sha=runSHA;

    fprintf('\n[A. EXACT BINDING]\n');
    fprintf(' REPLANNING_SYSTEM SHA = %s\n',srcSHA);
    fprintf(' AF7R58ZZB runner SHA   = %s\n',runSHA);

    %% B. Build temporary runner; original files stay byte-identical.
    R=read_utf8(pRun);

    anchor='replanner_instance = REPLANNING_SYSTEM(agent,nominal_ref,replan_opts);';
    assert(count_exact(R,anchor)==1, ...
        '[AF7R58ZZX] replanner constructor anchor unresolved in runner.');

    diagLines=sprintf([ ...
        '%% AF7R58ZZX TEMP DIAGNOSTIC OPTIONS -- original runner is untouched\\n' ...
        'replan_opts.ENABLE_LIGHT_RUNTIME_AUDIT = true;\\n' ...
        'replan_opts.SPIKE_CAPTURE_THRESHOLD_MS = 5.0;\\n\\n']);

    Rdiag=strrep(R,anchor,[diagLines anchor]);

    tempDir=tempname(fileparts(pRun));
    [ok,msg]=mkdir(tempDir); assert(ok,'%s',msg);
    tempRunner=fullfile(tempDir, ...
        'run_flight_headless_AF7R58ZZX_TEMP_DIAGNOSTIC.m');
    write_utf8(tempRunner,Rdiag);

    % Original source invariants remain unchanged.
    assert(strcmp(file_hash(pR),srcSHA));
    assert(strcmp(file_hash(pRun),runSHA));

    fprintf('\n[B. TEMP RUNNER]\n');
    fprintf(' light audit enabled only in temporary runner options\n');
    fprintf(' spike capture threshold = 5.0 ms\n');
    fprintf(' original production files modified = 0\n');

    %% C. Run exactly once.
    assignin('base','AF7R58ZZX_TEMP_RUNNER_PATH',tempRunner);
    evalin('base','run(AF7R58ZZX_TEMP_RUNNER_PATH);');
    OUT.flight_runs=1;

    % Immediately capture required variables before any cleanup/analysis.
    if evalin('base','exist(''AF7R58ZZB_RUNTIME_PROFILE'',''var'')')==1
        P=evalin('base','AF7R58ZZB_RUNTIME_PROFILE');
    elseif evalin('base','exist(''runtime_profile'',''var'')')==1
        P=evalin('base','runtime_profile');
    else
        error('[AF7R58ZZX] runtime profile missing immediately after runner.');
    end

    assert(evalin('base','exist(''logger'',''var'')')==1, ...
        '[AF7R58ZZX] logger missing immediately after runner.');
    L=evalin('base','logger');

    Sstage=struct();
    if evalin('base','exist(''replanner_instance'',''var'')')==1
        obj=evalin('base','replanner_instance');
        if isa(obj,'REPLANNING_SYSTEM') && ...
                ismethod(obj,'get_staged_pipeline_status')
            try
                Sstage=obj.get_staged_pipeline_status();
            catch
                Sstage=struct();
            end
        end
    end

    %% D. Convert logger timing state to one fixed numeric matrix.
    [tl,X,names]=extract_logger_timing(L);

    % Save RAW data FIRST. No heterogeneous struct-array construction.
    outdir=tempname(fileparts(pR));
    [ok,msg]=mkdir(outdir); assert(ok,'%s',msg);
    rawPath=fullfile(outdir,'AF7R58ZZX_RAW_CAPTURE.mat');

    RAW=struct();
    RAW.schema='AF7R58ZZX_RAW_CAPTURE_V1';
    RAW.source_sha=srcSHA;
    RAW.runner_sha=runSHA;
    RAW.runtime_profile=P;
    RAW.logger_time=tl;
    RAW.timing_names=names;
    RAW.timing_matrix=X;
    RAW.staged_pipeline_status=Sstage;
    save(rawPath,'RAW','-v7.3');

    OUT.raw_capture_path=rawPath;
    fprintf('\n[C. RAW CAPTURE SAVED BEFORE ANALYSIS]\n');
    fprintf(' %s\n',rawPath);

    %% E. Analyze ONLY the two requested windows.
    F=analyze_window(P,tl,X,names,4.95,5.20,'FLIGHT_ENTRY');
    G=analyze_window(P,tl,X,names,9.80,10.10,'REPLAN_WINDOW');

    OUT.flight_entry=F;
    OUT.replan_window=G;
    OUT.staged_pipeline_status=Sstage;

    fprintf('\n====================================================================================================\n');
    fprintf(' D1. FLIGHT ENTRY 4.95--5.20 s\n');
    fprintf('====================================================================================================\n');
    print_window(F);

    fprintf('\n====================================================================================================\n');
    fprintf(' D2. REPLAN WINDOW 9.80--10.10 s\n');
    fprintf('====================================================================================================\n');
    print_window(G);

    fprintf('\n====================================================================================================\n');
    fprintf(' E. INTERPRETATION NUMBERS\n');
    fprintf('====================================================================================================\n');

    fprintf(' Flight-entry peak reference      = %.4f ms @ %.3f s\n', ...
        F.peak_reference_ms,F.peak_reference_t_s);
    fprintf(' Flight-entry replanner at peak   = %.4f ms\n', ...
        F.peak_reference_replanner_ms);
    fprintf(' Flight-entry reference residual  = %.4f ms\n', ...
        F.peak_reference_residual_ms);
    fprintf(' Flight-entry dominant replanner stage = %s | %.4f ms\n', ...
        F.peak_reference_dominant_stage,F.peak_reference_dominant_stage_ms);

    fprintf(' Replan peak reference            = %.4f ms @ %.3f s\n', ...
        G.peak_reference_ms,G.peak_reference_t_s);
    fprintf(' Replan replanner at peak         = %.4f ms\n', ...
        G.peak_reference_replanner_ms);
    fprintf(' Replan reference residual        = %.4f ms\n', ...
        G.peak_reference_residual_ms);
    fprintf(' Replan dominant replanner stage  = %s | %.4f ms\n', ...
        G.peak_reference_dominant_stage,G.peak_reference_dominant_stage_ms);

    if isstruct(Sstage)&&~isempty(fieldnames(Sstage))
        fprintf('\n staged pipeline summary:\n');
        print_if_field(Sstage,'stage1_count');
        print_if_field(Sstage,'stage2_count');
        print_if_field(Sstage,'commit_count');
        print_if_field(Sstage,'last_stage1_ms');
        print_if_field(Sstage,'last_stage2_ms');
        print_if_field(Sstage,'last_stage3_ms');
        print_if_field(Sstage,'trigger_kernel_skip_count');
        print_if_field(Sstage,'postcommit_cache_prepare_count');
    end

    OUT.status='FINAL_TWO_WINDOW_ATTRIBUTION_COMPLETE';

    %% F. Save finished report/output.
    report=fullfile(outdir,'AF7R58ZZX_REPORT.txt');
    matp=fullfile(outdir,'AF7R58ZZX_OUT.mat');
    write_report(report,OUT);
    OUT.report_path=report;
    OUT.mat_path=matp;
    save(matp,'OUT','-v7.3');
    assignin('base','AF7R58ZZX_OUT',OUT);

    fprintf('\n[F. OUTPUT]\n');
    fprintf(' status = %s\n',OUT.status);
    fprintf(' report = %s\n',report);
    fprintf(' mat    = %s\n',matp);
    fprintf(' raw    = %s\n',rawPath);

catch ME
    OUT.status='FORENSIC_ERROR__PRODUCTION_UNCHANGED';
    OUT.error=struct('identifier',ME.identifier,'message',ME.message,'stack',{ME.stack});
    fprintf('\n[FORENSIC ERROR]\n%s | %s\n',ME.identifier,ME.message);

    % If a raw capture was already saved, preserve and advertise it.
    if isfield(OUT,'raw_capture_path') && isfile(OUT.raw_capture_path)
        fprintf(' RAW CAPTURE ALREADY SAVED -- no rerun will be required:\n');
        fprintf(' %s\n',OUT.raw_capture_path);
    end
end

% Cleanup TEMP runner only. Never clear classes or user workspace here.
try
    evalin('base','clear AF7R58ZZX_TEMP_RUNNER_PATH');
catch
end
try
    if ~isempty(tempRunner)&&isfile(tempRunner), delete(tempRunner); end
catch
end
try
    if ~isempty(tempDir)&&isfolder(tempDir), rmdir(tempDir,'s'); end
catch
end

% Prove originals stayed unchanged.
try
    OUT.production_modified= ...
        ~strcmp(file_hash(which('REPLANNING_SYSTEM')),EXPECTED_REPLANNER_SHA);
    OUT.original_runner_modified= ...
        ~strcmp(file_hash(which('run_flight_headless_AF7R58ZZB_reference_cycle_audit')), ...
        EXPECTED_RUNNER_SHA);
catch
end

fprintf('\n final production modified = %d\n',OUT.production_modified);
fprintf(' original runner modified = %d\n',OUT.original_runner_modified);
end


function W=analyze_window(P,tl,X,names,t0,t1,label)
t=double(P.t(:));
ph=char(P.phase(:));
ref=double(P.reference_s(:));

idx=find(ph=='f' & t>=t0-1e-12 & t<=t1+1e-12 & isfinite(ref));
assert(~isempty(idx),'[AF7R58ZZX] requested window has no Flight rows.');

cols={ ...
    't_s', ...
    'online_ms', ...
    'full_ms', ...
    'reference_ms', ...
    'replanner_e2e_ms', ...
    'reference_residual_ms', ...
    'base_cert_geom_ms', ...
    'sensor_trigger_ms', ...
    'output_state_ms', ...
    'base_ref_ms', ...
    'flight_guard_ms', ...
    'cert_guard_ms', ...
    'active_ref_ms', ...
    'geometry_ms', ...
    'gen_critical_ms', ...
    'gen_callback_ms', ...
    'request_delta', ...
    'commit_delta', ...
    'deadline_delta'};

M=nan(numel(idx),numel(cols));

for k=1:numel(idx)
    i=idx(k);
    tt=t(i);
    M(k,1)=tt;
    M(k,2)=1e3*get_profile(P,'online_before_plant_s',i);
    M(k,3)=1e3*get_profile(P,'full_iteration_s',i);
    M(k,4)=1e3*get_profile(P,'reference_s',i);

    [dmin,j]=min(abs(tl-tt));
    if ~isempty(j) && isfinite(dmin) && dmin<=1e-8*max(1,abs(tt))
        M(k,5)=get_timing(X,j,names,'rt_light_e2e_ms');

        if isfinite(M(k,4))&&isfinite(M(k,5))
            M(k,6)=M(k,4)-M(k,5);
        end

        % Fine split is populated only when built-in slow-cycle capture fired.
        M(k,7)=get_timing(X,j,names,'rt_spike_stage_base_cert_geom_ms');
        M(k,8)=get_timing(X,j,names,'rt_spike_stage_sensor_trigger_ms');
        M(k,9)=get_timing(X,j,names,'rt_spike_stage_output_state_ms');
        M(k,10)=get_timing(X,j,names,'rt_spike_base_ref_ms');
        M(k,11)=get_timing(X,j,names,'rt_spike_flight_guard_ms');
        M(k,12)=get_timing(X,j,names,'rt_spike_cert_guard_ms');
        M(k,13)=get_timing(X,j,names,'rt_spike_active_ref_ms');
        M(k,14)=get_timing(X,j,names,'rt_spike_geometry_ms');
    end

    M(k,15)=get_profile(P,'gen_last_cycle_critical_ms',i);
    M(k,16)=get_profile(P,'gen_last_callback_ms',i);
    M(k,17)=get_profile(P,'gen_request_delta',i);
    M(k,18)=get_profile(P,'gen_commit_delta',i);
    M(k,19)=get_profile(P,'gen_deadline_delta',i);
end

[~,kr]=max(M(:,4));
[~,kp]=max_finite(M(:,5));

W=struct();
W.label=label;
W.t0=t0;
W.t1=t1;
W.columns=cols;
W.samples=M;
W.peak_reference_row=kr;
W.peak_reference_t_s=M(kr,1);
W.peak_reference_ms=M(kr,4);
W.peak_reference_replanner_ms=M(kr,5);
W.peak_reference_residual_ms=M(kr,6);

[W.peak_reference_dominant_stage, ...
 W.peak_reference_dominant_stage_ms]= ...
    dominant_internal_stage(M(kr,:));

if isempty(kp)||~isfinite(kp)
    W.peak_replanner_row=NaN;
    W.peak_replanner_t_s=NaN;
    W.peak_replanner_ms=NaN;
else
    W.peak_replanner_row=kp;
    W.peak_replanner_t_s=M(kp,1);
    W.peak_replanner_ms=M(kp,5);
end
end


function print_window(W)
fprintf(' Columns:\n');
fprintf(' t | online | full | reference | replannerE2E | refResidual | baseCertGeom | sensorTrigger | outState | genCritical | req | commit\n');
fprintf(' [s]   [ms]     [ms]      [ms]          [ms]          [ms]          [ms]           [ms]       [ms]          [ms]\n');

M=W.samples;
for k=1:size(M,1)
    fprintf('%6.3f %8.3f %8.3f %10.3f %13.3f %12.3f %13.3f %14.3f %9.3f %12.3f %4.0f %6.0f\n', ...
        M(k,1),M(k,2),M(k,3),M(k,4),M(k,5),M(k,6), ...
        M(k,7),M(k,8),M(k,9),M(k,15),M(k,17),M(k,18));
end

fprintf('\n Peak reference row:\n');
fprintf('   t                 = %.3f s\n',W.peak_reference_t_s);
fprintf('   reference          = %.6f ms\n',W.peak_reference_ms);
fprintf('   REPLANNING_SYSTEM  = %.6f ms\n',W.peak_reference_replanner_ms);
fprintf('   reference residual = %.6f ms\n',W.peak_reference_residual_ms);
fprintf('   dominant internal  = %s | %.6f ms\n', ...
    W.peak_reference_dominant_stage,W.peak_reference_dominant_stage_ms);

r=W.samples(W.peak_reference_row,:);
fprintf('   fine BASE_CERT_GEOM: baseRef %.6f | flightGuard %.6f | certGuard %.6f | activeRef %.6f | geometry %.6f ms\n', ...
    r(10),r(11),r(12),r(13),r(14));
end


function [name,val]=dominant_internal_stage(r)
vals=[r(7),r(8),r(9)];
names={'BASE_CERT_GEOM','SENSOR_TRIGGER','OUTPUT_STATE'};
v=vals;
v(~isfinite(v))=-inf;
[val,k]=max(v);
if ~isfinite(val)
    name='NO_FINE_CAPTURE';
    val=NaN;
else
    name=names{k};
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
assert(~isempty(tl),'[AF7R58ZZX] logger.Data.t unavailable.');
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
    'rt_spike_geometry_ms', ...
    'rt_spike_state_handle_ms', ...
    'rt_spike_write_active_output_ms', ...
    'rt_spike_state_basic_ms', ...
    'rt_spike_state_trigger_scalar_ms', ...
    'rt_spike_state_trigger_heavy_ms', ...
    'rt_spike_state_cert_ms', ...
    'rt_spike_state_meta_ms', ...
    'rt_spike_active_count', ...
    'rt_spike_need_replan'};

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
if ~any(good)
    m=NaN; k=[];
    return;
end
ix=find(good);
[m,j]=max(vv(good));
k=ix(j);
end


function print_if_field(S,nm)
if isfield(S,nm)
    v=S.(nm);
    if isnumeric(v)&&isscalar(v)
        fprintf('   %-34s = %.9g\n',nm,double(v));
    else
        fprintf('   %-34s = %s\n',nm,char(string(v)));
    end
end
end


function write_report(path,OUT)
fid=fopen(path,'wt','n','UTF-8'); assert(fid>=0);
c=onCleanup(@()fclose(fid)); %#ok<NASGU>

fprintf(fid,'status=%s\n',OUT.status);
fprintf(fid,'sourceSHA=%s\n',OUT.source_sha);
fprintf(fid,'runnerSHA=%s\n',OUT.runner_sha);
fprintf(fid,'flightRuns=%d\n',OUT.flight_runs);
fprintf(fid,'productionModified=%d\n',OUT.production_modified);
fprintf(fid,'originalRunnerModified=%d\n',OUT.original_runner_modified);
fprintf(fid,'rawCapture=%s\n',OUT.raw_capture_path);

write_window_report(fid,OUT.flight_entry);
write_window_report(fid,OUT.replan_window);
end


function write_window_report(fid,W)
fprintf(fid,'\n[%s]\n',W.label);
fprintf(fid,'peakReferenceT=%.9g\n',W.peak_reference_t_s);
fprintf(fid,'peakReferenceMs=%.9g\n',W.peak_reference_ms);
fprintf(fid,'peakReferenceReplannerMs=%.9g\n',W.peak_reference_replanner_ms);
fprintf(fid,'peakReferenceResidualMs=%.9g\n',W.peak_reference_residual_ms);
fprintf(fid,'peakDominantInternal=%s\n',W.peak_reference_dominant_stage);
fprintf(fid,'peakDominantInternalMs=%.9g\n',W.peak_reference_dominant_stage_ms);
fprintf(fid,'columns=%s\n',strjoin(W.columns,','));
for k=1:size(W.samples,1)
    fprintf(fid,'row=');
    fprintf(fid,' %.9g',W.samples(k,:));
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


function s=read_utf8(path)
fid=fopen(path,'rb'); assert(fid>=0);
c=onCleanup(@()fclose(fid)); %#ok<NASGU>
b=fread(fid,'*uint8');
s=native2unicode(b.','UTF-8');
end


function write_utf8(path,s)
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
