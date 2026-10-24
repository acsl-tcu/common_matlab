% AF7R58R50_SOURCE_BOUND_CAPTURE_R1.m
% A single-command read-only capture of the ACTUAL current source + available
% runtime evidence. It does NOT alter REPLANNING_SYSTEM, trigger, HLC, plant,
% obstacle data, certificates, or reference. No default physical parameters.
% R49 results are archived only as older evidence, NOT current-flight proof.
% Usage (run after simulation while workspace data remain):
%   run('AF7R58R50_SOURCE_BOUND_CAPTURE_R1.m');
%   R50.report_file, R50.archive_file
%
fprintf('\n[R50] CURRENT-SOURCE + RUN EVIDENCE CAPTURE | READ ONLY\n');
R50 = af7r58r50_capture();
assignin('base','R50',R50);
fprintf('[R50] source_status   = %s\n',R50.source_status);
fprintf('[R50] workspace       = %s\n',R50.workspace_status);
fprintf('[R50] legacy_pairing = %s\n',R50.source_pairing_status);
fprintf('[R50] report          = %s\n',R50.report_file);
fprintf('[R50] archive         = %s\n',R50.archive_file);

function R=af7r58r50_capture()
root=fileparts(mfilename('fullpath'));
if isempty(root),root=pwd;end
outdir=fullfile(root,['AF7R58R50_CAPTURE_' datestr(now,'yyyymmdd_HHMMSS_FFF')]);
assert(~isfolder(outdir),'[R50] Capture directory exists.');
mkdir(outdir);
R=struct();
R.schema='AF7R58R50_READONLY_SOURCE_AND_EVIDENCE_CAPTURE_V1';
R.read_only=true;R.production_modified=false;
R.commit_modified=false;R.trigger_modified=false;
R.hlc_or_plant_modified=false;
R.controller_24_95_in_scope=false;
R.current_source_bound_to_logged_flight=false;
R.executed_C7_rejoin_certified=false;
R.whole_body_continuous_certified=false;
R.current_state_trigger_certified=false;
R.replanner_10ms_proved=false;
R.full_cycle_25ms_proved=false;
R.requirement='PRE-FLIGHT-VARIABLE NOMINAL TRAJECTORY / LIVE MASS / NO CLOCK CATCHUP';
R.outdir=outdir;
R.files=struct('name',{},'resolved_path',{},'sha256',{},'bytes',{},'status',{});

% Capture the source MATLAB would choose on THIS path, not a guessed copy.
primary=which('REPLANNING_SYSTEM');
if isempty(primary),primary=fullfile(root,'REPLANNING_SYSTEM.m');end
R.primary_source_selected=primary;
R.primary_source_inmem='UNKNOWN';
try
    loaded=inmem('-completenames');
    R.primary_source_inmem=mat2str(any(strcmpi(loaded,primary)));
catch
end

names={'REPLANNING_SYSTEM.m', ...
 'af7r36_core.hpp','af7r36_single_core_mex.cpp', ...
 'af7r33_active_flat_jet_batch.m','af7r33_flat_jet_batch_model.m', ...
 'g26_future_trigger_kernel_mex.cpp', ...
 'Controller_HL_Suspended_Load.m','HLC_SUSPENDED_LOAD.m', ...
 'SimHL.m', ...
 'AF7R58R49_THREE_PARALLEL_R1.m', ...
 'AF7R58R48R2_THREE_ISSUES_AUDIT.m'};
for k=1:numel(names)
    name=names{k};
    if k==1,src=primary;
    else
        src=which(name);
        if isempty(src),src=fullfile(root,name);end
    end
    R.files(end+1)=af7r58r50_capture_file(src,outdir,name); %#ok<AGROW>
end
if strcmp(R.files(1).status,'COPIED')
    R.source_status='COPIED_CURRENT_PATH_RESOLVED_SOURCE__NOT_FLIGHT_PAIRED';
else
    R.source_status='MISSING_CURRENT_REPLANNING_SYSTEM_SOURCE';
end

% Collect actual authoritatively generated MATLAB structs if present.
% Deliberately do not serialize live handle objects or invent missing values.
keys={'R44R2','R45R1','C44R2','L45','runtime_profile', ...
      'avoidance_status','replan_opts','R49', ...
      'AF7R58R49_OUT'};
WS=struct();
R.workspace_inventory=struct('name',{},'class',{},'size',{},'status',{});
for k=1:numel(keys)
    key=keys{k};
    status='NOT_IN_BASE_WORKSPACE';cls='';sz=[];
    try
        exists=evalin('base',sprintf('exist(''%s'',''var'')==1',key));
        if exists
            val=evalin('base',key);
            cls=class(val);sz=size(val);
            if isstruct(val) || isnumeric(val) || islogical(val)
                WS.(key)=val;
                status='CAPTURED';
            else
                status='IGNORED_NON_DATA_OBJECT';
            end
        end
    catch ME
        status=['READ_FAILED:' ME.identifier];
    end
    R.workspace_inventory(end+1)=struct( ...
        'name',key,'class',cls,'size',sz,'status',status); %#ok<AGROW>
end
if isempty(fieldnames(WS))
    R.workspace_status='NO_WORKSPACE_DATA__NOT_RUN';
    workspace_file='';
else
    workspace_file=fullfile(outdir,'AF7R58R50_WORKSPACE.mat');
    try
        save(workspace_file,'WS','-v7.3');
        R.workspace_status='WORKSPACE_DATA_CAPTURED__UNPAIRED';
    catch ME
        R.workspace_status=['WORKSPACE_SAVE_FAILED:' ME.identifier];
        workspace_file='';
    end
end
R.workspace_file=workspace_file;

% Include actual R44R2 snapshot from runtime authority, when available.
R.snapshot_status='NOT_PRESENT';R.snapshot_file='';
if isfield(WS,'R44R2') && isstruct(WS.R44R2) && ...
        isscalar(WS.R44R2) && isfield(WS.R44R2,'snapshot_file')
    path=WS.R44R2.snapshot_file;
    if (ischar(path) || (isstring(path)&&isscalar(path))) && ...
            isfile(char(path))
        [~,nm,ext]=fileparts(char(path));
        f=af7r58r50_capture_file(char(path),outdir,[nm ext]);
        R.files(end+1)=f;
        R.snapshot_status=f.status;
        if strcmp(f.status,'COPIED'),R.snapshot_file=fullfile(outdir,f.name);end
    else
        R.snapshot_status='SOURCE_SNAPSHOT_NOT_FOUND';
    end
end

% Capture the most recent R49/R48/R47 MAT results for provenance. These
% results are NOT automatically assumed to be from the same code revision.
patterns={...
 'AF7R58R49_RESULTS_*','AF7R58R49_OUT.mat'; ...
 'AF7R58R48R2_RESULTS_*','AF7R58R48R2_OUT.mat'; ...
 'AF7R58R47_R3_RESULTS_*','AF7R58R47_OUT.mat'};
R.previous_results=struct('name',{},'path',{},'status',{});
for k=1:size(patterns,1)
    D=dir(fullfile(root,patterns{k,1},patterns{k,2}));
    status='NOT_FOUND';src='';
    if ~isempty(D)
        [~,ix]=max([D.datenum]);
        src=fullfile(D(ix).folder,D(ix).name);
        [~,stem,ext]=fileparts(patterns{k,2});
        dest=[stem '_PRIOR' num2str(k) ext];
        f=af7r58r50_capture_file(src,outdir,dest);
        R.files(end+1)=f;
        status=f.status;
    end
    R.previous_results(end+1)=struct( ...
        'name',patterns{k,2},'path',src,'status',status); %#ok<AGROW>
end

% Optional original full-cycle report. Only used for historical comparison.
spike=fullfile(root,'AF7R58R23_SPIKE_REPORT.txt');
if isfile(spike)
    R.files(end+1)=af7r58r50_capture_file( ...
        spike,outdir,'AF7R58R23_SPIKE_REPORT.txt');
end

% Check only source-byte equality against the prior R48 audit. Equality
% does NOT establish that a previous Flight trace was generated by that source.
R.current_vs_r48_source_hash='NOT_AVAILABLE';
R.prior_r48_source_sha256='';
current_hash=R.files(1).sha256;
for k=1:numel(R.previous_results)
    if strcmp(R.previous_results(k).name,'AF7R58R48R2_OUT.mat') && ...
            strcmp(R.previous_results(k).status,'COPIED')
        try
            S48=load(R.previous_results(k).path,'OUT');
            if isfield(S48,'OUT') && isfield(S48.OUT,'trigger') && ...
                    isfield(S48.OUT.trigger,'source_sha256')
                R.prior_r48_source_sha256=char(S48.OUT.trigger.source_sha256);
                if numel(current_hash)==64 && numel(R.prior_r48_source_sha256)==64
                    if strcmpi(current_hash,R.prior_r48_source_sha256)
                        R.current_vs_r48_source_hash='MATCH_BYTES_ONLY__NOT_FLIGHT_PAIRED';
                    else
                        R.current_vs_r48_source_hash='DIFFERENT_SOURCE_BYTES';
                    end
                end
            end
        catch
            R.current_vs_r48_source_hash='PRIOR_HASH_READ_ERROR';
        end
    end
end
R.source_pairing_status= ...
 'NOT_VERIFIED__CAPTURED_SOURCE_AND_HISTORICAL_LOG_MAY_DIFFER';
R.three_track_status=struct( ...
 'return','NEEDS_FLIGHT_SPLICE_0_TO_7_JETS_AND_ACTIVE_PHASE', ...
 'timing','NEEDS_SAME_RUN_STAGE1_STAGE2_EXCLUSIVE_QPC_BUCKETS', ...
 'trigger','NEEDS_ESTIMATE_INITIALIZED_CLOSED_LOOP_AND_PROVED_REACHABLE_TUBE');
R.recommended_next= ...
 'Review current exact source plus actual snapshot and runtime structures; no production patch until safety and source binding close.';
R.report_file=fullfile(outdir,'AF7R58R50_CAPTURE_REPORT.txt');
[~, capture_name]=fileparts(outdir);
R.archive_file=fullfile(root,[capture_name '.zip']);

fid=fopen(R.report_file,'wt');
assert(fid>=0,'[R50] Could not create report.');
cleanup=onCleanup(@()fclose(fid));
fprintf(fid,'R50 | ACTUAL SOURCE + WORKSPACE EVIDENCE | READ ONLY\n');
fprintf(fid,'production_modified=0 | trigger_modified=0 | commit_modified=0\n');
fprintf(fid,'24.95s HLC spike is out of scope\n');
fprintf(fid,'source_selected=%s\n',primary);
fprintf(fid,'source_inmem=%s\n',R.primary_source_inmem);
fprintf(fid,'source_status=%s\n',R.source_status);
fprintf(fid,'source_pairing=%s\n',R.source_pairing_status);
fprintf(fid,'current_vs_R48_source_hash=%s\n',R.current_vs_r48_source_hash);
fprintf(fid,'workspace_status=%s\n',R.workspace_status);
fprintf(fid,'snapshot_status=%s\n',R.snapshot_status);
fprintf(fid,'\n[COPIED FILE INVENTORY]\n');
for k=1:numel(R.files)
    f=R.files(k);
    fprintf(fid,'%s | %s | %d | %s\n',f.name,f.status,f.bytes,f.sha256);
end
fprintf(fid,'\n[WORKSPACE CAPTURE]\n');
for k=1:numel(R.workspace_inventory)
    w=R.workspace_inventory(k);
    fprintf(fid,'%s | %s | %s\n',w.name,w.class,w.status);
end
fprintf(fid,'\n[THREE TRACKS]\n');
fprintf(fid,'RETURN: %s\n',R.three_track_status.return);
fprintf(fid,'TIMING: %s\n',R.three_track_status.timing);
fprintf(fid,'TRIGGER: %s\n',R.three_track_status.trigger);
fprintf(fid,'No continuity, collision, heading, performance, or commit certificate is claimed.\n');
clear cleanup;

% Preserve R with paths but never declare current-flight pairing verified.
result_file=fullfile(outdir,'AF7R58R50_CAPTURE_OUT.mat');
R.result_file=result_file;
save(result_file,'R','-v7.3');

% Copy-only evidence ZIP; no README, no production writes.
entries=dir(outdir);
entries=entries(~[entries.isdir]);
files=fullfile(outdir,{entries.name});
zip(R.archive_file,files,root);
end

function F=af7r58r50_capture_file(src,outdir,name)
F=struct('name',name,'resolved_path',char(src),'sha256','', ...
    'bytes',0,'status','NOT_FOUND');
if ~isfile(src),return;end
src=char(src);name=char(name);
try
    info=dir(src);
    F.bytes=info.bytes;
    dest=fullfile(outdir,name);
    % Always copy to new capture directory, never modify original source.
    [ok,msg]=copyfile(src,dest);
    if ~ok,F.status=['COPY_FAILED:' msg];return;end
    F.sha256=af7r58r50_sha256(dest);
    F.status='COPIED';
catch ME
    F.status=['COPY_FAILED:' ME.identifier];
end
end

function value=af7r58r50_sha256(file)
value='NOT_COMPUTED';
fid=fopen(file,'rb');
if fid<0,return;end
c=onCleanup(@()fclose(fid));
try
    md=java.security.MessageDigest.getInstance('SHA-256');
    while true
        data=fread(fid,2^19,'*uint8');
        if isempty(data),break;end
        md.update(typecast(data,'int8'));
    end
    bb=typecast(md.digest(),'uint8');
    value=lower(reshape(dec2hex(bb,2).',1,[]));
catch
end
end
