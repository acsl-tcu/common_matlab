function OUT = AF7R58ZZY_R24_EXHAUSTIVE_POSTRUN_AUDIT()
% AF7R58ZZY_R24_EXHAUSTIVE_POSTRUN_AUDIT
% =========================================================================
% PURPOSE
%   Exhaustive READ-ONLY post-run capture for the current R23/R21/R19
%   replanning-performance investigation.
%
% RUN IMMEDIATELY AFTER the normal R23 run:
%
%   >> AF7R58ZZY_R23_REPORT
%   >> OUT = AF7R58ZZY_R24_EXHAUSTIVE_POSTRUN_AUDIT;
%
% This file:
%   - DOES NOT rerun Flight.
%   - DOES NOT call replanner.do().
%   - DOES NOT patch/install/modify production source.
%   - DOES NOT change the safety/certificate/commit logic.
%   - Captures the ACTUAL post-run object, runtime_profile, all known
%     performance/status getters, readable object state, source/path/SHA
%     bindings, loaded common_matlab functions, every numeric timing table,
%     and source timing/caching/commit hot-path lines.
%
% OUTPUT
%   One ZIP is printed at the end. Return that ZIP.
%
% IMPORTANT
%   Raw structures are retained in MAT files even when REPORT.txt only prints
%   a summary. Numeric arrays/timing rings are separately exported as CSV,
%   so no measured timing table is intentionally discarded.
% =========================================================================

clc;
fprintf('\n');
fprintf('====================================================================================================\n');
fprintf(' AF7R58ZZY-R24 | EXHAUSTIVE POST-RUN ACCOUNTING CAPTURE\n');
fprintf('====================================================================================================\n');
fprintf('[AUTHORITY] readOnly=1 | flightRerun=0 | productionModified=0 | doCalled=0 | safetyMathChanged=0\n');

assert(evalin('base','exist(''replanner_instance'',''var'')==1'), ...
    '[R24] replanner_instance is missing. Run the normal R23 test first.');
assert(evalin('base','exist(''runtime_profile'',''var'')==1'), ...
    '[R24] runtime_profile is missing. Run the normal R23 test first.');

obj = evalin('base','replanner_instance');
P   = evalin('base','runtime_profile');

assert(isobject(obj),'[R24] replanner_instance is not an object.');
assert(isstruct(P) && isscalar(P),'[R24] runtime_profile is not a scalar struct.');

runnerPath = which('run_flight_headless');
if isempty(runnerPath)
    root = pwd;
else
    root = fileparts(runnerPath);
end

outdir = tempname(root);
[ok,msg] = mkdir(outdir);
assert(ok,'[R24] cannot create output directory: %s',msg);
mkdir(fullfile(outdir,'sources'));
mkdir(fullfile(outdir,'tables'));
mkdir(fullfile(outdir,'reports'));

reportPath = fullfile(outdir,'REPORT.txt');
fid = fopen(reportPath,'wt','n','UTF-8');
assert(fid>=0,'[R24] cannot create REPORT.txt.');
cleanupReport = onCleanup(@()safe_fclose(fid)); %#ok<NASGU>

OUT = struct();
OUT.schema = 'AF7R58ZZY_R24_EXHAUSTIVE_POSTRUN_AUDIT_V1';
OUT.read_only = true;
OUT.flight_rerun = false;
OUT.production_modified = false;
OUT.replanner_do_called = false;
OUT.matlab_version = version;
OUT.computer = computer;
OUT.timestamp = char(datetime('now','Format','yyyy-MM-dd HH:mm:ss.SSS'));
OUT.root = root;
OUT.output_directory = outdir;
OUT.object_class = class(obj);
OUT.runtime_profile = P;

say(fid,'');
say(fid,'[A. EXACT ACTIVE OBJECT / SESSION]');
say(fid,' active object class                    = %s',OUT.object_class);
say(fid,' MATLAB                                 = %s',OUT.matlab_version);
say(fid,' computer                               = %s',OUT.computer);
say(fid,' root                                   = %s',root);
say(fid,' capture timestamp                      = %s',OUT.timestamp);

% -------------------------------------------------------------------------
% A1. Inheritance / meta-class audit
% -------------------------------------------------------------------------
[OUT.class_lineage,OUT.meta_summary] = class_lineage_and_meta(obj);
say(fid,' class lineage                          = %s',strjoin(OUT.class_lineage,' <- '));
write_meta_report(fid,OUT.meta_summary);

% -------------------------------------------------------------------------
% A2. Loaded functions / MEX inventory
% -------------------------------------------------------------------------
loadedM = {};
loadedMex = {};
try
    [loadedM,loadedMex] = inmem('-completenames');
catch
    try
        loadedM = inmem('-completenames');
    catch
        loadedM = {};
    end
end
OUT.loaded_m = loadedM;
OUT.loaded_mex = loadedMex;

loadedRoot = {};
allLoaded = [force_cellstr(loadedM); force_cellstr(loadedMex)];
for k=1:numel(allLoaded)
    p = allLoaded{k};
    if is_under_root(p,root)
        loadedRoot{end+1,1} = p; %#ok<AGROW>
    end
end
loadedRoot = unique(loadedRoot,'stable');
OUT.loaded_under_root = loadedRoot;
say(fid,' loaded files under root                = %d',numel(loadedRoot));

% -------------------------------------------------------------------------
% B. Known read-only status getters
% -------------------------------------------------------------------------
say(fid,'');
say(fid,'[B. ALL KNOWN READ-ONLY STATUS GETTERS]');

getterNames = { ...
    'get_staged_pipeline_status', ...
    'get_r23_performance_status', ...
    'get_r21_performance_status', ...
    'get_r19_performance_status', ...
    'get_last_replan_status', ...
    'get_avoidance_mode_status', ...
    'get_runtime_certificate_status', ...
    'get_observer_hard_support_status', ...
    'get_last_trigger_event', ...
    'get_first_trigger_event', ...
    'get_avoidance_commit_policy', ...
    'get_avoidance_certificate_mode', ...
    'get_auto_replan_commit_enabled', ...
    'get_reference_only_certificate_enabled', ...
    'get_execution_aware_certificate_enabled'};

OUT.status = struct();
OUT.status_errors = struct();

for k=1:numel(getterNames)
    nm = getterNames{k};
    key = matlab.lang.makeValidName(nm);
    if ismethod(obj,nm)
        try
            OUT.status.(key) = feval(nm,obj);
            say(fid,' %-42s = CAPTURED',nm);
        catch ME
            OUT.status_errors.(key) = struct( ...
                'identifier',ME.identifier,'message',ME.message);
            say(fid,' %-42s = ERROR | %s | %s',nm,ME.identifier,ME.message);
        end
    else
        say(fid,' %-42s = NOT_PRESENT',nm);
    end
end

% Explicit properties used to identify current authority/commit timing.
propNames = { ...
    'last_replan_commit_time','prediction_dt','hard_modify_deadline_s', ...
    'active_reference_version','active_reference_mode','AVOIDANCE_COMMIT_POLICY', ...
    'ENABLE_STAGED_REPLAN_PIPELINE','reference_only_commit_count', ...
    'replan_commit_count','generator_request_count','generator_deadline_fail_count'};

OUT.explicit_properties = struct();
for k=1:numel(propNames)
    nm = propNames{k};
    key = matlab.lang.makeValidName(nm);
    try
        OUT.explicit_properties.(key) = obj.(nm);
        say(fid,' property %-34s = %s',nm,value_one_line(obj.(nm)));
    catch ME
        OUT.explicit_properties.(key) = struct('unreadable',true, ...
            'identifier',ME.identifier,'message',ME.message);
        say(fid,' property %-34s = UNREADABLE',nm);
    end
end

% -------------------------------------------------------------------------
% C. Exhaustive object-state snapshot.
% STRUCT(obj) is attempted because it can expose stored diagnostic rings that
% are not public properties. Object/handle-valued branches are summarized,
% never traversed recursively.
% -------------------------------------------------------------------------
say(fid,'');
say(fid,'[C. EXHAUSTIVE OBJECT STATE SNAPSHOT]');
OUT.object_struct_conversion_succeeded = false;
OUT.object_struct_conversion_error = struct();
OUT.object_safe_snapshot = struct();

try
    oldWarn = warning('query','MATLAB:structOnObject');
    warning('off','MATLAB:structOnObject');
    warnCleanup = onCleanup(@()restore_warning(oldWarn)); %#ok<NASGU>
    rawObj = struct(obj);
    OUT.object_struct_conversion_succeeded = true;
    OUT.object_safe_snapshot = sanitize_value(rawObj,0,8);
    say(fid,' struct(obj) conversion                  = SUCCESS');
catch ME
    OUT.object_struct_conversion_error = struct( ...
        'identifier',ME.identifier,'message',ME.message);
    say(fid,' struct(obj) conversion                  = FAILED | %s | %s', ...
        ME.identifier,ME.message);
end

if OUT.object_struct_conversion_succeeded
    objFieldCount = numel(fieldnames(OUT.object_safe_snapshot));
else
    objFieldCount = 0;
end
say(fid,' captured top-level object fields         = %d',objFieldCount);

% -------------------------------------------------------------------------
% D. Runtime profile: export every per-cycle field and identify all peaks.
% -------------------------------------------------------------------------
say(fid,'');
say(fid,'[D. COMPLETE RUNTIME_PROFILE ACCOUNTING]');
R = analyze_runtime_profile(P,fid,outdir);
OUT.runtime_analysis = R;

% -------------------------------------------------------------------------
% E. Correlate exact commit time and stage/status timing tables.
% -------------------------------------------------------------------------
say(fid,'');
say(fid,'[E. COMMIT / STAGE CORRELATION]');
commitTime = resolve_commit_time(OUT,obj);
OUT.commit_time = commitTime;
say(fid,' resolved commit time [s]                 = %.17g',commitTime);

if isfinite(commitTime) && isfield(P,'t')
    write_runtime_window(P,commitTime-0.20,commitTime+0.35, ...
        fullfile(outdir,'tables','runtime_commit_window.csv'));
    say(fid,' commit runtime window                   = [%.6f, %.6f] s', ...
        commitTime-0.20,commitTime+0.35);
else
    say(fid,' commit runtime window                   = UNAVAILABLE');
end

% -------------------------------------------------------------------------
% F. Export ALL numeric arrays/timing rings from statuses and object snapshot.
% -------------------------------------------------------------------------
say(fid,'');
say(fid,'[F. RAW NUMERIC TABLE EXPORT]');
tableManifest = {};
tableManifest = [tableManifest; ...
    export_numeric_tree(OUT.status,'status',fullfile(outdir,'tables'))]; %#ok<AGROW>
if OUT.object_struct_conversion_succeeded
    tableManifest = [tableManifest; ...
        export_numeric_tree(OUT.object_safe_snapshot,'object',fullfile(outdir,'tables'))]; %#ok<AGROW>
end
tableManifest = [tableManifest; ...
    export_numeric_tree(P,'runtime_profile_raw',fullfile(outdir,'tables'))]; %#ok<AGROW>
OUT.table_manifest = tableManifest;
say(fid,' exported numeric arrays/tables           = %d',numel(tableManifest));
for k=1:numel(tableManifest)
    say(fid,'  TABLE %04d | %s',k,tableManifest{k});
end

% -------------------------------------------------------------------------
% G. Source/path/SHA authority. Capture targeted files plus every currently
% loaded file below common_matlab. Duplicate paths are removed.
% -------------------------------------------------------------------------
say(fid,'');
say(fid,'[G. SOURCE / PATH / SHA AUTHORITY]');

targetNames = unique([{ ...
    'run_flight_headless', ...
    'AF7R58ZZY_R23_REPORT', ...
    'REPLANNING_SYSTEM_R23_COMPILED', ...
    'REPLANNING_SYSTEM_R21_BALANCED', ...
    'REPLANNING_SYSTEM_R19_FAST', ...
    'REPLANNING_SYSTEM', ...
    'TIME_VARYING_REFERENCE', ...
    'af7r76_compile_correction_packet_R23', ...
    'af7r76_compiled_eval_mex', ...
    'af7r75_posaccel_batch_R19', ...
    'af7r75_posaccel_core_R19', ...
    'af7r73_posaccel_core_R17', ...
    'af7r72_posaccel_core_R16', ...
    'af7r33_flat_jet_batch_model', ...
    'af7r58i_eval_canonical_reference', ...
    'af7r58i_prepare_batch_model', ...
    'af7r58i_commit_spacing_guard'}, ...
    OUT.class_lineage(:).'],'stable');

[OUT.sources,sourceFiles] = capture_sources(targetNames,loadedRoot,outdir,fid);

% -------------------------------------------------------------------------
% H. Source static scan: every line relevant to runtime, staging, packet
% build/eval, cache, request, commit, timing, MEX, assignment, or safety gate.
% This is not a timing claim. It ensures no obvious execution owner is omitted.
% -------------------------------------------------------------------------
say(fid,'');
say(fid,'[H. STATIC HOT-PATH SOURCE SCAN]');
scanPath = fullfile(outdir,'reports','SOURCE_HOTPATH_LINES.txt');
OUT.source_scan = scan_sources(sourceFiles,scanPath);
say(fid,' scanned source files                    = %d',numel(sourceFiles));
say(fid,' matching hot-path source lines          = %d',OUT.source_scan.match_count);
say(fid,' source scan file                        = %s',scanPath);

% -------------------------------------------------------------------------
% I. Workspace presence and lightweight summaries. We intentionally do not
% serialize handle-heavy agent/logger objects into OUT.
% -------------------------------------------------------------------------
say(fid,'');
say(fid,'[I. BASE WORKSPACE INVENTORY]');
W = evalin('base','whos');
OUT.base_whos = W;
for k=1:numel(W)
    say(fid,' %-36s | %-28s | %s | %.0f bytes', ...
        W(k).name,W(k).class,mat2str(W(k).size),double(W(k).bytes));
end

% -------------------------------------------------------------------------
% J. Existing reports beside the runner are copied read-only for comparison.
% -------------------------------------------------------------------------
say(fid,'');
say(fid,'[J. EXISTING R-SERIES REPORT SNAPSHOT]');
reportFiles = dir(fullfile(root,'AF7R58ZZY_R*_REPORT.txt'));
OUT.existing_reports = struct('name',{},'path',{},'sha256',{});
for k=1:numel(reportFiles)
    src = fullfile(reportFiles(k).folder,reportFiles(k).name);
    dst = fullfile(outdir,'reports',reportFiles(k).name);
    try
        copyfile(src,dst);
        h = sha_file(src);
        OUT.existing_reports(end+1) = struct( ... %#ok<AGROW>
            'name',reportFiles(k).name,'path',src,'sha256',h);
        say(fid,' copied %-42s | SHA=%s',reportFiles(k).name,h);
    catch ME
        say(fid,' copy failed %-38s | %s',reportFiles(k).name,ME.message);
    end
end
say(fid,' R-series reports copied                 = %d',numel(OUT.existing_reports));

% -------------------------------------------------------------------------
% K. Conservative interpretation contract.
% -------------------------------------------------------------------------
say(fid,'');
say(fid,'[K. INTERPRETATION CONTRACT]');
say(fid,' readOnly                                = 1');
say(fid,' flightRerun                             = 0');
say(fid,' productionModified                      = 0');
say(fid,' replannerDoCalled                       = 0');
say(fid,' trajectoryMathChanged                   = 0');
say(fid,' certificateAcceptanceChanged            = 0');
say(fid,' commitPolicyChanged                     = 0');
say(fid,' sourceDiskSHAEqualsLoadedBytesClaim     = 0');
say(fid,' timingTablesRetainedRaw                 = 1');
say(fid,' runtimeProfileRetainedRaw               = 1');
say(fid,' objectDiagnosticStateRetainedWhenReadable = %d', ...
    OUT.object_struct_conversion_succeeded);
say(fid,' NOTE: This capture diagnoses the existing run. It does not claim a 4-ms or 25-ms pass.');

% Save raw data before closing report.
rawPath = fullfile(outdir,'RAW_CAPTURE.mat');
save(rawPath,'OUT','-v7.3');

% Also save status/object/runtime separately for easier loading.
STATUS = OUT.status; %#ok<NASGU>
save(fullfile(outdir,'STATUS_ALL.mat'),'STATUS','-v7.3');
OBJECT_SAFE = OUT.object_safe_snapshot; %#ok<NASGU>
save(fullfile(outdir,'OBJECT_SAFE_SNAPSHOT.mat'),'OBJECT_SAFE','-v7.3');
RUNTIME_PROFILE = P; %#ok<NASGU>
save(fullfile(outdir,'RUNTIME_PROFILE_RAW.mat'),'RUNTIME_PROFILE','-v7.3');

say(fid,'');
say(fid,'[RETURN]');
say(fid,' raw capture                            = %s',rawPath);
say(fid,' report                                 = %s',reportPath);
say(fid,' Return ONE ZIP printed below.');

% Close report before zipping.
clear cleanupReport;
safe_fclose(fid);

zipPath = [outdir '_AF7R58ZZY_R24_EXHAUSTIVE.zip'];
zip(zipPath,{'REPORT.txt','RAW_CAPTURE.mat','STATUS_ALL.mat', ...
    'OBJECT_SAFE_SNAPSHOT.mat','RUNTIME_PROFILE_RAW.mat', ...
    'sources','tables','reports'},outdir);

OUT.return_zip = zipPath;
assignin('base','AF7R58ZZY_R24_EXHAUSTIVE_OUT',OUT);

fprintf('\n====================================================================================================\n');
fprintf(' AF7R58ZZY-R24 CAPTURE COMPLETE\n');
fprintf('====================================================================================================\n');
fprintf('active class  = %s\n',OUT.object_class);
fprintf('tables        = %d\n',numel(tableManifest));
fprintf('source files  = %d\n',numel(sourceFiles));
fprintf('commit time   = %.17g s\n',commitTime);
fprintf('\nRETURN THIS ONE FILE:\n%s\n',zipPath);
fprintf('====================================================================================================\n');
end

% =========================================================================
% Runtime profile analysis
% =========================================================================
function R = analyze_runtime_profile(P,fid,outdir)
R = struct();
assert(isfield(P,'t'),'[R24] runtime_profile.t missing.');
t = double(P.t(:));
N = numel(t);
R.N = N;

if isfield(P,'phase')
    phase = P.phase;
    if ischar(phase)
        phase = phase(:);
        flight = (phase=='f');
    else
        phase = string(phase(:));
        flight = strcmpi(phase,'f');
    end
else
    phase = repmat('?',N,1);
    flight = true(N,1);
end
flight = flight(:) & isfinite(t);
R.flight_count = nnz(flight);

fn = fieldnames(P);
cycleFields = {};
for k=1:numel(fn)
    v = P.(fn{k});
    if (isnumeric(v)||islogical(v)) && isvector(v) && numel(v)==N
        cycleFields{end+1} = fn{k}; %#ok<AGROW>
    end
end
R.per_cycle_numeric_fields = cycleFields;
say(fid,' samples                                = %d',N);
say(fid,' Flight samples                         = %d',R.flight_count);
say(fid,' per-cycle numeric fields               = %s',strjoin(cycleFields,', '));

% Export every per-cycle numeric field.
T = table(t,'VariableNames',{'t'});
if ischar(phase)
    T.phase = phase;
else
    T.phase = cellstr(string(phase));
end
for k=1:numel(cycleFields)
    nm = cycleFields{k};
    if strcmp(nm,'t'), continue; end
    vn = matlab.lang.makeUniqueStrings(matlab.lang.makeValidName(nm), ...
        T.Properties.VariableNames);
    T.(vn) = double(P.(nm)(:));
end
writetable(T,fullfile(outdir,'tables','runtime_profile_all_cycles.csv'));

if ~isfield(P,'online_before_plant_s')
    say(fid,' online_before_plant_s                 = MISSING');
    return;
end
online = 1e3*double(P.online_before_plant_s(:));
valid = flight & isfinite(online);
R.flight_online_median_ms = median(online(valid),'omitnan');
R.flight_online_max_ms = max(online(valid),[],'omitnan');
R.over25_count = nnz(valid & online>25);
R.over4_count = nnz(valid & online>4);

say(fid,' Flight online median [ms]              = %.9f',R.flight_online_median_ms);
say(fid,' Flight online max [ms]                 = %.9f',R.flight_online_max_ms);
say(fid,' Flight online >25ms count              = %d',R.over25_count);
say(fid,' Flight online >4ms count               = %d',R.over4_count);

idx = find(valid);
[~,ord] = sort(online(idx),'descend');
idxTop = idx(ord(1:min(40,numel(ord))));
R.top_indices = idxTop;
R.top_times = t(idxTop);
R.top_online_ms = online(idxTop);

say(fid,'');
say(fid,' TOP ONLINE CYCLES (all per-cycle fields):');
for j=1:numel(idxTop)
    ii = idxTop(j);
    print_runtime_row(fid,P,ii,cycleFields);
end

% All control-period violations.
idx25 = find(valid & online>25);
R.over25_indices = idx25;
if ~isempty(idx25)
    say(fid,'');
    say(fid,' ALL >25ms ONLINE CYCLES:');
    for j=1:numel(idx25)
        print_runtime_row(fid,P,idx25(j),cycleFields);
    end
end

% Distinct peak centers, separated by >= 0.15 s, then export +/-0.15 s.
selected = [];
for j=1:numel(idxTop)
    ii=idxTop(j);
    if isempty(selected) || all(abs(t(ii)-t(selected))>=0.15)
        selected(end+1)=ii; %#ok<AGROW>
    end
    if numel(selected)>=8, break; end
end
R.distinct_peak_indices = selected;
for j=1:numel(selected)
    tc=t(selected(j));
    ttag=strrep(sprintf('%.3f',tc),'.','p');
    fnm=sprintf('runtime_peak_%02d_t_%s.csv',j,ttag);
    write_runtime_window(P,tc-0.15,tc+0.15, ...
        fullfile(outdir,'tables',fnm));
end
end

function print_runtime_row(fid,P,ii,cycleFields)
t = double(P.t(ii));
parts = cell(1,0);
for k=1:numel(cycleFields)
    nm=cycleFields{k};
    if strcmp(nm,'t'), continue; end
    v=P.(nm);
    x=double(v(ii));
    if endsWith(lower(nm),'_s')
        parts{end+1}=sprintf('%s=%.6fms',nm,1e3*x); %#ok<AGROW>
    else
        parts{end+1}=sprintf('%s=%.9g',nm,x); %#ok<AGROW>
    end
end
say(fid,' t=%.17g | %s',t,strjoin(parts,' | '));
end

function write_runtime_window(P,t0,t1,path)
if ~isfield(P,'t'), return; end
t=double(P.t(:));
N=numel(t);
mask=isfinite(t)&t>=t0&t<=t1;
ix=find(mask);
if isempty(ix), return; end
fn=fieldnames(P);
T=table(t(ix),'VariableNames',{'t'});
if isfield(P,'phase')
    ph=P.phase;
    if ischar(ph)
        ph=ph(:);
        T.phase=ph(ix);
    else
        ph=string(ph(:));
        T.phase=cellstr(ph(ix));
    end
end
for k=1:numel(fn)
    nm=fn{k};
    if strcmp(nm,'t')||strcmp(nm,'phase'), continue; end
    v=P.(nm);
    if (isnumeric(v)||islogical(v))&&isvector(v)&&numel(v)==N
        vn=matlab.lang.makeUniqueStrings(matlab.lang.makeValidName(nm), ...
            T.Properties.VariableNames);
        T.(vn)=double(v(ix));
    end
end
writetable(T,path);
end

% =========================================================================
% Status/object numeric export
% =========================================================================
function manifest = export_numeric_tree(v,prefix,dirpath)
manifest = {};
walk(v,prefix,0);

    function walk(x,path,depth)
        if depth>10, return; end
        if isnumeric(x) || islogical(x)
            if isempty(x), return; end
            if ismatrix(x)
                safe = safe_filename(path);
                f = fullfile(dirpath,[safe '.csv']);
                try
                    writematrix(double(x),f);
                    manifest{end+1,1}=f; %#ok<AGROW>
                catch
                end
            end
            return;
        end
        if isstruct(x)
            for ii=1:numel(x)
                fns=fieldnames(x(ii));
                for jj=1:numel(fns)
                    p2=sprintf('%s__%s',path,fns{jj});
                    if numel(x)>1
                        p2=sprintf('%s__idx%d',p2,ii);
                    end
                    walk(x(ii).(fns{jj}),p2,depth+1);
                end
            end
            return;
        end
        if iscell(x)
            for ii=1:numel(x)
                walk(x{ii},sprintf('%s__cell%d',path,ii),depth+1);
            end
        end
    end
end

% =========================================================================
% Source capture
% =========================================================================
function [S,sourceFiles] = capture_sources(targetNames,loadedRoot,outdir,fid)
S = struct('name',{},'resolved_path',{},'path_alternatives',{}, ...
    'sha256',{},'bytes',{},'kind',{},'snapshot_file',{});
sourceFiles = {};

% Targeted names.
for k=1:numel(targetNames)
    nm=char(targetNames{k});
    p=which(nm);
    allp=which_all(nm);
    entry=make_source_entry(nm,p,allp,outdir);
    S(end+1)=entry; %#ok<AGROW>
    report_source_entry(fid,entry);
    if strcmp(entry.kind,'SOURCE')
        sourceFiles{end+1,1}=entry.resolved_path; %#ok<AGROW>
    end
end

% Every loaded file under root not already represented.
known={S.resolved_path};
for k=1:numel(loadedRoot)
    p=loadedRoot{k};
    if isempty(p)||any(strcmpi(known,p))||~isfile(p), continue; end
    [~,nm,ext]=fileparts(p);
    entry=make_source_entry([nm ext],p,{p},outdir);
    S(end+1)=entry; %#ok<AGROW>
    report_source_entry(fid,entry);
    known{end+1}=p; %#ok<AGROW>
    if strcmp(entry.kind,'SOURCE')
        sourceFiles{end+1,1}=p; %#ok<AGROW>
    end
end
sourceFiles=unique(sourceFiles,'stable');
end

function E=make_source_entry(nm,p,allp,outdir)
E=struct('name',nm,'resolved_path',p,'path_alternatives',{allp}, ...
    'sha256','','bytes',NaN,'kind','MISSING','snapshot_file','');
if isempty(p)||~isfile(p), return; end
D=dir(p);
E.bytes=D.bytes;
E.sha256=sha_file(p);
[~,base,ext]=fileparts(p);
if any(strcmpi(ext,{'.m','.cpp','.c','.cc','.h','.hpp','.txt'}))
    E.kind='SOURCE';
    dst=fullfile(outdir,'sources',[safe_filename(base) '__' E.sha256(1:12) ext]);
    try
        copyfile(p,dst);
        E.snapshot_file=dst;
    catch
        E.snapshot_file='';
    end
else
    E.kind='BINARY_HASH_ONLY';
end
end

function report_source_entry(fid,E)
say(fid,' %s',E.name);
say(fid,'   kind=%s | bytes=%.0f | SHA=%s',E.kind,E.bytes,E.sha256);
say(fid,'   resolved=%s',E.resolved_path);
for j=1:numel(E.path_alternatives)
    say(fid,'   candidate=%s',E.path_alternatives{j});
end
end

function A=which_all(nm)
try
    A=which(nm,'-all');
catch
    A={};
end
A=force_cellstr(A);
end

% =========================================================================
% Source scan
% =========================================================================
function R=scan_sources(sourceFiles,outPath)
keys = { ...
    'tic','toc','runtime','timing','qpc','stage1','stage2','stage3', ...
    'staged_replan','advance_staged','start_staged','request','commit', ...
    'packet','cache','compiled','mex','evaluate_active','reference_packet', ...
    'prepare','material','assignment','assign','fresh','stale','deadline', ...
    'certificate','observer','sensor','trigger','replan','do('};

fid=fopen(outPath,'wt','n','UTF-8');
assert(fid>=0,'cannot create source scan');
c=onCleanup(@()fclose(fid)); %#ok<NASGU>
count=0;
for k=1:numel(sourceFiles)
    p=sourceFiles{k};
    if ~isfile(p), continue; end
    try
        txt=fileread(p);
    catch
        continue;
    end
    lines=regexp(txt,'\r\n|\n|\r','split');
    fprintf(fid,'\n====================================================================================================\n');
    fprintf(fid,'%s\n',p);
    fprintf(fid,'====================================================================================================\n');
    for i=1:numel(lines)
        low=lower(lines{i});
        hit=false;
        for j=1:numel(keys)
            if contains(low,keys{j})
                hit=true; break;
            end
        end
        if hit
            count=count+1;
            fprintf(fid,'L%-7d %s\n',i,lines{i});
        end
    end
end
R=struct('match_count',count,'file',outPath);
end

% =========================================================================
% Object snapshot
% =========================================================================
function y=sanitize_value(x,depth,maxDepth)
if depth>maxDepth
    y=struct('__truncated__',true,'class',class(x),'size',size(x));
    return;
end

if isnumeric(x)||islogical(x)||ischar(x)||isstring(x)
    y=x;
    return;
end
if isa(x,'function_handle')
    y=struct('__function_handle__',func2str(x));
    return;
end
if isobject(x)
    y=struct('__object__',true,'class',class(x),'size',size(x));
    return;
end
if isstruct(x)
    if isempty(x)
        y=x;
        return;
    end
    y=[];
    for ii=1:numel(x)
        f=fieldnames(x(ii));
        z=struct();
        for jj=1:numel(f)
            try
                z.(f{jj})=sanitize_value(x(ii).(f{jj}),depth+1,maxDepth);
            catch ME
                z.(f{jj})=struct('__capture_error__',ME.message);
            end
        end
        if ii==1
            y=repmat(z,size(x));
        else
            y(ii)=z;
        end
    end
    return;
end
if iscell(x)
    y=cell(size(x));
    for ii=1:numel(x)
        y{ii}=sanitize_value(x{ii},depth+1,maxDepth);
    end
    return;
end
try
    y=struct('__unsupported__',true,'class',class(x),'size',size(x));
catch
    y=struct('__unsupported__',true);
end
end

% =========================================================================
% Meta class
% =========================================================================
function [lineage,M]=class_lineage_and_meta(obj)
mc=metaclass(obj);
lineage={};
queue={mc};
seen={};
while ~isempty(queue)
    q=queue{1}; queue(1)=[];
    if isempty(q)||any(strcmp(seen,q.Name)), continue; end
    seen{end+1}=q.Name; %#ok<AGROW>
    lineage{end+1}=q.Name; %#ok<AGROW>
    supers=q.SuperclassList;
    if ~isempty(supers)
        if iscell(supers)
            for k=1:numel(supers), queue{end+1}=supers{k}; end %#ok<AGROW>
        else
            for k=1:numel(supers), queue{end+1}=supers(k); end %#ok<AGROW>
        end
    end
end

M=struct();
M.class=mc.Name;
M.properties=struct('name',{},'getAccess',{},'setAccess',{}, ...
    'dependent',{},'transient',{},'hidden',{});
PL=mc.PropertyList;
for k=1:numel(PL)
    p=PL(k);
    if iscell(PL), p=PL{k}; end
    M.properties(end+1)=struct( ... %#ok<AGROW>
        'name',p.Name, ...
        'getAccess',access_text(p.GetAccess), ...
        'setAccess',access_text(p.SetAccess), ...
        'dependent',logical(p.Dependent), ...
        'transient',logical(p.Transient), ...
        'hidden',logical(p.Hidden));
end
M.methods=struct('name',{},'access',{},'static',{},'abstract',{},'hidden',{});
ML=mc.MethodList;
for k=1:numel(ML)
    m=ML(k);
    if iscell(ML), m=ML{k}; end
    M.methods(end+1)=struct( ... %#ok<AGROW>
        'name',m.Name, ...
        'access',access_text(m.Access), ...
        'static',logical(m.Static), ...
        'abstract',logical(m.Abstract), ...
        'hidden',logical(m.Hidden));
end
end

function write_meta_report(fid,M)
say(fid,' meta properties                         = %d',numel(M.properties));
say(fid,' meta methods                            = %d',numel(M.methods));
say(fid,' timing/performance/state-like properties:');
for k=1:numel(M.properties)
    nm=lower(M.properties(k).name);
    if contains(nm,'time')||contains(nm,'timing')||contains(nm,'stage')|| ...
            contains(nm,'runtime')||contains(nm,'count')||contains(nm,'cache')|| ...
            contains(nm,'packet')||contains(nm,'replan')||contains(nm,'compiled')
        say(fid,'  %-46s | get=%s | set=%s', ...
            M.properties(k).name,M.properties(k).getAccess,M.properties(k).setAccess);
    end
end
end

function s=access_text(a)
try
    if ischar(a), s=a; return; end
    if isstring(a), s=char(a); return; end
    if isa(a,'meta.class'), s=a.Name; return; end
    if iscell(a)
        c=cellfun(@access_text,a,'UniformOutput',false);
        s=strjoin(c,','); return;
    end
    s=char(string(a));
catch
    s='<complex>';
end
end

% =========================================================================
% Commit time resolution
% =========================================================================
function tc=resolve_commit_time(OUT,obj)
tc=NaN;
try
    x=obj.last_replan_commit_time;
    if isnumeric(x)&&isscalar(x)&&isfinite(x), tc=double(x); return; end
catch
end
try
    if isfield(OUT.explicit_properties,'last_replan_commit_time')
        x=OUT.explicit_properties.last_replan_commit_time;
        if isnumeric(x)&&isscalar(x)&&isfinite(x), tc=double(x); return; end
    end
catch
end
end

% =========================================================================
% Generic helpers
% =========================================================================
function say(fid,fmt,varargin)
s=sprintf(fmt,varargin{:});
fprintf(1,'%s\n',s);
fprintf(fid,'%s\n',s);
end

function safe_fclose(fid)
try
    if fid>=0, fclose(fid); end
catch
end
end

function restore_warning(S)
try
    warning(S.state,S.identifier);
catch
end
end

function tf=is_under_root(p,root)
tf=false;
if isempty(p)||isempty(root), return; end
try
    pc=char(java.io.File(p).getCanonicalPath());
    rc=char(java.io.File(root).getCanonicalPath());
    if ispc
        tf=startsWith(lower(pc),lower(rc));
    else
        tf=startsWith(pc,rc);
    end
catch
    if ispc
        tf=startsWith(lower(p),lower(root));
    else
        tf=startsWith(p,root);
    end
end
end

function C=force_cellstr(x)
if isempty(x)
    C={};
elseif iscell(x)
    C=cellfun(@char,x,'UniformOutput',false);
elseif isstring(x)
    C=cellstr(x(:));
elseif ischar(x)
    if size(x,1)>1
        C=cellstr(x);
    else
        C={x};
    end
else
    C={char(string(x))};
end
C=C(:);
end

function s=value_one_line(v)
try
    if ischar(v), s=v; return; end
    if isstring(v)&&isscalar(v), s=char(v); return; end
    if (isnumeric(v)||islogical(v))&&isscalar(v)
        s=sprintf('%.17g',double(v)); return;
    end
    if isnumeric(v)||islogical(v)
        s=sprintf('%s %s',class(v),mat2str(size(v))); return;
    end
    if isstruct(v)
        s=sprintf('struct %s fields=%d',mat2str(size(v)),numel(fieldnames(v))); return;
    end
    if isobject(v)
        s=sprintf('object %s %s',class(v),mat2str(size(v))); return;
    end
    s=sprintf('%s %s',class(v),mat2str(size(v)));
catch
    s='<unprintable>';
end
end

function s=safe_filename(s)
s=char(s);
s=regexprep(s,'[^A-Za-z0-9_\-]','_');
if isempty(s), s='unnamed'; end
if numel(s)>180, s=s(1:180); end
end

function h=sha_file(p)
fid=fopen(p,'rb');
assert(fid>=0,'cannot open %s',p);
c=onCleanup(@()fclose(fid)); %#ok<NASGU>
b=fread(fid,Inf,'*uint8');
md=java.security.MessageDigest.getInstance('SHA-256');
md.update(typecast(b,'int8'));
d=typecast(md.digest(),'uint8');
h=lower(reshape(dec2hex(d,2).',1,[]));
end
