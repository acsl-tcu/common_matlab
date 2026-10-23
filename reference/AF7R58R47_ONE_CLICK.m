function OUT = AF7R58R47_ONE_CLICK(varargin)
% AF7R58R47_ONE_CLICK: no-workspace-variable read-only R47 entry point.
% Use:
%     OUT = AF7R58R47_ONE_CLICK;
% Optionally bind exact authoritative paths:
%     OUT = AF7R58R47_ONE_CLICK('terminal','C:\...\R45R1_OUT.mat', ...
%         'towing','C:\...\R47_TOWING_LOG.mat', ...
%         'timing','C:\...\AF7R58R23_SPIKE_REPORT.txt');
%
% Input sources are NEVER synthesized, guessed from other versions, or used
% to modify REPLANNING_SYSTEM/plant/controller. Any absent track stays OPEN.
% Zero-argument mode searches base-workspace structs and R45R1-named .mat,
% a strictly schema-mapped towing log, and the latest R23 report. Selected
% sources are printed and recorded; different-run files are NOT assumed to
% have matching vehicle/controller/obstacle conditions.

assert(mod(numel(varargin),2)==0, ...
    '[R47] Supply name/value pairs: terminal, towing, timing.');
opt=struct('terminal','','towing','','timing','');
for k=1:2:numel(varargin)
    name=lower(char(varargin{k}));
    assert(isfield(opt,name),'[R47] Unknown option: %s',name);
    assert(ischar(varargin{k+1}) || ...
        (isstring(varargin{k+1}) && isscalar(varargin{k+1})), ...
        '[R47] Explicit option %s must be a file path.',name);
    opt.(name)=char(varargin{k+1});
    assert(isfile(opt.(name)), ...
        '[R47] Explicit %s file does not exist: %s',name,opt.(name));
end

scriptdir=fileparts(mfilename('fullpath'));
paths={pwd,scriptdir};
if ~strcmpi(pwd,scriptdir)
    [~,package_name]=fileparts(scriptdir);
    % Include parent ONLY for an extracted R47-named package directory;
    % never recursively crawl an unrelated GitHub/common_matlab ancestor.
    if startsWith(package_name,'AF7R58R47')
        parent=fileparts(scriptdir);
        if ~isempty(parent) && ~strcmpi(parent,scriptdir)
            paths{end+1}=parent; %#ok<AGROW>
        end
    end
end
paths=unique(paths,'stable');
OUT=struct();
OUT.schema='AF7R58R47_ONE_CLICK_READONLY_V2';
OUT.production_modified=false;
OUT.return=struct('status','NOT_RUN__R45R1_SCHEMA_SOURCE_NOT_FOUND');
OUT.towing=struct('status','NOT_RUN__MAPPED_LIVE_TOWING_LOG_NOT_FOUND');
OUT.timing=struct('status','NOT_RUN__R23_TIMING_REPORT_NOT_FOUND');
OUT.return_source='';OUT.towing_source='';OUT.timing_source='';
OUT.source_pairing_verified=false;
OUT.workspace_struct_inventory=base_struct_inventory();
OUT.production_reference_certified=false;
OUT.closed_loop_continuous_obstacle_certified=false;
OUT.control_period_25ms_certified=false;

fprintf('\n=============================================================\n');
fprintf(' AF7R58R47 3-TRACK READ-ONLY ONE CLICK\n');
fprintf(' No production modification / no fabricated source data\n');
fprintf('=============================================================\n');

% Track 1: exact R45R1 TERMINAL_AUDIT_V1, never R44R2 snapshot alone.
terminal=[];
if ~isempty(opt.terminal)
    terminal=opt.terminal;
    OUT.return_source=['EXPLICIT_FILE: ' opt.terminal];
else
    [terminal,src]=find_workspace_source('terminal');
    if ~isempty(src)
        OUT.return_source=src;
    else
        candidates=collect_matching(paths,{'*R45R1*.mat','*R45R1*OUT*.mat'});
        for k=1:numel(candidates)
            try
                S=load(candidates{k});
                if contains_terminal(S,0)
                    terminal=S;
                    OUT.return_source=['AUTO_EXACT_SCHEMA_FILE: ' candidates{k}];
                    break
                end
            catch ME
                fprintf('[R47] Terminal candidate unreadable: %s (%s)\n', ...
                    candidates{k},ME.identifier);
            end
        end
    end
end
if ~isempty(OUT.return_source)
    fprintf('\n[R47] RETURN source: %s\n',OUT.return_source);
    try
        OUT.return=AF7R58R47_RETURN_AUDIT(terminal);
    catch ME
        OUT.return=failed('RETURN_AUDIT_ERROR',ME);
    end
else
    fprintf('\n[R47] RETURN OPEN: actual R45R1 terminal audit not found.\n');
end

% Track 2: only *exact* recorded fields, never inferred state mapping.
tow=[];
if ~isempty(opt.towing)
    tow=opt.towing;
    OUT.towing_source=['EXPLICIT_FILE: ' opt.towing];
else
    [tow,src]=find_workspace_source('towing');
    if ~isempty(src)
        OUT.towing_source=src;
    else
        candidates=collect_matching(paths,{'AF7R58R47_TOWING_LOG*.mat'});
        for k=1:numel(candidates)
            try
                S=load(candidates{k});
                [mapped,found]=extract_towing(S,0);
                if found
                    tow=mapped;
                    OUT.towing_source=['AUTO_EXACT_FIELD_FILE: ' candidates{k}];
                    break
                end
            catch ME
                fprintf('[R47] Towing candidate unreadable: %s (%s)\n', ...
                    candidates{k},ME.identifier);
            end
        end
    end
end
if ~isempty(OUT.towing_source)
    fprintf('\n[R47] TOWING source: %s\n',OUT.towing_source);
    try
        if ischar(tow)
            S=load(tow);
            [tow,found]=extract_towing(S,0);
            assert(found,'[R47] Explicit towing MAT has no 7-field mapped log.');
        end
        OUT.towing=AF7R58R47_TOWING_LOG_AUDIT(tow);
    catch ME
        OUT.towing=failed('TOWING_AUDIT_ERROR',ME);
    end
else
    fprintf('\n[R47] TOWING OPEN: exact recorded log fields not found.\n');
end

% Track 3: the full-flight summary in the R23 report, not a made-up trace.
report='';
if ~isempty(opt.timing)
    report=opt.timing;
    OUT.timing_source=['EXPLICIT_FILE: ' opt.timing];
else
    candidates=collect_matching(paths,{'AF7R58R23_SPIKE_REPORT*.txt'});
    % The most recently modified report is selected, NOT declared to be
    % paired to the R45R1 snapshot or to the optional towing log.
    for k=1:numel(candidates)
        try
            file=fileread(candidates{k});
            if contains(file,'AF7R58R23 | FULL ITERATION SPIKE ATTRIBUTION')
                report=candidates{k};
                OUT.timing_source=['AUTO_LATEST_MATCHING_R23_REPORT: ' report];
                break
            end
        catch
            % Continue looking; other tracks remain independent.
        end
    end
end
if ~isempty(report)
    fprintf('\n[R47] TIMING source: %s\n',OUT.timing_source);
    try
        OUT.timing=AF7R58R47_TIMING_SOURCE_SPLIT(report);
    catch ME
        OUT.timing=failed('TIMING_AUDIT_ERROR',ME);
    end
else
    fprintf('\n[R47] TIMING OPEN: R23 spike report not found.\n');
end

OUT.finished_at=datestr(now,31);
name=['AF7R58R47_RESULTS_' datestr(now,'yyyymmdd_HHMMSS')];
outdir=fullfile(pwd,name);
ix=1;
while exist(outdir,'dir')
    outdir=fullfile(pwd,[name '_' num2str(ix)]);
    ix=ix+1;
end
mkdir(outdir);
OUT.result_directory=outdir;
OUT.mat_file=fullfile(outdir,'AF7R58R47_OUT.mat');
OUT.report_file=fullfile(outdir,'AF7R58R47_REPORT.txt');
save(OUT.mat_file,'OUT');
fid=fopen(OUT.report_file,'w');
assert(fid>0,'[R47] Could not write result report.');
cleaner=onCleanup(@() fclose(fid));
fprintf(fid,'AF7R58R47 THREE TRACKS | READ-ONLY V2\n');
fprintf(fid,'finished_at = %s\n',OUT.finished_at);
fprintf(fid,'production_modified = 0\n');
fprintf(fid,'return_source = %s\n',OUT.return_source);
fprintf(fid,'return_status = %s\n',OUT.return.status);
fprintf(fid,'towing_source = %s\n',OUT.towing_source);
fprintf(fid,'towing_status = %s\n',OUT.towing.status);
fprintf(fid,'timing_source = %s\n',OUT.timing_source);
fprintf(fid,'timing_status = %s\n',OUT.timing.status);
fprintf(fid,'source_pairing_verified = 0\n');
fprintf(fid,'base_workspace_struct_variables = %s\n', ...
    strjoin(OUT.workspace_struct_inventory,', '));
fprintf(fid,'production_reference_certified = 0\n');
fprintf(fid,'closed_loop_continuous_obstacle_certified = 0\n');
fprintf(fid,'control_period_25ms_certified = 0\n');
if isfield(OUT.return,'live_terminal_left_limit_norm_m')
    fprintf(fid,'live_return_offset_norm_m = %.17g\n', ...
        OUT.return.live_terminal_left_limit_norm_m);
end
if isfield(OUT.towing,'max_cable_length_abs_residual_m')
    fprintf(fid,'towing_sampled_cable_length_residual_m = %.17g\n', ...
        OUT.towing.max_cable_length_abs_residual_m);
end
if isfield(OUT.timing,'max_spike_full_ms')
    fprintf(fid,'timing_max_spike_full_ms = %.17g\n', ...
        OUT.timing.max_spike_full_ms);
end
clear cleaner;
fprintf('\n=============================================================\n');
fprintf('[R47] RETURN = %s\n',OUT.return.status);
fprintf('[R47] TOWING = %s\n',OUT.towing.status);
fprintf('[R47] TIMING = %s\n',OUT.timing.status);
fprintf('[R47] Result MAT = %s\n',OUT.mat_file);
fprintf('[R47] Report     = %s\n',OUT.report_file);
fprintf('[R47] No production commit or safety proof claim.\n');
fprintf('=============================================================\n');
end

function A=failed(label,ME)
A=struct('status',label,'identifier',ME.identifier,'message',ME.message);
fprintf('[R47] %s: %s\n',label,ME.message);
end

function [S,source]=find_workspace_source(kind)
S=[];source='';
try
    vars=evalin('base','whos');
catch
    return
end
for k=1:numel(vars)
    if ~strcmp(vars(k).class,'struct'),continue;end
    name=vars(k).name;
    try
        v=evalin('base',name);
        if strcmp(kind,'terminal') && contains_terminal(v,0)
            S=v; source=['BASE_WORKSPACE_EXACT_SCHEMA: ' name];return
        elseif strcmp(kind,'towing')
            [z,found]=extract_towing(v,0);
            if found
                S=z; source=['BASE_WORKSPACE_EXACT_FIELDS: ' name];return
            end
        end
    catch
        % A variable that cannot be inspected must not be treated as data.
    end
end
end

function tf=contains_terminal(S,depth)
tf=false;
if ~isstruct(S) || depth>16 || numel(S)>200,return;end
for i=1:numel(S)
    x=S(i);
    if isfield(x,'schema') && ...
            (ischar(x.schema)|| (isstring(x.schema)&&isscalar(x.schema))) && ...
            strcmp(char(x.schema),'AF7R58R45R1_TERMINAL_AUDIT_V1')
        tf=true;return
    end
    names=fieldnames(x);
    for j=1:numel(names)
        v=x.(names{j});
        if isstruct(v) && contains_terminal(v,depth+1)
            tf=true;return
        end
    end
end
end

function [T,found]=extract_towing(S,depth)
T=[];found=false;
if ~isstruct(S) || depth>10 || numel(S)>40,return;end
required={'t_s','pQ_m','pL_m','q_cable','L_m','mL_EKF_kg','mL_HLC_kg'};
for i=1:numel(S)
    x=S(i);
    if all(isfield(x,required))
        T=x;found=true;return
    end
    names=fieldnames(x);
    for j=1:numel(names)
        v=x.(names{j});
        if isstruct(v)
            [T,found]=extract_towing(v,depth+1);
            if found,return;end
        end
    end
end
end

function paths=collect_matching(folders,patterns)
% Local directory and subfolders only. Do not crawl machine-wide or web.
paths={};stamps=[];
for f=1:numel(folders)
    folder=folders{f};
    if ~isfolder(folder),continue;end
    for p=1:numel(patterns)
        D=[dir(fullfile(folder,patterns{p})); ...
           dir(fullfile(folder,'**',patterns{p}))]; %#ok<AGROW>
        for j=1:numel(D)
            if D(j).isdir,continue;end
            file=fullfile(D(j).folder,D(j).name);
            if ~any(strcmpi(paths,file))
                paths{end+1}=file; %#ok<AGROW>
                stamps(end+1)=D(j).datenum; %#ok<AGROW>
            end
        end
    end
end
if ~isempty(stamps)
    [~,order]=sort(stamps,'descend');
    paths=paths(order);
end
end

function names=base_struct_inventory()
names={};
try
    v=evalin('base','whos');
    for k=1:numel(v)
        if strcmp(v(k).class,'struct')
            names{end+1}=v(k).name; %#ok<AGROW>
        end
    end
catch
    % Workspace inventory is diagnostic metadata, not a required source.
end
end
