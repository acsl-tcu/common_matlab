function zipPath = CBF_COLLECT_ACTIVE_GUI(action, varargin)
% CBF_COLLECT_ACTIVE_GUI  Package the currently selected GUI simulation.
%
% Command Window (begin BEFORE running mainGUI):
%   CBF_COLLECT_ACTIVE_GUI('start');
%   mainGUI
%   % Complete the actual flight, plots, and other GUI callbacks.
%   CBF_COLLECT_ACTIVE_GUI('export');
%
% With optional additional roots or dynamically constructed names:
%   CBF_COLLECT_ACTIVE_GUI('export', 'ActiveSim', ...
%       'SimSuspendedLoadCBFIntegrated', 'ExtraRoots', {'D:\my_lib'}, ...
%       'ExtraNames', {'MyDynamicClass'}, 'ExtraFiles', {'D:\data\map.mat'});
%
% IMPORTANT:
% * mainGUI.m is INCLUDED but deliberately NEVER used as a static-analysis
%   root: the SimBaseMode candidate strings must not import inactive Sims.
% * Only the selected Sim and code reached from its dependencies are expanded.
% * Runtime profiling + iterative direct static dependencies + literal-name discovery
%   cover different cases, but CANNOT prove all dynamic/future branches complete.
% * This tool changes NO simulation, HLC, sensor, or DRAW source.
%
% See README_JA.md supplied with this file.

if nargin < 1 || isempty(action), action = 'export'; end
zipPath = '';
switch lower(char(action))
    case 'start'
        profile off;
        profile clear;
        profile on;
        fprintf('[CBF EXPORT] Recording started. Run mainGUI and exercise the GUI.\n');
        fprintf('[CBF EXPORT] After all GUI actions: CBF_COLLECT_ACTIVE_GUI(''export'')\n');
        return
    case 'export'
        % continue
    otherwise
        error('CBF_COLLECT_ACTIVE_GUI:Action','Use ''start'' or ''export''.');
end

ip = inputParser;
addParameter(ip,'ProjectRoot','',@(x)ischar(x)||isstring(x));
addParameter(ip,'ActiveSim','SimSuspendedLoadCBFIntegrated',@(x)ischar(x)||isstring(x));
addParameter(ip,'ExtraRoots',{},@(x)iscell(x)||isstring(x));
addParameter(ip,'ExtraNames',{},@(x)iscell(x)||isstring(x));
addParameter(ip,'ExtraFiles',{},@(x)iscell(x)||isstring(x));
addParameter(ip,'OutputZip','',@(x)ischar(x)||isstring(x));
addParameter(ip,'MaxRounds',50,@(x)isnumeric(x)&&isscalar(x)&&x>=1&&x<=200);
parse(ip,varargin{:}); o=ip.Results;

simName = char(o.ActiveSim);
[~,simBase,~] = fileparts(simName);
simName = simBase;
mainPath = which('mainGUI');
if isempty(char(o.ProjectRoot))
    if isempty(mainPath)
        error('CBF_COLLECT_ACTIVE_GUI:MainMissing', ...
            'mainGUI not found. Specify ProjectRoot and add project to path.');
    end
    root = cbf_abs(fileparts(mainPath));
else
    root = cbf_abs(char(o.ProjectRoot));
end
if ~isfolder(root), error('ProjectRoot not found: %s',root); end
simPath = cbf_resolve_name(simName);
if isempty(simPath), error('Selected Sim not found on path: %s',simName); end
if isempty(mainPath) && isfile(fullfile(root,'mainGUI.m'))
    mainPath = fullfile(root,'mainGUI.m');
end
if isempty(mainPath), error('mainGUI.m not found.'); end
mainPath = cbf_abs(mainPath);

extraRoots=cbf_as_cell(o.ExtraRoots);
for j=1:numel(extraRoots)
    extraRoots{j}=cbf_abs(extraRoots{j});
    if ~isfolder(extraRoots{j}), error('ExtraRoots{%d} not found',j); end
end
extraNames=cbf_as_cell(o.ExtraNames);
extraFiles=cbf_as_cell(o.ExtraFiles);

zipPath=char(o.OutputZip);
if isempty(zipPath)
    zipPath=fullfile(root,['CBF_ACTIVE_GUI_' datestr(now,'yyyymmdd_HHMMSS') '.zip']);
elseif ~cbf_is_absolute(zipPath)
    zipPath=fullfile(root,zipPath);
end
zipPath=cbf_abs(zipPath);
[outdir,~,~]=fileparts(zipPath);
if ~isfolder(outdir), mkdir(outdir); end

% Capture once. The profile is stopped by profile('info').
prof=struct(); profilerOn=false;
try
    s=profile('status');
    profilerOn=strcmpi(char(s.ProfilerStatus),'on');
    if profilerOn
        prof=profile('info');
    else
        warning('CBF_COLLECT_ACTIVE_GUI:ProfilerOff', ...
            'Profiler is OFF. Only selected-Sim/static dependencies will be collected.');
    end
catch ME
    warning('CBF_COLLECT_ACTIVE_GUI:ProfileError','Profiler: %s',ME.message);
end

found=containers.Map('KeyType','char','ValueType','char');
roots={};
rejected={}; problems={}; unresolved={}; runtimeFiles={};
toolboxNames={};

% Include mainGUI for replication but DO NOT ANALYZE IT. This is critical.
cbf_include(found,mainPath,'entry-only:not-analyzed');
cbf_include(found,simPath,'selected-sim');
roots{end+1}=simPath;

% Explicitly seed both CBF branches ONLY when they appear in uncommented
% code of the SELECTED Sim. They are not discovered via mainGUI candidates.
branchNames={'HLC_SUSPENDED_LOAD_CBF_REAL','HLC_SUSPENDED_LOAD_CBF_VIRTUAL'};
try
    selectedSource=cbf_strip_comments(fileread(simPath));
    for b=1:numel(branchNames)
        if ~isempty(regexp(selectedSource, ['(?<![A-Za-z0-9_])' ...
                branchNames{b} '(?![A-Za-z0-9_])'], 'once'))
            branchFile=cbf_resolve_name(branchNames{b});
            if isempty(branchFile)
                unresolved{end+1}=['CBF branch class not found: ' branchNames{b}]; %#ok<AGROW>
            else
                cbf_include(found,branchFile,'active-sim-cbf-branch');
                roots{end+1}=branchFile; %#ok<AGROW>
            end
        end
    end
catch ME
    problems{end+1}=['ActiveSim branch inspection: ' ME.message]; %#ok<AGROW>
end

% Runtime evidence includes callback methods, dynamic feval, class methods etc.
if isfield(prof,'FunctionTable')
    tab=prof.FunctionTable;
    for k=1:numel(tab)
        f='';
        if isfield(tab(k),'FileName'), f=char(tab(k).FileName); end
        if isempty(f) || ~isfile(f), continue; end
        f=cbf_abs(f);
        if cbf_is_matlabroot(f), continue; end
        if cbf_is_unselected_sim(f,simName)
            rejected{end+1}=['runtime: ' f]; %#ok<AGROW>
            continue
        end
        if cbf_is_exporter(f), continue; end
        cbf_include(found,f,'runtime-observed');
        runtimeFiles{end+1}=f; %#ok<AGROW>
        if cbf_is_source(f) && ~strcmpi(f,mainPath)
            roots{end+1}=f; %#ok<AGROW>
        end
    end
end
runtimeFiles=unique(runtimeFiles,'stable');
if profilerOn && ~any(strcmpi(runtimeFiles,simPath))
    problems{end+1}=['Selected Sim was not observed by Profiler: ' simPath ...
        '. Confirm the GUI was launched AFTER start and this is the active mode.']; %#ok<AGROW>
end

% Optional explicit dependencies for dynamic names/data that cannot be inferred.
for k=1:numel(extraNames)
    f=cbf_resolve_name(extraNames{k});
    if isempty(f)
        unresolved{end+1}=['ExtraNames not found: ' extraNames{k}]; %#ok<AGROW>
    elseif cbf_is_unselected_sim(f,simName)
        rejected{end+1}=['ExtraNames rejected inactive Sim: ' f]; %#ok<AGROW>
    else
        cbf_include(found,f,'manual-extra-name');
        if cbf_is_source(f), roots{end+1}=f; end %#ok<AGROW>
    end
end
for k=1:numel(extraFiles)
    f=cbf_abs(extraFiles{k});
    if ~isfile(f)
        unresolved{end+1}=['ExtraFiles not found: ' f]; %#ok<AGROW>
    elseif cbf_is_unselected_sim(f,simName)
        rejected{end+1}=['ExtraFiles rejected inactive Sim: ' f]; %#ok<AGROW>
    else
        cbf_include(found,f,'manual-extra-file');
        if cbf_is_source(f), roots{end+1}=f; end %#ok<AGROW>
    end
end

% Multiple rounds: MATLAB DIRECT static dependencies (toponly), iterated
% until closure without following other Sim classes + literal dynamic
% names and literal data resources discovered in those dependency sources.
% Entire folders are NEVER scanned or copied.
analyzed=containers.Map('KeyType','char','ValueType','logical');
scanned=containers.Map('KeyType','char','ValueType','logical');
roots=unique(roots,'stable');
for roundNo=1:o.MaxRounds
    todo={};
    for k=1:numel(roots)
        f=roots{k};
        if cbf_is_source(f) && ~strcmpi(f,mainPath) && ~isKey(analyzed,f)
            todo{end+1}=f; %#ok<AGROW>
            analyzed(f)=true;
        end
    end
    if ~isempty(todo)
        fprintf('[CBF EXPORT] Static round %d: %d roots\n',roundNo,numel(todo));
        try
            [deps,products]=matlab.codetools.requiredFilesAndProducts(todo,'toponly');
            [found,roots,rejected]=cbf_add_deps(found,roots,rejected,deps,simName);
            if ~isempty(products), toolboxNames=[toolboxNames,{products.Name}]; end %#ok<AGROW>
        catch ME
            problems{end+1}=['batch static round ' num2str(roundNo) ': ' ME.message]; %#ok<AGROW>
            for k=1:numel(todo)
                try
                    [deps,products]=matlab.codetools.requiredFilesAndProducts(todo{k},'toponly');
                    [found,roots,rejected]=cbf_add_deps(found,roots,rejected,deps,simName);
                    if ~isempty(products),toolboxNames=[toolboxNames,{products.Name}];end %#ok<AGROW>
                catch E
                    problems{end+1}=[todo{k} ' : ' E.message]; %#ok<AGROW>
                end
            end
        end
    end

    % Includes code/string-selected branch names, but ignores MATLAB
    % comments (including inactive commented-out reference declarations).
    scanPaths=keys(found);
    newly={};
    for k=1:numel(scanPaths)
        file=scanPaths{k};
        if ~strcmpi(cbf_ext(file),'.m') || isKey(scanned,file),continue;end
        scanned(file)=true;
        % Do not search mainGUI's candidate arrays: ONLY selected Sim's graph.
        if strcmpi(file,mainPath), continue; end
        try
            [names,assets]=cbf_literal_candidates(file);
            for j=1:numel(names)
                dep=cbf_resolve_name(names{j});
                if isempty(dep), continue; end
                if cbf_is_unselected_sim(dep,simName)
                    rejected{end+1}=['literal: ' dep]; %#ok<AGROW>
                    continue
                end
                if cbf_is_matlabroot(dep) || cbf_is_exporter(dep),continue;end
                wasPresent=isKey(found,dep);
                cbf_include(found,dep,['literal-in:' file]);
                if ~wasPresent && cbf_is_source(dep),newly{end+1}=dep;end %#ok<AGROW>
            end
            for j=1:numel(assets)
                dep=cbf_find_resource(assets{j},file,root,extraRoots);
                if ~isempty(dep) && ~cbf_is_exporter(dep)
                    cbf_include(found,dep,['literal-data-in:' file]);
                end
            end
        catch ME
            problems{end+1}=['literal scan ' file ': ' ME.message]; %#ok<AGROW>
        end
    end
    roots=unique([roots,newly],'stable');
    hasMore=false;
    for k=1:numel(roots)
        if cbf_is_source(roots{k}) && ~isKey(analyzed,roots{k})
            hasMore=true; break
        end
    end
    if ~hasMore,break;end
end
% Explicitly report any incomplete closure when iteration cap was hit.
for k=1:numel(roots)
    if cbf_is_source(roots{k}) && ~isKey(analyzed,roots{k})
        unresolved{end+1}=['UNANALYZED (MaxRounds reached): ' roots{k}]; %#ok<AGROW>
    end
end

% Warn about dynamic file loading in included executable sources. Comments
% are stripped before checking. This is a review list, not automatic copying.
dynamicReview={};
allPaths=sort(keys(found));
for k=1:numel(allPaths)
    f=allPaths{k};
    if ~strcmpi(cbf_ext(f),'.m') || strcmpi(f,mainPath),continue;end
    try
        code=cbf_strip_comments(fileread(f));
        pats=regexp(code,'(?<![A-Za-z0-9_])(feval|str2func|eval|load|fopen|readtable|readmatrix|imread|readstruct|importdata|addpath|run|load_system)\s*\(', 'tokens');
        if ~isempty(pats)
            dynamicReview{end+1}=sprintf('%s : %s',f,strjoin(unique(cellfun(@(x)x{1},pats,'UniformOutput',false)),', ')); %#ok<AGROW>
        end
    catch
    end
end

stage=tempname;
mkdir(stage);
cleanup=onCleanup(@() cbf_remove_stage(stage)); %#ok<NASGU>
original={}; inside={}; why={}; copyErrors={};
for k=1:numel(allPaths)
    f=allPaths{k};
    rel=cbf_archive_name(f,root,extraRoots);
    dest=fullfile(stage,rel);
    folder=fileparts(dest);
    if ~isfolder(folder),mkdir(folder);end
    [ok,msg]=copyfile(f,dest,'f');
    if ok
        original{end+1}=f;inside{end+1}=strrep(rel,filesep,'/');why{end+1}=found(f); %#ok<AGROW>
    else
        copyErrors{end+1}=[f ' : ' msg]; %#ok<AGROW>
    end
end
cbf_csv_manifest(fullfile(stage,'MANIFEST_FILES.csv'),original,inside,why);
cbf_write_list(fullfile(stage,'MANIFEST_EXECUTED.txt'),runtimeFiles);
cbf_write_list(fullfile(stage,'MANIFEST_EXCLUDED_OTHER_SIMS.txt'),unique(rejected,'stable'));
cbf_write_list(fullfile(stage,'MANIFEST_ANALYSIS_ERRORS.txt'),problems);
cbf_write_list(fullfile(stage,'MANIFEST_UNRESOLVED.txt'),unresolved);
cbf_write_list(fullfile(stage,'MANIFEST_COPY_ERRORS.txt'),copyErrors);
cbf_write_list(fullfile(stage,'MANIFEST_DYNAMIC_REVIEW.txt'),dynamicReview);
cbf_write_list(fullfile(stage,'MANIFEST_TOOLBOXES.txt'),unique(toolboxNames,'stable'));
summary={...
    ['Generated: ' datestr(now,31)], ...
    ['MATLAB version: ' version], ...
    ['ProjectRoot: ' root], ...
    ['mainGUI entry (NOT statically analyzed): ' mainPath], ...
    ['Selected Sim: ' simPath], ...
    ['Profiler captured: ' mat2str(profilerOn)], ...
    sprintf('Runtime files: %d',numel(runtimeFiles)),...
    sprintf('Archived files: %d',numel(original)),...
    sprintf('Rejected alternate Sim references: %d',numel(rejected)),...
    sprintf('Analyzer errors: %d',numel(problems)),...
    sprintf('Unresolved items: %d',numel(unresolved)),...
    sprintf('Copy errors: %d',numel(copyErrors)),...
    'No whole-project scanning and NO static scanning of mainGUI.m.',...
    'Selected Sim branch code (REAL/VIRTUAL) is included if statically resolvable.',...
    'WARNING: Dynamic paths, future branches, MEX external libraries, and runtime files may be missed.',...
    'Review all MANIFEST_* and remove any sensitive content before sharing.'};
cbf_write_list(fullfile(stage,'MANIFEST_SUMMARY.txt'),summary);
cbf_write_list(fullfile(stage,'README_RESTORE.txt'),{...
    'MATLAB active GUI source snapshot. Only selected-Sim dependency graph is expanded.',...
    'project/ contains relative paths from mainGUI directory.',...
    'extra_N/ contains relative files from ExtraRoots; external/ contains other resolved sources.',...
    'Extraction does not make all paths automatically usable. Configure MATLAB path as needed.',...
    'Check MANIFEST_FILES.csv for provenance: runtime-observed/static/literal/manual.',...
    'Other Sim scripts from inactive mainGUI choices are blocked intentionally.',...
    'MathWorks installed toolbox source not included. Check MANIFEST_TOOLBOXES.txt.',...
    'Run full GUI actions BEFORE export. Repeat when changing active Sim or dynamic branches.',...
    'This snapshot is not a proof of completeness. Check errors/unresolved/dynamic review.'});
zip(zipPath,'*',stage);
if ~isfile(zipPath),error('ZIP was not created: %s',zipPath);end
fprintf('[CBF EXPORT] Created %s\n',zipPath);
fprintf('[CBF EXPORT] Archived=%d, runtime=%d, excludedOtherSims=%d, analysisErrors=%d, missing=%d\n',...
    numel(original),numel(runtimeFiles),numel(rejected),numel(problems),numel(unresolved));
if ~profilerOn || ~isempty(problems) || ~isempty(copyErrors) || ~isempty(unresolved)
    warning('CBF_COLLECT_ACTIVE_GUI:Review', ...
        'Inspect MANIFEST_SUMMARY, ANALYSIS_ERRORS, UNRESOLVED and COPY_ERRORS before relying on the ZIP.');
end
end

function c=cbf_as_cell(x)
if isstring(x),c=cellstr(x);elseif iscell(x),c=x;else,c={x};end
c=c(:)';
for i=1:numel(c),c{i}=char(c{i});end
end

function [found,roots,rejected]=cbf_add_deps(found,roots,rejected,deps,simName)
if isstring(deps),deps=cellstr(deps);end
for j=1:numel(deps)
    f=char(deps{j});
    if ~isfile(f),continue;end
    f=cbf_abs(f);
    if cbf_is_unselected_sim(f,simName)
        rejected{end+1}=['static: ' f]; %#ok<AGROW>
    elseif ~cbf_is_matlabroot(f) && ~cbf_is_exporter(f)
        cbf_include(found,f,'static-dependency');
        if cbf_is_source(f),roots{end+1}=f;end %#ok<AGROW>
    end
end
end

function cbf_include(mp,f,reason)
if ~isfile(f),return;end
f=cbf_abs(f);
if cbf_is_matlabroot(f),return;end
if isKey(mp,f)
    val=mp(f);
    if ~contains(['|' val '|'],['|' reason '|'])
        mp(f)=[val '|' reason];
    end
else
    mp(f)=reason;
end
end

function out=cbf_resolve_name(name)
out='';
name=char(name);
if isempty(name),return;end
if isfile(name),out=cbf_abs(name);return;end
try
    x=which(name);
    if ~isempty(x) && isfile(x),out=cbf_abs(x);end
catch
end
end

function tf=cbf_is_unselected_sim(f,selected)
[~,name,~]=fileparts(f);
tf=~strcmpi(name,selected) && ~isempty(regexp(name,'^SimSuspendedLoad', 'once','ignorecase'));
end

function tf=cbf_is_source(f)
ext=lower(cbf_ext(f));
tf=any(strcmp(ext,{'.m','.p','.mlx','.mlapp','.slx','.mdl'}));
end
function ext=cbf_ext(f)
[~,~,ext]=fileparts(f);
end
function tf=cbf_is_exporter(f)
[~,name,~]=fileparts(f);
tf=strcmpi(name,'CBF_COLLECT_ACTIVE_GUI') || strcmpi(name,'CBF_COLLECT_RUN_PACKAGE');
end

function [names,assets]=cbf_literal_candidates(f)
% Heuristic for strings passed to waypoint generators, class dispatchers,
% feval/str2func and data readers. Comments are removed first.
code=cbf_strip_comments(fileread(f));
% Regular identifiers in executed code: catches direct constructor calls
% in both REAL and VIRTUAL branches, even if one branch wasn't run.
tokens=regexp(code,'(?<![A-Za-z0-9_.])([A-Za-z][A-Za-z0-9_]*)\s*\(', 'tokens');
names=cellfun(@(x)x{1},tokens,'UniformOutput',false);
% Quoted literal identifiers: catches route generator strings and dynamic
% functions selected via function name strings.
q=regexp(code,'[''"]([A-Za-z][A-Za-z0-9_]*)[''"]','tokens');
for j=1:numel(q),names{end+1}=q{j}{1};end %#ok<AGROW>
names=unique(names,'stable');
% Literal resource filenames with extensions, e.g. load('map.mat').
a=regexp(code,'[''"]([^''"\r\n]+\.(mat|csv|json|xml|yaml|yml|txt|dat|stl|obj|ply|urdf|png|jpg|jpeg|bmp|fig|slx|sldd))[''"]','tokens','ignorecase');
assets={};
for j=1:numel(a),assets{end+1}=a{j}{1};end %#ok<AGROW>
assets=unique(assets,'stable');
end

function code=cbf_strip_comments(code)
% Remove % line comments and %{ %} block comments, preserving strings.
out=repmat(' ',1,numel(code));
quoted=char(0); block=false; i=1;n=numel(code);
while i<=n
    c=code(i);
    if block
        if c=='%' && i<n && code(i+1)=='}'
            block=false;i=i+2;continue
        end
        if c==newline || c==char(13),out(i)=c;end
        i=i+1;continue
    end
    if quoted~=char(0)
        out(i)=c;
        if c==quoted
            if i<n && code(i+1)==quoted
                out(i+1)=code(i+1);i=i+2;continue
            end
            quoted=char(0);
        end
        i=i+1;continue
    end
    if c=='%' && i<n && code(i+1)=='{'
        block=true;i=i+2;continue
    end
    if c=='%'
        while i<=n && code(i)~=newline && code(i)~=char(13)
            i=i+1;
        end
        continue
    end
    if c=='"'
        quoted=c;
    elseif c==''''
        % MATLAB transpose apostrophe (x', A') is NOT a string opener.
        % This heuristic handles the common expression endings.
        before=' ';
        if i>1, before=code(i-1); end
        if isstrprop(before,'alphanum') || any(before=='_])}.')
            out(i)=c;i=i+1;continue
        end
        quoted=c;
    end
    out(i)=c;i=i+1;
end
code=out;
end

function f=cbf_find_resource(name,source,root,extraRoots)
f='';
if cbf_is_absolute(name)
    if isfile(name),f=cbf_abs(name);end
    return
end
folders=[{fileparts(source),pwd,root},extraRoots];
for j=1:numel(folders)
    candidate=fullfile(folders{j},name);
    if isfile(candidate),f=cbf_abs(candidate);return;end
end
w=which(name);
if ~isempty(w) && isfile(w),f=cbf_abs(w);end
end

function f=cbf_abs(f)
f=char(f);
if ~cbf_is_absolute(f),f=fullfile(pwd,f);end
try
    if usejava('jvm'),f=char(java.io.File(f).getCanonicalPath());end
catch
end
end
function tf=cbf_is_absolute(f)
f=char(f);
tf=(~isempty(f) && (f(1)=='/' || f(1)=='\')) || (numel(f)>1 && f(2)==':');
end
function tf=cbf_is_matlabroot(f)
tf=cbf_within(f,matlabroot);
end
function tf=cbf_within(f,folder)
a=strrep(char(f),'\','/');b=strrep(char(folder),'\','/');
if ispc,a=lower(a);b=lower(b);end
if ~isempty(b) && b(end)=='/',b(end)=[];end
tf=strcmp(a,b) || strncmp(a,[b '/'],numel(b)+1);
end
function rel=cbf_archive_name(f,root,extraRoots)
if cbf_within(f,root)
    rel=fullfile('project',cbf_rel(f,root));return
end
for j=1:numel(extraRoots)
    if cbf_within(f,extraRoots{j})
        rel=fullfile(sprintf('extra_%d',j),cbf_rel(f,extraRoots{j}));return
    end
end
p=strrep(f,'\','/');
if numel(p)>1 && p(2)==':',p=[p(1) '/' p(3:end)];end
p=regexprep(p,'^/+','');
rel=fullfile('external',strrep(p,'/',filesep));
end
function rel=cbf_rel(f,root)
a=strrep(f,'\','/');b=strrep(root,'\','/');
if ~isempty(b) && b(end)=='/',b(end)=[];end
rel=strrep(a(numel(b)+2:end),'/',filesep);
end
function cbf_csv_manifest(file,original,inside,reason)
fid=fopen(file,'w');if fid<0,error('Cannot create %s',file);end
c=onCleanup(@()fclose(fid)); %#ok<NASGU>
fprintf(fid,'"source_path","zip_path","reason"\n');
for i=1:numel(original)
    fprintf(fid,'"%s","%s","%s"\n',cbf_escape(original{i}),cbf_escape(inside{i}),cbf_escape(reason{i}));
end
end
function s=cbf_escape(s)
s=strrep(char(s),'"','""');
s=strrep(s,sprintf('\n'),' ');s=strrep(s,sprintf('\r'),' ');
end
function cbf_write_list(file,lines)
fid=fopen(file,'w');if fid<0,error('Cannot create %s',file);end
c=onCleanup(@()fclose(fid)); %#ok<NASGU>
for i=1:numel(lines),fprintf(fid,'%s\n',char(lines{i}));end
end
function cbf_remove_stage(stage)
try
    if isfolder(stage),rmdir(stage,'s');end
catch
end
end
