function zipPath = CBF_COLLECT_ACTIVE_GUI(action, varargin)
% CBF_COLLECT_ACTIVE_GUI  Runtime-first GUI dependency snapshot.
%   CBF_COLLECT_ACTIVE_GUI('start');
%   mainGUI
%   % Run flight, result plots, video and other required GUI callbacks.
%   zipPath = CBF_COLLECT_ACTIVE_GUI('export');
% All observed user files are included, regardless of Sim name.
% mainGUI is archived, but not expanded statically (candidate mode strings).
% Static dependencies of observed sources are expanded; CBF_GENERATOR is
% additionally archived in full. No simulation source is changed.
% See README_JA.txt. Profiling is not suitable for real-time benchmarks.

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
addParameter(ip,'ActiveSim','',@(x)ischar(x)||isstring(x));
addParameter(ip,'GeneratorRoots',{},@(x)iscell(x)||isstring(x)||ischar(x));
addParameter(ip,'ExtraRoots',{},@(x)iscell(x)||isstring(x));
addParameter(ip,'ExtraNames',{},@(x)iscell(x)||isstring(x));
addParameter(ip,'ExtraFiles',{},@(x)iscell(x)||isstring(x));
addParameter(ip,'OutputZip','',@(x)ischar(x)||isstring(x));
addParameter(ip,'MaxRounds',50,@(x)isnumeric(x)&&isscalar(x)&&x>=1&&x<=200);
parse(ip,varargin{:}); o=ip.Results;
if ~usejava('jvm'),error('CBF_COLLECT_ACTIVE_GUI:JVM','MATLAB with JVM is required for ZIP creation.');end

% Capture the table even if profiling was stopped manually. Do not clear it.
prof=struct(); profilerOn=false; profileError='';
try
    st=profile('status');
    profilerOn=strcmpi(char(st.ProfilerStatus),'on');
    prof=profile('info');
    profile off;
catch ME
    profileError=ME.message;
    try,profile off;catch,end
end

% Use the actual observed mainGUI path if available (not a Sim-name guess).
mainPath=which('mainGUI');
observedMain={};
if isfield(prof,'FunctionTable')
    for k=1:numel(prof.FunctionTable)
        f=cbf_profile_field(prof.FunctionTable(k),'FileName');
        [~,base,ext]=fileparts(f);
        if strcmpi(base,'mainGUI') && strcmpi(ext,'.m') && isfile(f)
            observedMain{end+1}=cbf_abs(f); %#ok<AGROW>
        end
    end
end
observedMain=unique(observedMain,'stable');
if isempty(char(o.ProjectRoot))
    if numel(observedMain)==1
        mainPath=observedMain{1};
    elseif numel(observedMain)>1
        error('CBF_COLLECT_ACTIVE_GUI:MultipleMain', ...
            'Multiple mainGUI paths observed. Specify ProjectRoot; all observed files will still be collected.');
    end
    if isempty(mainPath)
        error('CBF_COLLECT_ACTIVE_GUI:MainMissing','mainGUI not found. Specify ProjectRoot.');
    end
    root=cbf_abs(fileparts(mainPath));
else
    root=cbf_abs(char(o.ProjectRoot));
    mainPath=fullfile(root,'mainGUI.m');
end
if ~isfolder(root),error('ProjectRoot not found: %s',root);end
if ~isfile(mainPath),error('mainGUI.m not found: %s',mainPath);end
mainPath=cbf_abs(mainPath);
entryFiles=unique([{mainPath},observedMain],'stable');

extraRoots=cbf_as_cell(o.ExtraRoots);
for j=1:numel(extraRoots)
    extraRoots{j}=cbf_abs(extraRoots{j});
    if ~isfolder(extraRoots{j}), error('ExtraRoots{%d} not found',j); end
end
extraNames=cbf_as_cell(o.ExtraNames);
extraFiles=cbf_as_cell(o.ExtraFiles);

zipPath=char(o.OutputZip);
if isempty(zipPath)
    stem=['CBF_ACTIVE_GUI_' datestr(now,'yyyymmdd_HHMMSS')];
    zipPath=fullfile(root,[stem '.zip']);
    suffix=1;
    while isfile(zipPath) || isfolder(zipPath)
        zipPath=fullfile(root,sprintf('%s_%03d.zip',stem,suffix));
        suffix=suffix+1;
    end
elseif ~cbf_is_absolute(zipPath)
    zipPath=fullfile(root,zipPath);
end
[~,~,ext]=fileparts(zipPath);
if isempty(ext),zipPath=[zipPath '.zip'];elseif ~strcmpi(ext,'.zip'),error('OutputZip must end in .zip');end
zipPath=cbf_abs(zipPath);
if isfile(zipPath)||isfolder(zipPath),error('Output already exists: %s',zipPath);end
[outdir,~,~]=fileparts(zipPath);
if ~isfolder(outdir),mkdir(outdir);end

found=containers.Map('KeyType','char','ValueType','char');
roots={}; problems={}; unresolved={}; runtimeFiles={}; runtimeRows=cell(0,5);
toolboxNames={};
if ~isempty(profileError),problems{end+1}=['Profiler: ' profileError];end
if ~profilerOn
    problems{end+1}='Profiler was OFF at export: saved table may be incomplete or stale. Repeat start -> mainGUI -> GUI actions -> export.';
end
for k=1:numel(entryFiles)
    cbf_include(found,entryFiles{k},'entry-only:not-analyzed');
end

% No Sim-name exclusions. Every observed file is considered before analysis.
if isfield(prof,'FunctionTable')
    tab=prof.FunctionTable;
    for k=1:numel(tab)
        raw=cbf_profile_field(tab(k),'FileName');
        name=cbf_profile_field(tab(k),'FunctionName');
        calls=cbf_profile_field(tab(k),'NumCalls');
        f=raw;
        status='NO_FILE_REPORTED';
        if ~isempty(f)
            if isfile(f)
                f=cbf_abs(f);
                if cbf_is_matlabroot(f)
                    status='MATLAB_INSTALLATION_NOT_COPIED';
                elseif cbf_is_exporter(f)
                    status='COLLECTOR_NOT_COPIED';
                else
                    cbf_include(found,f,'runtime-observed');
                    runtimeFiles{end+1}=f; %#ok<AGROW>
                    status='RUNTIME_SOURCE';
                    if cbf_is_source(f) && ~any(strcmpi(f,entryFiles))
                        roots{end+1}=f; %#ok<AGROW>
                    end
                end
            elseif cbf_is_matlabroot(f)
                status='MATLAB_INSTALLATION_NOT_COPIED';
            else
                status='REPORTED_FILE_NOT_FOUND';
                unresolved{end+1}=['Profiler file not found: ' raw ' (' name ')']; %#ok<AGROW>
            end
        end
        runtimeRows(end+1,:)={name,raw,f,calls,status}; %#ok<AGROW>
    end
end
runtimeFiles=unique(runtimeFiles,'stable');
if isempty(runtimeFiles)
    problems{end+1}='No user runtime files observed. This is not a valid runtime snapshot.';
end
if ~any(ismember(runtimeFiles,entryFiles))
    problems{end+1}='mainGUI was not observed. Verify start was executed before mainGUI.';
end

% Legacy ActiveSim is accepted only as an optional extra seed; never filters.
if ~isempty(char(o.ActiveSim))
    extraNames=[extraNames,{char(o.ActiveSim)}];
end
for k=1:numel(extraNames)
    f=cbf_resolve_name(extraNames{k});
    if isempty(f)
        unresolved{end+1}=['ExtraNames not found: ' extraNames{k}]; %#ok<AGROW>
    elseif ~cbf_is_matlabroot(f) && ~cbf_is_exporter(f)
        cbf_include(found,f,'manual-extra-name');
        if cbf_is_source(f)&&~any(strcmpi(f,entryFiles)),roots{end+1}=f;end %#ok<AGROW>
    end
end
for k=1:numel(extraFiles)
    f=cbf_abs(extraFiles{k});
    if ~isfile(f)
        unresolved{end+1}=['ExtraFiles not found: ' f]; %#ok<AGROW>
    elseif ~cbf_is_matlabroot(f) && ~cbf_is_exporter(f)
        cbf_include(found,f,'manual-extra-file');
        if cbf_is_source(f)&&~any(strcmpi(f,entryFiles)),roots{end+1}=f;end %#ok<AGROW>
    end
end

% Multiple rounds: MATLAB DIRECT static dependencies (toponly), iterated
% until closure, with no Sim-name filter, plus literal names/data.
% mainGUI is intentionally not expanded. Generator is copied separately.
analyzed=containers.Map('KeyType','char','ValueType','logical');
scanned=containers.Map('KeyType','char','ValueType','logical');
roots=unique(roots,'stable');
for roundNo=1:o.MaxRounds
    todo={};
    for k=1:numel(roots)
        f=roots{k};
        if cbf_is_source(f) && ~any(strcmpi(f,entryFiles)) && ~isKey(analyzed,f)
            todo{end+1}=f; %#ok<AGROW>
            analyzed(f)=true;
        end
    end
    if ~isempty(todo)
        fprintf('[CBF EXPORT] Static round %d: %d roots\n',roundNo,numel(todo));
        try
            [deps,products]=matlab.codetools.requiredFilesAndProducts(todo,'toponly');
            [found,roots]=cbf_add_deps(found,roots,deps,entryFiles);
            if ~isempty(products), toolboxNames=[toolboxNames,{products.Name}]; end %#ok<AGROW>
        catch ME
            problems{end+1}=['batch static round ' num2str(roundNo) ': ' ME.message]; %#ok<AGROW>
            for k=1:numel(todo)
                try
                    [deps,products]=matlab.codetools.requiredFilesAndProducts(todo{k},'toponly');
                    [found,roots]=cbf_add_deps(found,roots,deps,entryFiles);
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
        % Do not expand mainGUI candidate arrays.
        if any(strcmpi(file,entryFiles)), continue; end
        try
            [names,assets]=cbf_literal_candidates(file);
            for j=1:numel(names)
                dep=cbf_resolve_name(names{j});
                if isempty(dep), continue; end
                if cbf_is_matlabroot(dep) || cbf_is_exporter(dep),continue;end
                wasPresent=isKey(found,dep);
                cbf_include(found,dep,['literal-in:' file]);
                if ~wasPresent && cbf_is_source(dep) && ~any(strcmpi(dep,entryFiles)),newly{end+1}=dep;end %#ok<AGROW>
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
        if cbf_is_source(roots{k}) && ~any(strcmpi(roots{k},entryFiles)) && ~isKey(analyzed,roots{k})
            hasMore=true; break
        end
    end
    if ~hasMore,break;end
end
% Explicitly report any incomplete closure when iteration cap was hit.
for k=1:numel(roots)
    if cbf_is_source(roots{k}) && ~any(strcmpi(roots{k},entryFiles)) && ~isKey(analyzed,roots{k})
        unresolved{end+1}=['UNANALYZED (MaxRounds reached): ' roots{k}]; %#ok<AGROW>
    end
end

% Full generator snapshot, independent of which generated function ran.
% Search known project locations plus ancestors of actual files/path entries.
% All discovered copies are retained separately; no generator is executed.
[generatorRoots,genIssues]=cbf_generator_roots(root,extraRoots,keys(found),o.GeneratorRoots);
unresolved=[unresolved,genIssues];
generatorDirs={}; generatorFiles={}; generatorRows=cell(0,4);
for k=1:numel(generatorRoots)
    gr=generatorRoots{k};
    [gf,gd,gi]=cbf_walk_generator(gr);
    generatorFiles=[generatorFiles,gf]; %#ok<AGROW>
    generatorDirs=[generatorDirs,gd]; %#ok<AGROW>
    unresolved=[unresolved,gi]; %#ok<AGROW>
    for j=1:numel(gf),cbf_include(found,gf{j},'generator-full-tree');end
    generatorRows(end+1,:)={gr,num2str(numel(gf)),num2str(numel(gd)),num2str(numel(gi))}; %#ok<AGROW>
end
generatorFiles=unique(generatorFiles,'stable');
generatorDirs=unique(generatorDirs,'stable');
if isempty(generatorRoots),unresolved{end+1}='CBF_GENERATOR folder not found. Use GeneratorRoots to specify its actual folder.';end

% Warn about dynamic file loading in included executable sources. Comments
% are stripped before checking. This is a review list, not automatic copying.
dynamicReview={};
allPaths=sort(keys(found));
for k=1:numel(allPaths)
    f=allPaths{k};
    if ~strcmpi(cbf_ext(f),'.m'),continue;end
    try
        code=cbf_strip_comments(fileread(f));
        pats=regexp(code,'(?<![A-Za-z0-9_])(feval|str2func|eval|load|fopen|readtable|readmatrix|imread|readstruct|importdata|addpath|run|load_system)\s*\(', 'tokens');
        if ~isempty(pats)
            dynamicReview{end+1}=sprintf('%s : %s',f,strjoin(unique(cellfun(@(x)x{1},pats,'UniformOutput',false)),', ')); %#ok<AGROW>
        end
    catch
    end
end

% Stage ordinary new files using byte copies, not source ACL/attributes.
% Only the private temporary directory is deleted; original files are not modified.
stage=cbf_abs(tempname);
mkdir(stage);
cleanup=onCleanup(@() cbf_remove_stage(stage)); %#ok<NASGU>
original={}; inside={}; why={}; copyErrors={};
for k=1:numel(generatorDirs)
    dest=fullfile(stage,cbf_archive_name(generatorDirs{k},root,extraRoots));
    if ~isfolder(dest)
        [ok,msg]=mkdir(dest);
        if ~ok,copyErrors{end+1}=[generatorDirs{k} ' : ' msg];end %#ok<AGROW>
    end
end
for k=1:numel(allPaths)
    f=allPaths{k};
    if strcmpi(f,zipPath),copyErrors{end+1}=['Output cannot be included in itself: ' f];continue;end %#ok<AGROW>
    rel=cbf_archive_name(f,root,extraRoots);
    dest=fullfile(stage,rel);
    try
        folder=fileparts(dest);
        if ~isfolder(folder),mkdir(folder);end
        if isfile(dest)||isfolder(dest),error('Archive path collision: %s',rel);end
        cbf_copy_bytes(f,dest);
        original{end+1}=f;inside{end+1}=strrep(rel,filesep,'/');why{end+1}=found(f); %#ok<AGROW>
    catch ME
        if isfile(dest) && ~any(strcmpi(strrep(rel,filesep,'/'),inside))
            try,delete(dest);catch,end
        end
        copyErrors{end+1}=[f ' : ' ME.message]; %#ok<AGROW>
    end
end
runtimeMissing=setdiff(runtimeFiles,original,'stable');
generatorMissing=setdiff(generatorFiles,original,'stable');
cbf_csv_manifest(fullfile(stage,'MANIFEST_FILES.csv'),original,inside,why);
cbf_write_csv(fullfile(stage,'MANIFEST_RUNTIME_FUNCTIONS.csv'), ...
    {'function_name','profiler_file','resolved_file','num_calls','status'},runtimeRows);
cbf_write_csv(fullfile(stage,'MANIFEST_GENERATOR_ROOTS.csv'), ...
    {'source_root','discovered_files','discovered_directories','walk_issues'},generatorRows);
cbf_write_list(fullfile(stage,'MANIFEST_EXECUTED.txt'),runtimeFiles);
cbf_write_list(fullfile(stage,'MANIFEST_RUNTIME_MISSING.txt'),runtimeMissing);
cbf_write_list(fullfile(stage,'MANIFEST_GENERATOR_MISSING.txt'),generatorMissing);
cbf_write_list(fullfile(stage,'MANIFEST_GENERATOR_FILES.txt'),generatorFiles);
cbf_write_list(fullfile(stage,'MANIFEST_GENERATOR_DIRS.txt'),generatorDirs);
cbf_write_list(fullfile(stage,'MANIFEST_ANALYSIS_ERRORS.txt'),unique(problems,'stable'));
cbf_write_list(fullfile(stage,'MANIFEST_UNRESOLVED.txt'),unique(unresolved,'stable'));
cbf_write_list(fullfile(stage,'MANIFEST_COPY_ERRORS.txt'),copyErrors);
cbf_write_list(fullfile(stage,'MANIFEST_DYNAMIC_REVIEW.txt'),dynamicReview);
cbf_write_list(fullfile(stage,'MANIFEST_TOOLBOXES.txt'),unique(toolboxNames,'stable'));
complete=profilerOn && ~isempty(runtimeFiles) && isempty(problems) && ...
    isempty(unresolved) && isempty(copyErrors) && isempty(runtimeMissing) && ...
    ~isempty(generatorRoots) && isempty(generatorMissing);
summary={...
    'Collector: runtime-first, full-generator, 20261125',...
    ['Generated: ' datestr(now,31)], ...
    ['MATLAB version: ' version], ...
    ['ProjectRoot: ' root], ...
    ['mainGUI entry (NOT statically expanded): ' mainPath], ...
    ['Profiler on at capture: ' mat2str(profilerOn)], ...
    sprintf('Profiler function records: %d',size(runtimeRows,1)),...
    sprintf('Runtime files: %d',numel(runtimeFiles)),...
    sprintf('Runtime files not archived: %d',numel(runtimeMissing)),...
    sprintf('Generator roots: %d',numel(generatorRoots)),...
    sprintf('Generator files: %d',numel(generatorFiles)),...
    sprintf('Generator files not archived: %d',numel(generatorMissing)),...
    sprintf('Archived files: %d',numel(original)),...
    sprintf('Analyzer/review errors: %d',numel(problems)),...
    sprintf('Unresolved items: %d',numel(unresolved)),...
    sprintf('Copy errors: %d',numel(copyErrors)),...
    ['Recorded-scope checks passed: ' mat2str(complete)],...
    'No selected-Sim filter; every observed user file is eligible.',...
    'Only CBF_GENERATOR is recursively copied in full; mainGUI is not statically expanded.',...
    'This is an export-time source snapshot, not a historical byte trace or proof of all dynamic dependencies.',...
    'Review all MANIFEST_* and remove sensitive content before sharing.'};
cbf_write_list(fullfile(stage,'MANIFEST_SUMMARY.txt'),summary);
cbf_write_list(fullfile(stage,'README_RESTORE.txt'),{...
    'Runtime-first MATLAB GUI snapshot; no Sim-name filtering.',...
    'project/ contains paths relative to mainGUI; extra_N/ and external/ contain other sources.',...
    'CBF_GENERATOR is included in full at its mapped original path (including unexecuted code/data).',...
    'MANIFEST_RUNTIME_FUNCTIONS.csv records profiler entries, including MATLAB/builtin records when reported.',...
    'MANIFEST_FILES.csv gives actual archived paths and provenance.',...
    'Review missing/error lists and generator roots; do not treat a ZIP as proof of completeness.',...
    'Configure MATLAB path after extracting. MathWorks installation is not copied.',...
    'Source content is read at export; do not edit files between flight and export.',...
    'No original file permissions, owner, ACL or elevated privileges are requested by the collector.'});
% Explicit ZIP entries (including hidden files and empty directories).
% Java ZIP stores ordinary path/content entries, not source file ACLs.
try
    cbf_zip_stage(stage,zipPath);
catch ME
    if isfile(zipPath),try,delete(zipPath);catch,end,end
    rethrow(ME);
end
if ~isfile(zipPath),error('ZIP was not created: %s',zipPath);end
fprintf('[CBF EXPORT] Created %s\n',zipPath);
fprintf('[CBF EXPORT] Archived=%d, runtime=%d (missing=%d), generator=%d (missing=%d)\n',...
    numel(original),numel(runtimeFiles),numel(runtimeMissing),numel(generatorFiles),numel(generatorMissing));
if ~complete
    warning('CBF_COLLECT_ACTIVE_GUI:Review', ...
        'ZIP created with review items. Inspect MANIFEST_SUMMARY and missing/error lists.');
end
end

function c=cbf_as_cell(x)
if isempty(x),c={};elseif isstring(x),c=cellstr(x);elseif iscell(x),c=x;else,c={x};end
c=c(:)';
for i=1:numel(c),c{i}=char(c{i});end
end

function [found,roots]=cbf_add_deps(found,roots,deps,entryFiles)
if isstring(deps),deps=cellstr(deps);end
for j=1:numel(deps)
    f=char(deps{j});
    if ~isfile(f),continue;end
    f=cbf_abs(f);
    if ~cbf_is_matlabroot(f) && ~cbf_is_exporter(f)
        cbf_include(found,f,'static-dependency');
        if cbf_is_source(f)&&~any(strcmpi(f,entryFiles)),roots{end+1}=f;end %#ok<AGROW>
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
if strcmp(a,b)||(ispc&&strcmpi(a,b)),rel='';return;end
if ~isempty(b) && b(end)=='/',b(end)=[];end
rel=strrep(a(numel(b)+2:end),'/',filesep);
end
function cbf_csv_manifest(file,original,inside,reason)
fid=fopen(file,'w','n','UTF-8');if fid<0,error('Cannot create %s',file);end
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
fid=fopen(file,'w','n','UTF-8');if fid<0,error('Cannot create %s',file);end
c=onCleanup(@()fclose(fid)); %#ok<NASGU>
for i=1:numel(lines),fprintf(fid,'%s\n',char(lines{i}));end
end
function cbf_remove_stage(stage)
try
    if isfolder(stage)
        [ok,msg]=rmdir(stage,'s');
        if ~ok,warning('CBF_COLLECT_ACTIVE_GUI:Cleanup','Temporary folder %s: %s',stage,msg);end
    end
catch ME
    warning('CBF_COLLECT_ACTIVE_GUI:Cleanup','Temporary folder %s: %s',stage,ME.message);
end
end

function val=cbf_profile_field(row,name)
val='';
if ~isfield(row,name),return;end
x=row.(name);
if ischar(x)||isstring(x),val=char(x);
elseif isnumeric(x)&&isscalar(x),val=num2str(x);end
end

function [gr,issues]=cbf_generator_roots(root,extraRoots,observed,manual)
gr={};issues={};
% Exact known layouts, not a recursive search of the project or disk.
base=[{root},extraRoots];
for j=1:numel(base)
    b=base{j};
    candidates={b,fullfile(b,'CBF_GENERATOR'), ...
        fullfile(b,'model','substance','CBF_GENERATOR'), ...
        fullfile(b,'substance','CBF_GENERATOR')};
    for k=1:numel(candidates)
        c=candidates{k};[~,n]=fileparts(c);
        if strcmpi(n,'CBF_GENERATOR')&&isfolder(c)&&~cbf_is_matlabroot(c)
            gr{end+1}=cbf_abs(c); %#ok<AGROW>
        end
    end
end
% Recognize generator ancestors of resolved files and MATLAB path entries.
pp=strsplit(path(),pathsep());
for j=1:numel(observed),pp{end+1}=fileparts(observed{j});end %#ok<AGROW>
for j=1:numel(pp)
    d=pp{j};
    if isempty(d)||cbf_is_matlabroot(d),continue;end
    d=cbf_abs(d);
    while ~isempty(d)
        [parent,n]=fileparts(d);
        if strcmpi(n,'CBF_GENERATOR')&&isfolder(d)&&~cbf_is_matlabroot(d)
            gr{end+1}=d;break; %#ok<AGROW>
        end
        if isempty(parent)||strcmp(parent,d),break;end
        d=parent;
    end
end
manual=cbf_as_cell(manual);
for j=1:numel(manual)
    d=cbf_abs(manual{j});[~,n]=fileparts(d);
    if isfolder(d)&&strcmpi(n,'CBF_GENERATOR')&&~cbf_is_matlabroot(d)
        gr{end+1}=d; %#ok<AGROW>
    else
        issues{end+1}=['GeneratorRoots entry must be an existing CBF_GENERATOR folder: ' d]; %#ok<AGROW>
    end
end
gr=unique(gr,'stable');
end

function [files,dirs,issues]=cbf_walk_generator(root)
% Explicit traversal includes hidden files and empty directories. Do not
% follow symbolic links/junctions to unrelated paths or loop recursively.
root=cbf_abs(root); files={};dirs={};issues={};queue={root};
visited=containers.Map('KeyType','char','ValueType','logical');
while ~isempty(queue)
    d=queue{end};queue(end)=[];
    key=cbf_abs(d);if ispc,key=lower(key);end
    if isKey(visited,key)
        issues{end+1}=['Repeated directory not followed: ' d];continue; %#ok<AGROW>
    end
    visited(key)=true;
    dirs{end+1}=d; %#ok<AGROW>
    try
        jf=java.io.File(d);
        stream=javaMethod('newDirectoryStream','java.nio.file.Files',jf.toPath());
        guard=onCleanup(@()stream.close());
        it=stream.iterator();
        while it.hasNext()
            jp=it.next();
            child=char(jp.toString());
            canonical=cbf_abs(child);
            if ispc,same=strcmpi(child,canonical);else,same=strcmp(child,canonical);end
            if javaMethod('isSymbolicLink','java.nio.file.Files',jp) || ...
                    ~same || ~cbf_within(canonical,root)
                issues{end+1}=['Linked or redirected entry not followed: ' child]; %#ok<AGROW>
            elseif isfolder(child)
                queue{end+1}=child; %#ok<AGROW>
            elseif isfile(child)
                files{end+1}=child; %#ok<AGROW>
            else
                issues{end+1}=['Unsupported or missing tree entry: ' child]; %#ok<AGROW>
            end
        end
        clear guard
    catch ME
        clear guard
        issues{end+1}=['Directory enumeration failed: ' d ' : ' ME.message]; %#ok<AGROW>
    end
end
files=sort(files);dirs=sort(dirs);
end

function cbf_write_csv(file,header,rows)
fid=fopen(file,'w','n','UTF-8');if fid<0,error('Cannot create %s',file);end
c=onCleanup(@()fclose(fid)); %#ok<NASGU>
allRows=[header;rows];
for i=1:size(allRows,1)
    line=cellfun(@(x)['"' cbf_escape(x) '"'],allRows(i,:),'UniformOutput',false);
    fprintf(fid,'%s\n',strjoin(line,','));
end
end

function cbf_copy_bytes(source,dest)
% No copyfile('f'), no ACL/owner/attribute copying, no chmod/attrib.
% Copy the bytes to a newly created ordinary file via OutputStream.
src=java.io.File(source);dst=java.io.File(dest);
fout=java.io.FileOutputStream(dst);
guard=onCleanup(@()fout.close());
n=javaMethod('copy','java.nio.file.Files',src.toPath(),fout);
clear guard
if n~=src.length()||n~=dst.length(),error('Copied length differs: %s',source);end
end

function cbf_zip_stage(stage,zipPath)
% ZIP built using ordinary ZipEntry paths; no Windows ACL/Unix mode metadata
% copied from the source. Explicit entries avoid wildcard/hidden-file issues.
stage=cbf_abs(stage);
[files,dirs,issues]=cbf_walk_generator(stage);
if ~isempty(issues),error('Cannot enumerate temporary stage: %s',strjoin(issues,' | '));end
fos=java.io.FileOutputStream(zipPath);
fosGuard=onCleanup(@()fos.close()); %#ok<NASGU>
zos=java.util.zip.ZipOutputStream(fos);
zipGuard=onCleanup(@()zos.close());
names={};sizes=[];
for j=1:numel(dirs)
    if strcmp(dirs{j},stage),continue;end
    name=[strrep(cbf_rel(dirs{j},stage),'\','/') '/'];
    ze=java.util.zip.ZipEntry(name);
    zos.putNextEntry(ze);zos.closeEntry();
    names{end+1}=name;sizes(end+1)=0; %#ok<AGROW>
end
for j=1:numel(files)
    f=files{j}; name=strrep(cbf_rel(f,stage),'\','/');
    ze=java.util.zip.ZipEntry(name);
    zos.putNextEntry(ze);
    jf=java.io.File(f);
    n=javaMethod('copy','java.nio.file.Files',jf.toPath(),zos);
    zos.closeEntry();
    if n~=jf.length(),error('ZIP copy length differs: %s',f);end
    names{end+1}=name;sizes(end+1)=double(n); %#ok<AGROW>
end
zos.finish();clear zipGuard
% Verify actual ZIP entry names and uncompressed sizes before success.
zf=java.util.zip.ZipFile(zipPath);
zfGuard=onCleanup(@()zf.close()); %#ok<NASGU>
if zf.size()~=numel(names),error('ZIP entry count mismatch.');end
for j=1:numel(names)
    entry=zf.getEntry(names{j});
    if isempty(entry)||double(entry.getSize())~=sizes(j)
        error('ZIP entry missing or wrong length: %s',names{j});
    end
end
end
