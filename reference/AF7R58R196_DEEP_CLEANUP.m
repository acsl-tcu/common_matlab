function R = AF7R58R196_DEEP_CLEANUP(action, argument)
% R196. Reversible archival of historical MATLAB research artifacts at
% common_matlab/ and common_matlab/reference/. Does not alter Flight code.
% PREVIEW      : list obsolete-looking files including Git-tracked entries
% QUARANTINE   : move untracked/ignored candidates only (explicit consent)
% QUARANTINE_ALL: move untracked and CLEAN Git-tracked candidates (consent)
% RESTORE      : restore an R196 quarantine; never overwrite live files
% SELFTEST     : check protective classifier against current installation
%
% MATLAB execution and dynamic function dispatch cannot be proven complete
% by a static text scan. The move is reversible, not an assertion of disuse.
if nargin<1||isempty(action),action='PREVIEW';end
if nargin<2,argument='';end
action=upper(char(action));
assert(any(strcmp(action,{'PREVIEW','QUARANTINE','QUARANTINE_ALL','RESTORE','SELFTEST'})), ...
    'R196:Action','Unknown action.');
ref=fileparts(mfilename('fullpath'));
[project,leaf]=fileparts(ref);
assert(strcmpi(leaf,'reference') && isfile(fullfile(ref,'REPLANNING_SYSTEM.m')) ...
    && isfile(fullfile(project,'run_flight_headless.m')), ...
    'R196:Location','Place R196 in the canonical common_matlab/reference.');
assert(strcmpi(r196_norm(which('REPLANNING_SYSTEM')), ...
                    r196_norm(fullfile(ref,'REPLANNING_SYSTEM.m'))), ...
    'R196:Shadow','MATLAB resolves a different REPLANNING_SYSTEM.');
output=fullfile(project,'_AF7_HANDOFF_EXPORT');
if ~isfolder(output),[ok,msg]=mkdir(output);assert(ok,'R196:Output','%s',msg);end
stamp=datestr(now,'yyyymmdd_HHMMSS_FFF');
R=struct('status','NOT_RUN','project_root',project,'action',action, ...
    'candidate_count',0,'tracked_count',0,'untracked_count',0, ...
    'candidate_files',0,'candidate_directories',0, ...
    'root_candidate_count',0,'reference_candidate_count',0, ...
    'protected_count',0,'candidate_paths',{{}},'moved_count',0, ...
    'moved_directories',0,'blocked_count',0,'blocked_paths',{{}}, ...
    'quarantine_path','','report_path', ...
    fullfile(output,['R196_' action '_' stamp '.txt']), ...
    'dependency_protection','SOURCE_TEXT_TRANSITIVE_AND_LIVE_PROFILE_IF_FOUND', ...
    'flight_modified',false);
if strcmp(action,'RESTORE')
    R=r196_restore(R,char(argument),output);return;
end
[gitOK,tracked,dirty]=r196_git(project);
assert(gitOK,'R196:Git','Git is not available; nothing moved.');
[protected,scanStatus]=r196_protected(project,ref,output);
R.dependency_protection=scanStatus;
loaded=inmem('-completenames');
loaded=cellfun(@r196_norm,loaded,'UniformOutput',false);
% Only the two immediate levels are screened. No arbitrary directory tree
% is traversed for candidate files. Historic dated capture folders are
% handled as a single item after directory safety checks.
rows=cell(0,6); % relative, absolute, isDir, isTracked, isDirty, type
for j=1:2
    if j==1,base=project;prefix='';else,base=ref;prefix='reference/';end
    d=dir(base);
    for k=1:numel(d)
        n=d(k).name;
        if any(strcmp(n,{'.','..'})),continue;end
        isDir=logical(d(k).isdir);rel=[prefix n];lo=lower(strrep(rel,'\','/'));
        if r196_absolute_protect(n,isDir),continue;end
        typ=r196_class(n,isDir);
        if isempty(typ),continue;end
        if isDir
            % Only timestamped AF7 research output directories.
            if ~r196_safe_dir(fullfile(base,n),project),continue;end
            if any(startsWith(protected,[lo '/']))||any(startsWith(loaded,[r196_norm(fullfile(base,n)) '/'])),continue;end
        end
        if any(strcmp(lo,protected)) || any(strcmp(r196_norm(fullfile(base,n)),loaded)),continue;end
        if ~isDir && any(strcmp(typ,{'CODE','LEGACY_CODE'})) && ...
                r196_source_reference(project,ref,n),continue;end
        tr=any(strcmp(lo,tracked)) || (isDir&&any(startsWith(tracked,[lo '/'])));
        di=any(strcmp(lo,dirty)) || (isDir&&any(startsWith(dirty,[lo '/'])));
        % A dirty Git source, even if otherwise obsolete, is NEVER moved.
        if di,continue;end
        rows(end+1,:)={rel,fullfile(base,n),isDir,tr,di,typ}; %#ok<AGROW>
    end
end
R.candidate_count=size(rows,1);
R.candidate_paths=rows(:,1);
if ~isempty(rows)
    R.candidate_directories=sum(cell2mat(rows(:,3)));
    R.candidate_files=R.candidate_count-R.candidate_directories;
    R.tracked_count=sum(cell2mat(rows(:,4)));
    R.untracked_count=R.candidate_count-R.tracked_count;
    R.reference_candidate_count=sum(startsWith(lower(string(rows(:,1))),'reference/'));
    R.root_candidate_count=R.candidate_count-R.reference_candidate_count;
end
header={sprintf('R196 %s candidates=%d files=%d folders=%d tracked=%d untracked=%d', ...
    action,R.candidate_count,R.candidate_files,R.candidate_directories,R.tracked_count,R.untracked_count),...
    ['Dependency protection: ' R.dependency_protection],...
    'Git-dirty files protected; loaded code protected; runner/main/HLC/EKF/plant protected.',...
    'MEX, C/C++ build sources and class/package directories protected.',...
    'Git-tracked clean candidates require QUARANTINE_ALL explicit confirmation.',...
    'Static closure does NOT prove dynamically constructed calls are unused.',...
    'No deletions, chmod, takeown, icacls or administrator elevation.',...
    '--- candidates: TRACKED/UNTRACKED  KIND  PROJECT-RELATIVE-PATH ---'};
entries=cell(size(rows,1),1);
for k=1:size(rows,1)
    if rows{k,4},t='TRACKED';else,t='UNTRACKED';end
    entries{k}=sprintf('%s\t%s\t%s',t,char(rows{k,6}),char(rows{k,1}));
end
if any(strcmp(action,{'PREVIEW','SELFTEST'}))
    if strcmp(action,'SELFTEST')
        assert(~any(strcmpi(R.candidate_paths,'reference/REPLANNING_SYSTEM.m')) && ...
               ~any(strcmpi(R.candidate_paths,'run_flight_headless.m')), ...
               'R196:SelfTest','Protected core is candidate.');
        R.status='PASS_R196_PROTECTIVE_CLASSIFIER__NO_FILES_MOVED';
    else
        R.status='R196_PREVIEW_COMPLETE__NO_FILES_MOVED';
    end
    r196_lines(R.report_path,[header(:);entries(:)]);
    fprintf('[R196] %d candidates (root=%d, reference=%d, tracked=%d, untracked=%d)\n%s\n', ...
        R.candidate_count,R.root_candidate_count,R.reference_candidate_count, ...
        R.tracked_count,R.untracked_count,R.report_path);
    return;
end
if strcmp(action,'QUARANTINE')
    assert(strcmp(char(argument),'CONFIRM_R196_UNTRACKED'), ...
        'R196:Confirm','Pass exact CONFIRM_R196_UNTRACKED.');
    if ~isempty(rows),rows=rows(~cell2mat(rows(:,4)),:);end
else
    assert(strcmp(char(argument),'CONFIRM_R196_TRACKED_GIT_CHANGES'), ...
        'R196:ConfirmTracked','Pass exact CONFIRM_R196_TRACKED_GIT_CHANGES.');
end
R=r196_move(R,rows,output,stamp,project,header,entries);
end

function [ok,tracked,dirty]=r196_git(project)
[st,t]=system(sprintf('git -c core.quotePath=false -C "%s" ls-files --cached',project));
[sd,d]=system(sprintf('git -c core.quotePath=false -C "%s" ls-files --modified',project));
[ss,s]=system(sprintf('git -c core.quotePath=false -C "%s" diff --cached --name-only',project));
ok=st==0&&sd==0&&ss==0 && ...
   isempty(regexp([sprintf('\n') t sprintf('\n') d sprintf('\n') s], ...
    '[\r\n]\"','once'));
tracked=r196_lines_parse(t);dirty=unique([r196_lines_parse(d);r196_lines_parse(s)]);
end
function c=r196_lines_parse(s)
x=regexp(strrep(char(s),sprintf('\r'),''),'\n','split');c={};
for j=1:numel(x)
    v=strtrim(x{j});if isempty(v),continue;end
    % Git may quote exotic filenames; protect those by not proposing them.
    if startsWith(v,'"'),continue;end
    c{end+1,1}=lower(strrep(v,'\','/')); %#ok<AGROW>
end
c=unique(c);
end
function [protected,status]=r196_protected(project,ref,out)
% Lexical transitive closure over actual names. This avoids MATLAB depfun's
% BadSyntax failure from historic files, while erring toward preservation.
protected={'reference/replanning_system.m';'run_flight_headless.m'};
files={};names={};rels={};
% Read top-level callable names as resolution candidates.
for i=1:2
    if i==1,p=project;pre='';else,p=ref;pre='reference/';end
    d=dir(fullfile(p,'*.m'));
    for j=1:numel(d)
        f=fullfile(p,d(j).name);[~,stem]=fileparts(f);
        files{end+1,1}=f;names{end+1,1}=lower(stem); %#ok<AGROW>
        rels{end+1,1}=lower([pre d(j).name]); %#ok<AGROW>
    end
end
% Resolve core classes from MATLAB's active path, not historical snapshots.
seeds={fullfile(ref,'REPLANNING_SYSTEM.m');fullfile(project,'run_flight_headless.m')};
core={'HLC_SUSPENDED_LOAD','Controller_HL_Suspended_Load','EKF', ...
    'Estimator_EKF_SuspendedLoad','TIME_VARYING_REFERENCE', ...
    'SUSPENDED_LOAD_REF_ADJUST','Controller_HL_Suspended_Load', ...
    'SUSPENDED_LOAD','SUSPENDED_LOAD_PLANT'};
for j=1:numel(core)
    w=which(core{j});
    if ~isempty(w)&&isfile(w),seeds{end+1,1}=w;end %#ok<AGROW>
end
% Also protect runtime files already captured by R193, including traces
% whose static dependency audit failed. Ignore missing paths.
last=fullfile(out,'R193_LAST_STATUS.mat');
if isfile(last)
    try
        S=load(last,'R');
        if isfield(S,'R')&&isstruct(S.R)&&isfield(S.R,'active_project_paths')
            a=S.R.active_project_paths;
            for j=1:numel(a)
                rel=lower(strrep(char(a{j}),'\','/'));
                if r196_rel_safe(rel)
                    protected{end+1,1}=rel; %#ok<AGROW>
                    p=fullfile(project,strrep(rel,'/',filesep));
                    if isfile(p)&&endsWith(lower(p),'.m'),seeds{end+1,1}=p;end %#ok<AGROW>
                end
            end
        end
    catch
    end
end
% Include subfiles of active HLC/EKF class folders in lexical scan; they
% may call AF7 functions through methods not visible in the top-level main.
for j=1:numel(core)
    w=which(core{j});
    if isempty(w)||~isfile(w),continue;end
    parent=fileparts(w);
    if contains(lower(strrep(parent,'\','/')),'/@')
        d=dir(fullfile(parent,'*.m'));
        for k=1:numel(d),seeds{end+1,1}=fullfile(parent,d(k).name);end %#ok<AGROW>
    end
end
% Include project-base .m files that are not old AF7 scripts to capture
% traditional controller and environment entry points / string dispatch.
for j=1:numel(files)
    if ~startsWith(names{j},'af7') && ~startsWith(names{j},'g27') && ...
       ~startsWith(names{j},'g26') && ~startsWith(names{j},'replanning_')
        seeds{end+1,1}=files{j}; %#ok<AGROW>
    end
end
queue=unique(seeds,'stable');seen=containers.Map('KeyType','char','ValueType','logical');
nameIndex=containers.Map('KeyType','char','ValueType','any');
for j=1:numel(names)
    stem=names{j};
    if isKey(nameIndex,stem),nameIndex(stem)=[nameIndex(stem),j];
    else,nameIndex(stem)=j;end
end
processed=0;
while ~isempty(queue)
    file=queue{end};queue(end)=[];
    key=r196_norm(file);
    if isKey(seen,key)||~isfile(file),continue;end
    seen(key)=true;processed=processed+1;
    if ~endsWith(lower(file),'.m'),continue;end
    try
        txt=lower(fileread(file));
    catch
        continue;
    end
    % Any occurrence, including in quoted strings/comments, is protective.
    toks=unique(regexp(txt,'[a-z_][a-z_0-9]{2,}','match'));
    for t=1:numel(toks)
        token=toks{t};
        if ~isKey(nameIndex,token),continue;end
        ix=nameIndex(token);
        for h=1:numel(ix)
            id=ix(h);protected{end+1,1}=rels{id}; %#ok<AGROW>
            if ~isKey(seen,r196_norm(files{id}))
                queue{end+1,1}=files{id}; %#ok<AGROW>
            end
        end
    end
end
protected=unique(protected);
status=sprintf('LEXICAL_TRANSITIVE_%d_FILES_%d_PROTECTED',processed,numel(protected));
end
function tf=r196_source_reference(project,ref,n)
% Additional guard for direct mention in source entry points, cached.
persistent priorKey text
try
    a=dir(fullfile(ref,'REPLANNING_SYSTEM.m'));
    b=dir(fullfile(project,'run_flight_headless.m'));
    stamp=[a.datenum a.bytes b.datenum b.bytes];
    if isempty(priorKey)||~isequal(priorKey,stamp)
        text=[lower(fileread(fullfile(ref,'REPLANNING_SYSTEM.m'))) newline ...
              lower(fileread(fullfile(project,'run_flight_headless.m')))];
        priorKey=stamp;
    end
    [~,stem]=fileparts(n);
    pat=['(?<![a-zA-Z0-9_])' regexptranslate('escape',lower(stem)) '(?![a-zA-Z0-9_])'];
    tf=~isempty(regexp(text,pat,'once'));
catch
    tf=true; % unreadable main => do not remove code
end
end
function tf=r196_absolute_protect(n,isDir)
lo=lower(char(n));
if isDir
    tf=any(strcmp(lo,{'.git','reference','_af7_handoff_export', ...
      'functions','baseline','controller','estimator','plant','sensor', ...
      'environment','model','models','system','runtime_support', ...
      'suspendedload','docs','private'})) || startsWith(lo,'@') || startsWith(lo,'+');
else
    tf=any(strcmp(lo,{'replanning_system.m','run_flight_headless.m', ...
         'time_varying_reference.m','suspended_load_ref_adjust.m', ...
         'hlc_suspended_load.m','controller_hl_suspended_load.m', ...
         'ekf.m','estimator_ekf_suspendedload.m', ...
         'af7r58r170_generate.m','af7r58r172_strict_trigger.m', ...
         'af7r58r172_strict_generator.m','af7r58r181_live_trigger.m', ...
         'af7r58r182_interval_trigger.m','af7r58r187_rt_episode.m', ...
         'af7r58r188_rt_event.m','af7r58r189_c7_live_trial.m', ...
         'af7r58r189_reference_hull.m','af7r58r191_ekf_tick_trigger.m', ...
         'af7r58r191_hull_separate.m','af7r58r191_sim_verify.m', ...
         'af7r58r191_postflight.m'})) || ...
         startsWith(lo,'af7r58r196_') || ...
         ~isempty(regexp(lo,'\.(mexw64|mexa64|mexmaci64|cpp|c|cc|h|hpp|p)$','once'));
end
end
function typ=r196_class(n,isDir)
lo=lower(char(n));typ='';
if isDir
    if ~isempty(regexp(lo,'^af7[a-z0-9_\-]*(result|view|viz|audit|recover|collect|capture|report|preview|output|staged|test)[a-z0-9_\-]*_20\d{6}_\d{6}(_\d+)?$','once'))
        typ='OLD_DATED_FOLDER';
    end
    return;
end
[~,stem,ext]=fileparts(lo);
% Research patch and experiment scripts, not ordinary application code.
old=startsWith(stem,'af7') || ...
    ~isempty(regexp(stem,'^g(26|27)[a-z0-9_]+$','once'));
legacy=startsWith(stem,'replanning_') && ...
    ~strcmp(stem,'replanning_system');
if strcmp(ext,'.m')
    if old,typ='CODE';elseif legacy,typ='LEGACY_CODE';end
elseif any(strcmp(ext,{'.source','.asv','.bak','.backup','.txt','.md', ...
                        '.mat','.fig','.png','.csv','.zip','.py','.dat'}))
    if old||legacy||startsWith(stem,'readme')|| ...
      startsWith(stem,'r79_')||startsWith(stem,'g27')|| ...
      contains(stem,'backup')||contains(stem,'report')
        typ='OLD_ARTIFACT';
    end
elseif startsWith(ext,'.r19')||contains(lo,'_backup_')
    if old,typ='OLD_BACKUP';end
end
end
function tf=r196_rel_safe(rel)
tf=~isempty(rel)&&~startsWith(rel,'/')&&~contains(rel,'..') ...
    &&~contains(rel,':') &&~contains(rel,'//');
end
function s=r196_norm(p)
s=lower(strrep(char(p),'\','/'));
while endsWith(s,'/')&&numel(s)>1,s=s(1:end-1);end
end
function tf=r196_safe_dir(folder,root)
tf=false;
try
    a=java.io.File(folder);b=java.io.File(root);
    ca=r196_norm(char(a.getCanonicalPath()));
    ab=r196_norm(char(a.getAbsolutePath()));
    br=r196_norm(char(b.getCanonicalPath()));
    tf=strcmp(ca,ab)&&startsWith(ca,[br '/']);
catch
end
end
function R=r196_move(R,rows,out,stamp,project,header,entries)
q=fullfile(out,['R196_QUARANTINE_' stamp]);
[ok,msg]=mkdir(q);assert(ok,'R196:QuarantineFolder','%s',msg);
manifest=fullfile(q,'R196_MANIFEST.tsv');
f=fopen(manifest,'wt');assert(f>0,'R196:Manifest','Cannot write manifest.');
cl=onCleanup(@()fclose(f)); %#ok<NASGU>
fprintf(f,'relative_path\tkind\tsha256\tfile_count\tgit_tracked\n');
moved={};blocked={};movedDirs=0;
for k=1:size(rows,1)
    rel=char(rows{k,1});src=char(rows{k,2});isDir=logical(rows{k,3});
    if ~r196_rel_safe(rel)
        blocked{end+1,1}=[rel ' / UNSAFE_RELATIVE_PATH'];continue;end %#ok<AGROW>
    if ~isfile(src)&&~isfolder(src)
        blocked{end+1,1}=[rel ' / NO_LONGER_EXISTS'];continue;end %#ok<AGROW>
    if isDir&&~r196_safe_dir(src,project)
        blocked{end+1,1}=[rel ' / JUNCTION_OR_OUTSIDE_ROOT'];continue;end %#ok<AGROW>
    try
        [before,count]=r196_digest(src,isDir,project);
        if isempty(before),error('R196:Unreadable','Could not hash candidate');end
        dst=fullfile(q,strrep(rel,'/',filesep));
        parent=fileparts(dst);if ~isfolder(parent),mkdir(parent);end
        if isfile(dst)||isfolder(dst),error('R196:Collision','Destination exists');end
        [ok,msg]=movefile(src,dst);
        if ~ok,error('R196:Move','%s',msg);end
        [after,count2]=r196_digest(dst,isDir,q);
        if ~strcmp(before,after)||count~=count2
            [ok,msg]=movefile(dst,src);
            if ~ok,error('R196:Rollback','Digest mismatch; rollback failed: %s',msg);end
            error('R196:Integrity','Moved item digest mismatch, reverted');
        end
        if isDir,movedDirs=movedDirs+1;end
        fprintf(f,'%s\t%s\t%s\t%d\t%d\n',rel,r196_kind(isDir),before,count,logical(rows{k,4}));
        moved{end+1,1}=rel; %#ok<AGROW>
    catch ME
        blocked{end+1,1}=[rel ' / ' ME.identifier ' / ' ME.message]; %#ok<AGROW>
    end
end
R.quarantine_path=q;
R.moved_count=numel(moved);R.moved_directories=movedDirs;
R.blocked_count=numel(blocked);R.blocked_paths=blocked;
if isempty(blocked),R.status='R196_QUARANTINE_VERIFIED__NO_DELETION';
else,R.status='R196_PARTIAL_QUARANTINE__SEE_BLOCKED_REPORT';end
r196_lines(R.report_path,[{R.status};header(:);entries(:); ...
    {'--- moved ---'};moved(:);{'--- blocked ---'};blocked(:); ...
    {'QUARANTINE_DIRECTORY';q}]);
fprintf('[R196] moved=%d (folders=%d), blocked=%d\n%s\n', ...
    R.moved_count,R.moved_directories,R.blocked_count,R.report_path);
rehash;
end
function s=r196_kind(isDir)
if isDir,s='DIR';else,s='FILE';end
end
function [hash,count]=r196_digest(src,isDir,safeRoot)
hash='';count=0;
if ~isDir
    hash=r196_sha(src);if ~isempty(hash),count=1;end
    return;
end
if ~isfolder(src),return;end
entries={};stack={{src,''}};
while ~isempty(stack)
    node=stack{end};stack(end)=[];
    kids=dir(node{1});
    for k=1:numel(kids)
        n=kids(k).name;
        if any(strcmp(n,{'.','..'})),continue;end
        path=fullfile(node{1},n);rel=[node{2} n];
        if kids(k).isdir
            if ~r196_safe_dir(path,safeRoot),return;end
            entries{end+1,1}=['DIR:' rel]; %#ok<AGROW>
            stack{end+1}={path,[rel '/']}; %#ok<AGROW>
        else
            h=r196_sha(path);if isempty(h),return;end
            entries{end+1,1}=['FILE:' rel ':' h]; %#ok<AGROW>
            count=count+1;
        end
    end
end
hash=r196_sha_text(strjoin(sort(entries),sprintf('\n')));
end
function hash=r196_sha(p)
hash='';fid=fopen(p,'rb');if fid<0,return;end
cl=onCleanup(@()fclose(fid)); %#ok<NASGU>
try
    md=java.security.MessageDigest.getInstance('SHA-256');
    while ~feof(fid)
        b=fread(fid,1048576,'*uint8');
        if ~isempty(b),md.update(typecast(b(:),'int8'));end
    end
    hash=lower(reshape(dec2hex(typecast(md.digest(),'uint8'),2).',1,[]));
catch
end
end
function hash=r196_sha_text(s)
% SHA-256 of empty UTF-8 sequence; vital for old empty capture folders.
if isempty(s)
    hash='e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855';
    return;
end
hash='';
try
    md=java.security.MessageDigest.getInstance('SHA-256');
    md.update(typecast(uint8(s(:)),'int8'));
    hash=lower(reshape(dec2hex(typecast(md.digest(),'uint8'),2).',1,[]));
catch
end
end
function R=r196_restore(R,folder,out)
[parent,name]=fileparts(folder);
assert(strcmpi(r196_norm(parent),r196_norm(out)) && ...
    startsWith(name,'R196_QUARANTINE_')&&isfolder(folder), ...
    'R196:RestorePath','Expected an R196 quarantine directly under _AF7_HANDOFF_EXPORT.');
file=fullfile(folder,'R196_MANIFEST.tsv');
assert(isfile(file),'R196:ManifestMissing','Restore manifest not found.');
f=fopen(file,'rt');assert(f>0,'R196:ManifestRead','Cannot open manifest.');
cl=onCleanup(@()fclose(f));fgetl(f); %#ok<NASGU>
moved={};blocked={};
while true
    line=fgetl(f);if ~ischar(line),break;end
    a=strsplit(line,sprintf('\t'));
    if numel(a)<4,continue;end
    rel=a{1};isDir=strcmp(a{2},'DIR');
    if ~r196_rel_safe(rel)
        blocked{end+1,1}=[rel ' / INVALID_RELATIVE_PATH'];continue;end %#ok<AGROW>
    src=fullfile(folder,strrep(rel,'/',filesep));
    dst=fullfile(R.project_root,strrep(rel,'/',filesep));
    if isfile(dst)||isfolder(dst)
        blocked{end+1,1}=[rel ' / DESTINATION_EXISTS'];continue;end %#ok<AGROW>
    try
        [before,count]=r196_digest(src,isDir,folder);
        if isempty(before)||~strcmp(before,a{3})||count~=str2double(a{4})
            error('R196:RestoreDigest','Quarantine contents changed');end
        parent=fileparts(dst);if ~isfolder(parent),mkdir(parent);end
        [ok,msg]=movefile(src,dst);
        if ~ok,error('R196:RestoreMove','%s',msg);end
        moved{end+1,1}=rel; %#ok<AGROW>
    catch ME
        blocked{end+1,1}=[rel ' / ' ME.message]; %#ok<AGROW>
    end
end
R.moved_count=numel(moved);R.blocked_count=numel(blocked);R.blocked_paths=blocked;
R.quarantine_path=folder;
if isempty(blocked),R.status='R196_RESTORE_COMPLETED';
else,R.status='R196_PARTIAL_RESTORE__SEE_REPORT';end
r196_lines(R.report_path,[{R.status};moved(:);{'--- blocked ---'};blocked(:)]);
rehash;
end
function r196_lines(file,items)
f=fopen(file,'wt');assert(f>0,'R196:Report','Cannot write report.');
cl=onCleanup(@()fclose(f)); %#ok<NASGU>
for i=1:numel(items),fprintf(f,'%s\n',char(items{i}));end
end
