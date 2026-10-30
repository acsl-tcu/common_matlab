function R=AF7R58R195_PROJECT_QUARANTINE(action,argument,confirmation)
% R195: reversible cleanup of obsolete diagnostic files AND timestamped
% result/capture folders in common_matlab and common_matlab/reference.
% NEVER alters REPLANNING_SYSTEM, Flight, HLC, EKF, plant or their runtime.
% NEVER deletes files, removes ownership, changes ACL, or requires admin.
% No R192/R193 status MAT dependency: each move is checked by SHA-256.
%
% R=AF7R58R195_PROJECT_QUARANTINE('PREVIEW');
% R=AF7R58R195_PROJECT_QUARANTINE('QUARANTINE','CONFIRM_R195');
%% R=AF7R58R195_PROJECT_QUARANTINE('PREVIEW_ALL');
% R=AF7R58R195_PROJECT_QUARANTINE('TRACKED_PREVIEW');
% R=AF7R58R195_PROJECT_QUARANTINE('QUARANTINE_TRACKED', ...
%       'CONFIRM_R195_TRACKED_GIT_CHANGES');
% R=AF7R58R195_PROJECT_QUARANTINE('TRACKED_PREVIEW');
% R=AF7R58R195_PROJECT_QUARANTINE('QUARANTINE_SELECTED',...
%       {'reference/AF7R58R136_VIEW_20261026_120856_700'},'CONFIRM_R195_SELECTED');
% R=AF7R58R195_PROJECT_QUARANTINE('RESTORE',R.quarantine_path);
if nargin<1||isempty(action),action='PREVIEW';end
if nargin<2,argument='';end
if nargin<3,confirmation='';end
action=upper(char(action));
valid={'PREVIEW','PREVIEW_ALL','TRACKED_PREVIEW','QUARANTINE',...
       'QUARANTINE_TRACKED','QUARANTINE_SELECTED','RESTORE'};
assert(any(strcmp(action,valid)), 'R195:Action', 'Unknown action: %s',action);
ref=fileparts(mfilename('fullpath'));
[project,leaf]=fileparts(ref);
assert(strcmpi(leaf,'reference') && isfile(fullfile(ref,'REPLANNING_SYSTEM.m')) && ...
    isfile(fullfile(project,'run_flight_headless.m')), ...
    'R195:Root','Put this file in the active common_matlab/reference.');
assert(strcmpi(which('REPLANNING_SYSTEM'),fullfile(ref,'REPLANNING_SYSTEM.m')), ...
    'R195:Shadow','MATLAB is resolving a different REPLANNING_SYSTEM.');
out=fullfile(project,'_AF7_HANDOFF_EXPORT');
if ~isfolder(out),[ok,msg]=mkdir(out);assert(ok,'R195:Output','%s',msg);end
stamp=datestr(now,'yyyymmdd_HHMMSS_FFF');
R=struct('status','NOT_RUN','action',action,'project_root',project, ...
    'candidate_count',0,'candidate_files',0,'candidate_directories',0, ...
    'root_candidate_count',0,'reference_candidate_count',0, ...
    'candidate_paths',{{}},'tracked_candidate_count',0,'moved_count',0, ...
    'moved_files',0,'moved_directories',0,'blocked_count',0, ...
    'blocked_paths',{{}},'quarantine_path','','report_path', ...
    fullfile(out,['R195_' action '_' stamp '.txt']), ...
    'static_dependency_status','NOT_RUN', ...
    'backup_required',false,'content_verified',false, ...
    'git_worktree_will_change',strcmp(action,'QUARANTINE_TRACKED'));
if strcmp(action,'RESTORE')
    R=r195_restore(R,char(argument),out);return;
end
[st1,text1]=system(sprintf('git -c core.quotePath=false -C "%s" ls-files --cached',project));
[st2,text2]=system(sprintf('git -c core.quotePath=false -C "%s" ls-files --others --exclude-standard',project));
assert(st1==0 && st2==0,'R195:Git','Git unavailable; nothing moved.');
tracked=r195_lines(text1);untracked=r195_lines(text2);
% Static closure is protective only; absence of a successful static closure
% NEVER turns an unknown library or dated non-diagnostic directory into junk.
[deps,depStatus]=r195_protected(project,ref);
R.static_dependency_status=depStatus;
loaded=inmem('-completenames');
loaded=cellfun(@r195_norm,loaded,'UniformOutput',false);
% Search ONLY these two levels; never recurse into arbitrary code trees.
rows=cell(0,5); % project-relative, absolute, directory, old diagnostic, tracked
folders={project,ref};prefixes={'','reference/'};
for i=1:2
    ds=dir(folders{i});
    for k=1:numel(ds)
        name=ds(k).name;
        if any(strcmp(name,{'.','..'})),continue;end
        isDir=logical(ds(k).isdir);
        if ~isDir && ~endsWith(lower(name),'.m'),continue;end
        rel=[prefixes{i} name];lowerRel=lower(strrep(rel,'\','/'));
        if r195_always_protected(name,isDir),continue;end
        isTracked=any(strcmp(lowerRel,tracked));
        if isDir
            isTracked=isTracked || any(startsWith(tracked,[lowerRel '/']));
            isUntracked=any(startsWith(untracked,[lowerRel '/']));
        else
            isUntracked=any(strcmp(lowerRel,untracked));
        end
        % The narrow diagnostic rules may include .gitignored leftovers.
        % Git TRACKED files/directories remain protected.
        if any(strcmp(lowerRel,deps)) || ...
           (isDir && any(startsWith(deps,[lowerRel '/']))),continue;end
        src=fullfile(folders{i},name);
        if any(strcmp(r195_norm(src),loaded)) || ...
           (isDir && any(startsWith(loaded,[r195_norm(src) '/']))),continue;end
        if ~isDir && r195_seed_mentioned(project,ref,name),continue;end
        if isDir && ~r195_folder_pattern(name),continue;end
        old=(isDir && r195_folder_pattern(name)) || ...
             (~isDir && r195_diagnostic_pattern(name));
        rows(end+1,:)={rel,src,isDir,old,isTracked}; %#ok<AGROW>
    end
end
% Git tracked source entries NEVER move automatically.
selection=false(size(rows,1),1);
switch action
    case {'PREVIEW','QUARANTINE'}
        for k=1:size(rows,1)
            selection(k)=logical(rows{k,4}) && ~logical(rows{k,5});
        end
    case 'PREVIEW_ALL'
        for k=1:size(rows,1),selection(k)=~logical(rows{k,5});end
    case {'TRACKED_PREVIEW','QUARANTINE_TRACKED'}
        for k=1:size(rows,1),selection(k)=logical(rows{k,4}) && logical(rows{k,5});end
    case 'QUARANTINE_SELECTED'
        assert(iscell(argument)&&~isempty(argument),'R195:Selection', ...
            'Provide an explicit cell array of relative paths.');
        for i=1:numel(argument)
            rel=char(argument{i});
            assert(r195_valid_rel(rel),'R195:SelectionPath','Invalid path: %s',rel);
            ix=find(strcmpi(rel,rows(:,1)));
            assert(numel(ix)==1 && ~logical(rows{ix,5}), ...
                'R195:Protected','Tracked, active, excluded, or nonexistent: %s',rel);
            selection(ix)=true;
        end
end
rows=rows(selection,:);
R.candidate_count=size(rows,1);
R.candidate_paths=rows(:,1);
if ~isempty(rows)
    R.candidate_directories=sum(cell2mat(rows(:,3)));
    R.candidate_files=R.candidate_count-R.candidate_directories;
    R.tracked_candidate_count=sum(cell2mat(rows(:,5)));
    R.reference_candidate_count=sum(startsWith(lower(string(rows(:,1))),'reference/'));
    R.root_candidate_count=R.candidate_count-R.reference_candidate_count;
end
header={sprintf('R195 %s; %d candidates; files=%d folders=%d',action,...
    R.candidate_count,R.candidate_files,R.candidate_directories), ...
    ['Static dependency scan: ' depStatus], ...
    'NO deletions, NO admin privileges, NO automatic Git-tracked moves.', ...
    'QUARANTINE_TRACKED requires explicit confirmation and clean committed Git sources.', ...
    'Narrow dated historical folders may also be .gitignored.', ...
    'R192/R193 backup status is NOT required for reversible, SHA-checked move.', ...
    'Warning: unexecuted dynamic functions may not appear in dependencies.', ...
    '--- candidates (project-relative) ---'};
if any(strcmp(action,{'PREVIEW','PREVIEW_ALL','TRACKED_PREVIEW'}))
    R.status='R195_PREVIEW_COMPLETE__NO_FILES_MOVED';
    r195_write(R.report_path,[header(:);R.candidate_paths(:)]);
    fprintf('[R195] %d candidates (files=%d dirs=%d, root=%d, reference=%d)\n%s\n', ...
      R.candidate_count,R.candidate_files,R.candidate_directories,...
      R.root_candidate_count,R.reference_candidate_count,R.report_path);
    return;
end
if strcmp(action,'QUARANTINE')
    assert(strcmp(char(argument),'CONFIRM_R195'), 'R195:Confirm', ...
        'Use exact CONFIRM_R195 to move candidates.');
elseif strcmp(action,'QUARANTINE_TRACKED')
    assert(strcmp(char(argument),'CONFIRM_R195_TRACKED_GIT_CHANGES'), ...
        'R195:TrackedConfirm', 'Only previously committed tracked diagnostics may move.');
else
    assert(strcmp(char(confirmation),'CONFIRM_R195_SELECTED'), ...
        'R195:SelectedConfirm', 'Use exact CONFIRM_R195_SELECTED.');
end
R=r195_do_move(R,rows,out,stamp,project,tracked,untracked,loaded,header);
end

function R=r195_do_move(R,rows,out,stamp,project,tracked,untracked,loaded,header)
q=fullfile(out,['R195_QUARANTINE_' stamp]);
[ok,msg]=mkdir(q);assert(ok,'R195:QuarantineDir','%s',msg);
manifest=fullfile(q,'R195_MANIFEST.tsv');
f=fopen(manifest,'wt');assert(f>0,'R195:Manifest','Cannot open manifest.');
cl=onCleanup(@()fclose(f)); %#ok<NASGU>
fprintf(f,'relative_path\tkind\tsha256\tfile_count\n');
blocked={};moved={};R.content_verified=true;
for k=1:size(rows,1)
    rel=char(rows{k,1});src=char(rows{k,2});isDir=logical(rows{k,3});
    lowerRel=lower(strrep(rel,'\','/'));
    % A final Git-state, in-memory and existence check on each candidate.
    if ~isfile(src) && ~isfolder(src)
        blocked{end+1,1}=[rel ' / MISSING'];continue;end %#ok<AGROW>
    if any(strcmp(r195_norm(src),loaded)) || ...
       (isDir&&any(startsWith(loaded,[r195_norm(src) '/'])))
        blocked{end+1,1}=[rel ' / CURRENTLY_LOADED'];continue;end %#ok<AGROW>
    trackedNow=any(strcmp(lowerRel,tracked)) || ...
        (isDir && any(startsWith(tracked,[lowerRel '/'])));
    if trackedNow~=logical(rows{k,5})
        blocked{end+1,1}=[rel ' / GIT_STATUS_CHANGED'];continue;end %#ok<AGROW>
    if trackedNow
        if ~strcmp(R.action,'QUARANTINE_TRACKED')
            blocked{end+1,1}=[rel ' / GIT_TRACKED'];continue;end %#ok<AGROW>
        % Never move a dirty tracked file/folder, even on explicit request.
        [sc,~]=system(sprintf('git -C "%s" diff --quiet -- "%s"',project,rel));
        [ss,~]=system(sprintf('git -C "%s" diff --cached --quiet -- "%s"',project,rel));
        if sc~=0||ss~=0
            blocked{end+1,1}=[rel ' / GIT_DIRTY_OR_UNCOMMITTED'];continue;end %#ok<AGROW>
    end
    % Untracked and .gitignored dated diagnostic artifacts are eligible.
    % Re-verify by querying Git's cached/indexed set before moving.
    if isDir && ~r195_safe_directory(src,project)
        blocked{end+1,1}=[rel ' / JUNCTION_OR_UNSAFE_DIRECTORY'];continue;end %#ok<AGROW>
    try
        [before,count]=r195_digest(src,isDir);
        if isempty(before),error('R195:Unreadable','Cannot hash input');end
        dst=fullfile(q,strrep(rel,'/',filesep));
        dp=fileparts(dst);if ~isfolder(dp),mkdir(dp);end
        if isfile(dst)||isfolder(dst),error('R195:Collision','Destination exists');end
        [did,why]=movefile(src,dst);
        if ~did,error('R195:MoveFailed','%s',why);end
        [after,countAfter]=r195_digest(dst,isDir);
        if ~strcmp(before,after)||count~=countAfter
            [back,err]=movefile(dst,src);
            if ~back,error('R195:RollbackFailed','%s',err);end
            error('R195:SHA','File contents changed after move');
        end
        fprintf(f,'%s\t%s\t%s\t%d\n',rel,r195_kind(isDir),before,count);
        moved{end+1,1}=rel; %#ok<AGROW>
        if isDir
            % Remove stale entries from MATLAB path after a dated legacy
            % folder has been moved (never changes saved pathdef).
            dirs=strsplit(path,pathsep);
            for p=1:numel(dirs)
                if startsWith(r195_norm(dirs{p}),[r195_norm(src) '/']) || ...
                        strcmp(r195_norm(dirs{p}),r195_norm(src))
                    try,rmpath(dirs{p});catch,end
                end
            end
        end
    catch ME
        blocked{end+1,1}=[rel ' / ' ME.identifier ' / ' ME.message]; %#ok<AGROW>
        % Other candidates remain untouched; successful moves are verified.
    end
end
R.quarantine_path=q;R.moved_count=numel(moved);
R.blocked_count=numel(blocked);R.blocked_paths=blocked;
R.moved_directories=sum(cellfun(@(x)r195_was_dir(rows,x),moved));
R.moved_files=R.moved_count-R.moved_directories;
if isempty(blocked)
    R.status='R195_QUARANTINED_WITH_HASH_VERIFICATION__NO_DELETE';
else
    R.status='R195_PARTIAL_QUARANTINE__SEE_BLOCKED_REPORT';
end
r195_write(R.report_path,[{R.status};header(:);{'--- moved ---'};moved(:);...
    {'--- blocked ---'};blocked(:);{'Quarantine:';q}]);
fprintf('[R195] moved=%d (files=%d folders=%d), blocked=%d\n%s\n',...
    R.moved_count,R.moved_files,R.moved_directories,R.blocked_count,q);
rehash;
end
function yes=r195_was_dir(rows,rel)
idx=find(strcmp(rows(:,1),rel),1);
yes=~isempty(idx) && logical(rows{idx,3});
end
function s=r195_kind(d)
if d,s='DIR';else,s='FILE';end
end
function R=r195_restore(R,q,out)
% RESTORE never overwrites a new file/directory. Manifest is authoritative.
q=char(q);[parent,name]=fileparts(q);
assert(strcmpi(parent,out) && startsWith(name,'R195_QUARANTINE_') && isfolder(q), ...
    'R195:RestoreDir','Expected R195 quarantine in _AF7_HANDOFF_EXPORT.');
file=fullfile(q,'R195_MANIFEST.tsv');
assert(isfile(file),'R195:RestoreManifest','Missing manifest.');
f=fopen(file,'rt');assert(f>0,'R195:RestoreRead','Could not open manifest.');
cl=onCleanup(@()fclose(f)); %#ok<NASGU>
fgetl(f);moved={};blocked={};
while true
    line=fgetl(f);if ~ischar(line),break;end
    a=strsplit(line,sprintf('\t'));if numel(a)~=4,continue;end
    rel=a{1};isDir=strcmp(a{2},'DIR');sha=a{3};
    if ~r195_valid_rel(rel)
        blocked{end+1,1}=[rel ' / INVALID_PATH'];continue;end %#ok<AGROW>
    from=fullfile(q,strrep(rel,'/',filesep));
    to=fullfile(R.project_root,strrep(rel,'/',filesep));
    if isfile(to)||isfolder(to)
        blocked{end+1,1}=[rel ' / DEST_EXISTS'];continue;end %#ok<AGROW>
    try
        [h,~]=r195_digest(from,isDir);
        if ~strcmp(h,sha),error('R195:RestoreHash','Hash mismatch');end
        dstDir=fileparts(to);if ~isfolder(dstDir),mkdir(dstDir);end
        [ok,msg]=movefile(from,to);
        if ~ok,error('R195:RestoreMove','%s',msg);end
        moved{end+1,1}=rel; %#ok<AGROW>
    catch ME
        blocked{end+1,1}=[rel ' / ' ME.message]; %#ok<AGROW>
    end
end
R.status='R195_RESTORE_COMPLETE__NO_OVERWRITE';
R.moved_count=numel(moved);R.blocked_count=numel(blocked);R.blocked_paths=blocked;
R.quarantine_path=q;
r195_write(R.report_path,[{R.status;'--- restored ---'};moved(:);...
    {'--- blocked ---'};blocked(:)]);
rehash;
end
function yes=r195_valid_rel(s)
s=strrep(char(s),'\','/');
parts=strsplit(s,'/');
yes=(numel(parts)==1|| (numel(parts)==2&&strcmpi(parts{1},'reference'))) && ...
    ~contains(s,'..') && ~startsWith(s,'/') && ~contains(s,':') && ...
    ~isempty(regexp(parts{end},'^AF7[A-Za-z0-9_.-]+$','once'));
end
function yes=r195_folder_pattern(n)
% Evidence-backed dated folders: e.g. AF7R58R170_RESULT_20261028_213418_103
% Includes R119 historical captured source snapshots, not general libraries.
n=upper(char(n));
yes=~isempty(regexp(n,'^AF7[A-Z0-9_]*(RESULT|RESULTS|VIEW|VIZ|AUDIT|RECOVER|COLLECT|CAPTURE|EXTRACT|REPORT|PREVIEW|OUTPUT|STAGED|TEST)[A-Z0-9_]*_20[0-9]{6}_[0-9]{6}(_[0-9]+)?$','once'));
end
function yes=r195_diagnostic_pattern(n)
lo=lower(char(n));
yes=startsWith(lo,'af7') && endsWith(lo,'.m') && ...
    (contains(lo,'_install') || contains(lo,'_selftest') || ...
    contains(lo,'_audit') || contains(lo,'_postflight') || ...
    contains(lo,'_preflight') || contains(lo,'_viewer') || ...
    contains(lo,'_visual') || contains(lo,'_report') || ...
    contains(lo,'_collect') || contains(lo,'_source_check') || ...
    contains(lo,'_open_3d') || contains(lo,'_plot'));
end
function yes=r195_always_protected(n,isDir)
lo=lower(char(n));
if isDir
    yes=any(strcmp(lo,{'reference','.git','functions','baseline','docs',...
        'runtime_support','suspendedload','_af7_handoff_export'})) || ...
        startsWith(lo,'@') || startsWith(lo,'＠');
else
    yes=any(strcmp(lo,{'replanning_system.m','run_flight_headless.m', ...
      'af7r58r191_ekf_tick_trigger.m','af7r58r191_hull_separate.m', ...
      'af7r58r189_c7_live_trial.m','af7r58r191_postflight.m', ...
      'af7r58r191_sim_verify.m','af7r58r170_generate.m', ...
      'af7r58r172_strict_trigger.m','af7r58r172_strict_generator.m'})) || ...
      startsWith(lo,'af7r58r192_') || startsWith(lo,'af7r58r192r2_') || ...
      startsWith(lo,'af7r58r193_') || startsWith(lo,'af7r58r194_') || ...
      startsWith(lo,'af7r58r195_');
end
end
function [deps,status]=r195_protected(project,ref)
deps={};status='NOT_AVAILABLE';
try
    seeds={fullfile(project,'run_flight_headless.m'), ...
           fullfile(ref,'REPLANNING_SYSTEM.m')};
    names={'EKF','Estimator_EKF_SuspendedLoad','HLC_SUSPENDED_LOAD', ...
           'Controller_HL_Suspended_Load','TIME_VARYING_REFERENCE',...
           'SUSPENDED_LOAD_REF_ADJUST'};
    for j=1:numel(names)
        f=which(names{j});if ~isempty(f)&&isfile(f),seeds{end+1}=f;end %#ok<AGROW>
    end
    [found,~]=matlab.codetools.requiredFilesAndProducts(seeds);
    found=[found(:);seeds(:)];
    for j=1:numel(found)
        f=r195_norm(found{j});p=r195_norm(project);
        if startsWith(f,[p '/'])
            deps{end+1,1}=f(numel(p)+2:end); %#ok<AGROW>
        end
    end
    deps=unique(deps);status=sprintf('STATIC_CLOSURE_%d',numel(deps));
catch ME
    status=['STATIC_CLOSURE_UNAVAILABLE__' char(ME.identifier)];
end
end
function yes=r195_seed_mentioned(project,ref,name)
% Source-level mention is deliberately overprotective for dynamic dispatch.
[~,stem,~]=fileparts(name);persistent cached text stamp
s=[dir(fullfile(ref,'REPLANNING_SYSTEM.m'));dir(fullfile(project,'run_flight_headless.m'))];
cur=[s.datenum s.bytes];
if isempty(stamp)||~isequal(stamp,cur)
    text=[fileread(fullfile(ref,'REPLANNING_SYSTEM.m')) newline ...
          fileread(fullfile(project,'run_flight_headless.m'))];stamp=cur;
end
pat=['(?<![a-zA-Z0-9_])' regexptranslate('escape',stem) '(?![a-zA-Z0-9_])'];
yes=~isempty(regexpi(text,pat,'once'));
end
function lines=r195_lines(t)
items=regexp(strrep(t,sprintf('\r'),''),'\n','split');lines={};
for k=1:numel(items)
    x=strtrim(items{k});
    if isempty(x)||startsWith(x,'"'),continue;end
    lines{end+1,1}=lower(strrep(x,'\','/')); %#ok<AGROW>
end
lines=unique(lines);
end
function s=r195_norm(x)
s=lower(strrep(char(x),'\','/'));
while endsWith(s,'/') && numel(s)>1,s=s(1:end-1);end
end
function yes=r195_safe_directory(dirPath,root)
% Reject junctions or symbolic links resolving outside the repo.
yes=false;
try
    object=java.io.File(dirPath);origin=java.io.File(root);
    canonical=r195_norm(char(object.getCanonicalPath()));
    raw=r195_norm(char(object.getAbsolutePath()));
    base=r195_norm(char(origin.getCanonicalPath()));
    yes=strcmp(canonical,raw)&&startsWith(canonical,[base '/']);
catch
end
end
function [h,count]=r195_digest(src,isDir)
h='';count=0;
if ~isDir
    if isfile(src),h=r195_sha_file(src);count=1;end
    return;
end
if ~isfolder(src),return;end
entries={};
stack={{src,''}};
while ~isempty(stack)
    x=stack{end};stack(end)=[];
    children=dir(x{1});
    for k=1:numel(children)
        item=children(k);n=item.name;
        if any(strcmp(n,{'.','..'})),continue;end
        full=fullfile(x{1},n);rel=[x{2} n];
        if item.isdir
            % Directory junctions are not followed.
            if ~r195_safe_directory(full,src),h='';return;end
            stack{end+1}={full,[rel '/']}; %#ok<AGROW>
            entries{end+1,1}=['DIR|' rel]; %#ok<AGROW>
        else
            sha=r195_sha_file(full);if isempty(sha),h='';return;end
            entries{end+1,1}=['FILE|' rel '|' sha]; %#ok<AGROW>
            count=count+1;
        end
    end
end
entries=sort(entries);
h=r195_sha_bytes(uint8(strjoin(entries,sprintf('\n'))));
end
function h=r195_sha_file(src)
h='';fid=fopen(src,'rb');if fid<0,return;end
cl=onCleanup(@()fclose(fid)); %#ok<NASGU>
try
 md=java.security.MessageDigest.getInstance('SHA-256');
 while ~feof(fid)
     b=fread(fid,1048576,'*uint8');
     if ~isempty(b),md.update(typecast(b(:),'int8'));end
 end
 h=lower(reshape(dec2hex(typecast(md.digest(),'uint8'),2).',1,[]));
catch
end
end
function h=r195_sha_bytes(bytes)
h='';
try
 md=java.security.MessageDigest.getInstance('SHA-256');
 md.update(typecast(uint8(bytes(:)),'int8'));
 h=lower(reshape(dec2hex(typecast(md.digest(),'uint8'),2).',1,[]));
catch
end
end
function r195_write(filepath,rows)
f=fopen(filepath,'wt');assert(f>0,'R195:Report','Cannot write report.');
c=onCleanup(@()fclose(f)); %#ok<NASGU>
for k=1:numel(rows),fprintf(f,'%s\n',char(rows{k}));end
end
