function C = AF7R58R194_PROJECT_CLEANUP(action,arg,confirmation)
% R194 - Project root + reference cleanup; does not edit Flight/plant/HLC/EKF.
% No Flight/profile required. Requires the independently verified R192/R193
% archive before MOVING.  All movement is reversible, never delete or chmod.
%
% C=AF7R58R194_PROJECT_CLEANUP('PREVIEW');
% C=AF7R58R194_PROJECT_CLEANUP('QUARANTINE', ...
%                    'CONFIRM_R194_UNTRACKED_DIAGNOSTICS');
% C=AF7R58R194_PROJECT_CLEANUP('PREVIEW_ALL');
% C=AF7R58R194_PROJECT_CLEANUP('TRACKED_PREVIEW');
% C=AF7R58R194_PROJECT_CLEANUP('QUARANTINE_TRACKED', ...
%                    'CONFIRM_R194_TRACKED_DIAGNOSTICS_GIT_CHANGES');
% C=AF7R58R194_PROJECT_CLEANUP('QUARANTINE_SELECTED', ...
%                    {'reference/EXAMPLE_OLD_AUDIT.m'},'CONFIRM_R194_SELECTED');
% C=AF7R58R194_PROJECT_CLEANUP('RESTORE',C.quarantine_path, ...
%                    'CONFIRM_R194_RESTORE');
if nargin<1 || isempty(action),action='PREVIEW';end
if nargin<2,arg='';end
if nargin<3,confirmation='';end
action=upper(char(action));
valid={'PREVIEW','PREVIEW_ALL','TRACKED_PREVIEW','QUARANTINE', ...
       'QUARANTINE_TRACKED','QUARANTINE_SELECTED','RESTORE'};
assert(any(strcmp(action,valid)),'R194:BadAction', ...
    'Unsupported action. Use PREVIEW, QUARANTINE, PREVIEW_ALL, TRACKED_PREVIEW or RESTORE.');
ref=fileparts(which('REPLANNING_SYSTEM'));
[project,leaf]=fileparts(ref);
assert(strcmpi(leaf,'reference') && isfile(fullfile(ref,'REPLANNING_SYSTEM.m')), ...
    'R194:WrongRoot','Cannot resolve canonical common_matlab/reference/REPLANNING_SYSTEM.m.');
assert(isfile(fullfile(project,'run_flight_headless.m')), ...
    'R194:RunnerMissing','Original project root runner is missing.');
assert(strcmpi(fileparts(mfilename('fullpath')),ref), ...
    'R194:ToolNotInReference','Copy this R194 .m to canonical reference folder.');
out=fullfile(project,'_AF7_HANDOFF_EXPORT');
if ~isfolder(out),mkdir(out);end
stamp=datestr(now,'yyyymmdd_HHMMSS_FFF');
report=fullfile(out,['R194_' action '_' stamp '.txt']);
C=struct('status','PREVIEW_ONLY','action',action,'project_root',project, ...
    'candidate_count',0,'candidate_paths',{{}},'root_candidate_count',0, ...
    'reference_candidate_count',0,'tracked_candidate_count',0, ...
    'moved_count',0,'blocked_count',0,'blocked_paths',{{}}, ...
    'quarantine_path','','report_path',report,'backup_verified',false, ...
    'backup_path','','static_closure_status','NOT_RUN', ...
    'protected_active_count',0,'git_worktree_will_change',false);
if strcmp(action,'RESTORE')
    assert(strcmp(char(confirmation),'CONFIRM_R194_RESTORE'), ...
       'R194:RestoreConfirm','Exact RESTORE confirmation required.');
    C=r194_restore(C,out,arg);
    return;
end
[statusT,trackedText]=system(sprintf( ...
 'git -c core.quotePath=false -C "%s" ls-files --cached',project));
[statusU,untrackedText]=system(sprintf( ...
 'git -c core.quotePath=false -C "%s" ls-files --others --exclude-standard',project));
assert(statusT==0 && statusU==0,'R194:GitRequired', ...
  'Git file tracking unavailable. No files changed.');
tracked=r194_git_paths(trackedText);
untracked=r194_git_paths(untrackedText);
% Existing small R193 capture (including partial recordings) can ONLY add
% protective names. It never grants authority to move/delete anything.
protected={};
s193=fullfile(out,'R193_LAST_STATUS.mat');
if isfile(s193)
    try
        T=load(s193,'R');
        if isfield(T.R,'active_project_paths')
            for j=1:numel(T.R.active_project_paths)
                p=char(T.R.active_project_paths{j});
                if ~isempty(p),protected{end+1,1}=lower(strrep(p,'\','/'));end %#ok<AGROW>
            end
        end
    catch
    end
end
% Source-level closure. This is NOT a proof that every dynamic name/data
% used by all possible configurations is captured. Failure blocks moves.
[staticOK,staticList,staticReason]=r194_static_protect(project,ref);
C.static_closure_status=staticReason;
protected=[protected;staticList(:)];
protected=unique(protected);
C.protected_active_count=numel(protected);
% Protect historical R191 diagnostics that are still used to assess flight.
% The R194 tools and all real controller/estimator/plant files are excluded
% by restricting this scan to ONLY the two top-level project directories.
D={dir(fullfile(project,'*.m')),dir(fullfile(ref,'*.m'))};
allRows=cell(0,4);
for area=1:2
    if area==1,prefix='';fold=project;else,prefix='reference/';fold=ref;end
    ds=D{area};
    for k=1:numel(ds)
        if ds(k).isdir,continue;end
        n=ds(k).name; rel=[prefix n];lo=lower(rel);
        isT=any(strcmp(lo,tracked));
        isU=any(strcmp(lo,untracked));
        if ~(isT||isU),continue;end % ignored / unknown Git status: protected
        if r194_protected_name(n) || any(strcmp(lo,protected)),continue;end
        % Exact symbol references in runtime seeds are additional protection
        % for str2func or other name-based dynamic invocation.
        if r194_seed_mentions(project,ref,n),continue;end
        old=r194_is_old_diagnostic(n);
        allRows(end+1,:)={rel,old,isT,fullfile(fold,n)}; %#ok<AGROW>
    end
end
choose=false(size(allRows,1),1);
switch action
    case {'PREVIEW','QUARANTINE'}
        for k=1:size(allRows,1)
            choose(k)=logical(allRows{k,2}) && ~logical(allRows{k,3});
        end
    case 'PREVIEW_ALL'
        for k=1:size(allRows,1),choose(k)=~logical(allRows{k,3});end
    case {'TRACKED_PREVIEW','QUARANTINE_TRACKED'}
        for k=1:size(allRows,1)
            choose(k)=logical(allRows{k,2}) && logical(allRows{k,3});
        end
    case 'QUARANTINE_SELECTED'
        assert(iscell(arg)&&~isempty(arg),'R194:ExplicitSelection', ...
            'Select project-relative .m paths from PREVIEW_ALL.');
        for j=1:numel(arg)
            p=char(arg{j});
            assert(~contains(p,'..') && ~contains(p,'\') && ...
                   ~startsWith(p,'/') && ...
                   ~isempty(regexp(p,'^(reference/)?[A-Za-z][A-Za-z0-9_]*\.m$','once')), ...
                'R194:InvalidPath','Invalid relative path: %s',p);
            ix=find(strcmpi(p,allRows(:,1)));
            assert(numel(ix)==1 && ~logical(allRows{ix,3}), ...
                'R194:NotEligible','Not eligible/untracked: %s',p);
            choose(ix)=true;
        end
end
rows=allRows(choose,:);
C.candidate_count=size(rows,1);
C.candidate_paths=rows(:,1);
C.root_candidate_count=sum(~startsWith(lower(string(C.candidate_paths)),'reference/'));
C.reference_candidate_count=C.candidate_count-C.root_candidate_count;
if ~isempty(rows),C.tracked_candidate_count=sum(cell2mat(rows(:,3)));end
[archiveOK,archivePath,archiveStatus]=r194_backup_status(out);
C.backup_verified=archiveOK;C.backup_path=archivePath;
header={['R194 ' action], 'No files deleted; only top-level common_matlab and reference.', ...
 ['Verified backup: ' num2str(archiveOK) ' / ' archiveStatus], ...
 ['Backup ZIP: ' archivePath], ...
 ['Static closure: ' staticReason], ...
 'Git tracked sources are NEVER included in ordinary QUARANTINE.', ...
 'TRACKED_PREVIEW / QUARANTINE_TRACKED is separate and modifies Git worktree.', ...
 'Not called in one Flight does not imply unused.', ...
 sprintf('Candidates: %d (root %d, reference %d)', ...
    C.candidate_count,C.root_candidate_count,C.reference_candidate_count), ...
 '--- relative source paths ---'};
if any(strcmp(action,{'PREVIEW','PREVIEW_ALL','TRACKED_PREVIEW'}))
    C.status='R194_PREVIEW_ONLY__NO_FILES_MOVED';
    r194_write(report,[header(:);rows(:,1)]);
    fprintf('[R194] %s: %d candidates (root=%d, reference=%d)\n%s\n', ...
      action,C.candidate_count,C.root_candidate_count,C.reference_candidate_count,report);
    return;
end
% Require a verified existing archive. A failed static scan does not
% prevent ONLY narrow historical untracked diagnostic quarantine, as the
% backup is present and movement is reversible. Never relax this for a
% tracked or explicitly selected arbitrary source file.
assert(archiveOK,'R194:VerifiedBackupRequired', ...
    'No verified R192/R193 ZIP found locally; no files changed.');
if ~staticOK
    assert(strcmp(action,'QUARANTINE'), 'R194:StaticClosureRequired', ...
      'Static closure failed; only narrow UNTRACKED diagnostic quarantine is allowed. %s',staticReason);
    C.static_closure_status=['LIMITED_TO_UNTRACKED_HISTORICAL_DIAGNOSTICS__' staticReason];
end
if strcmp(action,'QUARANTINE')
    assert(strcmp(char(arg),'CONFIRM_R194_UNTRACKED_DIAGNOSTICS'), ...
      'R194:Confirm','Confirm untracked diagnostics with exact token.');
elseif strcmp(action,'QUARANTINE_TRACKED')
    assert(strcmp(char(arg),'CONFIRM_R194_TRACKED_DIAGNOSTICS_GIT_CHANGES'), ...
      'R194:ConfirmTracked','This will show as deletions in Git. Use explicit token.');
    C.git_worktree_will_change=true;
else
    assert(strcmp(char(confirmation),'CONFIRM_R194_SELECTED'), ...
      'R194:ConfirmSelected','Explicit selection confirmation required.');
end
q=fullfile(out,['R194_QUARANTINE_' stamp]);
assert(mkdir(q),'R194:QuarantineCreate','Cannot create quarantine directory.');
manifest=fullfile(q,'R194_MANIFEST.tsv');
fid=fopen(manifest,'wt');
assert(fid>0,'R194:ManifestWrite','Cannot write quarantine manifest.');
cl=onCleanup(@()fclose(fid));
fprintf(fid,'relative_path\tsha256\tgit_tracked\n');
blocked={};moved={};
loaded=inmem('-completenames');
loaded=cellfun(@(x)lower(strrep(char(x),'\','/')),loaded,'UniformOutput',false);
for k=1:size(rows,1)
    rel=char(rows{k,1});src=char(rows{k,4});
    if ~isfile(src),blocked{end+1,1}=[rel ' / NOT_FOUND'];continue;end %#ok<AGROW>
    if any(strcmp(lower(strrep(src,'\','/')),loaded)) || ...
        r194_seed_mentions(project,ref,src)
        blocked{end+1,1}=[rel ' / LOADED_OR_LIVE_REFERENCE'];continue;end %#ok<AGROW>
    % Recheck Git classification just before the move: it must match PREVIEW.
    if logical(rows{k,3}) ~= any(strcmp(lower(rel),tracked)) || ...
       (~logical(rows{k,3}) && ~any(strcmp(lower(rel),untracked)))
        blocked{end+1,1}=[rel ' / GIT_CLASSIFICATION_CHANGED'];continue;end %#ok<AGROW>
    h=r194_sha(src);
    if isempty(h),blocked{end+1,1}=[rel ' / SOURCE_UNREADABLE'];continue;end %#ok<AGROW>
    dst=fullfile(q,strrep(rel,'/',filesep)); parent=fileparts(dst);
    if ~isfolder(parent),mkdir(parent);end
    if isfile(dst),blocked{end+1,1}=[rel ' / DEST_EXISTS'];continue;end %#ok<AGROW>
    [ok,msg]=movefile(src,dst);
    if ~ok,blocked{end+1,1}=[rel ' / ' msg];continue;end %#ok<AGROW>
    if ~strcmp(r194_sha(dst),h)
        [back,~]=movefile(dst,src); %#ok<ASGLU>
        blocked{end+1,1}=[rel ' / HASH_CHANGED_ON_MOVE'];continue; %#ok<AGROW>
    end
    fprintf(fid,'%s\t%s\t%d\n',rel,h,logical(rows{k,3}));
    moved{end+1,1}=rel; %#ok<AGROW>
end
C.quarantine_path=q;C.moved_count=numel(moved);
C.blocked_count=numel(blocked);C.blocked_paths=blocked;
C.status='R194_FILES_MOVED_TO_REVERSIBLE_QUARANTINE';
r194_write(report,[{C.status};header(:);{'--- MOVED ---'}; ...
    moved(:);{'--- BLOCKED ---'};blocked(:)]);
fprintf('[R194] moved=%d blocked=%d (root=%d reference=%d)\n%s\n', ...
    C.moved_count,C.blocked_count,C.root_candidate_count, ...
    C.reference_candidate_count,q);
rehash;
end

function p=r194_git_paths(output)
lines=regexp(strrep(output,sprintf('\r'),''),'\n','split');
p=cell(0,1);
for k=1:numel(lines)
    s=strtrim(lines{k});
    if isempty(s) || startsWith(s,'"'),continue;end % quoted names fail closed
    p{end+1,1}=lower(strrep(s,'\','/')); %#ok<AGROW>
end
p=unique(p);
end

function yes=r194_protected_name(name)
lo=lower(name);
fixed={'replanning_system.m','run_flight_headless.m', ...
 'af7r58r191_ekf_tick_trigger.m','af7r58r191_hull_separate.m', ...
 'af7r58r189_c7_live_trial.m','af7r58r191_postflight.m', ...
 'af7r58r191_sim_verify.m','af7r58r23_spike_report.m', ...
 'af7r58r170_generate.m','af7r58r172_strict_generator.m', ...
 'af7r58r172_strict_trigger.m','af7r58r71_exact_hlc_plant_preview.m'};
yes=any(strcmp(lo,fixed)) || startsWith(lo,'af7r58r192_') || ...
     startsWith(lo,'af7r58r192r2_') || startsWith(lo,'af7r58r193_') || ...
     startsWith(lo,'af7r58r194_');
end
function yes=r194_is_old_diagnostic(name)
lo=lower(name);
% Narrow class: installers, selftests, post-flight audits, view/report tools.
% Cannot treat unknown numeric libraries, solver/MEX and helper .m as unused.
yes=startsWith(lo,'af7') && endsWith(lo,'.m') && ...
  (contains(lo,'_install') || contains(lo,'_selftest') || ...
   contains(lo,'_audit') || contains(lo,'_postflight') || ...
   contains(lo,'_preflight') || contains(lo,'_viewer') || ...
   contains(lo,'_visual') || contains(lo,'_report') || ...
   contains(lo,'_collect') || contains(lo,'_source_check'));
end
function yes=r194_seed_mentions(project,ref,name)
[~,stem,~]=fileparts(name);yes=false;
% Main and runner must NEVER acquire a dangling named call.  Byte/regexp
% matching may overprotect commented mentions; that is safer than moving.
persistent rootRemember txtRemember stampRemember
current=[dir(fullfile(ref,'REPLANNING_SYSTEM.m')); ...
         dir(fullfile(project,'run_flight_headless.m'))];
sig=[current.datenum current.bytes];
if isempty(rootRemember) || ~strcmp(rootRemember,project) || ...
        ~isequal(sig,stampRemember)
    rootRemember=project;stampRemember=sig;
    txtRemember=[fileread(fullfile(ref,'REPLANNING_SYSTEM.m')) newline ...
                 fileread(fullfile(project,'run_flight_headless.m'))];
end
pat=['(?<![A-Za-z0-9_])' regexptranslate('escape',stem) '(?![A-Za-z0-9_])'];
yes=~isempty(regexpi(txtRemember,pat,'once'));
end
function [ok,files,status]=r194_static_protect(project,ref)
ok=false;files={};status='UNKNOWN';
try
    seed={fullfile(project,'run_flight_headless.m'); ...
          fullfile(ref,'REPLANNING_SYSTEM.m')};
    names={'HLC_SUSPENDED_LOAD','Controller_HL_Suspended_Load', ...
           'Estimator_EKF_SuspendedLoad','EKF', ...
           'SUSPENDED_LOAD_REF_ADJUST','TIME_VARYING_REFERENCE'};
    for k=1:numel(names)
        w=which(names{k});
        if ~isempty(w) && isfile(w) && r194_within(project,w)
            seed{end+1,1}=w; %#ok<AGROW>
        end
    end
    [deps,~]=matlab.codetools.requiredFilesAndProducts(seed);
    deps=[deps(:);seed(:)];
    for k=1:numel(deps)
        f=char(deps{k});
        if r194_within(project,f)
            rel=strrep(f(numel(project)+2:end),'\','/');
            files{end+1,1}=lower(rel); %#ok<AGROW>
        end
    end
    files=unique(files);ok=true;
    status=sprintf('PASS_STATIC_SOURCE_CLOSURE_%d_PATHS',numel(files));
catch ME
    status=['FAIL_CLOSED_' char(ME.identifier) ' ' char(ME.message)];
end
end
function yes=r194_within(root,file)
root=lower(strrep(root,'/','\'));file=lower(strrep(char(file),'/','\'));
yes=startsWith(file,[root '\']);
end
function [ok,zipPath,status]=r194_backup_status(out)
ok=false;zipPath='';status='NO_VERIFIED_LOCAL_BACKUP';
for fn={'LATEST_SNAPSHOT_STATUS.mat','R193_LAST_STATUS.mat'}
    path=fullfile(out,fn{1});
    if ~isfile(path),continue;end
    try
        D=load(path,'R');R=D.R;
        if isfield(R,'zip_path') && isfield(R,'snapshot_verified') && ...
                logical(R.snapshot_verified) && isfile(R.zip_path)
            ok=true;zipPath=char(R.zip_path);status='R192_SNAPSHOT_VERIFIED';return;
        end
        if isfield(R,'zip_path') && isfield(R,'zip_verified') && ...
                logical(R.zip_verified) && isfile(R.zip_path) && ...
                isfield(R,'included_files') && R.included_files>0
            ok=true;zipPath=char(R.zip_path);status='R193_ZIP_VERIFIED';return;
        end
    catch
    end
end
end
function h=r194_sha(path)
h='';f=fopen(path,'rb');if f<0,return;end
c=onCleanup(@()fclose(f)); %#ok<NASGU>
try
    md=java.security.MessageDigest.getInstance('SHA-256');
    while ~feof(f)
        b=fread(f,1048576,'*uint8');
        if ~isempty(b),md.update(typecast(b(:),'int8'));end
    end
    h=lower(reshape(dec2hex(typecast(md.digest(),'uint8'),2).',1,[]));
catch
end
end
function C=r194_restore(C,out,path)
path=char(path);
[par,leaf]=fileparts(path);
assert(strcmpi(par,out) && startsWith(leaf,'R194_QUARANTINE_') && ...
    isfolder(path),'R194:RestorePath','Only an R194 quarantine under export can be restored.');
manifest=fullfile(path,'R194_MANIFEST.tsv');
assert(isfile(manifest),'R194:ManifestMissing');
f=fopen(manifest,'rt');assert(f>=0);cl=onCleanup(@()fclose(f)); %#ok<NASGU>
fgetl(f);moved={};blocked={};
while true
    line=fgetl(f);if ~ischar(line),break;end
    x=strsplit(line,sprintf('\t'));if numel(x)<3,continue;end
    rel=x{1};h=x{2};
    if isempty(regexp(rel,'^(reference/)?[A-Za-z][A-Za-z0-9_]*\.m$','once'))
        blocked{end+1,1}=[rel ' / BAD_RELATIVE_PATH'];continue;end %#ok<AGROW>
    src=fullfile(path,strrep(rel,'/',filesep));
    dst=fullfile(C.project_root,strrep(rel,'/',filesep));
    if ~isfile(src)
        blocked{end+1,1}=[rel ' / QUARANTINE_FILE_MISSING'];continue;end %#ok<AGROW>
    if isfile(dst)
        blocked{end+1,1}=[rel ' / CURRENT_FILE_EXISTS_NO_OVERWRITE'];continue;end %#ok<AGROW>
    if ~strcmp(r194_sha(src),h)
        blocked{end+1,1}=[rel ' / QUARANTINE_HASH_MISMATCH'];continue;end %#ok<AGROW>
    [ok,msg]=movefile(src,dst);
    if ok,moved{end+1,1}=rel; %#ok<AGROW>
    else,blocked{end+1,1}=[rel ' / ' msg];end %#ok<AGROW>
end
C.status='R194_RESTORE_ATTEMPTED__NO_OVERWRITES';
C.moved_count=numel(moved);C.blocked_count=numel(blocked);
C.blocked_paths=blocked;C.quarantine_path=path;
r194_write(C.report_path,[{C.status};{'RESTORED:'};moved(:); ...
    {'NOT RESTORED:'};blocked(:)]);
rehash;
end
function r194_write(path,rows)
f=fopen(path,'wt');assert(f>=0,'R194:ReportWrite');cl=onCleanup(@()fclose(f)); %#ok<NASGU>
for k=1:numel(rows),fprintf(f,'%s\n',char(rows{k}));end
end
