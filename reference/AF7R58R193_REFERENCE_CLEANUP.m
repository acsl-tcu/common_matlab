function C=AF7R58R193_REFERENCE_CLEANUP(action,token)
% Uses an R193 profiled + statically closed Flight export to prepare cleanup.
% PREVIEW: lists safe-ish UNTRACKED historic diagnostics, does NOT delete.
% QUARANTINE: MOVES only those reviewed, untracked history files, never core.
% No privilege escalation, chmod, takeown, permanent deletion or MEX moves.
if nargin<1||isempty(action),action='PREVIEW';end
if nargin<2,token='';end
action=upper(char(action));
assert(any(strcmp(action,{'PREVIEW','QUARANTINE','PREVIEW_ALL','QUARANTINE_SELECTED'})), ...
 'R193:Action','Use PREVIEW, PREVIEW_ALL, QUARANTINE or QUARANTINE_SELECTED.');
ref=fileparts(mfilename('fullpath'));
[root,leaf]=fileparts(ref);
assert(strcmpi(leaf,'reference') && isfile(fullfile(ref,'REPLANNING_SYSTEM.m')), ...
 'R193:WrongRoot','Must be in canonical common_matlab/reference.');
out=fullfile(root,'_AF7_HANDOFF_EXPORT');
statusFile=fullfile(out,'R193_LAST_STATUS.mat');
assert(isfile(statusFile),'R193:NoTrace', ...
 'Run AF7R58R193_USED_FLIGHT_EXPORT first.');
T=load(statusFile,'R');R=T.R;
assert(isfield(R,'zip_verified')&&logical(R.zip_verified)&& ...
 strcmp(R.status,'R193_ACTUAL_FLIGHT_AND_STATIC_CLOSURE_ZIP_VERIFIED') && ...
 R.flight_completed && isfile(R.zip_path), ...
 'R193:IncompleteTrace','Complete Flight, static closure and ZIP verification required.');
[rc,txt]=system(sprintf('git -c core.quotePath=false -C "%s" ls-files --cached',root));
assert(rc==0,'R193:GitUnavailable','Git tracking cannot be checked. No moves.');
tracked=regexp(strrep(txt,sprintf('\r'),''),'\n','split');
tracked=cellfun(@lower,strrep(tracked,filesep,'/'),'UniformOutput',false);
active=cellfun(@lower,strrep(R.active_project_paths,filesep,'/'),'UniformOutput',false);
assert(~isempty(active),'R193:EmptyActive','No captured active-source closure.');
% A source change since the trace invalidates the classification.
assert(r193_main_unchanged(R,root), 'R193:SourceChanged', ...
 'REPLANNING_SYSTEM or runner changed since profiling; recollect before cleanup.');
% Limit selection to canonical reference root. Do not recursively move code.
D=dir(fullfile(ref,'*.m'));
select={};reason={};allInactive={};
for k=1:numel(D)
 if D(k).isdir,continue;end
 name=D(k).name;
 rel=['reference/' name];
 if any(strcmpi(rel,tracked)),continue;end
 if any(strcmpi(rel,active)),continue;end
 if r193_core_protect(name),continue;end
 allInactive{end+1,1}=name; %#ok<AGROW>
 if ~r193_old_diagnostic(name),continue;end
 select{end+1,1}=name; %#ok<AGROW>
 reason{end+1,1}='UNTRACKED_OLD_DIAGNOSTIC_NOT_IN_R193_EXECUTED_OR_STATIC_CLOSURE'; %#ok<AGROW>
end
stamp=datestr(now,'yyyymmdd_HHMMSS_FFF');
report=fullfile(out,['R193_REFERENCE_CLEANUP_' action '_' stamp '.txt']);
if strcmp(action,'PREVIEW_ALL'),select=allInactive;end
if strcmp(action,'QUARANTINE_SELECTED')
    assert(iscell(token)&&~isempty(token), 'R193:SelectionRequired', ...
      'Pass reviewed filenames as a cell array, e.g. {''old_audit.m''}.');
    requested=token;select={};
    for j=1:numel(requested)
      n=char(requested{j});
      assert(~contains(n,'/') && ~contains(n,'\') && ...
        ~contains(n,'..') && any(strcmp(n,allInactive)), ...
        'R193:NotEligible','Not eligible for quarantine: %s',n);
      select{end+1,1}=n; %#ok<AGROW>
    end
end
C=struct('status','PREVIEW_ONLY','candidate_count',numel(select), ...
 'candidate_paths',{select},'reason_codes',{reason},'report_path',report, ...
 'moved_count',0,'blocked_count',0,'quarantine_path','', ...
 'active_zip_path',R.zip_path);
if any(strcmp(action,{'PREVIEW','PREVIEW_ALL'}))
 r193_write(report,[{'R193 PREVIEW: No file changes'; ...
 'Not in one Flight is NOT proof that source can be deleted.'};select]);
 fprintf('[R193 CLEANUP] preview=%d; %s\n',numel(select),report);
 return;
end
if strcmp(action,'QUARANTINE')
 assert(strcmp(char(token),'CONFIRM_R193_UNTRACKED_DIAGNOSTIC_QUARANTINE'), ...
 'R193:ConfirmationNeeded','Review PREVIEW and pass exact confirmation token.');
end
% Before moving, repeat Git and current file existence checks.
Q=fullfile(out,['R193_QUARANTINE_' stamp]);mkdir(Q);
failed={};oknames={};
for k=1:numel(select)
 n=select{k}; src=fullfile(ref,n);
 if ~isfile(src),continue;end
 % A previous capture may be stale after changes; protect all currently
 % loaded code too. Manual review needed for these.
 loaded=inmem('-completenames');
 if any(strcmpi(strrep(loaded,'/',filesep),strrep(src,'/',filesep)))
    failed{end+1,1}=['LOADED_IN_MATLAB: ' n]; %#ok<AGROW>
    continue;
 end
 dest=fullfile(Q,n);
 [ok,msg]=movefile(src,dest);
 if ok,oknames{end+1,1}=n; %#ok<AGROW>
 else,failed{end+1,1}=[n ' : ' msg];end %#ok<AGROW>
end
C.moved_count=numel(oknames);C.blocked_count=numel(failed);
C.quarantine_path=Q;
C.status='R193_QUARANTINED_UNTRACKED_DIAGNOSTICS__RESTORABLE';
r193_write(report,[{C.status;'MOVED:'};oknames;{'BLOCKED:'};failed]);
fprintf('[R193 CLEANUP] moved=%d blocked=%d; %s\n', ...
 C.moved_count,C.blocked_count,Q);
end
function ok=r193_main_unchanged(R,root)
ok=false;
f=fopen(R.manifest_path,'rt');if f<0,return;end
c=onCleanup(@()fclose(f));
found=zeros(1,2);good=zeros(1,2);
while true
 l=fgetl(f);if ~ischar(l),break;end
 a=strsplit(l,sprintf('\t'));if numel(a)<4,continue;end
 for j=1:2
  if j==1,target='reference/REPLANNING_SYSTEM.m';
  else,target='run_flight_headless.m';end
  if strcmpi(a{1},target)
   found(j)=1;path=fullfile(root,strrep(target,'/',filesep));
   if isfile(path),good(j)=strcmp(r193_sha(path),a{4});end
  end
 end
end
ok=all(found==1) && all(good==1);
end
function h=r193_sha(f)
md=java.security.MessageDigest.getInstance('SHA-256');
id=fopen(f,'rb');if id<0,h='';return;end
c=onCleanup(@()fclose(id));
while ~feof(id)
 b=fread(id,1048576,'*uint8');
 if ~isempty(b),md.update(typecast(b(:),'int8'));end
end
h=lower(reshape(dec2hex(typecast(md.digest(),'uint8'),2).',1,[]));
end
function yes=r193_core_protect(n)
lo=lower(n);
yes=strcmpi(n,'REPLANNING_SYSTEM.m') || startsWith(lo,'af7r58r193') || ...
     any(strcmpi(n,{'AF7R58R191_EKF_TICK_TRIGGER.m', ...
     'AF7R58R191_HULL_SEPARATE.m','AF7R58R189_C7_LIVE_TRIAL.m', ...
     'AF7R58R172_STRICT_GENERATOR.m','AF7R58R172_STRICT_TRIGGER.m', ...
     'AF7R58R170_GENERATE.m','AF7R58R71_EXACT_HLC_PLANT_PREVIEW.m'}));
end
function yes=r193_old_diagnostic(n)
lo=lower(n);
if startsWith(lo,'af7r58r193'),yes=false;return;end
% Conservative: do not bulk-move possible production helpers/generators.
yes=startsWith(lo,'af7') && ...
 (contains(lo,'_selftest')||contains(lo,'_postflight')|| ...
  contains(lo,'_preflight')||contains(lo,'_install')|| ...
  contains(lo,'_audit')||contains(lo,'_visual')|| ...
  contains(lo,'_collect')||contains(lo,'_viewer')|| ...
  contains(lo,'_report'));
% Old REPLANNING_SYSTEM_pre*.m intentionally left for separate Git restore.
end
function r193_write(file,lines)
f=fopen(file,'wt');assert(f>=0,'R193:WriteFailed');c=onCleanup(@()fclose(f));
for k=1:numel(lines),fprintf(f,'%s\n',char(lines{k}));end
end
