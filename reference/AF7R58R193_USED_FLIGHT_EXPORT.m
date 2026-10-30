function R = AF7R58R193_USED_FLIGHT_EXPORT(mode)
% R193: Capture ACTUALLY EXECUTED Flight MATLAB files, plus static closure.
% No existing controller, EKF, plant, runner, or replanner source is changed.
% Default CAPTURE_FLIGHT: executes the existing HEADLESS SIMULATION once.
% FROM_PROFILE: imports an already recorded 'profile' trace without a Flight.
% MATLAB profile can miss dynamically accessed DATA and unexecuted branches.
if nargin<1 || isempty(mode),mode='CAPTURE_FLIGHT';end
mode=upper(char(mode));
assert(any(strcmp(mode,{'CAPTURE_FLIGHT','FROM_PROFILE'})), ...
    'R193:Mode','Use CAPTURE_FLIGHT or FROM_PROFILE.');
ref=fileparts(mfilename('fullpath'));
[project,leaf]=fileparts(ref);
assert(strcmpi(leaf,'reference') && ...
    isfile(fullfile(ref,'REPLANNING_SYSTEM.m')), ...
    'R193:WrongLocation','Put R193 .m files in common_matlab/reference.');
runner=fullfile(project,'run_flight_headless.m');
assert(isfile(runner),'R193:RunnerMissing','Original headless runner missing.');
resolved=which('REPLANNING_SYSTEM');
canonical=fullfile(ref,'REPLANNING_SYSTEM.m');
assert(~isempty(resolved) && strcmpi(strrep(char(resolved),'/',filesep), ...
 strrep(canonical,'/',filesep)), 'R193:ShadowedReplanner', ...
 'MATLAB path resolves a different REPLANNING_SYSTEM.m; fix path before Flight.');
runnerCode=fileread(runner);
assert(contains(runnerCode,'REPLANNING_SYSTEM'), ...
    'R193:RunnerUnexpected','Original runner source must be reviewed.');
exportDir=fullfile(project,'_AF7_HANDOFF_EXPORT');
if ~isfolder(exportDir),mkdir(exportDir);end
stamp=datestr(now,'yyyymmdd_HHMMSS_FFF');
stage=fullfile(exportDir,['R193_STAGE_' stamp]);
mkdir(stage);mkdir(fullfile(stage,'SOURCE'));
R=struct('status','STARTED','project_root',project,'runner_path',runner, ...
 'mode',mode,'flight_completed',false,'flight_error','', ...
 'profiled_function_count',0,'profiler_project_file_count',0, ...
 'static_file_count',0,'included_files',0,'static_status','NOT_RUN', ...
 'zip_path',fullfile(exportDir,['R193_USED_FLIGHT_' stamp '.zip']), ...
 'manifest_path',fullfile(stage,'R193_MANIFEST.tsv'), ...
 'profile_path',fullfile(stage,'R193_PROFILE.mat'), ...
 'active_project_paths',{{}},'zip_verified',false, ...
 'errors',{{}},'external_paths',{{}});
if strcmp(mode,'CAPTURE_FLIGHT')
    fprintf('[R193] MATLAB HEADLESS SIM FLIGHT, profiler ON. Timing will be distorted.\n');
    profile clear;
    profile on;
    try
        if usejava('desktop'),edit(runner);end
        % Run in BASE: original runner may contain 'clearvars'. Collector's
        % local variables and file gathering logic remain intact.
        escaped=strrep(runner,'''','''''');
        evalin('base',['run(''' escaped ''');']);
        R.flight_completed=true;
    catch ME
        R.flight_error=[char(ME.identifier) ' : ' char(ME.message)];
        fprintf(2,'[R193] Flight stopped: %s\n',R.flight_error);
    end
    profile off;
end
try
    P=profile('info');
    assert(isfield(P,'FunctionTable') && ~isempty(P.FunctionTable), ...
        'R193:NoTrace','No recorded Flight calls in the MATLAB profiler.');
catch ME
    R.status='NO_USABLE_PROFILER_TRACE__NOT_A_COMPLETE_EXPORT';
    R.errors={ME.message};
    save(fullfile(exportDir,'R193_LAST_STATUS.mat'),'R','-v7');
    return;
end
save(R.profile_path,'P','-v7.3');
R.profiled_function_count=numel(P.FunctionTable);
fileMap=containers.Map('KeyType','char','ValueType','char');
roleMap=containers.Map('KeyType','char','ValueType','double');
external={};profileFiles={};
for k=1:numel(P.FunctionTable)
    F=P.FunctionTable(k);
    if ~isfield(F,'FileName') || isempty(F.FileName),continue;end
    path=char(F.FileName);
    if ~isfile(path),continue;end
    if r193_inside(project,path)
        [fileMap,roleMap]=r193_add(fileMap,roleMap,path,1);
        profileFiles{end+1,1}=path; %#ok<AGROW>
    else
        external{end+1,1}=path; %#ok<AGROW>
    end
end
profileFiles=unique(profileFiles,'stable');
R.profiler_project_file_count=numel(profileFiles);
% Hard seeds: scripts may be omitted from the profiler's FunctionTable, and
% runtime uses controller, model and estimator classes via dynamic dispatch.
seeds={runner, fullfile(ref,'REPLANNING_SYSTEM.m'), ...
 fullfile(ref,'AF7R58R193_USED_FLIGHT_EXPORT.m'), ...
 fullfile(ref,'AF7R58R193_REFERENCE_CLEANUP.m')};
seedNames={'HLC_SUSPENDED_LOAD','Controller_HL_Suspended_Load', ...
 'Estimator_EKF_SuspendedLoad','EKF','TIME_VARYING_REFERENCE', ...
 'SUSPENDED_LOAD_REF_ADJUST', 'AF7R58R191_EKF_TICK_TRIGGER', ...
 'AF7R58R189_C7_LIVE_TRIAL'};
for k=1:numel(seedNames)
    w=which(seedNames{k});
    if ~isempty(w) && isfile(w) && r193_inside(project,w)
        seeds{end+1}=w; %#ok<AGROW>
    end
end
% Source dependencies: static closure also includes branches that were NOT
% entered in this particular Flight (not proof of every dynamic feval/str2func).
staticFiles={};
try
    src=unique([seeds(:);profileFiles(:)],'stable');
    isCode=cellfun(@(p)~isempty(regexp(lower(char(p)), ...
       '\.(m|mlx|p)$','once')),src);
    src=src(isCode);
    [deps,~]=matlab.codetools.requiredFilesAndProducts(src);
    staticFiles=deps(:);
    R.static_status='MATLAB_CODETOOLS_TRANSITIVE_STATIC_CLOSURE';
catch ME
    R.static_status=['STATIC_CLOSURE_FAILED__' char(ME.identifier)];
    R.errors{end+1,1}=ME.message;
    % A reduced backup with no static closure is NOT considered verified.
end
for k=1:numel(staticFiles)
    f=char(staticFiles{k});
    if isfile(f) && r193_inside(project,f)
        [fileMap,roleMap]=r193_add(fileMap,roleMap,f,2);
    elseif isfile(f)
        external{end+1,1}=f; %#ok<AGROW>
    end
end
R.static_file_count=numel(staticFiles);
for k=1:numel(seeds)
    [fileMap,roleMap]=r193_add(fileMap,roleMap,seeds{k},4);
end
% MEX binaries may be loaded via name/string handles; include compact MEX
% binaries in canonical reference, plus binaries recorded by profiler above.
mx=dir(fullfile(ref,['*.' mexext]));
for k=1:numel(mx)
    if ~mx(k).isdir
        [fileMap,roleMap]=r193_add(fileMap,roleMap, ...
          fullfile(mx(k).folder,mx(k).name),8);
        [~,stem]=fileparts(mx(k).name);
        for ex={'.cpp','.c','.hpp','.h'}
          bf=fullfile(ref,[stem char(ex{1})]);
          if isfile(bf),[fileMap,roleMap]=r193_add(fileMap,roleMap,bf,16);end
        end
    end
end
% Conservative literal data reads from the actual used code and static
% sources. Computed filenames/paths still require manual review.
resourceNotes={};assetFiles={};
fs=values(fileMap);
assetPattern='(?:load|fileread|readtable|readmatrix|readcell|imread)\s*\(\s*["'']([^"''\r\n]+)["'']';
for k=1:numel(fs)
 f=char(fs{k});
 if ~endsWith(lower(f),'.m'),continue;end
 try
  srcText=fileread(f);
  toks=regexp(srcText,assetPattern,'tokens');
 catch,continue;end
 for j=1:numel(toks)
  token=char(toks{j}{1});
  if contains(token,'*') || contains(token,'?') || ...
    contains(token,'..') || startsWith(token,filesep) || ...
    ~isempty(regexp(token,'^[A-Za-z]:','once')),continue;end
  candidates={fullfile(fileparts(f),token),fullfile(project,token), ...
      fullfile(ref,token)};
  [~,~,ext]=fileparts(token);
  if isempty(ext)
   candidates=[candidates,cellfun(@(x)[x '.mat'],candidates, ...
      'UniformOutput',false)]; %#ok<AGROW>
  end
  for a=1:numel(candidates)
   item=candidates{a};
   if ~isfile(item) || ~r193_inside(project,item),continue;end
   st=dir(item);
   if st.bytes<=16*1024*1024
    [fileMap,roleMap]=r193_add(fileMap,roleMap,item,32);
    assetFiles{end+1,1}=item; %#ok<AGROW>
   else
    resourceNotes{end+1,1}=['LARGE_LITERAL_RESOURCE_NOT_PACKED ' item]; %#ok<AGROW>
   end
   break;
  end
 end
end
R.literal_resource_count=numel(unique(assetFiles));
% Do not add all historical MAT/fig/zip: they were not runtime dependencies.
% Dynamic paths can escape this literal scan, so list the limitation.

ks=keys(fileMap);
manifest=cell(numel(ks)+1,4);
manifest(1,:)={'project_relative_path','source_role_bitmask','bytes','sha256'};
copied={};copyErrors={};
for k=1:numel(ks)
    f=fileMap(ks{k});
    rel=r193_rel(project,f);
    dest=fullfile(stage,'SOURCE',strrep(rel,'/',filesep));
    parent=fileparts(dest);
    if ~isfolder(parent),mkdir(parent);end
    [ok,msg]=copyfile(f,dest,'f');
    if ~ok
        copyErrors{end+1,1}=[rel ' : ' msg]; %#ok<AGROW>
        continue;
    end
    a=r193_sha(f);b=r193_sha(dest);
    if ~strcmp(a,b)
        copyErrors{end+1,1}=['SHA_MISMATCH: ' rel]; %#ok<AGROW>
        continue;
    end
    dinfo=dir(f);
    manifest(k+1,:)={rel,num2str(roleMap(ks{k})),num2str(dinfo.bytes),a};
    copied{end+1,1}=rel; %#ok<AGROW>
end
manifest=manifest([true;~cellfun(@isempty,manifest(2:end,1))],:);
R.active_project_paths=copied;
R.included_files=numel(copied);
R.external_paths=unique(external,'stable');
R.errors=[R.errors(:);copyErrors(:)];
r193_tsv(R.manifest_path,manifest);
r193_lines(fullfile(stage,'R193_EXTERNAL_DEPENDENCIES.txt'),R.external_paths);
r193_lines(fullfile(stage,'R193_RESOURCE_WARNINGS.txt'),resourceNotes);
notes=fullfile(ref,'AF7R58R193_HANDOFF_NOTES.txt');
if isfile(notes),copyfile(notes,fullfile(stage,'R193_HANDOFF_NOTES.txt'));end
r193_lines(fullfile(stage,'R193_LIMITATIONS.txt'),{ ...
 'Profilar dynamic: functions actually called during THIS headless Flight.', ...
 'Static: matlab.codetools.requiredFilesAndProducts on runner and profiled sources.', ...
 'Dynamic feval/str2func/eval and data load by filename can escape static analysis.', ...
 'Unexecuted alternate obstacle branches are not proven reachable or unnecessary.', ...
 'All top-level reference MEX binaries included; code generation/build artifacts may need more.', ...
 'MATLAB/toolbox sources outside common_matlab are listed but not copied.', ...
 'This ZIP is a working-source handoff, not proof of collision avoidance.', ...
 'Flight profiling changes runtime and MUST NOT be used for 25ms timing claims.', ...
 'Active execution data/resources can require separate manual inclusion.'});
% Optionally pack profile call graph when its size is small enough.
profileDetail=dir(R.profile_path);
includeTrace=~isempty(profileDetail) && profileDetail.bytes<=10*1024*1024;
% Store the real source hashes, root locations and Git status for provenance.
[rcGit,gitInfo]=system(sprintf('git -C "%s" status --short',project));
if rcGit~=0,gitInfo='Git status unavailable';end
r193_lines(fullfile(stage,'R193_GIT_STATUS.txt'),{gitInfo});
R.status='ZIP_CREATION_PENDING_VERIFICATION';
save(fullfile(stage,'R193_METADATA.mat'),'R','-v7');
zipItems={'SOURCE','R193_MANIFEST.tsv','R193_EXTERNAL_DEPENDENCIES.txt', ...
 'R193_LIMITATIONS.txt','R193_RESOURCE_WARNINGS.txt', ...
 'R193_GIT_STATUS.txt','R193_METADATA.mat'};
if isfile(fullfile(stage,'R193_HANDOFF_NOTES.txt'))
 zipItems{end+1}='R193_HANDOFF_NOTES.txt';
end
if includeTrace,zipItems{end+1}='R193_PROFILE.mat';end
zip(R.zip_path,zipItems,stage);
% Verify extracted bytes against source SHA-256 before permitting housekeeping.
testDir=fullfile(exportDir,['R193_VERIFY_' stamp]);
mkdir(testDir);
try
    unzip(R.zip_path,testDir);
    assert(R.profiler_project_file_count>0 && numel(copied)>0 && isempty(R.errors) && ...
       strcmp(R.static_status,'MATLAB_CODETOOLS_TRANSITIVE_STATIC_CLOSURE'), ...
       'R193:ClosureIncomplete','Static closure/copies did not pass.');
    for k=2:size(manifest,1)
        f=fullfile(testDir,'SOURCE',strrep(manifest{k,1},'/',filesep));
        assert(isfile(f) && strcmp(r193_sha(f),manifest{k,4}), ...
            'R193:VerificationFailed','ZIP mismatch at %s',manifest{k,1});
    end
    R.zip_verified=true;
    R.status='R193_ACTUAL_FLIGHT_AND_STATIC_CLOSURE_ZIP_VERIFIED';
catch ME
    R.status='R193_INCOMPLETE_EXPORT__DO_NOT_CLEAN_CODE';
    R.errors{end+1,1}=ME.message;
end
if isfolder(testDir),rmdir(testDir,'s');end
save(fullfile(exportDir,'R193_LAST_STATUS.mat'),'R','-v7');
fprintf('[R193] %s | %d files | %s\n',R.status,R.included_files,R.zip_path);
if ~R.flight_completed && strcmp(mode,'CAPTURE_FLIGHT')
 fprintf(2,'[R193] Flight ended with error; profile represents a PARTIAL Flight.\n');
 R.zip_verified=false;
 R.status='R193_PARTIAL_FLIGHT__NO_CLEANUP_AUTHORIZATION';
 save(fullfile(exportDir,'R193_LAST_STATUS.mat'),'R','-v7');
end
end

function [fm,rm]=r193_add(fm,rm,p,mask)
if ~isfile(p),return;end
key=lower(strrep(char(p),'/',filesep));
if isKey(rm,key),rm(key)=bitor(rm(key),mask);
else,rm(key)=mask;fm(key)=char(p);end
end
function yes=r193_inside(root,file)
r=lower(strrep(char(root),'/',filesep));
f=lower(strrep(char(file),'/',filesep));
yes=startsWith(f,[r filesep]) && ~contains(f,[filesep '_AF7_HANDOFF_EXPORT' filesep]);
end
function rel=r193_rel(root,file)
rel=strrep(char(file(numel(root)+2:end)),filesep,'/');
end
function r193_lines(file,lines)
f=fopen(file,'wt');assert(f>=0,'R193:WriteFailed');c=onCleanup(@()fclose(f));
for k=1:numel(lines),fprintf(f,'%s\n',char(lines{k}));end
end
function r193_tsv(file,rows)
f=fopen(file,'wt');assert(f>=0,'R193:WriteFailed');c=onCleanup(@()fclose(f));
for i=1:size(rows,1)
 fprintf(f,'%s\t%s\t%s\t%s\n',rows{i,1},rows{i,2},rows{i,3},rows{i,4});
end
end
function s=r193_sha(file)
md=java.security.MessageDigest.getInstance('SHA-256');
f=fopen(file,'rb');assert(f>=0,'R193:HashReadFailed');c=onCleanup(@()fclose(f));
while ~feof(f)
 b=fread(f,1048576,'*uint8');
 if ~isempty(b),md.update(typecast(b(:),'int8'));end
end
s=lower(reshape(dec2hex(typecast(md.digest(),'uint8'),2).',1,[]));
end
