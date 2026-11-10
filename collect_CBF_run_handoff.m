function REPORT = collect_CBF_run_handoff(projectRoot, outputZip)
% collect_CBF_run_handoff  Create a portable ZIP of the local CBF MATLAB project.
%  REPORT=collect_CBF_run_handoff();
%  REPORT=collect_CBF_run_handoff('C:\path\to\common_matlab');
%
% Runs AFTER your experiment. Does NOT execute the controller/plant and does
% NOT change source files. It collects the ENTIRE project code subtree
% (including indirect/dynamic function calls), statically detected MATLAB
% dependencies and functions left in memory. Common MAT/CSV/config/image
% datasets are included by default if inside projectRoot. Read SECURITY_NOTES before sharing the ZIP:
% this is a broad project snapshot, not only selected CBF files.
%
% MATLAB's static dependency analyzer can miss eval, str2func, dynamic class
% registration and data loaded by file names. To avoid missing these, every
% project-local source/data of common extensions are included as a superset.
% Dependencies on external MATLAB paths are gathered via static analysis,
% inmem, and an explicit list of known CBF components; unknown dynamic
% dependencies outside the project root may still require manual inclusion.
if nargin<1 || isempty(projectRoot)
    p=which('run_flight_CBF_dual_compare');
    if isempty(p),p=which('run_flight_CBF');end
    if isempty(p),p=which('run_VERTICAL_CBF_DUAL_INPUT_COMPARISON');end
    if isempty(p), p=which('run_VERTICAL_CBF_PHYSICAL_FLIGHT_READY'); end
    if isempty(p),p=which('run_VERTICAL_HLC_FLIGHT_READY');end
    if isempty(p)
        error('CBF:NoRunner','Add the project to path or pass projectRoot.');
    end
    projectRoot=fileparts(p);
end
projectRoot=char(projectRoot);
if exist(projectRoot,'dir')~=7
    error('CBF:MissingRoot','Project directory not found: %s',projectRoot);
end
projectRoot=char(java.io.File(projectRoot).getCanonicalPath());
nowTag=datestr(now,'yyyymmdd_HHMMSS');
if nargin<2 || isempty(outputZip)
    outputZip=fullfile(projectRoot,'CBF_ALL_RUN_SOURCES_HANDOFF_LATEST.zip');
end
outputZip=char(outputZip);
stage=tempname(tempdir);
mkdir(stage);
stageCleanup=onCleanup(@()localCleanStage(stage)); %#ok<NASGU>
mkdir(fullfile(stage,'PROJECT'));
warningLog={};
paths={};
% A broad superset is deliberate: dynamic dispatch and files invoked by
% the original SimSuspendedLoad framework cannot be inferred statically.
extensions={'.m','.p','.mexw64','.mexa64','.mexmaci64', ...
    '.slx','.sldd','.mat','.csv','.json','.yaml','.yml', ...
    '.txt','.xml','.ini','.dat','.fig','.xlsx','.xls', ...
    '.png','.jpg','.jpeg','.urdf','.stl','.obj'};
files=dir(fullfile(projectRoot,'**','*'));
for ii=1:numel(files)
    ff=files(ii);
    if ff.isdir,continue;end
    full=fullfile(ff.folder,ff.name);
    if localExcluded(full,projectRoot),continue;end
    [~,~,e]=fileparts(ff.name);
    if ~any(strcmpi(e,extensions)),continue;end
    paths{end+1,1}=full; %#ok<AGROW>
end
% Some MATLAB dir('**') implementations omit files in root itself.
rootFiles=dir(projectRoot);
for ii=1:numel(rootFiles)
    ff=rootFiles(ii);
    if ff.isdir,continue;end
    [~,~,e]=fileparts(ff.name);
    if any(strcmpi(e,extensions))
        paths{end+1,1}=fullfile(projectRoot,ff.name); %#ok<AGROW>
    end
end
% Include the input script explicitly and static dependencies (including
% external project folders outside projectRoot).
entryNames={'run_flight_CBF_dual_compare','run_flight_CBF', ...
    'run_VERTICAL_CBF_DUAL_INPUT_COMPARISON', ...
    'run_VERTICAL_CBF_PHYSICAL_FLIGHT_READY','run_VERTICAL_HLC_FLIGHT_READY'};
entryFiles={};
for ii=1:numel(entryNames)
    p=which(entryNames{ii});
    if ~isempty(p),entryFiles{end+1}=p;paths{end+1,1}=p;end %#ok<AGROW>
end
productInfo=[];
if ~isempty(entryFiles)
    try
        [staticFiles,productInfo]=matlab.codetools.requiredFilesAndProducts(entryFiles);
        paths=[paths;staticFiles(:)]; %#ok<AGROW>
    catch ME
        warningLog{end+1}=sprintf('STATIC_ANALYSIS: %s',ME.message); %#ok<AGROW>
    end
end
% Dynamic path resolution for functions currently loaded in this MATLAB
% session, after the user's actual Virtual/Physical experiment.
try
    loaded=inmem('-completenames');
    for ii=1:numel(loaded)
        p=loaded{ii};
        if isfile(p)
            paths{end+1,1}=p; %#ok<AGROW>
        end
    end
catch ME
    warningLog{end+1}=sprintf('INMEM: %s',ME.message); %#ok<AGROW>
end
critical={'HLC_SUSPENDED_CBF','Generate_CBF_DualInput_6Methods', ...
    'Generate_CBF_ActuatorAware_6Methods', ...
    'HLC_SUSPENDED_CBF_PHYSICAL_FLIGHT','CBF_PREFLIGHT_WARMUP', ...
    'ENVIRONMENT_OBSTACLE_ELLIPSE_MOVE','Model_Suspended_Load', ...
    'with_load_model_euler_for_HL','with_load_model_mL_euler_for_HL', ...
    'generateModel_SupendedLoad','TIME_VARYING_REFERENCE', ...
    'Controller_HL_Suspended_Load','HLC_SUSPENDED_LOAD', ...
    'SUSPENDED_LOAD_STATE_MANAGER','DRONE','DRONE_PARAM_SUSPENDED_LOAD', ...
    'Estimator_EKF_SuspendedLoad','EKF','TAKEOFF_REFERENCE', ...
    'LANDING_REFERENCE','LOGGER','STATE_CLASS','MODEL_CLASS', ...
    'gen_ref_line','gen_ref_for_rigid_body','Beta2_SuspendedLoadxyDst', ...
    'V2_alpha2_SuspendedLoadxyDst','Uf_SuspendedLoadxyDst', ...
    'Vfd_SuspendedLoadxyDst','Vs_SuspendedLoadxyDst'};
missing={};
for ii=1:numel(critical)
    p=which(critical{ii});
    if isempty(p)
        missing{end+1}=critical{ii}; %#ok<AGROW>
    elseif isfile(p)
        paths{end+1,1}=p; %#ok<AGROW>
    end
end
paths=unique(paths,'stable');
source={}; archived={}; sizes=[]; hashes={};
for ii=1:numel(paths)
    src=char(paths{ii});
    if ~isfile(src),continue;end
    % Never copy proprietary MathWorks installation/toolbox files.
    if localInside(src,matlabroot),continue;end
    inProject=localInside(src,projectRoot);
    if inProject
        relative=src(numel(projectRoot)+2:end);
        relative=strrep(relative,'\',filesep);
        target=fullfile(stage,'PROJECT',relative);
        archivePath=['PROJECT/' strrep(relative,filesep,'/')];
    else
        [~,nm,ex]=fileparts(src);
        destBase=sprintf('%05d_%s%s',ii,nm,ex);
        target=fullfile(stage,'EXTERNAL_MATLAB_PATH_FILES',destBase);
        archivePath=['EXTERNAL_MATLAB_PATH_FILES/' destBase];
    end
    parent=fileparts(target);
    if exist(parent,'dir')~=7,mkdir(parent);end
    [ok,msg]=copyfile(src,target,'f');
    if ~ok
        warningLog{end+1}=sprintf('COPY_FAILED %s: %s',src,msg); %#ok<AGROW>
        continue;
    end
    source{end+1,1}=src; %#ok<AGROW>
    archived{end+1,1}=archivePath; %#ok<AGROW>
    dd=dir(target);sizes(end+1,1)=dd.bytes; %#ok<AGROW>
    hashes{end+1,1}=localHash(target); %#ok<AGROW>
end
T=table(source,archived,sizes,hashes,'VariableNames', ...
    {'original_file','archive_relative_path','size_bytes','sha256'});
writetable(T,fullfile(stage,'FILE_MANIFEST.csv'));
localWriteText(fullfile(stage,'MISSING_OR_UNRESOLVED.txt'), ...
    [{'Known entry points/function names not found on current MATLAB path:'}; ...
    missing(:);{''};{'Collection warnings:'};warningLog(:)]);
allMatlabPaths=strsplit(path,pathsep);
localWriteText(fullfile(stage,'MATLAB_PATH_USED.txt'),allMatlabPaths(:));
try
    v=ver;
    names=cell(numel(v)+1,1);names{1}=version;
    for i=1:numel(v),names{i+1}=[v(i).Name ' ' v(i).Version];end
    localWriteText(fullfile(stage,'MATLAB_VERSION_AND_TOOLBOXES.txt'),names);
catch
end
if ~isempty(productInfo)
    try
        names=cell(numel(productInfo),1);
        for i=1:numel(productInfo),names{i}=[productInfo(i).Name ' ' productInfo(i).Version];end
        localWriteText(fullfile(stage,'STATIC_DEPENDENCY_PRODUCTS.txt'),names);
    catch
    end
end
localWriteText(fullfile(stage,'SECURITY_AND_COMPLETENESS.txt'),{ ...
    'Contains all source/code and MAT files under the selected projectRoot, plus discovered external dependencies.', ...
    'Review the archive before sharing: project MAT files may contain private data.', ...
    'MATLAB installation/toolbox files under matlabroot are deliberately NOT redistributed.', ...
    'Dynamic dependencies outside projectRoot may not be found by static analysis or inmem.', ...
    'Check MISSING_OR_UNRESOLVED.txt and FILE_MANIFEST.csv.', ...
    'Generated sources are not altered. Archived contents are copied read-only.', ...
    'The original zip is NOT a flight-safety certificate or a reproducer without MATLAB/toolboxes.', ...
    'To include code from other project roots, select a common parent root and rerun.'});
% If an exported handoff note was put on path, add exactly this note.
note=which('CBF_RESEARCH_HANDOFF_20261109.txt');
if isempty(note)
    note=fullfile(projectRoot,'CBF_RESEARCH_HANDOFF_20261109.txt');
end
if exist(note,'file')==2
    copyfile(note,fullfile(stage,'CBF_RESEARCH_HANDOFF_20261109.txt'),'f');
else
    localWriteText(fullfile(stage,'CBF_RESEARCH_HANDOFF_20261109.txt'),{ ...
        'Research: hierarchical linearization HLC + flight-only CBF for quadrotor suspended load', ...
        'Parallel Virtual and Physical CBF research; same S3 obstacle geometry and initial/reference condition.', ...
        'Virtual/S3 QP stopped 11.050 s (LP feasible, numerical conditioning to investigate).', ...
        'Physical/S3 QP stopped 13.225 s (LP infeasible; obs3 cable row).', ...
        'Keep original HLC, mass EKF, generic 3D fixed obstacle geometry and takeoff/landing unchanged.', ...
        'Read the full handoff TXT from the associated ChatGPT response if available.'});
end
[zipFolder,~,~]=fileparts(outputZip);
if ~isempty(zipFolder)&&exist(zipFolder,'dir')~=7,mkdir(zipFolder);end
if exist(outputZip,'file')==2,delete(outputZip);end
stageEntries=dir(stage);
archiveItems={};
for ii=1:numel(stageEntries)
    if ~ismember(stageEntries(ii).name,{'.','..'})
        archiveItems{end+1}=stageEntries(ii).name; %#ok<AGROW>
    end
end
zip(outputZip,archiveItems,stage);
REPORT=struct('zip',outputZip,'projectRoot',projectRoot, ...
    'includedFiles',numel(source),'missingCritical',{missing}, ...
    'warnings',{warningLog},'totalSourceBytes',sum(sizes));
fprintf('[CBF-HANDOFF] %d source/data files, %.1f MB; ZIP: %s\n', ...
    REPORT.includedFiles,REPORT.totalSourceBytes/1024^2,outputZip);
if ~isempty(missing)
    fprintf('[CBF-HANDOFF] Missing known functions: %s\n',strjoin(missing,', '));
end
end

function yes=localInside(file,root)
file=lower(strrep(char(file),'/',filesep));
root=lower(strrep(char(root),'/',filesep));
yes=startsWith(file,[root filesep]);
end
function yes=localExcluded(file,root)
relative=file(numel(root)+1:end);
relative=strrep(relative,'\','/');
yes=contains(relative,'/.git/') || contains(relative,'/node_modules/') || ...
    contains(relative,'/CBF_handoff_stage_') || ...
    contains(relative,'/CBF_DUAL_FROM_ORIGINAL_RELEASE_') || ...
    ~isempty(regexp(relative,'/CBF_dual_comparison_[^/]*/','once')) || ...
    ~isempty(regexp(relative,'/CBF_DUAL_FROM_ORIGINAL_[^/]*/','once')) || ...
    ~isempty(regexp(relative,'/CBF_DUAL_WORK_[^/]*/','once')); 
end
function localWriteText(name,lines)
fid=fopen(name,'w','n','UTF-8');
if fid<0,error('CBF:Write','Cannot write %s',name);end
cl=onCleanup(@()fclose(fid));
for i=1:numel(lines)
    fprintf(fid,'%s\n',char(string(lines{i})));
end
end
function hs=localHash(file)
hs='';
try
    md=java.security.MessageDigest.getInstance('SHA-256');
    fid=fopen(file,'rb');
    if fid<0,return;end
    cl=onCleanup(@()fclose(fid));
    while true
        b=fread(fid,1024*1024,'*uint8');
        if isempty(b),break;end
        md.update(b);
    end
    hv=typecast(md.digest(),'uint8');
    hs=lower(reshape(dec2hex(hv,2).',1,[]));
catch
end
end

function localCleanStage(d)
try
    if exist(d,'dir')==7,rmdir(d,'s');end
catch ME
    warning('CBF:TempCleanup','Could not remove temporary source staging folder: %s',ME.message);
end
end
