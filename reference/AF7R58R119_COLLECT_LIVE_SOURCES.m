function OUT = AF7R58R119_COLLECT_LIVE_SOURCES(includeRecentPostflightMat)
% Read-only source collection for EKF/HLC/plant 4s trigger + C7 corridor planner.
% NEVER edits active sources, MATLAB path, HLC, native MEX, or current state.
% Put this file in any chosen audit folder; output ZIP is written beside it.
% Call: S = AF7R58R119_COLLECT_LIVE_SOURCES();
% Optionally include the newest R116 postflight MAT (may be large):
%       S = AF7R58R119_COLLECT_LIVE_SOURCES(true);
if nargin<1, includeRecentPostflightMat=false;end
assert(isscalar(includeRecentPostflightMat) && ...
    (islogical(includeRecentPostflightMat)||isnumeric(includeRecentPostflightMat)), ...
    'R119:MatFlag','Argument must be true/false.');
scriptPath=mfilename('fullpath');outDir=fileparts(scriptPath);
if isempty(outDir),error('R119:ScriptPath','Put script in an actual folder.');end
stamp=datestr(now,'yyyymmdd_HHMMSS_FFF');
work=fullfile(outDir,['AF7R58R119_COLLECT_' stamp]);
stage=fullfile(work,'sources');mkdir(stage);
root=discover_root();
refDir=fullfile(root,'reference');
if ~exist(refDir,'dir')
    error('R119:Root','Could not find live common_matlab/reference at %s.',root);
end
OUT=struct('status','COLLECTING','source_root',root,...
 'archive','','included_count',0,'missing',{{}},'dependency_errors',{{}}, ...
 'not_included_toolbox_count',0,'bytes',0,'manifest',fullfile(work,'manifest.tsv'));
logLines={'STATUS\tFILE\tDETAIL'};
sourceList={}; archiveList={}; sourceBytes=[];
required={ ...
 'REPLANNING_SYSTEM','run_flight_headless','SimHL',...
 'AF7R58R71_EXACT_HLC_PLANT_PREVIEW',...
 'AF7R58R118_DUAL_REPLACEMENT','AF7R58R118_EKF_TRIGGER',...
 'AF7R58R118_C7_CORRIDOR_PLANNER','AF7R58R118_CORRIDOR_C7_QP',...
 'AF7R58R118_HEADING_C6_CERT','AF7R58R118_YAW_JETS_C6',...
 'AF7R58R118_SELFTEST','AF7R58R117_DUAL_REPLACEMENT',...
 'AF7R58R80_CONTINUOUS_SEPARATE_BODY_CERT',...
 'AF7R58R91_STABLE_KINEMATIC_CERT',...
 'AF7R58R92_C7_MINIMUM8_ROUTE','AF7R58R92_SUPPORT_SEED_ROUTES',...
 'AF7R58R99_MULTI_SIZE_KNOT_CORRIDOR',...
 'HLC_SUSPENDED_LOAD','Controller_HL_Suspended_Load',...
 'SUSPENDED_LOAD_REF_ADJUST','with_load_model_euler_for_HL',...
 'with_load_model_mL_euler_for_HL','Model_Suspended_Load',...
 'Estimator_EKF_SuspendedLoad','EKF','SUSPENDED_LOAD_STATE_MANAGER',...
 'DRONE_PARAM_SUSPENDED_LOAD','ENVIRONMENT_OBSTACLE_ELLIPSE_MOVE',...
 'Vfd_SuspendedLoadxyDst','Vs_SuspendedLoadxyDst',...
 'Uf_SuspendedLoadxyDst','Beta2_SuspendedLoadxyDst',...
 'V2_alpha2_SuspendedLoadxyDst',...
 'TIME_VARYING_REFERENCE','gen_ref_circle','gen_ref_line','gen_ref_saddle'};
% All matches are logged, to expose shadows; only the active file is copied.
for i=1:numel(required)
    nm=required{i};hits=which(nm,'-all');
    if ischar(hits),hits=cellstr(hits);end
    if isempty(hits)
        OUT.missing{end+1}=nm; %#ok<AGROW>
        logLines{end+1}=sprintf('MISSING\t%s\tNOT_ON_MATLAB_PATH',nm); %#ok<AGROW>
    else
        for j=1:numel(hits)
            h=char(hits{j});
            if j==1, status='ACTIVE';else,status='SHADOWED';end
            logLines{end+1}=sprintf('%s\t%s\t%s',status,nm,h); %#ok<AGROW>
        end
        if exist(hits{1},'file')==2 || exist(hits{1},'file')==3
            add_file(hits{1},'required');
        end
    end
end
% All directly stored historical modules for the NEW method, independent
% of assistant-assigned revision numbers and irrespective of current PATH.
% This ensures older R102 supporting scripts are not silently left out.
allResearch=dir(fullfile(refDir,'AF7R58R*.m'));
for i=1:numel(allResearch)
    name=allResearch(i).name;
    % Collect implementation sources in broad recent design range, not
    % unrelated performance instrumentation, unless dependencies request it.
    mt=regexp(name,'^AF7R58R([0-9]+)','tokens','once');
    if isempty(mt),continue;end
    idx=str2double(mt{1});
    if idx>=71 && idx<=119
        add_file(fullfile(refDir,name),'history');
    end
end
% Collect matching native geometric kernels and current MEX (no build).
native={'af7r36_core.hpp','af7r36_single_core_mex.cpp',...
    'af7r36_single_core_mex.mexw64'};
for i=1:numel(native)
    p=fullfile(refDir,native{i});
    if exist(p,'file')==2 || exist(p,'file')==3
        add_file(p,'native');
    else
        OUT.missing{end+1}=native{i}; %#ok<AGROW>
        logLines{end+1}=sprintf('MISSING\t%s\t%s',native{i},p); %#ok<AGROW>
    end
end
% Dynamic static-dependency capture: obtains generated HLC auxiliary m-files,
% class files and source-bound R71 dependents, without guessing names.
entry={'AF7R58R71_EXACT_HLC_PLANT_PREVIEW',...
 'AF7R58R118_DUAL_REPLACEMENT','AF7R58R118_C7_CORRIDOR_PLANNER',...
 'REPLANNING_SYSTEM','run_flight_headless','HLC_SUSPENDED_LOAD',...
 'Controller_HL_Suspended_Load','Estimator_EKF_SuspendedLoad'};
for i=1:numel(entry)
    p=which(entry{i});
    if isempty(p)||~exist(p,'file'),continue;end
    try
        [deps,~]=matlab.codetools.requiredFilesAndProducts(p);
        for j=1:numel(deps)
            dep=char(deps{j});
            if local_under_root(dep,root)
                [~,~,ext]=fileparts(dep);
                if any(strcmpi(ext,{'.m','.p','.mlx','.cpp','.hpp','.h','.mexw64'}))
                    add_file(dep,'dependency');
                end
            else
                OUT.not_included_toolbox_count=OUT.not_included_toolbox_count+1;
            end
        end
    catch ME
        OUT.dependency_errors{end+1}=[entry{i} ': ' ME.message]; %#ok<AGROW>
        logLines{end+1}=sprintf('DEP_ERROR\t%s\t%s',entry{i},ME.message); %#ok<AGROW>
    end
end
% Optional: the most recent postflight file, never every MAT in root.
if logical(includeRecentPostflightMat)
    m=dir(fullfile(refDir,'AF7R58R116_POSTFLIGHT_*.mat'));
    if ~isempty(m)
        [~,order]=sort([m.datenum],'descend');
        add_file(fullfile(refDir,m(order(1)).name),'postflight_latest');
    end
end
% Root-aware inventory for verifying that the real Flight sources match
% the earlier transferred snapshots (critical when PATH is shadowed).
logLines{end+1}=sprintf('SUMMARY\tSOURCE_ROOT\t%s',root);
logLines{end+1}=sprintf('SUMMARY\tFILE_COUNT\t%d',OUT.included_count);
logLines{end+1}=sprintf('SUMMARY\tBYTE_COUNT\t%.0f',OUT.bytes);
logLines{end+1}=sprintf('SUMMARY\tMISSING_COUNT\t%d',numel(OUT.missing));
fid=fopen(OUT.manifest,'w');
if fid<0,error('R119:ManifestOpen','Cannot open manifest for writing');end
cleanup=onCleanup(@() fclose(fid)); %#ok<NASGU>
for i=1:numel(logLines)
    fprintf(fid,'%s\n',strrep(logLines{i},'\','\\'));
end
clear cleanup;
[~,zipBase]=fileparts(work);zipPath=fullfile(outDir,[zipBase '.zip']);
% Zip stage files using relative paths, plus manifest outside sources.
include={};
for j=1:numel(archiveList)
    include{end+1}=fullfile('sources',archiveList{j}); %#ok<AGROW>
end
include{end+1}='manifest.tsv';
zip(zipPath,include,work);
OUT.archive=zipPath;
OUT.status='COLLECTED_READ_ONLY__NOT_RUNTIME_VALIDATION';
fprintf('[R119] %s\n',OUT.status);
fprintf('[R119] ZIP: %s\n',OUT.archive);
fprintf('[R119] Included: %d files, %.1f MiB; missing named: %d; dependency errors: %d\n',...
 OUT.included_count,OUT.bytes/1048576,numel(OUT.missing),numel(OUT.dependency_errors));
fprintf('[R119] Original source, HLC, MEX, PATH and Flight state unchanged.\n');

    function add_file(p,tag)
        if ~(exist(p,'file')==2 || exist(p,'file')==3),return;end
        p=char(p);
        [~,~,ext]=fileparts(p);
        if ~any(strcmpi(ext,{'.m','.p','.mlx','.hpp','.h','.cpp','.mexw64','.mat'})),return;end
        if any(strcmpi(sourceList,p)),return;end
        inf=dir(p);
        if isempty(inf)||inf(1).bytes>200*1048576
            logLines{end+1}=sprintf('SKIP_TOO_LARGE\t%s\t%s',tag,p); %#ok<AGROW>
            return;
        end
        if local_under_root(p,root)
            relative=p(numel(root)+2:end);
            % fileparts+fullfile handle Windows file separators.
            relative=strrep(relative,'\','/');
        else
            [~,fn,ex]=fileparts(p);
            relative=fullfile('other_path',sprintf('%03d_%s%s',numel(sourceList)+1,fn,ex));
            relative=strrep(relative,'\','/');
        end
        dest=fullfile(stage,strrep(relative,'/',filesep));
        parent=fileparts(dest);
        if ~exist(parent,'dir'),mkdir(parent);end
        [yes,msg]=copyfile(p,dest);
        if ~yes
            logLines{end+1}=sprintf('COPY_ERROR\t%s\t%s',p,msg); %#ok<AGROW>
            return;
        end
        sourceList{end+1}=p;archiveList{end+1}=relative; %#ok<AGROW>
        sourceBytes(end+1)=inf(1).bytes; %#ok<AGROW>
        OUT.included_count=OUT.included_count+1;
        OUT.bytes=OUT.bytes+inf(1).bytes;
        logLines{end+1}=sprintf('COPIED_%s\t%s\t%s',upper(tag),relative,p); %#ok<AGROW>
    end
end

function root=discover_root()
p=which('REPLANNING_SYSTEM');
assert(~isempty(p),'R119:NoReplanner','REPLANNING_SYSTEM not on MATLAB path.');
current=fileparts(p);
for i=1:7
    if exist(fullfile(current,'reference','REPLANNING_SYSTEM.m'),'file')==2
        root=current;return;
    end
    parent=fileparts(current);
    if strcmp(parent,current),break;end
    current=parent;
end
error('R119:RootNotFound','Find common_matlab project root near active REPLANNING_SYSTEM.');
end
function yes=local_under_root(path,root)
p=strrep(char(path),'\','/');r=strrep(char(root),'\','/');
if ispc,p=lower(p);r=lower(r);end
if r(end)~='/',r=[r '/'];end
yes=length(p)>length(r)&&strncmp(p,r,length(r));
end
