function R=AF7R58R70_EXPORT_MISSING_DYNAMICS()
% R70 read-only collection of LIVE model/estimator/flat-reference sources.
% No Flight rerun, no modification of HLC, plant, parameter or Replanner.
    outdir=fileparts(mfilename('fullpath'));
    names={ ...
        'Model_Suspended_Load','Load_mL_HL','get_model_name', ...
        'Estimator_EKF_SuspendedLoad','EKF','STATE_CLASS', ...
        'DRONE_PARAM_SUSPENDED_LOAD','SUSPENDED_LOAD_STATE_MANAGER', ...
        'Model_Quat13','MODEL_CLASS','HLC_SUSPENDED_LOAD', ...
        'Controller_HL_Suspended_Load','run_flight_headless', ...
        'TIME_VARYING_REFERENCE','gen_ref_line','gen_ref_circle', ...
        'gen_ref_saddle','SUSPENDED_LOAD_REF_ADJUST', ...
        'ENVIRONMENT_OBSTACLE_ELLIPSE_MOVE','REPLANNING_SYSTEM'};
    liveMethods={};
    if evalin('base','exist(''agent'',''var'')')==1
        try
            A=evalin('base','agent');
            if isprop(A,'plant')&&isa(A.plant.method,'function_handle')
                FF=functions(A.plant.method);
                if isfield(FF,'file')&&~isempty(FF.file)
                    liveMethods{end+1}=char(FF.file); %#ok<AGROW>
                else
                    liveMethods{end+1}=which(func2str(A.plant.method)); %#ok<AGROW>
                end
            end
            if isprop(A,'estimator')&&isprop(A.estimator,'method') && ...
                    isa(A.estimator.method,'function_handle')
                liveMethods{end+1}=which(func2str(A.estimator.method)); %#ok<AGROW>
            end
        catch
            % Inventory reports missing entries; never guess model sources.
        end
    end
    report=cell(0,1);
    sourcePaths={};sourceNames={};
    for k=1:numel(names)
        path=which(names{k});
        if isempty(path)||~isfile(path)
            report{end+1,1}=sprintf('MISSING | %s',names{k}); %#ok<AGROW>
        else
            report{end+1,1}=sprintf('FOUND | %s | %s',names{k},path); %#ok<AGROW>
            sourcePaths{end+1}=path;sourceNames{end+1}=names{k}; %#ok<AGROW>
        end
    end
    for k=1:numel(liveMethods)
        path=liveMethods{k};
        if ~isempty(path)&&isfile(path)
            report{end+1,1}=sprintf('LIVE_METHOD | %s',path); %#ok<AGROW>
            sourcePaths{end+1}=path;sourceNames{end+1}='LIVE_ODE_METHOD'; %#ok<AGROW>
        else
            report{end+1,1}='LIVE_METHOD | UNRESOLVED'; %#ok<AGROW>
        end
    end
    keep=true(1,numel(sourcePaths));
    for k=1:numel(sourcePaths)
        for j=1:k-1
            if strcmp(sourcePaths{k},sourcePaths{j}),keep(k)=false;break;end
        end
    end
    sourcePaths=sourcePaths(keep);sourceNames=sourceNames(keep);
    tmp=tempname;mkdir(tmp);
    cleanup=onCleanup(@()rmdir(tmp,'s'));
    files=cell(1,numel(sourcePaths));
    for k=1:numel(sourcePaths)
        [~,name,ext]=fileparts(sourcePaths{k});
        newname=sprintf('%02d_%s%s',k,name,ext);
        destination=fullfile(tmp,newname);
        [ok,msg]=copyfile(sourcePaths{k},destination);
        assert(ok,'[R70] Failed copying actual source: %s',msg);
        files{k}=newname;
    end
    zipfile=fullfile(outdir,'AF7R58R70_ACTUAL_DYNAMICS_SOURCES.zip');
    if ~isempty(files),zip(zipfile,files,tmp);end
    inventory=fullfile(outdir,'AF7R58R70_ACTUAL_DYNAMICS_INVENTORY.txt');
    fid=fopen(inventory,'w');assert(fid>0,'[R70] Cannot create inventory.');
    closef=onCleanup(@()fclose(fid));
    fprintf(fid,'R70 | ACTUAL LIVE MATLAB SOURCES | READ ONLY\n');
    fprintf(fid,'Settings must be read from live external authorities on EACH Flight.\n');
    fprintf(fid,'Most recent EKF mL is authoritative EACH CONTROL STEP.\n');
    fprintf(fid,'No model source may be inferred from MODEL_CLASS alone.\n');
    for k=1:numel(report),fprintf(fid,'%s\n',report{k});end
    clear closef cleanup;
    R=struct('zip_file',zipfile,'report_file',inventory, ...
        'resolved_sources',numel(sourcePaths),'report',{report});
end
