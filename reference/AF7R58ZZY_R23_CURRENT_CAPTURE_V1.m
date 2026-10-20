function OUT=AF7R58ZZY_R23_CURRENT_CAPTURE_V1()
% Run AFTER the existing R23 result. No rerun/Flight/installation is performed.
% Reads source files and copies runtime_profile from base workspace.
% Returns ONE ZIP with raw timing, status and the current runner/class sources.
% Does not call controller/replanner.do, construct objects, clear classes or patch files.
root=fileparts(mfilename('fullpath')); outdir=tempname(root);
[ok,msg]=mkdir(outdir); assert(ok,'%s',msg);
f=fopen(fullfile(outdir,'REPORT.txt'),'wt','n','UTF-8'); assert(f>=0);
cl=onCleanup(@()fclose(f)); %#ok<NASGU>
OUT=struct(); OUT.schema='AF7R58ZZY_R23_CURRENT_CAPTURE_V1';
OUT.flight_run=false; OUT.production_modified=false;
OUT.matlab_version=version; OUT.computer=computer;
OUT.runtime_profile=[]; OUT.object_class=''; OUT.status=struct();
OUT.status_errors=struct(); OUT.loaded_functions={};
try, OUT.loaded_functions=inmem('-completenames'); catch, end
if evalin('base','exist(''runtime_profile'',''var'')==1')
    OUT.runtime_profile=evalin('base','runtime_profile');
end
if evalin('base','exist(''replanner_instance'',''var'')==1')
    obj=evalin('base','replanner_instance'); OUT.object_class=class(obj);
    % Known read-only status getters only; no serialization of handle object.
    getters={'get_staged_pipeline_status','get_r23_performance_status', ...
        'get_r21_performance_status','get_r19_performance_status'};
    for k=1:numel(getters)
        nm=getters{k};
        if ismethod(obj,nm)
            try
                OUT.status.(nm)=feval(nm,obj);
            catch ME
                OUT.status_errors.(nm)=ME.message;
            end
        end
    end
    fields={'last_replan_commit_time','prediction_dt','hard_modify_deadline_s', ...
        'active_reference_version','active_reference_mode','AVOIDANCE_COMMIT_POLICY'};
    for k=1:numel(fields)
        nm=fields{k};
        try, OUT.status.(nm)=obj.(nm); catch ME, OUT.status_errors.(nm)=ME.message; end
    end
end
say(f,'R23 CURRENT EXECUTION CAPTURE - NO SIMULATION RERUN');
say(f,'active class = %s',OUT.object_class);
say(f,'MATLAB = %s',OUT.matlab_version);
say(f,'Disk source SHA does not prove an already loaded class has identical bytes.');
say(f,'Current class, path alternatives, and loaded function inventory are retained separately.');
% Files actually owning the R23 execution route, including runner and baseline.
names={'run_flight_headless','REPLANNING_SYSTEM_R23_COMPILED', ...
 'REPLANNING_SYSTEM_R21_BALANCED','REPLANNING_SYSTEM_R19_FAST','REPLANNING_SYSTEM', ...
 'TIME_VARYING_REFERENCE','AF7R58ZZY_R23_REPORT', ...
 'af7r76_compile_correction_packet_R23','af7r76_compiled_eval_mex', ...
 'af7r75_posaccel_batch_R19','af7r75_posaccel_core_R19', ...
 'af7r73_posaccel_core_R17','af7r72_posaccel_core_R16', ...
 'af7r33_flat_jet_batch_model','af7r58i_eval_canonical_reference', ...
 'af7r58i_prepare_batch_model','af7r58i_commit_spacing_guard'};
if ~isempty(OUT.object_class)&&~any(strcmp(names,OUT.object_class))
    names{end+1}=OUT.object_class;
end
sources=struct('name',{},'resolved_path',{},'path_alternatives',{}, ...
 'sha256',{},'snapshot_file',{},'kind',{});
mkdir(fullfile(outdir,'sources'));
for k=1:numel(names)
    nm=names{k}; p=which(nm); allp=which(nm,'-all');
    s=struct('name',nm,'resolved_path',p,'path_alternatives',{allp}, ...
        'sha256','','snapshot_file','','kind','MISSING');
    if ~isempty(p)&&isfile(p)
        [~,~,ext]=fileparts(p); s.sha256=sha_file(p);
        if strcmpi(ext,'.m')||strcmpi(ext,'.cpp')||strcmpi(ext,'.hpp')
            dest=fullfile(outdir,'sources',[nm ext]);
            copyfile(p,dest); s.snapshot_file=dest; s.kind='SOURCE';
            assert(strcmp(s.sha256,sha_file(dest)),'source changed while copying: %s',nm);
        else
            s.kind='BINARY_HASH_ONLY';
        end
    end
    sources(end+1)=s; %#ok<AGROW>
    say(f,'%s | %s | SHA=%s',nm,s.kind,s.sha256);
    say(f,' path=%s',p);
    if iscell(allp)
        for j=1:numel(allp), say(f,' candidate=%s',allp{j}); end
    end
end
OUT.sources=sources;
P=OUT.runtime_profile;
if isstruct(P)&&isscalar(P)&&all(isfield(P,{'t','phase','online_before_plant_s'}))
    t=double(P.t(:)); phase=P.phase(:); ms=1e3*double(P.online_before_plant_s(:));
    assert(numel(t)==numel(phase)&&numel(t)==numel(ms),'runtime_profile length mismatch');
    if ischar(phase), fl=phase=='f'; else, fl=strcmp(string(phase),'f'); end
    fl=fl&isfinite(t)&isfinite(ms);
    OUT.flight_online_count=nnz(fl);
    OUT.flight_online_over4_count=nnz(fl&ms>4);
    OUT.flight_online_max_ms=NaN;
    if any(fl), OUT.flight_online_max_ms=max(ms(fl)); end
    say(f,'full online metric includes non-replanner work; not a replanner-only deadline metric.');
    say(f,'Flight full-online count=%d | over4ms=%d | max=%.6fms', ...
        OUT.flight_online_count,OUT.flight_online_over4_count,OUT.flight_online_max_ms);
    if isfield(OUT.status,'last_replan_commit_time')
        tc=double(OUT.status.last_replan_commit_time);
        if isscalar(tc)&&isfinite(tc)
            ix=find(fl&t>=tc-.15&t<=tc+.2);
            say(f,'commit-window full-online samples (milliseconds):');
            for j=ix(:).', say(f,' t=%.17g | online=%.6f',t(j),ms(j)); end
        end
    end
    OUT.runtime_profile_fields=fieldnames(P);
    say(f,'raw timing fields: %s',strjoin(fieldnames(P),', '));
else
    say(f,'runtime_profile missing or incompatible: sources still captured; do NOT rerun automatically.');
end
OUT.output_directory=outdir;
save(fullfile(outdir,'RAW_CAPTURE.mat'),'OUT','-v7');
say(f,'No speed improvement or 4ms pass is claimed by this capture.');
say(f,'Return the ZIP printed below. No extra files and no Flight rerun are needed.');
% Close before zipping so REPORT is complete on disk.
clear cl;
zipPath=[outdir '_R23_CURRENT.zip'];
zip(zipPath,{'REPORT.txt','RAW_CAPTURE.mat','sources'},outdir);
OUT.return_zip=zipPath; assignin('base','AF7R58ZZY_R23_CURRENT_CAPTURE_OUT',OUT);
fprintf('\nRETURN THIS ONE FILE:\n%s\n',zipPath);
end
function say(f,fmt,varargin)
s=sprintf(fmt,varargin{:});fprintf(1,'%s\n',s);fprintf(f,'%s\n',s);
end
function h=sha_file(p)
f=fopen(p,'rb');assert(f>=0,'cannot open %s',p);c=onCleanup(@()fclose(f)); %#ok<NASGU>
b=fread(f,Inf,'*uint8');m=java.security.MessageDigest.getInstance('SHA-256');
m.update(typecast(b,'int8'));d=typecast(m.digest(),'uint8');
h=lower(reshape(dec2hex(d,2).',1,[]));
end
