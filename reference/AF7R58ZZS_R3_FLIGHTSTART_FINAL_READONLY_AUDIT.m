function OUT = AF7R58ZZS_R3_FLIGHTSTART_FINAL_READONLY_AUDIT()
% AF7R58ZZS-R3
% Final read-only Flight-start spike audit.
% Uses the already completed AF7R58ZZQ-R2 flight run.
% Does NOT rerun Flight and does NOT modify production.

EXPECTED_SHA='bc97df3a0a7341ac5605d52f4dd94923bfd831a11be535450666ec6d10ff41ec';

OUT=struct();
OUT.schema='AF7R58ZZS_R3_FLIGHTSTART_FINAL_READONLY_V1';
OUT.production_modified=false;
OUT.status='OPEN';
OUT.error=struct();

disp('====================================================================================================');
disp(' AF7R58ZZS-R3 | FLIGHT-START FINAL READ-ONLY AUDIT');
disp('====================================================================================================');
disp('[AUTHORITY] readOnly=1 | noFlightRerun=1 | noProductionWrite=1 | fixedHorizon4s=1');

try
    pR=which('REPLANNING_SYSTEM');
    assert(~isempty(pR)&&isfile(pR),'[AF7R58ZZS-R3] REPLANNING_SYSTEM unresolved.');
    OUT.source_sha=file_hash(pR);
    OUT.source_exact=strcmp(OUT.source_sha,EXPECTED_SHA);
    assert(OUT.source_exact,'[AF7R58ZZS-R3] source differs from exact AF7R58ZZQ-R2 authority.');

    assert(evalin('base','exist(''AF7R58ZZB_RUNTIME_PROFILE'',''var'')')==1, ...
        '[AF7R58ZZS-R3] AF7R58ZZB_RUNTIME_PROFILE missing.');
    P=evalin('base','AF7R58ZZB_RUNTIME_PROFILE');

    t=double(P.t(:));
    ph=char(P.phase(:));
    full=1e3*double(P.full_iteration_s(:));
    online=1e3*double(P.online_before_plant_s(:));
    refms=1e3*double(P.reference_s(:));
    req=double(P.gen_request_delta(:));

    fw=find(ph=='f' & t>=5.0-1e-12 & t<=5.55+1e-12 & isfinite(full));
    assert(~isempty(fw),'[AF7R58ZZS-R3] no finite Flight-start runtime rows.');
    [~,j]=max(full(fw));
    ip=fw(j);

    OUT.flight_peak=struct('index',ip,'t_s',t(ip),'full_ms',full(ip), ...
        'online_ms',online(ip),'reference_ms',refms(ip),'request_delta',req(ip));

    disp('[A. CURRENT-RUN FLIGHT-START PEAK]');
    disp(sprintf(' t=%.6f s | full=%.6f ms | online=%.6f ms | reference=%.6f ms | request=%.0f', ...
        t(ip),full(ip),online(ip),refms(ip),req(ip)));

    F={ ...
        'rt_spike_capture_time', ...
        'rt_spike_total_ms', ...
        'rt_spike_stage_front_ms', ...
        'rt_spike_stage_loadmass_ms', ...
        'rt_spike_stage_dependency_ms', ...
        'rt_spike_stage_base_cert_geom_ms', ...
        'rt_spike_stage_sensor_trigger_ms', ...
        'rt_spike_stage_output_state_ms', ...
        'rt_spike_base_ref_ms', ...
        'rt_spike_flight_guard_ms', ...
        'rt_spike_cert_guard_ms', ...
        'rt_spike_active_ref_ms', ...
        'rt_spike_geometry_ms', ...
        'rt_spike_state_handle_ms', ...
        'rt_spike_write_active_output_ms', ...
        'rt_spike_state_basic_ms', ...
        'rt_spike_state_trigger_scalar_ms', ...
        'rt_spike_state_trigger_heavy_ms', ...
        'rt_spike_state_cert_ms', ...
        'rt_spike_state_meta_ms'};

    OUT.logger_found=false;
    OUT.logger_capture_found=false;
    OUT.capture=struct();

    if evalin('base','exist(''logger'',''var'')')==1
        L=evalin('base','logger');
        D=member(L,'Data');
        A=member(D,'agent');
        if numel(A)>1, A=A(1); end
        C=member(member(A,'reference'),'result');
        if ~iscell(C), C=num2cell(C); end
        C=C(:);
        tl=member(D,'t');
        if isempty(tl)
            tl=(0:numel(C)-1)'*0.025;
        else
            tl=double(tl(:));
        end
        N=min(numel(C),numel(tl));
        C=C(1:N);
        tl=tl(1:N);
        X=nan(N,numel(F));
        for k=1:N
            st=member(C{k},'state');
            if isempty(st), continue; end
            for q=1:numel(F)
                [v,ok]=scalar(member(st,F{q}));
                if ok, X(k,q)=v; end
            end
        end
        OUT.logger_found=true;

        icap=find(strcmp(F,'rt_spike_capture_time'),1);
        itot=find(strcmp(F,'rt_spike_total_ms'),1);
        cap=X(:,icap);
        cand=find(isfinite(cap) & cap>=4.9 & cap<=5.6);
        if ~isempty(cand)
            vals=X(cand,itot);
            vals2=vals;
            vals2(~isfinite(vals2))=-inf;
            if any(isfinite(vals))
                [~,jj]=max(vals2);
                ir=cand(jj);
            else
                [~,jj]=min(abs(cap(cand)-t(ip)));
                ir=cand(jj);
            end
            OUT.logger_capture_found=true;
            OUT.capture.log_index=ir;
            OUT.capture.log_time_s=tl(ir);
            OUT.capture.values=struct();
            for q=1:numel(F)
                OUT.capture.values.(F{q})=X(ir,q);
            end
        end
    end

    OUT.live_object_found=false;
    OUT.live_capture=struct();
    if evalin('base','exist(''replanner_instance'',''var'')')==1
        obj=evalin('base','replanner_instance');
        if isa(obj,'REPLANNING_SYSTEM')
            st=member(member(obj,'result'),'state');
            OUT.live_object_found=true;
            for q=1:numel(F)
                [v,ok]=scalar(member(st,F{q}));
                if ok
                    OUT.live_capture.(F{q})=v;
                else
                    OUT.live_capture.(F{q})=NaN;
                end
            end
        end
    end

    if OUT.logger_capture_found
        V=OUT.capture.values;
        source='LOGGER_SPIKE_CAPTURE';
    elseif OUT.live_object_found && isfield(OUT.live_capture,'rt_spike_capture_time') && ...
            isfinite(OUT.live_capture.rt_spike_capture_time) && ...
            OUT.live_capture.rt_spike_capture_time>=4.9 && ...
            OUT.live_capture.rt_spike_capture_time<=5.6
        V=OUT.live_capture;
        source='LIVE_OBJECT_SPIKE_CAPTURE';
    else
        V=struct();
        source='NO_CURRENT_SPIKE_CAPTURE';
    end
    OUT.capture_source=source;

    stageNames={ ...
        'rt_spike_stage_front_ms', ...
        'rt_spike_stage_loadmass_ms', ...
        'rt_spike_stage_dependency_ms', ...
        'rt_spike_stage_base_cert_geom_ms', ...
        'rt_spike_stage_sensor_trigger_ms', ...
        'rt_spike_stage_output_state_ms'};

    OUT.stage_ranking=struct('name',{},'ms',{});
    for q=1:numel(stageNames)
        nm=stageNames{q};
        vv=NaN;
        if isstruct(V) && isfield(V,nm), vv=double(V.(nm)); end
        OUT.stage_ranking(end+1)=struct('name',nm,'ms',vv); %#ok<AGROW>
    end
    ms=[OUT.stage_ranking.ms];
    valid=isfinite(ms);

    disp('[B. CURRENT SPIKE CAPTURE]');
    disp([' source = ' source]);
    if isstruct(V) && isfield(V,'rt_spike_capture_time')
        disp(sprintf(' capture time = %.6f s',V.rt_spike_capture_time));
    end
    if isstruct(V) && isfield(V,'rt_spike_total_ms')
        disp(sprintf(' captured REPLANNING_SYSTEM total = %.6f ms',V.rt_spike_total_ms));
    end
    for q=1:numel(OUT.stage_ranking)
        disp(sprintf(' %-42s %.6f ms',OUT.stage_ranking(q).name,OUT.stage_ranking(q).ms));
    end

    if any(valid)
        msc=ms;
        msc(~valid)=-inf;
        [mx,jx]=max(msc);
        OUT.dominant_stage=OUT.stage_ranking(jx).name;
        OUT.dominant_stage_ms=mx;
        if strcmp(OUT.dominant_stage,'rt_spike_stage_base_cert_geom_ms')
            OUT.classification='FLIGHTSTART_SPIKE_DOMINATED_BY_BASE_CERT_GEOMETRY';
        elseif strcmp(OUT.dominant_stage,'rt_spike_stage_output_state_ms')
            OUT.classification='FLIGHTSTART_SPIKE_DOMINATED_BY_OUTPUT_STATE';
        elseif strcmp(OUT.dominant_stage,'rt_spike_stage_sensor_trigger_ms')
            OUT.classification='FLIGHTSTART_SPIKE_DOMINATED_BY_SENSOR_TRIGGER';
        elseif strcmp(OUT.dominant_stage,'rt_spike_stage_dependency_ms')
            OUT.classification='FLIGHTSTART_SPIKE_DOMINATED_BY_DEPENDENCY_REFRESH';
        else
            OUT.classification=['FLIGHTSTART_SPIKE_DOMINATED_BY_' upper(strrep(OUT.dominant_stage,'rt_spike_stage_',''))];
        end
        OUT.status='CURRENT_FLIGHTSTART_SPIKE_STAGE_AVAILABLE__NO_PRODUCTION_CHANGE';
    else
        OUT.dominant_stage='';
        OUT.dominant_stage_ms=NaN;
        OUT.classification='CURRENT_FINE_SPIKE_CAPTURE_UNAVAILABLE__USE_RUNTIME_PEAK_ONLY';
        OUT.status='RUNTIME_PEAK_CONFIRMED__FINE_CAPTURE_UNAVAILABLE__NO_PRODUCTION_CHANGE';
    end

    disp('[C. CLASSIFICATION]');
    disp([' classification = ' OUT.classification]);
    if ~isempty(OUT.dominant_stage)
        disp(sprintf(' dominant stage = %s | %.6f ms',OUT.dominant_stage,OUT.dominant_stage_ms));
    end
    disp([' status = ' OUT.status]);

catch ME
    OUT.status='AUDIT_ERROR__NO_PRODUCTION_CHANGE';
    OUT.error=struct('identifier',ME.identifier,'message',ME.message,'stack',{ME.stack});
    disp('[AUDIT ERROR]');
    disp([ME.identifier ' | ' ME.message]);
end

root=fileparts(which('REPLANNING_SYSTEM'));
if isempty(root), root=pwd; end
outdir=tempname(root);
[ok,msg]=mkdir(outdir);
assert(ok,'%s',msg);
report=fullfile(outdir,'AF7R58ZZS_R3_REPORT.txt');
matp=fullfile(outdir,'AF7R58ZZS_R3_OUT.mat');
fid=fopen(report,'wt','n','UTF-8');
assert(fid>=0);
fprintf(fid,'status=%s',OUT.status); fprintf(fid,'%c',10);
if isfield(OUT,'source_sha'), fprintf(fid,'sourceSHA=%s',OUT.source_sha); fprintf(fid,'%c',10); end
if isfield(OUT,'flight_peak')
    fprintf(fid,'flightPeakT=%.17g',OUT.flight_peak.t_s); fprintf(fid,'%c',10);
    fprintf(fid,'flightPeakFullMs=%.9g',OUT.flight_peak.full_ms); fprintf(fid,'%c',10);
    fprintf(fid,'flightPeakOnlineMs=%.9g',OUT.flight_peak.online_ms); fprintf(fid,'%c',10);
    fprintf(fid,'flightPeakReferenceMs=%.9g',OUT.flight_peak.reference_ms); fprintf(fid,'%c',10);
    fprintf(fid,'flightPeakRequestDelta=%.9g',OUT.flight_peak.request_delta); fprintf(fid,'%c',10);
end
if isfield(OUT,'capture_source'), fprintf(fid,'captureSource=%s',OUT.capture_source); fprintf(fid,'%c',10); end
if isfield(OUT,'classification'), fprintf(fid,'classification=%s',OUT.classification); fprintf(fid,'%c',10); end
if isfield(OUT,'dominant_stage'), fprintf(fid,'dominantStage=%s',OUT.dominant_stage); fprintf(fid,'%c',10); end
if isfield(OUT,'dominant_stage_ms'), fprintf(fid,'dominantStageMs=%.9g',OUT.dominant_stage_ms); fprintf(fid,'%c',10); end
if isfield(OUT,'error')&&isstruct(OUT.error)&&isfield(OUT.error,'message')
    fprintf(fid,'errorIdentifier=%s',OUT.error.identifier); fprintf(fid,'%c',10);
    fprintf(fid,'errorMessage=%s',OUT.error.message); fprintf(fid,'%c',10);
end
fclose(fid);
OUT.report_path=report;
OUT.mat_path=matp;
save(matp,'OUT','-v7');
assignin('base','AF7R58ZZS_R3_OUT',OUT);
disp(['RETURN: ' report]);
disp(['RETURN: ' matp]);
end

function [x,ok]=scalar(v)
x=NaN; ok=false;
for k=1:3
    if iscell(v)&&numel(v)==1
        v=v{1};
    else
        break;
    end
end
if (isnumeric(v)||islogical(v))&&isscalar(v)&&isfinite(double(v))
    x=double(v);
    ok=true;
end
end

function v=member(x,n)
v=[];
try
    for k=1:3
        if iscell(x)&&numel(x)==1
            x=x{1};
        else
            break;
        end
    end
    if isstruct(x)&&isscalar(x)&&isfield(x,n)
        v=x.(n);
    elseif isobject(x)&&isscalar(x)&&isprop(x,n)
        v=x.(n);
    end
catch
    v=[];
end
end

function h=file_hash(path)
md=java.security.MessageDigest.getInstance('SHA-256');
fid=fopen(path,'rb');
assert(fid>=0);
c=onCleanup(@()fclose(fid)); %#ok<NASGU>
while true
    b=fread(fid,1024*1024,'*uint8');
    if isempty(b), break; end
    md.update(b);
end
u=typecast(md.digest(),'uint8');
h=lower(reshape(dec2hex(u,2).',1,[]));
end
