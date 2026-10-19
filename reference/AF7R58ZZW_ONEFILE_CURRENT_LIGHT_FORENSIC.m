function OUT = AF7R58ZZW_ONEFILE_CURRENT_LIGHT_FORENSIC()
% AF7R58ZZW
% One-run, source-bound forensic of the CURRENT ZZV-R2 build.
%
% Temporarily changes ONLY:
%   ENABLE_LIGHT_RUNTIME_AUDIT = 0;
% to:
%   ENABLE_LIGHT_RUNTIME_AUDIT = 1;
%
% Then runs the existing AF7R58ZZB runner once, reads built-in rt_spike_*
% fields, and restores the original REPLANNING_SYSTEM source bytes.
%
% No algorithm, horizon, sensing, certificate, controller, worker, or runner
% source is changed. Diagnostic timing only; not WCET evidence.

EXPECTED_SHA = ...
    '310cfdd01f9554a80b71e8d90464816a721859709ad7c4b9137cdc0f3a6eaf49';

OUT=struct();
OUT.schema='AF7R58ZZW_CURRENT_LIGHT_FORENSIC_V1';
OUT.production_modified=false;
OUT.source_restored=false;
OUT.status='OPEN';
OUT.error=struct();

fprintf('\n====================================================================================================\n');
fprintf(' AF7R58ZZW | CURRENT REFERENCE/REPLANNER SPIKE FORENSIC\n');
fprintf('====================================================================================================\n');
fprintf('[AUTHORITY] currentZZV_R2=1 | runnerUnchanged=1 | diagnosticOnly=1\n');
fprintf('[TEMP PATCH] ENABLE_LIGHT_RUNTIME_AUDIT 0->1 only | exact-byte restore=1\n');
fprintf('[WINDOWS] FlightEntry=1 | MidFlightNonRequest=1 | ReplanWindow=1\n');

pR=which('REPLANNING_SYSTEM');
assert(~isempty(pR)&&isfile(pR), ...
    '[AF7R58ZZW] REPLANNING_SYSTEM unresolved.');

root=fileparts(pR);
origBytes=read_bytes(pR);
origSHA=sha_bytes(origBytes);
OUT.pre_sha=origSHA;
assert(strcmp(origSHA,EXPECTED_SHA), ...
    '[AF7R58ZZW] source is not exact current ZZV-R2 authority.');

restoreGuard=onCleanup(@()restore_source_exact(pR,origBytes)); %#ok<NASGU>

try
    S=native2unicode(origBytes.','UTF-8');

    old='        ENABLE_LIGHT_RUNTIME_AUDIT = 0; % AF7R50C clean production-timing run: no in-run light audit';
    new='        ENABLE_LIGHT_RUNTIME_AUDIT = 1; % AF7R58ZZW TEMP diagnostic; exact source restored after run';

    assert(count_exact(S,old)==1, ...
        '[AF7R58ZZW] exact light-audit property anchor unresolved.');

    Sdiag=strrep(S,old,new);

    % Static no-algorithm-change check.
    assert(count_exact(Sdiag,'ENABLE_LIGHT_RUNTIME_AUDIT = 1;')>=1);
    assert(count_exact(Sdiag,'horizon_s     = 4.0;')==1);
    assert(~contains(Sdiag,'process_worker_'));
    assert(~contains(Sdiag,'parfeval'));
    assert(contains(Sdiag,'AF7R58ZZV_PREPARED_COMMIT_V2'));
    assert(contains(Sdiag,'detected_only_obstacle_contract'));
    assert(contains(Sdiag,'evaluate_reference_only_proposal_certificate'));

    % Parse full temporary class before touching production path.
    parseDir=tempname(root);
    [ok,msg]=mkdir(parseDir); assert(ok,'%s',msg);
    parseName='AF7R58ZZW_PARSE_REPLANNING_SYSTEM';
    parseText=strrep(Sdiag,'classdef REPLANNING_SYSTEM < handle', ...
        ['classdef ' parseName ' < handle']);
    assert(~strcmp(parseText,Sdiag), ...
        '[AF7R58ZZW] temporary parse rename failed.');
    parsePath=fullfile(parseDir,[parseName '.m']);
    write_utf8(parsePath,parseText);
    addpath(parseDir,'-begin');
    cp=onCleanup(@()safe_rmpath(parseDir)); %#ok<NASGU>
    eval(['clear ' parseName]);
    mc=meta.class.fromName(parseName);
    assert(~isempty(mc),'[AF7R58ZZW] temporary diagnostic class parse failed.');
    eval(['clear ' parseName]);

    % Write temporary diagnostic source.
    write_utf8(pR,Sdiag);
    OUT.production_modified=true;
    OUT.temporary_diagnostic_sha=file_hash(pR);

    fprintf('\n[A. TEMP DIAGNOSTIC SOURCE]\n');
    fprintf(' original SHA  = %s\n',origSHA);
    fprintf(' temporary SHA = %s\n',OUT.temporary_diagnostic_sha);
    fprintf(' algorithm changes = 0\n');

    evalin('base','clear classes');
    evalin('base','rehash');

    runner='run_flight_headless_AF7R58ZZB_reference_cycle_audit';
    pRun=which(runner);
    assert(~isempty(pRun)&&isfile(pRun), ...
        '[AF7R58ZZW] existing AF7R58ZZB runner unresolved.');
    OUT.runner_path=pRun;
    OUT.runner_sha=file_hash(pRun);

    fprintf('\n[B. RUN]\n');
    fprintf(' runner = %s\n',pRun);
    fprintf(' running one fresh diagnostic Flight...\n');

    evalin('base',[runner ';']);

    assert(evalin('base','exist(''AF7R58ZZB_RUNTIME_PROFILE'',''var'')')==1, ...
        '[AF7R58ZZW] runtime profile missing after run.');
    assert(evalin('base','exist(''logger'',''var'')')==1, ...
        '[AF7R58ZZW] logger missing after run.');

    P=evalin('base','AF7R58ZZB_RUNTIME_PROFILE');
    L=evalin('base','logger');

    t=double(P.t(:));
    ph=char(P.phase(:));

    D=member(L,'Data');
    A=member(D,'agent');
    if numel(A)>1, A=A(1); end
    Rres=member(member(A,'reference'),'result');
    if ~iscell(Rres), Rres=num2cell(Rres); end
    Rres=Rres(:);

    tl=member(D,'t');
    if isempty(tl)
        tl=t;
    else
        tl=double(tl(:));
    end

    N=min(numel(Rres),numel(tl));
    Rres=Rres(1:N);
    tl=tl(1:N);

    names={ ...
        'rt_light_e2e_ms', ...
        'rt_observer_ms', ...
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
        'rt_spike_state_meta_ms', ...
        'rt_spike_active_count', ...
        'rt_spike_need_replan'};

    X=nan(N,numel(names));
    for i=1:N
        st=member(Rres{i},'state');
        if isempty(st), continue; end
        for j=1:numel(names)
            [v,okv]=scalar(member(st,names{j}));
            if okv, X(i,j)=v; end
        end
    end

    W=[4.95 5.20; 6.40 6.60; 9.80 10.10];
    labels={'FLIGHT_ENTRY','MIDFLIGHT_NONREQUEST','REPLAN_WINDOW'};
    rows=struct([]);

    fprintf('\n[C. CURRENT-RUN ATTRIBUTION]\n');

    for w=1:size(W,1)
        idx=find(ph=='f' & ...
            t>=W(w,1)-1e-12 & t<=W(w,2)+1e-12 & ...
            isfinite(P.online_before_plant_s(:)));

        assert(~isempty(idx), ...
            '[AF7R58ZZW] no runtime samples in diagnostic window.');

        [~,jj]=max(P.online_before_plant_s(idx));
        ip=idx(jj);
        tp=t(ip);

        [dtmin,il]=min(abs(tl-tp));
        assert(isfinite(dtmin)&&dtmin<=1e-8*max(1,abs(tp)), ...
            '[AF7R58ZZW] logger/runtime time alignment failed.');

        q=struct();
        q.label=labels{w};
        q.t_s=tp;
        q.online_ms=1e3*double(P.online_before_plant_s(ip));
        q.full_ms=1e3*double(P.full_iteration_s(ip));
        q.reference_ms=get_profile_ms(P,'reference_s',ip);
        q.sensor_ms=get_profile_ms(P,'sensor_s',ip);
        q.estimator_ms=get_profile_ms(P,'estimator_s',ip);
        q.controller_ms=get_profile_ms(P,'controller_s',ip);
        q.input_transform_ms=get_profile_ms(P,'input_transform_s',ip);
        q.gen_request_delta=get_profile_value(P,'gen_request_delta',ip);
        q.gen_commit_delta=get_profile_value(P,'gen_commit_delta',ip);
        q.gen_deadline_delta=get_profile_value(P,'gen_deadline_delta',ip);
        q.gen_last_cycle_critical_ms=get_profile_value(P,'gen_last_cycle_critical_ms',ip);
        q.gen_last_callback_ms=get_profile_value(P,'gen_last_callback_ms',ip);

        for j=1:numel(names)
            q.(names{j})=X(il,j);
        end

        if isfinite(q.reference_ms)&&isfinite(q.rt_spike_total_ms)
            q.reference_minus_replanner_ms= ...
                q.reference_ms-q.rt_spike_total_ms;
        else
            q.reference_minus_replanner_ms=NaN;
        end

        stageVals=[ ...
            q.rt_spike_stage_front_ms, ...
            q.rt_spike_stage_loadmass_ms, ...
            q.rt_spike_stage_dependency_ms, ...
            q.rt_spike_stage_base_cert_geom_ms, ...
            q.rt_spike_stage_sensor_trigger_ms, ...
            q.rt_spike_stage_output_state_ms];
        stageNames={'FRONT','LOADMASS','DEPENDENCY','BASE_CERT_GEOM', ...
            'SENSOR_TRIGGER','OUTPUT_STATE'};
        temp=stageVals;
        temp(~isfinite(temp))=-inf;
        [mx,kx]=max(temp);
        q.dominant_stage=stageNames{kx};
        q.dominant_stage_ms=mx;

        rows(w)=q; %#ok<AGROW>

        fprintf('\n %-24s t=%.3f s\n',q.label,q.t_s);
        fprintf('   online / full / reference = %.4f / %.4f / %.4f ms\n', ...
            q.online_ms,q.full_ms,q.reference_ms);
        fprintf('   REPLANNING_SYSTEM light e2e = %.4f ms\n',q.rt_spike_total_ms);
        fprintf('   reference - replanner residual = %.4f ms\n', ...
            q.reference_minus_replanner_ms);
        fprintf('   front/load/dependency = %.4f / %.4f / %.4f ms\n', ...
            q.rt_spike_stage_front_ms,q.rt_spike_stage_loadmass_ms, ...
            q.rt_spike_stage_dependency_ms);
        fprintf('   baseCertGeom / sensorTrigger / outputState = %.4f / %.4f / %.4f ms\n', ...
            q.rt_spike_stage_base_cert_geom_ms, ...
            q.rt_spike_stage_sensor_trigger_ms, ...
            q.rt_spike_stage_output_state_ms);
        fprintf('   baseRef/flightGuard/certGuard/activeRef/geometry = %.4f / %.4f / %.4f / %.4f / %.4f ms\n', ...
            q.rt_spike_base_ref_ms,q.rt_spike_flight_guard_ms, ...
            q.rt_spike_cert_guard_ms,q.rt_spike_active_ref_ms, ...
            q.rt_spike_geometry_ms);
        fprintf('   dominant internal stage = %s | %.4f ms\n', ...
            q.dominant_stage,q.dominant_stage_ms);
        fprintf('   request/commit/deadline = %.0f / %.0f / %.0f | genCritical=%.4f callback=%.4f ms\n', ...
            q.gen_request_delta,q.gen_commit_delta,q.gen_deadline_delta, ...
            q.gen_last_cycle_critical_ms,q.gen_last_callback_ms);
    end

    OUT.rows=rows;
    OUT.status='CURRENT_SOURCE_SPIKE_ATTRIBUTION_COMPLETE';

catch ME
    OUT.status='AUDIT_ERROR__SOURCE_WILL_BE_RESTORED';
    OUT.error=struct('identifier',ME.identifier,'message',ME.message,'stack',{ME.stack});
    fprintf('\n[AUDIT ERROR]\n%s | %s\n',ME.identifier,ME.message);
end

restore_source_exact(pR,origBytes);
OUT.production_modified=false;
OUT.source_restored=strcmp(file_hash(pR),origSHA);

try
    evalin('base','clear classes');
    evalin('base','rehash');
catch
end

fprintf('\n[D. RESTORE]\n');
fprintf(' exact source restored = %d\n',OUT.source_restored);
fprintf(' restored SHA = %s\n',file_hash(pR));
fprintf(' status = %s\n',OUT.status);

outdir=tempname(root);
[ok,msg]=mkdir(outdir); assert(ok,'%s',msg);
report=fullfile(outdir,'AF7R58ZZW_REPORT.txt');
matp=fullfile(outdir,'AF7R58ZZW_OUT.mat');

fid=fopen(report,'wt','n','UTF-8'); assert(fid>=0);
fprintf(fid,'status=%s\n',OUT.status);
fprintf(fid,'sourceRestored=%d\n',OUT.source_restored);
fprintf(fid,'preSHA=%s\n',OUT.pre_sha);
if isfield(OUT,'temporary_diagnostic_sha')
    fprintf(fid,'temporaryDiagnosticSHA=%s\n',OUT.temporary_diagnostic_sha);
end
if isfield(OUT,'runner_sha')
    fprintf(fid,'runnerSHA=%s\n',OUT.runner_sha);
end
if isfield(OUT,'rows')
    for k=1:numel(OUT.rows)
        q=OUT.rows(k);
        fprintf(fid,['window=%s | t=%.9g | online=%.9g | full=%.9g | ' ...
            'reference=%.9g | replanner=%.9g | residual=%.9g | ' ...
            'front=%.9g | load=%.9g | dep=%.9g | baseCertGeom=%.9g | ' ...
            'sensorTrigger=%.9g | outputState=%.9g | baseRef=%.9g | ' ...
            'flightGuard=%.9g | certGuard=%.9g | activeRef=%.9g | ' ...
            'geometry=%.9g | dominant=%s | dominantMs=%.9g\n'], ...
            q.label,q.t_s,q.online_ms,q.full_ms,q.reference_ms, ...
            q.rt_spike_total_ms,q.reference_minus_replanner_ms, ...
            q.rt_spike_stage_front_ms,q.rt_spike_stage_loadmass_ms, ...
            q.rt_spike_stage_dependency_ms,q.rt_spike_stage_base_cert_geom_ms, ...
            q.rt_spike_stage_sensor_trigger_ms,q.rt_spike_stage_output_state_ms, ...
            q.rt_spike_base_ref_ms,q.rt_spike_flight_guard_ms, ...
            q.rt_spike_cert_guard_ms,q.rt_spike_active_ref_ms, ...
            q.rt_spike_geometry_ms,q.dominant_stage,q.dominant_stage_ms);
    end
end
if isfield(OUT,'error')&&isstruct(OUT.error)&&isfield(OUT.error,'message')
    fprintf(fid,'errorIdentifier=%s\n',OUT.error.identifier);
    fprintf(fid,'errorMessage=%s\n',OUT.error.message);
end
fclose(fid);

OUT.report_path=report;
OUT.mat_path=matp;
save(matp,'OUT','-v7');
assignin('base','AF7R58ZZW_OUT',OUT);

fprintf('RETURN: %s\n',report);
fprintf('RETURN: %s\n',matp);
end


function x=get_profile_ms(P,nm,i)
x=NaN;
if isfield(P,nm)
    v=P.(nm);
    if numel(v)>=i && isfinite(double(v(i)))
        x=1e3*double(v(i));
    end
end
end


function x=get_profile_value(P,nm,i)
x=NaN;
if isfield(P,nm)
    v=P.(nm);
    if numel(v)>=i && isfinite(double(v(i)))
        x=double(v(i));
    end
end
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


function n=count_exact(s,pat)
n=numel(strfind(s,pat));
end


function b=read_bytes(path)
fid=fopen(path,'rb'); assert(fid>=0);
c=onCleanup(@()fclose(fid)); %#ok<NASGU>
b=fread(fid,'*uint8');
end


function write_utf8(path,s)
b=unicode2native(s,'UTF-8');
fid=fopen(path,'wb'); assert(fid>=0);
c=onCleanup(@()fclose(fid)); %#ok<NASGU>
fwrite(fid,uint8(b),'uint8');
end


function restore_source_exact(path,b)
try
    fid=fopen(path,'wb');
    if fid>=0
        fwrite(fid,uint8(b),'uint8');
        fclose(fid);
    end
catch
end
end


function h=file_hash(path)
h=sha_bytes(read_bytes(path));
end


function h=sha_bytes(b)
md=java.security.MessageDigest.getInstance('SHA-256');
md.update(uint8(b));
u=typecast(md.digest(),'uint8');
h=lower(reshape(dec2hex(u,2).',1,[]));
end


function safe_rmpath(p)
try, rmpath(p); catch, end
end
