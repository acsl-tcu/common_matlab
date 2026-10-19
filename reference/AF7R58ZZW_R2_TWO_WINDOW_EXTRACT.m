function OUT = AF7R58ZZW_R2_TWO_WINDOW_EXTRACT()
% AF7R58ZZW-R2
% Read the ALREADY COMPLETED diagnostic run from the current MATLAB workspace.
% No Flight rerun. No source modification. No class clearing.
%
% Only two windows are inspected:
%   1) Flight-entry   : 4.95--5.20 s
%   2) Replan window : 9.80--10.10 s
%
% The previous ZZW run already enabled the built-in light timing and completed
% the simulation. Its failure occurred only while storing heterogeneous result
% structs. This extractor avoids struct-array assignment entirely.

OUT=struct();
OUT.schema='AF7R58ZZW_R2_TWO_WINDOW_EXTRACT_V1';
OUT.production_modified=false;
OUT.flight_rerun=false;
OUT.status='OPEN';
OUT.error=struct();

fprintf('\n====================================================================================================\n');
fprintf(' AF7R58ZZW-R2 | TWO-WINDOW CURRENT-RUN EXTRACT\n');
fprintf('====================================================================================================\n');
fprintf('[READ ONLY] noFlightRerun=1 | noSourceWrite=1 | windows=FlightEntry,ReplanOnly\n');

try
    assert(evalin('base','exist(''AF7R58ZZB_RUNTIME_PROFILE'',''var'')')==1, ...
        '[AF7R58ZZW-R2] AF7R58ZZB_RUNTIME_PROFILE missing from current workspace.');
    assert(evalin('base','exist(''logger'',''var'')')==1, ...
        '[AF7R58ZZW-R2] logger missing from current workspace.');

    P=evalin('base','AF7R58ZZB_RUNTIME_PROFILE');
    L=evalin('base','logger');

    t=double(P.t(:));
    ph=char(P.phase(:));

    % Extract logger reference result states.
    D=member(L,'Data');
    A=member(D,'agent');
    if numel(A)>1, A=A(1); end
    RR=member(member(A,'reference'),'result');
    if ~iscell(RR), RR=num2cell(RR); end
    RR=RR(:);

    tl=member(D,'t');
    if isempty(tl)
        tl=t(1:min(numel(t),numel(RR)));
    else
        tl=double(tl(:));
    end

    N=min(numel(RR),numel(tl));
    RR=RR(1:N);
    tl=tl(1:N);

    names={ ...
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
        st=member(RR{i},'state');
        if isempty(st), continue; end
        for j=1:numel(names)
            [v,ok]=scalar(member(st,names{j}));
            if ok, X(i,j)=v; end
        end
    end

    capCol=find(strcmp(names,'rt_spike_capture_time'),1);
    totCol=find(strcmp(names,'rt_spike_total_ms'),1);
    cap=X(:,capCol);
    tot=X(:,totCol);

    W=[4.95 5.20; 9.80 10.10];
    labels={'FLIGHT_ENTRY','REPLAN_WINDOW'};

    % Fixed template prevents heterogeneous-struct assignment.
    T=struct( ...
        'label','', ...
        'runtime_peak_t_s',NaN, ...
        'runtime_peak_online_ms',NaN, ...
        'runtime_peak_full_ms',NaN, ...
        'runtime_peak_reference_ms',NaN, ...
        'runtime_peak_request_delta',NaN, ...
        'runtime_peak_commit_delta',NaN, ...
        'runtime_peak_gen_critical_ms',NaN, ...
        'capture_found',false, ...
        'capture_t_s',NaN, ...
        'capture_total_ms',NaN, ...
        'front_ms',NaN, ...
        'loadmass_ms',NaN, ...
        'dependency_ms',NaN, ...
        'base_cert_geom_ms',NaN, ...
        'sensor_trigger_ms',NaN, ...
        'output_state_ms',NaN, ...
        'base_ref_ms',NaN, ...
        'flight_guard_ms',NaN, ...
        'cert_guard_ms',NaN, ...
        'active_ref_ms',NaN, ...
        'geometry_ms',NaN, ...
        'state_handle_ms',NaN, ...
        'write_active_output_ms',NaN, ...
        'state_basic_ms',NaN, ...
        'state_trigger_scalar_ms',NaN, ...
        'state_trigger_heavy_ms',NaN, ...
        'state_cert_ms',NaN, ...
        'state_meta_ms',NaN, ...
        'active_count',NaN, ...
        'need_replan',NaN, ...
        'dominant_stage','', ...
        'dominant_stage_ms',NaN);

    OUT.windows=repmat(T,2,1);

    for w=1:2
        q=T;
        q.label=labels{w};

        % Current runtime-profile reference peak in this window.
        idx=find(ph=='f' & ...
            t>=W(w,1)-1e-12 & t<=W(w,2)+1e-12 & ...
            isfinite(double(P.reference_s(:))));
        assert(~isempty(idx), ...
            '[AF7R58ZZW-R2] no Flight runtime row in requested window.');

        [~,jj]=max(double(P.reference_s(idx)));
        ip=idx(jj);

        q.runtime_peak_t_s=t(ip);
        q.runtime_peak_online_ms=1e3*double(P.online_before_plant_s(ip));
        q.runtime_peak_full_ms=1e3*double(P.full_iteration_s(ip));
        q.runtime_peak_reference_ms=1e3*double(P.reference_s(ip));
        q.runtime_peak_request_delta=get_profile(P,'gen_request_delta',ip);
        q.runtime_peak_commit_delta=get_profile(P,'gen_commit_delta',ip);
        q.runtime_peak_gen_critical_ms=get_profile(P,'gen_last_cycle_critical_ms',ip);

        % Built-in light-audit captures only rare slow REPLANNING_SYSTEM cycles.
        ci=find(isfinite(cap) & isfinite(tot) & ...
            cap>=W(w,1)-1e-12 & cap<=W(w,2)+1e-12);

        if ~isempty(ci)
            [~,kc]=max(tot(ci));
            ir=ci(kc);
            q.capture_found=true;
            q.capture_t_s=cap(ir);
            q.capture_total_ms=tot(ir);

            q.front_ms=getx(X,ir,names,'rt_spike_stage_front_ms');
            q.loadmass_ms=getx(X,ir,names,'rt_spike_stage_loadmass_ms');
            q.dependency_ms=getx(X,ir,names,'rt_spike_stage_dependency_ms');
            q.base_cert_geom_ms=getx(X,ir,names,'rt_spike_stage_base_cert_geom_ms');
            q.sensor_trigger_ms=getx(X,ir,names,'rt_spike_stage_sensor_trigger_ms');
            q.output_state_ms=getx(X,ir,names,'rt_spike_stage_output_state_ms');

            q.base_ref_ms=getx(X,ir,names,'rt_spike_base_ref_ms');
            q.flight_guard_ms=getx(X,ir,names,'rt_spike_flight_guard_ms');
            q.cert_guard_ms=getx(X,ir,names,'rt_spike_cert_guard_ms');
            q.active_ref_ms=getx(X,ir,names,'rt_spike_active_ref_ms');
            q.geometry_ms=getx(X,ir,names,'rt_spike_geometry_ms');

            q.state_handle_ms=getx(X,ir,names,'rt_spike_state_handle_ms');
            q.write_active_output_ms=getx(X,ir,names,'rt_spike_write_active_output_ms');
            q.state_basic_ms=getx(X,ir,names,'rt_spike_state_basic_ms');
            q.state_trigger_scalar_ms=getx(X,ir,names,'rt_spike_state_trigger_scalar_ms');
            q.state_trigger_heavy_ms=getx(X,ir,names,'rt_spike_state_trigger_heavy_ms');
            q.state_cert_ms=getx(X,ir,names,'rt_spike_state_cert_ms');
            q.state_meta_ms=getx(X,ir,names,'rt_spike_state_meta_ms');

            q.active_count=getx(X,ir,names,'rt_spike_active_count');
            q.need_replan=getx(X,ir,names,'rt_spike_need_replan');

            vals=[q.front_ms,q.loadmass_ms,q.dependency_ms, ...
                q.base_cert_geom_ms,q.sensor_trigger_ms,q.output_state_ms];
            nms={'FRONT','LOADMASS','DEPENDENCY','BASE_CERT_GEOM', ...
                'SENSOR_TRIGGER','OUTPUT_STATE'};
            vv=vals; vv(~isfinite(vv))=-inf;
            [mx,kx]=max(vv);
            q.dominant_stage=nms{kx};
            q.dominant_stage_ms=mx;
        end

        OUT.windows(w)=q;

        fprintf('\n--------------------------------------------------------------------------------\n');
        fprintf(' %s\n',q.label);
        fprintf('--------------------------------------------------------------------------------\n');
        fprintf(' runtime reference peak: t=%.3f s | reference=%.4f ms | online=%.4f ms | full=%.4f ms\n', ...
            q.runtime_peak_t_s,q.runtime_peak_reference_ms, ...
            q.runtime_peak_online_ms,q.runtime_peak_full_ms);
        fprintf(' request/commit = %.0f / %.0f | genCritical=%.4f ms\n', ...
            q.runtime_peak_request_delta,q.runtime_peak_commit_delta, ...
            q.runtime_peak_gen_critical_ms);

        if q.capture_found
            fprintf(' captured REPLANNING_SYSTEM slow cycle: t=%.3f s | total=%.4f ms\n', ...
                q.capture_t_s,q.capture_total_ms);
            fprintf('   front/load/dependency           = %.4f / %.4f / %.4f ms\n', ...
                q.front_ms,q.loadmass_ms,q.dependency_ms);
            fprintf('   baseCertGeom/sensorTrigger/out  = %.4f / %.4f / %.4f ms\n', ...
                q.base_cert_geom_ms,q.sensor_trigger_ms,q.output_state_ms);
            fprintf('   baseRef/flightGuard/certGuard   = %.4f / %.4f / %.4f ms\n', ...
                q.base_ref_ms,q.flight_guard_ms,q.cert_guard_ms);
            fprintf('   activeRef/geometry              = %.4f / %.4f ms\n', ...
                q.active_ref_ms,q.geometry_ms);
            fprintf('   dominant stage                  = %s | %.4f ms\n', ...
                q.dominant_stage,q.dominant_stage_ms);
            fprintf('   captured need_replan            = %.0f\n',q.need_replan);
        else
            fprintf(' captured REPLANNING_SYSTEM slow cycle: NONE IN THIS WINDOW\n');
        end
    end

    OUT.status='TWO_WINDOW_CURRENT_RUN_EXTRACT_COMPLETE';
    fprintf('\nstatus = %s\n',OUT.status);

catch ME
    OUT.status='EXTRACT_ERROR__NO_SOURCE_CHANGE';
    OUT.error=struct('identifier',ME.identifier,'message',ME.message,'stack',{ME.stack});
    fprintf('\n[EXTRACT ERROR]\n%s | %s\n',ME.identifier,ME.message);
end

assignin('base','AF7R58ZZW_R2_OUT',OUT);
end


function x=get_profile(P,nm,i)
x=NaN;
if isfield(P,nm)
    v=P.(nm);
    if numel(v)>=i && isfinite(double(v(i)))
        x=double(v(i));
    end
end
end


function x=getx(X,i,names,nm)
x=NaN;
j=find(strcmp(names,nm),1);
if ~isempty(j) && size(X,1)>=i && isfinite(X(i,j))
    x=X(i,j);
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
