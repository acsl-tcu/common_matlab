function OUT = AF7R28_BLUE_SPIKE_ATTRIBUTION(runtime_profile,logger)
% AF7R28_BLUE_SPIKE_ATTRIBUTION
% =========================================================================
% Read-only attribution of "online before plant" blue spikes.
%
% Uses the existing REPLANNING_SYSTEM rare-spike snapshots:
%   rt_spike_stage_front_ms
%   rt_spike_stage_loadmass_ms
%   rt_spike_stage_dependency_ms
%   rt_spike_stage_base_cert_geom_ms
%   rt_spike_stage_sensor_trigger_ms
%   rt_spike_stage_output_state_ms
%
% The SENSOR_TRIGGER stage contains raw sensor/trigger/replan request work,
% including generator invocation and its post-callback diagnostic/certificate
% work. The audit aligns those snapshots with generator request edges/status.
% =========================================================================

fprintf('\n');
fprintf('====================================================================================================\n');
fprintf(' AF7R28 | BLUE ONLINE-SPIKE ATTRIBUTION\n');
fprintf('====================================================================================================\n');
fprintf('[AUTHORITY] readOnly=1 | diagnosticOnly=1 | safetyLogicUnchanged=1\n');

t=double(runtime_profile.t(:));
online=1e3*double(runtime_profile.online_before_plant_s(:));
phase=runtime_profile.phase(:);
N=numel(t);

D=logger.Data;
A=D.agent;
if numel(A)>1, A=A(1); end
R=result_cells_local(member_local(member_local(A,'reference'),'result'));
NL=numel(R);

if isfield(D,'t')&&numel(D.t)>=NL
    tl=double(D.t(1:NL)); tl=tl(:);
else
    tl=nan(NL,1);
end

paths={ ...
    'state.generator_request_count', ...
    'state.generator_last_status_code', ...
    'state.bezier_shadow_last_reason_code', ...
    'state.r9_hr_callback_wall_ms', ...
    'state.rt_spike_capture_time', ...
    'state.rt_spike_total_ms', ...
    'state.rt_spike_stage_front_ms', ...
    'state.rt_spike_stage_loadmass_ms', ...
    'state.rt_spike_stage_dependency_ms', ...
    'state.rt_spike_stage_base_cert_geom_ms', ...
    'state.rt_spike_stage_sensor_trigger_ms', ...
    'state.rt_spike_stage_output_state_ms', ...
    'state.rt_spike_base_ref_ms', ...
    'state.rt_spike_flight_guard_ms', ...
    'state.rt_spike_cert_guard_ms', ...
    'state.rt_spike_active_ref_ms', ...
    'state.rt_spike_geometry_ms', ...
    'state.rt_spike_state_cert_ms', ...
    'state.rt_spike_state_meta_ms', ...
    'state.rt_spike_need_replan' ...
    };

V=cell(size(paths));
for i=1:numel(paths), V{i}=series_local(R,paths{i}); end

req=V{1}; st=V{2}; rsn=V{3}; qcb=V{4};
cap=V{5}; rt=V{6};
front=V{7}; mass=V{8}; dep=V{9}; base=V{10}; trig=V{11}; out=V{12};
bref=V{13}; fguard=V{14}; cguard=V{15}; aref=V{16}; geom=V{17};
statecert=V{18}; statemeta=V{19}; need=V{20};

edge=false(NL,1);
prev=0;
for k=1:NL
    if isfinite(req(k))&&req(k)>prev
        edge(k)=true;
        prev=req(k);
    end
end

Aout=repmat(struct('t',NaN,'online_ms',NaN,'request_edge',false, ...
    'status',NaN,'reason',NaN,'callback_qpc_ms',NaN,'spike_total_ms',NaN, ...
    'front_ms',NaN,'loadmass_ms',NaN,'dependency_ms',NaN, ...
    'base_cert_geom_ms',NaN,'sensor_trigger_ms',NaN,'output_state_ms',NaN, ...
    'base_ref_ms',NaN,'flight_guard_ms',NaN,'cert_guard_ms',NaN, ...
    'active_ref_ms',NaN,'geometry_ms',NaN,'state_cert_ms',NaN, ...
    'state_meta_ms',NaN,'need_replan',NaN),1,N);

for i=1:N
    a=Aout(i);
    a.t=t(i); a.online_ms=online(i);
    if all(~isfinite(tl)), Aout(i)=a; continue; end
    [d,j]=min(abs(tl-t(i)));
    if isfinite(d)&&d<=1e-9+16*eps(max(1,abs(t(i))))
        a.request_edge=edge(j);
        a.status=st(j); a.reason=rsn(j); a.callback_qpc_ms=qcb(j);
        if isfinite(cap(j)) && abs(cap(j)-t(i))<=1e-9+16*eps(max(1,abs(t(i))))
            a.spike_total_ms=rt(j);
            a.front_ms=front(j); a.loadmass_ms=mass(j); a.dependency_ms=dep(j);
            a.base_cert_geom_ms=base(j); a.sensor_trigger_ms=trig(j);
            a.output_state_ms=out(j); a.base_ref_ms=bref(j);
            a.flight_guard_ms=fguard(j); a.cert_guard_ms=cguard(j);
            a.active_ref_ms=aref(j); a.geometry_ms=geom(j);
            a.state_cert_ms=statecert(j); a.state_meta_ms=statemeta(j);
            a.need_replan=need(j);
        end
    end
    Aout(i)=a;
end

I=find(phase=='f' & online>=20);
fprintf(' Flight online>=20 ms cycles             = %d\n',numel(I));
fprintf(' #    t[s] online  req stat rsn qpcCB  rtTotal  front mass dep baseGeom sensorTrig outState [ms]\n');
for k=1:numel(I)
    a=Aout(I(k));
    fprintf('%2d %8.3f %7.3f  %d  %4.0f %3.0f %6.3f %8.3f %6.3f %5.3f %5.3f %8.3f %10.3f %8.3f\n', ...
        k,a.t,a.online_ms,a.request_edge,a.status,a.reason,a.callback_qpc_ms, ...
        a.spike_total_ms,a.front_ms,a.loadmass_ms,a.dependency_ms, ...
        a.base_cert_geom_ms,a.sensor_trigger_ms,a.output_state_ms);
end

% Focus on post-collision request spikes, excluding the known 9.5--10.75 burst
J=I(t(I)>10.75 & t(I)<20);
fprintf('\n[POST-10.75 s ONLINE SPIKES]\n');
fprintf(' count                                    = %d\n',numel(J));
if ~isempty(J)
    reqn=sum([Aout(J).request_edge]);
    fprintf(' coincident with generator request edge   = %d / %d\n',reqn,numel(J));
    vals=[Aout(J).sensor_trigger_ms]; vals=vals(isfinite(vals));
    if ~isempty(vals)
        fprintf(' sensor/trigger/replan stage min/med/max  = %.6g / %.6g / %.6g ms\n', ...
            min(vals),median(vals),max(vals));
    end
    vals=[Aout(J).callback_qpc_ms]; vals=vals(isfinite(vals));
    if ~isempty(vals)
        fprintf(' backend callback QPC min/med/max         = %.6g / %.6g / %.6g ms\n', ...
            min(vals),median(vals),max(vals));
    end
end

fprintf('\n[INTERPRETATION GUIDE]\n');
fprintf(' large sensor_trigger_ms + tiny qpcCB     => post-callback replanner/diagnostic work\n');
fprintf(' large base_cert_geom_ms                  => reference/certificate/geometry front path\n');
fprintf(' large output_state_ms                    => result/state write path\n');
fprintf(' request_edge=0                           => not caused by a new generator request\n');

OUT=struct('schema','AF7R28_BLUE_SPIKE_ATTRIBUTION_V1', ...
    'cycles',Aout,'online20_indices',I,'post1075_indices',J);
assignin('base','AF7R28_BLUE_SPIKE_ATTRIBUTION_OUT',OUT);
fprintf('====================================================================================================\n');
end


function x=member_local(S,name)
x=[];
try
    if isempty(S), return; end
    if isstruct(S)
        if isfield(S,name), x=S.(name); end
    elseif isobject(S)
        if isprop(S,name), x=S.(name); end
    end
catch
end
end

function C=result_cells_local(X)
if isempty(X), C={};
elseif iscell(X), C=X(:);
elseif isstruct(X)||isobject(X), C=num2cell(X(:));
else, C={};
end
end

function x=nested_local(S,path)
x=S; P=strsplit(path,'.');
for k=1:numel(P)
    x=member_local(x,P{k});
    if isempty(x), return; end
end
end

function v=series_local(C,path)
v=nan(numel(C),1);
for k=1:numel(C)
    x=nested_local(C{k},path);
    try
        if (isnumeric(x)||islogical(x))&&isscalar(x), v(k)=double(x); end
    catch
    end
end
end
