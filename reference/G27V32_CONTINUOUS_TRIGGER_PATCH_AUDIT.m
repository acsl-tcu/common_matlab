function OUT = G27V32_CONTINUOUS_TRIGGER_PATCH_AUDIT()
% G27V32_CONTINUOUS_TRIGGER_PATCH_AUDIT
% =========================================================================
% PURPOSE
%   Verify that the current REPLANNING_SYSTEM implementation contains the
%   G27-V32 continuous-time future-trigger patch:
%
%   - raw sensor geometry remains unchanged;
%   - NOMINAL active speed certificate is exact and version-bound;
%   - obstacle packet includes dt/2 intersample sweep inflation;
%   - stale mode/version/dt/nominal-func packet reuse fails closed.
%
% This audit does not modify REPLANNING_SYSTEM.m.
%
% RUN AFTER replacing the current REPLANNING_SYSTEM.m:
%
%   clear classes
%   rehash
%   G27V32_CONTINUOUS_TRIGGER_PATCH_AUDIT
% =========================================================================

clc;

src=which('REPLANNING_SYSTEM');
assert(~isempty(src),'[G27-V32 ERROR] REPLANNING_SYSTEM not found.');

txt=fileread(src);

fprintf('\n');
fprintf('====================================================================================================\n');
fprintf(' G27-V32 | CONTINUOUS-TIME FUTURE-TRIGGER PATCH AUDIT\n');
fprintf('====================================================================================================\n');
fprintf('[SOURCE]\n');
fprintf(' path = %s\n',src);

% -------------------------------------------------------------------------
% A. Source markers.
% -------------------------------------------------------------------------
M=struct();

M.active_speed_property=contains(txt,'active_speed_certificate = struct()');
M.sweep_property=contains(txt,'trigger_sweep_inflation  = struct()');
M.nominal_builder=contains(txt,'build_nominal_active_speed_certificate');
M.install_interface=contains(txt,'install_active_speed_certificate');
M.packet_guard=contains(txt,'assert_trigger_packet_certificate_current');
M.delta_payload=contains(txt,'sweep.payload_m');
M.delta_uav=contains(txt,'sweep.uav_m');
M.delta_cable=contains(txt,'sweep.cable_m');
M.packet_version=contains(txt,'pack.reference_version');
M.packet_dt=contains(txt,'pack.sweep_dt');
M.func_signature_guard=contains(txt,'nominal reference function changed');
M.raw_sensor_present=contains(txt,'function [active,min_dist,min_id,dists] = raw_sensor_gate');
M.raw_surface_present=contains(txt,'function d = raw_surface_distance');
M.trigger_mex_present=contains(txt,'g26_future_trigger_kernel_mex');

fprintf('\n[SOURCE MARKERS]\n');
fn=fieldnames(M);
for k=1:numel(fn)
    fprintf(' %-32s = %d\n',fn{k},M.(fn{k}));
end

required={ ...
    'active_speed_property','sweep_property','nominal_builder', ...
    'install_interface','packet_guard','delta_payload','delta_uav', ...
    'delta_cable','packet_version','packet_dt','func_signature_guard', ...
    'raw_sensor_present','raw_surface_present','trigger_mex_present'};

sourcePass=true;
for k=1:numel(required)
    sourcePass=sourcePass && M.(required{k});
end

% -------------------------------------------------------------------------
% B. Verify raw sensor has NOT been replaced by ellipsoid sensor.
% -------------------------------------------------------------------------
sensorEllipsoidMixed=false;

sensorStart=strfind(txt,'function [active,min_dist,min_id,dists] = raw_sensor_gate');
sensorEnd=strfind(txt,'function d = raw_surface_distance');

if ~isempty(sensorStart) && ~isempty(sensorEnd)
    s0=sensorStart(1);
    eCandidates=sensorEnd(sensorEnd>s0);
    if ~isempty(eCandidates)
        block=txt(s0:eCandidates(1)-1);
        sensorEllipsoidMixed=contains(block,'Apay') || ...
            contains(block,'Auav') || contains(block,'Acable') || ...
            contains(block,'outer_ellipsoid');
    end
end

fprintf('\n[RAW SENSOR SEPARATION]\n');
fprintf(' raw sensor uses trigger ellipsoid packet = %d\n',sensorEllipsoidMixed);
fprintf(' expected = 0\n');

% -------------------------------------------------------------------------
% C. Current nominal reference exact speed truth.
% -------------------------------------------------------------------------
REF=g27_resolve_nominal_ref();

fstr=func2str(REF.func);
[xd,t]=g27_func_to_sym(fstr);

assert(numel(xd)>=11,'[G27-V32 ERROR] nominal xd length < 11.');

pL=simplify(xd(1:3));
vL=simplify(xd(5:7));
aL=simplify(xd(9:11));

deriv1=g27_sym_vec_equal(diff(pL,t),vL);
deriv2=g27_sym_vec_equal(diff(vL,t),aL);

speed2=simplify(vL.'*vL);
speedConst=g27_sym_zero(diff(speed2,t));

assert(deriv1 && deriv2 && speedConst, ...
    '[G27-V32 ERROR] nominal exact speed theorem does not close.');

VL=double(simplify(sqrt(speed2)));

aZero=true;
for k=1:numel(aL)
    aZero=aZero && g27_sym_zero(aL(k));
end

assert(aZero, ...
    '[G27-V32 ERROR] nominal aL is nonzero; general qdot theorem required.');

VQ=VL;

% current dt
dt=g27_resolve_dt();

dSensor=VQ*dt;
dL=VL*dt/2;
dQ=VQ*dt/2;
dC=max(VL,VQ)*dt/2;

fprintf('\n[EXACT CURRENT NOMINAL SPEED]\n');
fprintf(' func2str = %s\n',fstr);
fprintf(' d/dt pL == vL = %d\n',deriv1);
fprintf(' d/dt vL == aL = %d\n',deriv2);
fprintf(' aL == 0         = %d\n',aZero);
fprintf(' EXACT V_L = %.15g m/s\n',VL);
fprintf(' EXACT V_Q = %.15g m/s\n',VQ);
fprintf(' dt        = %.15g s\n',dt);
fprintf(' sensor one-period travel = %.15g m\n',dSensor);
fprintf(' trigger delta payload    = %.15g m\n',dL);
fprintf(' trigger delta UAV        = %.15g m\n',dQ);
fprintf(' trigger delta cable      = %.15g m\n',dC);

% -------------------------------------------------------------------------
% D. Optional live REPLANNING_SYSTEM object audit.
% -------------------------------------------------------------------------
[RPL,rplSource]=g27_find_live_replanner();

LIVE=struct();
LIVE.available=~isempty(RPL);
LIVE.source=rplSource;
LIVE.pass=false;

fprintf('\n[LIVE REPLANNING_SYSTEM AUDIT]\n');

if isempty(RPL)
    fprintf(' no live REPLANNING_SYSTEM object currently reachable.\n');
    fprintf(' source patch + nominal theorem can still be audited offline.\n');
else
    fprintf(' source = %s\n',rplSource);
    fprintf(' active mode    = %s\n',char(RPL.active_reference_mode));
    fprintf(' active version = %.0f\n',double(RPL.active_reference_version));

    C=RPL.active_speed_certificate;
    S=RPL.trigger_sweep_inflation;
    P=RPL.obstacle_packet;

    cpass=isstruct(C) && isfield(C,'certified') && logical(C.certified) && ...
        strcmpi(char(C.reference_mode),char(RPL.active_reference_mode)) && ...
        double(C.reference_version)==double(RPL.active_reference_version);

    spass=isstruct(S) && isfield(S,'certified') && logical(S.certified) && ...
        abs(S.payload_m-dL)<=eps(max(1,abs(dL))) && ...
        abs(S.uav_m-dQ)<=eps(max(1,abs(dQ))) && ...
        abs(S.cable_m-dC)<=eps(max(1,abs(dC))) && ...
        double(S.dt)==double(RPL.prediction_dt);

    ppass=isstruct(P) && isfield(P,'sweep_certified') && ...
        logical(P.sweep_certified) && ...
        isfield(P,'reference_version') && ...
        double(P.reference_version)==double(RPL.active_reference_version) && ...
        isfield(P,'sweep_dt') && ...
        double(P.sweep_dt)==double(RPL.prediction_dt) && ...
        abs(P.delta_payload_m-dL)<=eps(max(1,abs(dL))) && ...
        abs(P.delta_uav_m-dQ)<=eps(max(1,abs(dQ))) && ...
        abs(P.delta_cable_m-dC)<=eps(max(1,abs(dC)));

    fprintf(' active speed cert current = %d\n',cpass);
    fprintf(' sweep inflation exact     = %d\n',spass);
    fprintf(' obstacle packet bound     = %d\n',ppass);

    LIVE.pass=cpass&&spass&&ppass;
end

fprintf('\n[THEOREM STATUS]\n');
fprintf(' source patch complete                  = %d\n',sourcePass);
fprintf(' raw sensor / trigger representation split = %d\n',~sensorEllipsoidMixed);
fprintf(' exact nominal continuous V_L,V_Q       = 1\n');
fprintf(' future-trigger dt/2 inflation formula  = %d\n', ...
    M.delta_payload&&M.delta_uav&&M.delta_cable);

fprintf('\n----------------------------------------------------------------------------------------------------\n');
if sourcePass
    fprintf('[G27-V32 PASS] required source-level continuous-trigger patch markers present\n');
else
    fprintf('[G27-V32 FAIL] one or more required source-level patch markers missing\n');
end

if ~sensorEllipsoidMixed
    fprintf('[G27-V32 PASS] raw true-shape sensor remains unchanged\n');
else
    fprintf('[G27-V32 FAIL] sensor/trigger geometry representations were mixed\n');
end

fprintf('[G27-V32 PASS] current nominal continuous-time V_L/V_Q rederived exactly\n');
fprintf('[G27-V32 PASS] exact nominal trigger sweep distances rederived from live dt\n');

if LIVE.available && LIVE.pass
    fprintf('[G27-V32 PASS] live packet is mode/version/dt-bound to the active speed certificate\n');
elseif LIVE.available
    fprintf('[G27-V32 FAIL] live packet/certificate binding is inconsistent\n');
else
    fprintf('[G27-V32 INFO] live runtime packet audit deferred until REPLANNING_SYSTEM is instantiated\n');
end

fprintf('[G27-V32 OPEN] sensor polling delay still contributes V_Q*dt to first-detection timing\n');
fprintf('[G27-V32 OPEN] newly detected obstacle prefix/switch deadline theorem\n');
fprintf('[G27-V32 OPEN] REPAIRED backend must provide its own version-bound V_L/V_Q certificate at commit\n');
fprintf('====================================================================================================\n');

OUT=struct();
OUT.source=src;
OUT.SOURCE_MARKERS=M;
OUT.source_patch_complete=sourcePass;
OUT.raw_sensor_unchanged=~sensorEllipsoidMixed;
OUT.reference_func=fstr;
OUT.VL=VL;
OUT.VQ=VQ;
OUT.dt=dt;
OUT.delta_sensor=dSensor;
OUT.delta_payload=dL;
OUT.delta_uav=dQ;
OUT.delta_cable=dC;
OUT.LIVE=LIVE;
OUT.nominal_continuous_speed_closed=true;
OUT.nominal_trigger_intersample_closed=sourcePass && ~sensorEllipsoidMixed;
OUT.live_packet_binding_closed=LIVE.available && LIVE.pass;

assignin('base','G27V32_OUT',OUT);

end

% =========================================================================
function REF=g27_resolve_nominal_ref()
names={'nominal_ref','agent.reference.timevarying', ...
       'replanner_instance','mov.replanner_ref'};
objs={};
for k=1:numel(names)
    try, x=evalin('base',names{k}); catch, x=[]; end
    if ~isempty(x)&&isa(x,'TIME_VARYING_REFERENCE')
        objs{end+1}=x; %#ok<AGROW>
    end
end
assert(~isempty(objs),'[G27-V32 ERROR] nominal TIME_VARYING_REFERENCE not found.');
REF=objs{1};
for k=2:numel(objs)
    assert(isequal(REF,objs{k}), ...
        '[G27-V32 ERROR] nominal reference aliases no longer agree.');
end
end

function [xd,t]=g27_func_to_sym(fstr)
assert(startsWith(strtrim(fstr),'@(t)'), ...
    '[G27-V32 ERROR] nominal func is not @(t)...');
body=strtrim(extractAfter(string(strtrim(fstr)),strlength('@(t)')));
body=strrep(body,'.^','^');
body=strrep(body,'.*','*');
body=strrep(body,'./','/');
syms t real
xd=str2sym(char(body));
xd=xd(:);
end

function tf=g27_sym_vec_equal(a,b)
r=simplify(a-b);
tf=true;
for k=1:numel(r)
    if ~g27_sym_zero(r(k))
        tf=false;
        return;
    end
end
end

function tf=g27_sym_zero(x)
tf=isAlways(simplify(x)==0,'Unknown','false');
end

function dt=g27_resolve_dt()
cands={'agent.plant.dt','dt','time.dt'};
for k=1:numel(cands)
    try, x=evalin('base',cands{k}); catch, x=[]; end
    if isnumeric(x)&&isscalar(x)&&isfinite(x)&&x>0
        dt=double(x);
        return;
    end
end
error('[G27-V32 ERROR] current dt unavailable.');
end

function [R,source]=g27_find_live_replanner()
R=[];
source='';
W=evalin('base','whos');
C={};
P={};
for k=1:numel(W)
    n=W(k).name;
    if startsWith(n,'G27V')||startsWith(n,'G27_'), continue; end
    try, x=evalin('base',n); catch, continue; end
    [cc,pp]=g27_scan(x,['BASE:' n],0,4);
    C=[C,cc]; %#ok<AGROW>
    P=[P,pp]; %#ok<AGROW>
end
if isempty(C), return; end

keep=true(1,numel(C));
for i=1:numel(C)
    if ~keep(i), continue; end
    for j=i+1:numel(C)
        if keep(j)&&isequal(C{i},C{j})
            keep(j)=false;
            P{i}=[P{i} ' | ' P{j}];
        end
    end
end
C=C(keep);
P=P(keep);
if numel(C)==1
    R=C{1};
    source=P{1};
end
end

function [C,P]=g27_scan(x,prefix,depth,maxDepth)
C={}; P={};
if depth>maxDepth||isempty(x), return; end
if isobject(x)&&isa(x,'REPLANNING_SYSTEM')
    C={x}; P={prefix}; return;
end
if isnumeric(x)||islogical(x)||ischar(x)||isstring(x)||iscell(x), return; end
if isstruct(x)
    if numel(x)~=1, return; end
    fn=fieldnames(x);
    for k=1:numel(fn)
        f=fn{k};
        if any(strcmp(f,{'logger','log','history','data','profiling'})), continue; end
        try, y=x.(f); catch, continue; end
        [cc,pp]=g27_scan(y,[prefix '.' f],depth+1,maxDepth);
        C=[C,cc]; P=[P,pp]; %#ok<AGROW>
    end
elseif isobject(x)
    if numel(x)~=1, return; end
    try, pn=properties(x); catch, pn={}; end
    for k=1:numel(pn)
        f=pn{k};
        if any(strcmp(f,{'logger','log','history','data','profiling', ...
                         'nominal_table_pL','nominal_table_q'})), continue; end
        try, y=x.(f); catch, continue; end
        [cc,pp]=g27_scan(y,[prefix '.' f],depth+1,maxDepth);
        C=[C,cc]; P=[P,pp]; %#ok<AGROW>
    end
end
end
