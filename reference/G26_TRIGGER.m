function varargout = G26_TRIGGER(action,varargin)
% G26_TRIGGER
%
% Single-file trigger module for integration into the main MATLAB system.
%
% This ONE .m file contains:
%   - external parameter template
%   - initialization
%   - self-generated MEX source / compilation
%   - obstacle precomputation
%   - one-cycle trigger query
%   - MATLAB fallback kernel
%   - research logger
%   - MAT/CSV export
%
% -------------------------------------------------------------------------
% QUICK USAGE
% -------------------------------------------------------------------------
%
% 1) PARAMETERS
%
%   P = G26_TRIGGER('param');
%
%   % IMPORTANT: overwrite from the actual main system
%   P.L_cable_m         = MAIN_PARAM.L;
%   P.R_payload_m       = MAIN_PARAM.R_payload;
%   P.R_uav_m           = MAIN_PARAM.R_uav;
%   P.R_cable_m         = MAIN_PARAM.R_cable;
%   P.geometry_margin_m = MAIN_PARAM.trigger_margin;
%
%   % policy parameters -- externally changeable
%   P.sensor_range_m    = 20.0;
%   P.horizon_s         = 4.0;      % provisional default
%   P.prediction_dt_s   = 0.025;
%   P.control_dt_s      = MAIN_PARAM.dt;
%
% 2) INITIALIZE ONCE
%
%   [RUNTIME,P] = G26_TRIGGER('init',P);
%   LOGGER = G26_TRIGGER('log_init',P,RUNTIME);
%
% 3) WHEN THE KNOWN OBSTACLE SET CHANGES
%
%   [OBS_PACKET,OBS_TIMING] = G26_TRIGGER('prepare_obstacles',known_obstacles,P);
%
% 4) EVERY CONTROL CYCLE
%
%   % Build the ORIGINAL future reference packet.
%   %
%   % pL_ref(:,j) = payload position at:
%   %   t_now + (j-1)*P.prediction_dt_s
%   %
%   % q_ref(:,j) = cable direction UAV -> load.
%   %
%   % Prefer q_d actually used by the controller.
%   % If unavailable:
%   %   r = ddot_pL_ref(:,j) + g*[0;0;1];
%   %   q_ref(:,j) = -r/norm(r);
%
%   SAMPLE = G26_TRIGGER('step', ...
%       t_now,pL_ref,q_ref,OBS_PACKET,P,RUNTIME);
%
%   LOGGER = G26_TRIGGER('log_append',LOGGER,SAMPLE);
%
%   if SAMPLE.triggered
%       % FIRST integration stage: LOG ONLY.
%       % Do not modify the reference yet.
%   end
%
% 5) END OF RUN
%
%   FILES = G26_TRIGGER('log_save',LOGGER,'experiment_001');
%
% -------------------------------------------------------------------------
% REFERENCE / GEOMETRY CONTRACT
% -------------------------------------------------------------------------
%
% q convention:
%   q = UAV -> load
%
% cable kinematics:
%   pL - pQ = L*q
%   pQ = pL - L*q
%
% component codes:
%   1 = PAYLOAD
%   2 = UAV
%   3 = CABLE
%
% -------------------------------------------------------------------------
% TIMING
% -------------------------------------------------------------------------
%
% SAMPLE.timing.kernel_ms
%   MEX/MATLAB collision-trigger kernel only.
%
% SAMPLE.timing.wrapper_total_ms
%   Complete G26_TRIGGER('step',...) call.
%
% OBS_TIMING.obstacle_prepare_ms
%   Obstacle conversion/precomputation.
%   Measure separately because this should run only when the known obstacle
%   set changes.
%
% FINAL E2E TIMING:
% Place one tic/toc around the ACTUAL integrated path:
%
%   sensor
%   -> raw obstacle extraction / update
%   -> original reference evaluation pL,q
%   -> obstacle packet preparation if changed
%   -> G26_TRIGGER('step',...)
%
% Do NOT add independently measured p50 values and call that WCET.
%
% -------------------------------------------------------------------------
% CURRENT VALIDATED RAW OBSTACLE TYPE
% -------------------------------------------------------------------------
%
% finite cylinder with arbitrary R_obs.
%
% Required obstacle fields:
%   type       = 'cylinder'
%   p_center   [3x1]
%   raw_param  [radius height]
%   R_obs      [3x3] proper rotation matrix
%   d_margin   scalar >= 0
%   id         finite scalar
%
% An empty obstacle set is valid.
%
% -------------------------------------------------------------------------
% CURRENT AUTHORITY
% -------------------------------------------------------------------------
%
% Integration candidate:
%   YES, after V226 pre-integration tests.
%
% Not yet claimed:
%   - continuous-time no-miss trigger proof
%   - C6 repair
%   - final system safety certificate
%   - production support for non-cylinder raw obstacle types
%
% -------------------------------------------------------------------------

assert(nargin>=1,'G26TRIGGER:Usage','First input must be an action string.');

switch lower(action)

    case 'param'
        varargout{1} = local_param_template();

    case 'init'
        P = varargin{1};
        [RUNTIME,P] = local_init(P);
        varargout{1} = RUNTIME;
        if nargout>=2
            varargout{2} = P;
        end

    case 'prepare_obstacles'
        OBS = varargin{1};
        P   = varargin{2};
        [PACK,TIMING] = local_prepare_obstacles(OBS,P);
        varargout{1} = PACK;
        if nargout>=2
            varargout{2} = TIMING;
        end

    case 'step'
        t_now_s = varargin{1};
        pL_ref  = varargin{2};
        q_ref   = varargin{3};
        PACK    = varargin{4};
        P       = varargin{5};
        RUNTIME = varargin{6};

        varargout{1} = local_step(t_now_s,pL_ref,q_ref,PACK,P,RUNTIME);

    case 'log_init'
        P       = varargin{1};
        RUNTIME = varargin{2};
        varargout{1} = local_log_init(P,RUNTIME);

    case 'log_append'
        LOGGER = varargin{1};
        SAMPLE = varargin{2};
        varargout{1} = local_log_append(LOGGER,SAMPLE);

    case 'log_save'
        LOGGER = varargin{1};
        prefix = varargin{2};
        varargout{1} = local_log_save(LOGGER,prefix);

    case 'component_name'
        varargout{1} = local_component_name(varargin{1});

    otherwise
        error('G26TRIGGER:Action','Unknown action: %s',action);
end

end

%% =========================================================================
% PARAMETER TEMPLATE
% =========================================================================
function P = local_param_template()

P = struct();

% Trigger policy -- provisional defaults, externally changeable.
P.sensor_range_m        = 20.0;
P.horizon_s             = 4.0;
P.prediction_dt_s       = 0.025;
P.control_dt_s          = 0.025;

% Physical geometry -- REPLACE from the actual main system.
P.L_cable_m             = 1.0;
P.R_payload_m           = 0.25;
P.R_uav_m               = 0.25;
P.R_cable_m             = 0.15;
P.geometry_margin_m     = 0.15;

% Input / diagnostic policy.
P.q_unit_tolerance      = 1e-9;

% false is useful close to the terminal end of an existing reference.
% The output always reports effective_horizon_s and full_horizon_covered.
P.require_full_horizon  = false;

% Runtime.
P.use_mex               = true;
P.force_mex_rebuild     = false;
P.runtime_validate      = true;

% Logger.
P.logger_initial_capacity = 20000;

% Research metadata stored once in MAT header.
P.reference_source_label = 'MAIN_TIME_VARYING_REFERENCE';
P.q_source_label         = 'MAIN_QD_OR_FLATNESS_DERIVED';
P.experiment_note        = '';

end

%% =========================================================================
% INITIALIZATION / SELF-BUILD MEX
% =========================================================================
function [RUNTIME,P] = local_init(P)

P = local_validate_param(P);

RUNTIME = struct();
RUNTIME.version = 'G26-TRIGGER-SINGLEFILE-V1';
RUNTIME.mex_name = 'g26_trigger_packet_kernel_mex';
RUNTIME.mex_available = false;
RUNTIME.cpp_created = false;
RUNTIME.compile_requested = false;
RUNTIME.cpp_path = '';
RUNTIME.mex_path = '';
RUNTIME.message = '';

root = fileparts(mfilename('fullpath'));
RUNTIME.cpp_path = fullfile(root,[RUNTIME.mex_name '.cpp']);
RUNTIME.mex_path = fullfile(root,[RUNTIME.mex_name '.' mexext]);

if ~P.use_mex
    RUNTIME.message = 'MEX disabled by P.use_mex.';
    return;
end

if exist(RUNTIME.cpp_path,'file') ~= 2
    local_write_mex_source(RUNTIME.cpp_path);
    RUNTIME.cpp_created = true;
end

compileNeeded = P.force_mex_rebuild || exist(RUNTIME.mex_path,'file') ~= 3;

if ~compileNeeded && exist(RUNTIME.cpp_path,'file')==2 && exist(RUNTIME.mex_path,'file')==3
    ds = dir(RUNTIME.cpp_path);
    dm = dir(RUNTIME.mex_path);

    if ~isempty(ds) && ~isempty(dm)
        compileNeeded = ds.datenum > dm.datenum;
    end
end

if compileNeeded
    RUNTIME.compile_requested = true;

    try
        old = pwd;
        cleanup = onCleanup(@()cd(old)); %#ok<NASGU>
        cd(root);

        mex('-R2018a',RUNTIME.cpp_path,'-output',RUNTIME.mex_name);
        rehash;

    catch ME
        RUNTIME.message = ['MEX build failed; MATLAB fallback remains available. ' ME.message];
        RUNTIME.mex_available = false;
        return;
    end
end

RUNTIME.mex_available = ...
    exist(RUNTIME.mex_path,'file')==3 || exist(RUNTIME.mex_name,'file')==3;

if RUNTIME.mex_available
    if RUNTIME.cpp_created
        RUNTIME.message = 'C++ source generated and MEX compiled successfully.';
    elseif RUNTIME.compile_requested
        RUNTIME.message = 'MEX recompiled successfully.';
    else
        RUNTIME.message = 'Existing MEX reused.';
    end
else
    RUNTIME.message = 'MEX unavailable; MATLAB fallback will be used.';
end

end

function P = local_validate_param(P)

required = { ...
    'sensor_range_m','horizon_s','prediction_dt_s','control_dt_s', ...
    'L_cable_m','R_payload_m','R_uav_m','R_cable_m','geometry_margin_m', ...
    'q_unit_tolerance','require_full_horizon','use_mex','force_mex_rebuild', ...
    'runtime_validate','logger_initial_capacity', ...
    'reference_source_label','q_source_label','experiment_note'};

for k=1:numel(required)
    assert(isfield(P,required{k}), ...
        'G26TRIGGER:Param','Missing parameter field: %s',required{k});
end

positiveFields = { ...
    'sensor_range_m','horizon_s','prediction_dt_s','control_dt_s', ...
    'L_cable_m','R_payload_m','R_uav_m','R_cable_m','q_unit_tolerance'};

for k=1:numel(positiveFields)
    v = P.(positiveFields{k});

    assert(isscalar(v) && isfinite(v) && v>0, ...
        'G26TRIGGER:Param','%s must be positive finite.',positiveFields{k});
end

assert(isscalar(P.geometry_margin_m) && ...
    isfinite(P.geometry_margin_m) && P.geometry_margin_m>=0);

assert(isscalar(P.logger_initial_capacity) && ...
    P.logger_initial_capacity>=1 && ...
    floor(P.logger_initial_capacity)==P.logger_initial_capacity);

end

%% =========================================================================
% OBSTACLE PREPARATION
% =========================================================================
function [PACK,TIMING] = local_prepare_obstacles(OBS,P)

tTotal = tic;

if isempty(OBS)
    M = 0;
elseif isstruct(OBS)
    M = numel(OBS);
else
    error('G26TRIGGER:Obstacle','OBS must be a struct array or empty.');
end

PACK = struct();
PACK.count = M;
PACK.C = zeros(3,M);
PACK.raw_r = zeros(1,M);
PACK.raw_h = zeros(1,M);
PACK.R = zeros(9,M);
PACK.Apay = zeros(3,M);
PACK.Auav = zeros(3,M);
PACK.Acable = zeros(3,M);
PACK.ids = zeros(1,M);

for i=1:M

    local_validate_obstacle(OBS(i),i);

    r  = OBS(i).raw_param(1);
    hh = OBS(i).raw_param(2)/2;

    Dp = OBS(i).d_margin + P.geometry_margin_m + P.R_payload_m;
    Dq = OBS(i).d_margin + P.geometry_margin_m + P.R_uav_m;
    Dc = OBS(i).d_margin + P.geometry_margin_m + P.R_cable_m;

    PACK.C(:,i) = OBS(i).p_center(:);
    PACK.raw_r(i) = r;
    PACK.raw_h(i) = OBS(i).raw_param(2);
    PACK.R(:,i) = OBS(i).R_obs(:);

    % Conservative enclosing ellipsoid of expanded finite cylinder.
    PACK.Apay(:,i) = [ ...
        sqrt(3/2)*(r+Dp); ...
        sqrt(3/2)*(r+Dp); ...
        sqrt(3)*(hh+Dp)];

    PACK.Auav(:,i) = [ ...
        sqrt(3/2)*(r+Dq); ...
        sqrt(3/2)*(r+Dq); ...
        sqrt(3)*(hh+Dq)];

    PACK.Acable(:,i) = [ ...
        sqrt(3/2)*(r+Dc); ...
        sqrt(3/2)*(r+Dc); ...
        sqrt(3)*(hh+Dc)];

    PACK.ids(i) = OBS(i).id;
end

PACK.source_type = 'RAW_CYLINDER_TO_COMPONENT_ELLIPSOIDS';
PACK.geometry_margin_m = P.geometry_margin_m;

TIMING = struct();
TIMING.obstacle_prepare_ms = 1e3*toc(tTotal);

end

function local_validate_obstacle(O,i)

required = {'type','p_center','raw_param','R_obs','d_margin','id'};

for k=1:numel(required)
    assert(isfield(O,required{k}), ...
        'G26TRIGGER:Obstacle','Obstacle %d missing %s.',i,required{k});
end

assert(strcmpi(O.type,'cylinder'), ...
    'G26TRIGGER:ObstacleType', ...
    ['Current validated production base supports cylinder only. ' ...
     'Obstacle %d is %s.'],i,O.type);

assert(numel(O.p_center)==3 && all(isfinite(O.p_center(:))));
assert(numel(O.raw_param)==2 && ...
    all(isfinite(O.raw_param(:))) && all(O.raw_param(:)>0));

assert(isequal(size(O.R_obs),[3 3]) && all(isfinite(O.R_obs(:))));

assert(norm(O.R_obs'*O.R_obs-eye(3),'fro')<=1e-9 && ...
    abs(det(O.R_obs)-1)<=1e-9, ...
    'G26TRIGGER:Rotation', ...
    'Obstacle %d R_obs is not a proper rotation matrix.',i);

assert(isscalar(O.d_margin) && isfinite(O.d_margin) && O.d_margin>=0);
assert(isscalar(O.id) && isfinite(O.id));

end

%% =========================================================================
% ONE CONTROL-CYCLE STEP
% =========================================================================
function SAMPLE = local_step(t_now_s,pL_ref,q_ref,PACK,P,RUNTIME)

tWrapper = tic;

if P.runtime_validate
    local_validate_step_inputs(t_now_s,pL_ref,q_ref,PACK,P);
end

N = size(pL_ref,2);

packet_horizon_s = (N-1)*P.prediction_dt_s;
effective_horizon_s = min(P.horizon_s,packet_horizon_s);

full_horizon_covered = ...
    packet_horizon_s + 10*eps(max(1,P.horizon_s)) >= P.horizon_s;

if P.require_full_horizon
    assert(full_horizon_covered, ...
        'G26TRIGGER:ShortPacket', ...
        'Reference packet horizon %.6f s is shorter than requested %.6f s.', ...
        packet_horizon_s,P.horizon_s);
end

pL_now = pL_ref(:,1);
q_now  = q_ref(:,1);
pQ_now = pL_now - P.L_cable_m*q_now;

tKernel = tic;

if P.use_mex && RUNTIME.mex_available

    fun = str2func(RUNTIME.mex_name);

    [triggered,jhit,obstacle_id,component,visible_count] = fun( ...
        1, ...
        P.horizon_s, ...
        P.prediction_dt_s, ...
        P.sensor_range_m, ...
        P.L_cable_m, ...
        pL_ref, ...
        q_ref, ...
        PACK.C, ...
        PACK.raw_r, ...
        PACK.raw_h, ...
        PACK.R, ...
        PACK.Apay, ...
        PACK.Auav, ...
        PACK.Acable, ...
        PACK.ids);

    backend = 'MEX';

else

    [triggered,jhit,obstacle_id,component,visible_count] = ...
        local_query_matlab(pL_ref,q_ref,PACK,P);

    backend = 'MATLAB';
end

kernel_ms = 1e3*toc(tKernel);

SAMPLE = struct();

SAMPLE.t_now_s = t_now_s;

SAMPLE.triggered = logical(triggered);
SAMPLE.ttc_s = NaN;
SAMPLE.collision_time_s = NaN;
SAMPLE.collision_index = NaN;
SAMPLE.obstacle_id = NaN;
SAMPLE.component_code = NaN;
SAMPLE.component = 'NONE';

SAMPLE.visible_obstacle_count = visible_count;
SAMPLE.obstacle_packet_count = PACK.count;

SAMPLE.reference_point_count = N;
SAMPLE.packet_horizon_s = packet_horizon_s;
SAMPLE.effective_horizon_s = effective_horizon_s;
SAMPLE.full_horizon_covered = full_horizon_covered;

SAMPLE.pL_now = pL_now;
SAMPLE.q_now = q_now;
SAMPLE.pQ_now = pQ_now;

SAMPLE.pL_collision = [NaN;NaN;NaN];
SAMPLE.q_collision  = [NaN;NaN;NaN];
SAMPLE.pQ_collision = [NaN;NaN;NaN];

SAMPLE.backend = backend;

if triggered

    jhit = round(jhit);

    assert(jhit>=1 && jhit<=N, ...
        'G26TRIGGER:Kernel','Kernel returned invalid collision index.');

    SAMPLE.collision_index = jhit;
    SAMPLE.ttc_s = (jhit-1)*P.prediction_dt_s;
    SAMPLE.collision_time_s = t_now_s + SAMPLE.ttc_s;

    SAMPLE.obstacle_id = obstacle_id;
    SAMPLE.component_code = component;
    SAMPLE.component = local_component_name(component);

    SAMPLE.pL_collision = pL_ref(:,jhit);
    SAMPLE.q_collision  = q_ref(:,jhit);
    SAMPLE.pQ_collision = ...
        SAMPLE.pL_collision - P.L_cable_m*SAMPLE.q_collision;
end

SAMPLE.timing = struct();
SAMPLE.timing.kernel_ms = kernel_ms;
SAMPLE.timing.wrapper_total_ms = 1e3*toc(tWrapper);

end

function local_validate_step_inputs(t_now_s,pL,q,PACK,P)

assert(isscalar(t_now_s) && isfinite(t_now_s));

assert(isnumeric(pL) && size(pL,1)==3 && ...
    ~isempty(pL) && all(isfinite(pL(:))), ...
    'G26TRIGGER:Input','pL_ref must be finite 3xN.');

assert(isnumeric(q) && size(q,1)==3 && ...
    size(q,2)==size(pL,2) && all(isfinite(q(:))), ...
    'G26TRIGGER:Input','q_ref must be finite 3xN and match pL_ref.');

qn = vecnorm(q,2,1);

assert(max(abs(qn-1)) <= P.q_unit_tolerance, ...
    'G26TRIGGER:Input','q_ref violates unit-vector tolerance.');

required = {'count','C','raw_r','raw_h','R','Apay','Auav','Acable','ids'};

for k=1:numel(required)
    assert(isfield(PACK,required{k}), ...
        'G26TRIGGER:Packet','Obstacle packet missing %s.',required{k});
end

M = PACK.count;

assert(size(PACK.C,1)==3 && size(PACK.C,2)==M);
assert(numel(PACK.raw_r)==M && numel(PACK.raw_h)==M);
assert(isequal(size(PACK.R),[9 M]));
assert(isequal(size(PACK.Apay),[3 M]));
assert(isequal(size(PACK.Auav),[3 M]));
assert(isequal(size(PACK.Acable),[3 M]));
assert(numel(PACK.ids)==M);

end

%% =========================================================================
% MATLAB FALLBACK QUERY
% =========================================================================
function [found,jhit,oid,comp,visibleCount] = ...
    local_query_matlab(pL,q,PACK,P)

N = size(pL,2);
M = PACK.count;

pQ0 = pL(:,1) - P.L_cable_m*q(:,1);

active = false(1,M);

for i=1:M

    active(i) = local_point_cylinder_distance( ...
        pQ0, ...
        PACK.C(:,i), ...
        PACK.raw_r(i), ...
        PACK.raw_h(i), ...
        reshape(PACK.R(:,i),3,3)) ...
        <= P.sensor_range_m;
end

visibleCount = sum(active);

found = false;
jhit = NaN;
oid = NaN;
comp = NaN;

if ~any(active)
    return;
end

nH = floor(P.horizon_s/P.prediction_dt_s + 1e-12);
i1 = min(N,1+nH);

for j=1:i1

    pLj = pL(:,j);
    pQj = pLj - P.L_cable_m*q(:,j);

    best = inf;
    bestComp = NaN;
    bestOid = NaN;

    for i=1:M

        if ~active(i)
            continue;
        end

        c = PACK.C(:,i);
        R = reshape(PACK.R(:,i),3,3);

        sp = local_point_level( ...
            pLj,c,R,PACK.Apay(:,i)) - 1;

        if sp<best
            best=sp;
            bestComp=1;
            bestOid=PACK.ids(i);
        end

        sq = local_point_level( ...
            pQj,c,R,PACK.Auav(:,i)) - 1;

        if sq<best
            best=sq;
            bestComp=2;
            bestOid=PACK.ids(i);
        end

        sc = local_segment_level( ...
            pLj,pQj,c,R,PACK.Acable(:,i)) - 1;

        if sc<best
            best=sc;
            bestComp=3;
            bestOid=PACK.ids(i);
        end
    end

    if best<=0
        found=true;
        jhit=j;
        oid=bestOid;
        comp=bestComp;
        return;
    end
end

end

%% =========================================================================
% GEOMETRY
% =========================================================================
function d = local_point_cylinder_distance(p,c,r,h,R)

z = R'*(p-c);

dr = max(hypot(z(1),z(2))-r,0);
dz = max(abs(z(3))-h/2,0);

d = hypot(dr,dz);

end

function v = local_point_level(p,c,R,a)

z = (R'*(p-c))./a;
v = norm(z);

end

function v = local_segment_level(p0,p1,c,R,a)

z0 = (R'*(p0-c))./a;
z1 = (R'*(p1-c))./a;

d = z1-z0;
den = dot(d,d);

if den<=eps
    v = norm(z0);
    return;
end

u = -dot(z0,d)/den;
u = min(1,max(0,u));

v = norm(z0+u*d);

end

function c = local_component_name(code)

if code==1
    c='PAYLOAD';
elseif code==2
    c='UAV';
elseif code==3
    c='CABLE';
else
    c='NONE';
end

end

%% =========================================================================
% LOGGER
% =========================================================================
function L = local_log_init(P,RUNTIME)

cap = P.logger_initial_capacity;

L = struct();

L.version = 'G26-TRIGGER-LOGGER-V1';
L.param = P;
L.runtime_header = RUNTIME;

L.n = 0;
L.capacity = cap;

L.seq = zeros(cap,1);
L.t_now_s = nan(cap,1);

L.triggered = false(cap,1);
L.ttc_s = nan(cap,1);
L.collision_time_s = nan(cap,1);
L.obstacle_id = nan(cap,1);
L.component_code = nan(cap,1);

L.visible_obstacle_count = zeros(cap,1);
L.obstacle_packet_count = zeros(cap,1);

L.reference_point_count = zeros(cap,1);
L.packet_horizon_s = nan(cap,1);
L.effective_horizon_s = nan(cap,1);
L.full_horizon_covered = false(cap,1);

L.kernel_ms = nan(cap,1);
L.wrapper_total_ms = nan(cap,1);

L.pL_now = nan(cap,3);
L.q_now  = nan(cap,3);
L.pQ_now = nan(cap,3);

L.pL_collision = nan(cap,3);
L.q_collision  = nan(cap,3);
L.pQ_collision = nan(cap,3);

% 1=MEX, 2=MATLAB
L.backend_code = zeros(cap,1);

end

function L = local_log_append(L,S)

if L.n>=L.capacity
    L = local_log_grow(L);
end

i = L.n+1;
L.n = i;

L.seq(i) = i;
L.t_now_s(i) = S.t_now_s;

L.triggered(i) = S.triggered;
L.ttc_s(i) = S.ttc_s;
L.collision_time_s(i) = S.collision_time_s;
L.obstacle_id(i) = S.obstacle_id;
L.component_code(i) = S.component_code;

L.visible_obstacle_count(i) = S.visible_obstacle_count;
L.obstacle_packet_count(i) = S.obstacle_packet_count;

L.reference_point_count(i) = S.reference_point_count;
L.packet_horizon_s(i) = S.packet_horizon_s;
L.effective_horizon_s(i) = S.effective_horizon_s;
L.full_horizon_covered(i) = S.full_horizon_covered;

L.kernel_ms(i) = S.timing.kernel_ms;
L.wrapper_total_ms(i) = S.timing.wrapper_total_ms;

L.pL_now(i,:) = S.pL_now(:).';
L.q_now(i,:)  = S.q_now(:).';
L.pQ_now(i,:) = S.pQ_now(:).';

L.pL_collision(i,:) = S.pL_collision(:).';
L.q_collision(i,:)  = S.q_collision(:).';
L.pQ_collision(i,:) = S.pQ_collision(:).';

if strcmpi(S.backend,'MEX')
    L.backend_code(i)=1;
else
    L.backend_code(i)=2;
end

end

function L = local_log_grow(L)

old = L.capacity;
new = max(old+1,2*old);

names = fieldnames(L);

for k=1:numel(names)

    name = names{k};
    x = L.(name);

    if isnumeric(x) || islogical(x)

        if size(x,1)==old

            add = new-old;

            if islogical(x)
                fill = false(add,size(x,2));
            else
                fill = nan(add,size(x,2));

                if any(strcmp(name,{ ...
                        'seq', ...
                        'visible_obstacle_count', ...
                        'obstacle_packet_count', ...
                        'reference_point_count', ...
                        'backend_code'}))
                    fill = zeros(add,size(x,2));
                end
            end

            L.(name) = [x;fill];
        end
    end
end

L.capacity = new;

end

function files = local_log_save(L,prefix)

n = L.n;

D = struct();

D.version = L.version;
D.param = L.param;
D.runtime_header = L.runtime_header;
D.n = n;

fields = { ...
    'seq', ...
    't_now_s', ...
    'triggered', ...
    'ttc_s', ...
    'collision_time_s', ...
    'obstacle_id', ...
    'component_code', ...
    'visible_obstacle_count', ...
    'obstacle_packet_count', ...
    'reference_point_count', ...
    'packet_horizon_s', ...
    'effective_horizon_s', ...
    'full_horizon_covered', ...
    'kernel_ms', ...
    'wrapper_total_ms', ...
    'pL_now', ...
    'q_now', ...
    'pQ_now', ...
    'pL_collision', ...
    'q_collision', ...
    'pQ_collision', ...
    'backend_code'};

for k=1:numel(fields)

    name = fields{k};
    x = L.(name);
    D.(name) = x(1:n,:);
end

matFile = [prefix '_trigger_log.mat'];
csvFile = [prefix '_trigger_log.csv'];

save(matFile,'D');

T = table( ...
    D.seq, ...
    D.t_now_s, ...
    D.triggered, ...
    D.ttc_s, ...
    D.collision_time_s, ...
    D.obstacle_id, ...
    D.component_code, ...
    D.visible_obstacle_count, ...
    D.obstacle_packet_count, ...
    D.reference_point_count, ...
    D.packet_horizon_s, ...
    D.effective_horizon_s, ...
    D.full_horizon_covered, ...
    D.kernel_ms, ...
    D.wrapper_total_ms, ...
    D.backend_code, ...
    D.pL_now(:,1), ...
    D.pL_now(:,2), ...
    D.pL_now(:,3), ...
    D.q_now(:,1), ...
    D.q_now(:,2), ...
    D.q_now(:,3), ...
    D.pQ_now(:,1), ...
    D.pQ_now(:,2), ...
    D.pQ_now(:,3), ...
    D.pL_collision(:,1), ...
    D.pL_collision(:,2), ...
    D.pL_collision(:,3), ...
    D.q_collision(:,1), ...
    D.q_collision(:,2), ...
    D.q_collision(:,3), ...
    D.pQ_collision(:,1), ...
    D.pQ_collision(:,2), ...
    D.pQ_collision(:,3), ...
    'VariableNames',{ ...
    'seq', ...
    't_now_s', ...
    'triggered', ...
    'ttc_s', ...
    'collision_time_s', ...
    'obstacle_id', ...
    'component_code', ...
    'visible_obstacle_count', ...
    'obstacle_packet_count', ...
    'reference_point_count', ...
    'packet_horizon_s', ...
    'effective_horizon_s', ...
    'full_horizon_covered', ...
    'kernel_ms', ...
    'wrapper_total_ms', ...
    'backend_code', ...
    'pL_now_x', ...
    'pL_now_y', ...
    'pL_now_z', ...
    'q_now_x', ...
    'q_now_y', ...
    'q_now_z', ...
    'pQ_now_x', ...
    'pQ_now_y', ...
    'pQ_now_z', ...
    'pL_col_x', ...
    'pL_col_y', ...
    'pL_col_z', ...
    'q_col_x', ...
    'q_col_y', ...
    'q_col_z', ...
    'pQ_col_x', ...
    'pQ_col_y', ...
    'pQ_col_z'});

writetable(T,csvFile);

files = struct();
files.mat = matFile;
files.csv = csvFile;

end

%% =========================================================================
% MEX SOURCE GENERATION
% =========================================================================
function local_write_mex_source(path)

L = local_mex_source_lines();

fid = fopen(path,'w');

assert(fid>=0, ...
    'G26TRIGGER:MEX','Unable to create MEX source: %s',path);

cleanup = onCleanup(@()fclose(fid)); %#ok<NASGU>

for k=1:numel(L)
    fprintf(fid,'%s\n',L{k});
end

end

function L = local_mex_source_lines()

L = {
'// G26 trigger packet kernel'
'// Auto-generated by G26_TRIGGER.m'
'#include "mex.h"'
'#include <cmath>'
'#include <algorithm>'
'#include <limits>'
''
'struct V3{double x,y,z;};'
''
'static inline V3 sub(V3 a,V3 b){'
' return {a.x-b.x,a.y-b.y,a.z-b.z};'
'}'
''
'static inline V3 add(V3 a,V3 b){'
' return {a.x+b.x,a.y+b.y,a.z+b.z};'
'}'
''
'static inline V3 mul(double s,V3 a){'
' return {s*a.x,s*a.y,s*a.z};'
'}'
''
'static inline double dot(V3 a,V3 b){'
' return a.x*b.x+a.y*b.y+a.z*b.z;'
'}'
''
'static inline double nrm(V3 a){'
' return std::sqrt(dot(a,a));'
'}'
''
'static inline V3 col3(const double*A,mwSize j){'
' return {A[3*j],A[3*j+1],A[3*j+2]};'
'}'
''
'static inline V3 Rt(const double*R,V3 v){'
' return {'
'   R[0]*v.x+R[1]*v.y+R[2]*v.z,'
'   R[3]*v.x+R[4]*v.y+R[5]*v.z,'
'   R[6]*v.x+R[7]*v.y+R[8]*v.z};'
'}'
''
'static double pointCylinder(V3 p,V3 c,double r,double h,const double*R){'
' V3 z=Rt(R,sub(p,c));'
' double dr=std::max(std::hypot(z.x,z.y)-r,0.0);'
' double dz=std::max(std::fabs(z.z)-0.5*h,0.0);'
' return std::hypot(dr,dz);'
'}'
''
'static double pointLevel(V3 p,V3 c,const double*R,V3 a){'
' V3 z=Rt(R,sub(p,c));'
' z.x/=a.x;'
' z.y/=a.y;'
' z.z/=a.z;'
' return nrm(z);'
'}'
''
'static double segmentLevel(V3 p0,V3 p1,V3 c,const double*R,V3 a){'
' V3 z0=Rt(R,sub(p0,c));'
' V3 z1=Rt(R,sub(p1,c));'
''
' z0.x/=a.x;'
' z0.y/=a.y;'
' z0.z/=a.z;'
''
' z1.x/=a.x;'
' z1.y/=a.y;'
' z1.z/=a.z;'
''
' V3 d=sub(z1,z0);'
' double den=dot(d,d);'
''
' if(den<=std::numeric_limits<double>::epsilon())'
'   return nrm(z0);'
''
' double u=-dot(z0,d)/den;'
' u=std::max(0.0,std::min(1.0,u));'
''
' return nrm(add(z0,mul(u,d)));'
'}'
''
'void mexFunction(int nlhs,mxArray*plhs[],int nrhs,const mxArray*prhs[]){'
''
' if(nrhs!=15)'
'  mexErrMsgIdAndTxt("G26TRIGGER:MEX","Expected 15 inputs.");'
''
' if(nlhs>5)'
'  mexErrMsgIdAndTxt("G26TRIGGER:MEX","At most 5 outputs.");'
''
' mwSize i0m=(mwSize)mxGetScalar(prhs[0]);'
' double H=mxGetScalar(prhs[1]);'
' double dt=mxGetScalar(prhs[2]);'
' double rangeMax=mxGetScalar(prhs[3]);'
' double L=mxGetScalar(prhs[4]);'
''
' const mxArray*pLmx=prhs[5];'
' const mxArray*qmx=prhs[6];'
' const mxArray*Cmx=prhs[7];'
' const mxArray*rrmx=prhs[8];'
' const mxArray*rhmx=prhs[9];'
' const mxArray*Rmx=prhs[10];'
' const mxArray*Apmx=prhs[11];'
' const mxArray*Aqmx=prhs[12];'
' const mxArray*Acmx=prhs[13];'
' const mxArray*Idmx=prhs[14];'
''
' if(mxGetM(pLmx)!=3 || mxGetM(qmx)!=3)'
'  mexErrMsgIdAndTxt("G26TRIGGER:MEX","pL,q must be 3xN.");'
''
' mwSize N=mxGetN(pLmx);'
''
' if(mxGetN(qmx)!=N)'
'  mexErrMsgIdAndTxt("G26TRIGGER:MEX","pL/q length mismatch.");'
''
' if(i0m<1 || i0m>N)'
'  mexErrMsgIdAndTxt("G26TRIGGER:MEX","start index invalid.");'
''
' if(mxGetM(Cmx)!=3 || mxGetM(Rmx)!=9 || '
'    mxGetM(Apmx)!=3 || mxGetM(Aqmx)!=3 || mxGetM(Acmx)!=3)'
'  mexErrMsgIdAndTxt("G26TRIGGER:MEX","bad obstacle matrices.");'
''
' mwSize M=mxGetN(Cmx);'
''
' if(mxGetNumberOfElements(rrmx)!=M || '
'    mxGetNumberOfElements(rhmx)!=M || '
'    mxGetN(Rmx)!=M || '
'    mxGetN(Apmx)!=M || '
'    mxGetN(Aqmx)!=M || '
'    mxGetN(Acmx)!=M || '
'    mxGetNumberOfElements(Idmx)!=M)'
'  mexErrMsgIdAndTxt("G26TRIGGER:MEX","obstacle size mismatch.");'
''
' const double*pL=mxGetPr(pLmx);'
' const double*q=mxGetPr(qmx);'
' const double*C=mxGetPr(Cmx);'
' const double*rr=mxGetPr(rrmx);'
' const double*rh=mxGetPr(rhmx);'
' const double*Rf=mxGetPr(Rmx);'
' const double*Ap=mxGetPr(Apmx);'
' const double*Aq=mxGetPr(Aqmx);'
' const double*Ac=mxGetPr(Acmx);'
' const double*ids=mxGetPr(Idmx);'
''
' mwSize i0=i0m-1;'
''
' V3 pL0=col3(pL,i0);'
' V3 q0=col3(q,i0);'
' V3 pQ0=sub(pL0,mul(L,q0));'
''
' unsigned char*active=(unsigned char*)mxCalloc(M,sizeof(unsigned char));'
''
' bool any=false;'
' mwSize visibleCount=0;'
''
' for(mwSize i=0;i<M;i++){'
'  V3 c={C[3*i],C[3*i+1],C[3*i+2]};'
''
'  if(pointCylinder(pQ0,c,rr[i],rh[i],Rf+9*i)<=rangeMax){'
'   active[i]=1;'
'   any=true;'
'   visibleCount++;'
'  }'
' }'
''
' bool found=false;'
' double jhit=mxGetNaN();'
' double oid=mxGetNaN();'
' double comp=mxGetNaN();'
''
' if(any){'
''
'  mwSize nH=(mwSize)std::floor(H/dt+1e-12);'
'  mwSize i1=std::min(N-1,i0+nH);'
''
'  for(mwSize j=i0;j<=i1;j++){'
''
'   V3 pLj=col3(pL,j);'
'   V3 qj=col3(q,j);'
'   V3 pQj=sub(pLj,mul(L,qj));'
''
'   double best=std::numeric_limits<double>::infinity();'
'   double bestComp=mxGetNaN();'
'   double bestOid=mxGetNaN();'
''
'   for(mwSize i=0;i<M;i++){'
''
'    if(!active[i])'
'     continue;'
''
'    V3 c={C[3*i],C[3*i+1],C[3*i+2]};'
'    const double*R=Rf+9*i;'
''
'    V3 ap={Ap[3*i],Ap[3*i+1],Ap[3*i+2]};'
'    V3 aq={Aq[3*i],Aq[3*i+1],Aq[3*i+2]};'
'    V3 ac={Ac[3*i],Ac[3*i+1],Ac[3*i+2]};'
''
'    double sp=pointLevel(pLj,c,R,ap)-1.0;'
''
'    if(sp<best){'
'     best=sp;'
'     bestComp=1;'
'     bestOid=ids[i];'
'    }'
''
'    double sq=pointLevel(pQj,c,R,aq)-1.0;'
''
'    if(sq<best){'
'     best=sq;'
'     bestComp=2;'
'     bestOid=ids[i];'
'    }'
''
'    double sc=segmentLevel(pLj,pQj,c,R,ac)-1.0;'
''
'    if(sc<best){'
'     best=sc;'
'     bestComp=3;'
'     bestOid=ids[i];'
'    }'
'   }'
''
'   if(best<=0){'
'    found=true;'
'    jhit=(double)(j+1);'
'    oid=bestOid;'
'    comp=bestComp;'
'    break;'
'   }'
'  }'
' }'
''
' mxFree(active);'
''
' if(nlhs>0)'
'  plhs[0]=mxCreateLogicalScalar(found);'
''
' if(nlhs>1)'
'  plhs[1]=mxCreateDoubleScalar(jhit);'
''
' if(nlhs>2)'
'  plhs[2]=mxCreateDoubleScalar(oid);'
''
' if(nlhs>3)'
'  plhs[3]=mxCreateDoubleScalar(comp);'
''
' if(nlhs>4)'
'  plhs[4]=mxCreateDoubleScalar((double)visibleCount);'
'}'
};

end
