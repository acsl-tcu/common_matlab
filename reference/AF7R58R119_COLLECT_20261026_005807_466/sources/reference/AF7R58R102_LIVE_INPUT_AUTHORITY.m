function [I,S]=AF7R58R102_LIVE_INPUT_AUTHORITY(agent,optionalTension)
% Explicit live actuator source. Does not guess mass, thrust or torque.
% This is a verified HLC COMMAND CLAMP authority, NOT a closed-loop
% tracking bound, a torque certificate, or a strength-rating authority.
% For saved Flight replay the payload mass is taken from the exact R88
% snapshot by the calling runner; it is NOT replaced by a stale mass.
I=struct();
S=struct('status','MISSING_VERIFIED_HLC_COMMAND_AUTHORITY', ...
    'input_authorities_available',false,'cable_strength_available',false, ...
    'hlc_binding_verified',false,'production_commit_authorized',false);
obj=locate(agent,5);
if isempty(obj),S.status='REPLANNER_NOT_FOUND';return;end
if ~isprop(obj,'command_limit_certificate') || ...
        ~isprop(obj,'self'),return;end
cert=obj.command_limit_certificate;
req={'certified','live_controller_verified','live_gain_verified', ...
    'controller_class','command_order','limits'};
if ~isstruct(cert)||~all(isfield(cert,req)),return;end
if ~logical(cert.certified)|| ...
        ~logical(cert.live_controller_verified)|| ...
        ~logical(cert.live_gain_verified)|| ...
        ~strcmp(char(cert.controller_class),'HLC_SUSPENDED_LOAD')|| ...
        ~isequal(cert.command_order,{'F','Mx','My','Mz'})
    S.status='LIVE_HLC_SOURCE_OR_GAIN_NOT_VERIFIED';return;
end
lim=double(cert.limits);
if ~isequal(size(lim),[4,2])||~all(isfinite(lim(:)))|| ...
        any(lim(:,1)>=lim(:,2))||lim(1,1)<0
    S.status='LIVE_HLC_COMMAND_LIMITS_MALFORMED';return;
end
self=obj.self;
if ~isobject(self)||~isprop(self,'parameter')
    S.status='LIVE_VEHICLE_PARAMETER_UNAVAILABLE';return;
end
params=self.parameter;
if isobject(params)&&isprop(params,'mass')
    m=params.mass;
elseif isstruct(params)&&isfield(params,'mass')
    m=params.mass;
else
    S.status='LIVE_VEHICLE_MASS_UNAVAILABLE';return;
end
if ~isnumeric(m)||~isscalar(m)||~isfinite(m)||m<=0
    S.status='LIVE_VEHICLE_MASS_INVALID';return;
end
I=struct('vehicle_mass_kg',double(m), ...
    'thrust_bounds_N',lim(1,:), ...
    'torque_bounds_Nm',lim(2:4,:));
if nargin>=2 && ~isempty(optionalTension)
    assert(isnumeric(optionalTension)&&isscalar(optionalTension)&& ...
        isfinite(optionalTension)&&optionalTension>0, ...
        'R102:ExternalTensionRatingRequired', ...
        'Cable tensile strength must come from live/external authority.');
    I.tension_max_N=double(optionalTension);
    S.cable_strength_available=true;
end
S.status='SOURCE_BOUND_HLC_COMMAND_LIMITS_ONLY__NOT_TORQUE_PROOF';
S.input_authorities_available=true;
S.hlc_binding_verified=true;
end
function obj=locate(x,depth)
obj=[];
if isa(x,'REPLANNING_SYSTEM')&&isscalar(x),obj=x;return;end
if depth<1||isempty(x)||numel(x)~=1,return;end
if isobject(x)
    try,n=properties(x);catch,return;end
elseif isstruct(x)
    n=fieldnames(x);
elseif iscell(x)
    for j=1:numel(x)
        obj=locate(x{j},depth-1);
        if ~isempty(obj),return;end
    end
    return;
else,return;end
skip={'self','result','parameter','plant','sensor','estimator', ...
    'controller','logger','time','state','parent'};
for j=1:numel(n)
    if any(strcmpi(n{j},skip)),continue;end
    try,obj=locate(x.(n{j}),depth-1);catch,obj=[];end
    if ~isempty(obj),return;end
end
end
