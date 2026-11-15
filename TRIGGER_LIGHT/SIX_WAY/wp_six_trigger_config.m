function cfg = wp_six_trigger_config(systemMode,obstacleApproximation,externalGeometry,model)
%WP_SIX_TRIGGER_CONFIG Configure exactly ONE geometry combination.
%
% Examples:
%   model = wp_six_trigger_model_snapshot(agent);
%   original = wp_system_geometry_config('B'); % external provisional file
%   cfg = wp_six_trigger_config('B','multiple_ellipsoids',original,model);
%
% Radii, margin, segment count and D-ellipsoid eta come from the EXTERNAL
% geometry definition. Cable length and physics come from the MODEL.
% No guessed geometry/physical defaults live in this function.
if nargin~=4, error('LightTrigger:Config','Need mode, approximation, external geometry, model snapshot');end
systemMode=upper(char(systemMode));
obstacleApproximation=char(obstacleApproximation);
if ~ismember(systemMode,{'C','B','D'}) || ...
        ~ismember(obstacleApproximation,{'multiple_spheres','multiple_ellipsoids'})
    error('LightTrigger:Config','One of C/B/D and one of the two obstacle covers is required');
end
if ~isstruct(externalGeometry) || ~isstruct(model)
    error('LightTrigger:Config','External geometry and model snapshot must be structs');
end
fields={'droneRadius','loadRadius','cableRadius','safetyMargin', ...
    'cableSegments','ellipsoidEta','checkGround'};
for j=1:numel(fields)
    if ~isfield(externalGeometry,fields{j})
        error('LightTrigger:Config','Missing external geometry field: %s',fields{j});
    end
end
if isfield(externalGeometry,'method') && ...
        ~strcmpi(char(externalGeometry.method),systemMode)
    error('LightTrigger:Config','Geometry source is for a DIFFERENT system mode');
end
if ~isfield(model,'cableLength') || ~isfield(model,'loadMass')
    error('LightTrigger:Config','Model snapshot lacks actual cable length or current mL');
end
cfg=struct();
cfg.systemMode=systemMode;
cfg.obstacleApproximation=obstacleApproximation;
for j=1:numel(fields)
    cfg.(fields{j})=externalGeometry.(fields{j});
end
cfg.cableLength=model.cableLength;  % never use a provisional hardcoded length
cfg.currentLoadMass=model.loadMass;
cfg.isProvisional=isfield(externalGeometry,'isProvisional') && logical(externalGeometry.isProvisional);
cfg.cbfGeometryCertified=false;
nums={'droneRadius','loadRadius','cableRadius','safetyMargin','cableLength','ellipsoidEta'};
for j=1:numel(nums)
    x=cfg.(nums{j});
    if ~isnumeric(x) || ~isscalar(x) || ~isfinite(x) || x<0
        error('LightTrigger:Config','Invalid geometry field %s',nums{j});
    end
end
if cfg.droneRadius<=0 || cfg.loadRadius<=0 || cfg.cableLength<=0 || cfg.ellipsoidEta<=0 || ...
        cfg.cableSegments<1 || cfg.cableSegments~=floor(cfg.cableSegments) || ...
        ~isscalar(cfg.checkGround)
    error('LightTrigger:Config','Invalid geometry parameters');
end
end
