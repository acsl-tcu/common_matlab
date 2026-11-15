function known = wp_six_trigger_prepare(observedWorld,approximation)
%WP_SIX_TRIGGER_PREPARE Cache ONE chosen observed/memorized obstacle cover.
% Never pass the simulator's unobserved full world to an online trigger.
% Call only when the observed obstacle memory changes.
if nargin<1 || nargin>2
    error('LightTrigger:Prepare','Need observedWorld [, expected method].');
end
% For direct part.kind inputs, infer only the METHOD LABEL; never refit.
info=ra_obstacle_method_info(observedWorld);
if nargin==2 && ~isempty(approximation) && ...
        ~strcmp(char(approximation),info.method)
    error('LightTrigger:Prepare','Explicit method differs from original parts/world');
end
if ~isfield(observedWorld,'approximation') || isempty(observedWorld.approximation)
    observedWorld.approximation=info.method; % temporary metadata only
end
known=wp_trigger_prepare_known(observedWorld);
if any(known.kind~=info.kindCode)
    error('LightTrigger:Prepare','Obstacle parts do not match world definition');
end
approximation=info.method;
% Exact world-axis half widths of each rotated ellipsoid's AABB.
% For O=c+R*diag(a)*B(1), h_O(e_i)-e_i'*c = ||diag(a)*R'*e_i||.
% These are derived from the actual OBSERVED outer covers, not a new
% guessed radius, and are prepared only when observation memory changes.
known.orientedAabbHalfWidth=zeros(3,known.numParts);
for j=1:known.numParts
    transform=known.R(:,:,j);
    semiAxes=known.axes(:,j);
    known.orientedAabbHalfWidth(:,j)= ...
        sqrt(sum((transform.*repmat(semiAxes.',3,1)).^2,2));
end
known.geometryCacheFormatVersion=4;
% V3/V4/V8 geometry dispatch recognizes two SHAPES, not four generator
% labels. Preserve the original generator label independently. This does
% NOT re-fit, replace, scale, or transform any original part geometry.
if strcmp(info.kind,'sphere')
    known.approximation='multiple_spheres';
else
    known.approximation='multiple_ellipsoids';
end
known.obstacleSourceApproximation=approximation;
known.obstacleKind=info.kind;
known.singlePerObject=info.singlePerObject;
known.observationProvenanceVerified=false;
known.outerCoverageVerified=false;
end
