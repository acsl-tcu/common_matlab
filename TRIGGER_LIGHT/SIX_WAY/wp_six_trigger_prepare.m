function known = wp_six_trigger_prepare(observedWorld,approximation)
%WP_SIX_TRIGGER_PREPARE Cache ONE chosen observed/memorized obstacle cover.
% Never pass the simulator's unobserved full world to an online trigger.
% Call only when the observed obstacle memory changes.
if nargin~=2,error('LightTrigger:Prepare','Need one observed cover and its approximation');end
approximation=char(approximation);
if ~ismember(approximation,{'multiple_spheres','multiple_ellipsoids'})
    error('LightTrigger:Prepare','Unsupported obstacle approximation');
end
known=wp_trigger_prepare_known(observedWorld);
kind=1;
if strcmp(approximation,'multiple_ellipsoids'),kind=2;end
if any(known.kind~=kind)
    error('LightTrigger:Prepare','Obstacle parts do not match selected approximation');
end
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
known.approximation=approximation;
known.observationProvenanceVerified=false;
known.outerCoverageVerified=false;
end
