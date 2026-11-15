function [pDrone,meta]=wp_v7_nominal_baseline_taut_projection(pLoad,pDroneRaw,cableDirection,L)
%WP_V7_NOMINAL_BASELINE_TAUT_PROJECTION Test-only original-trace normalization.
% MATLAB ODE integration can give a taut-cable residual ~1e-6 m,
% invalidating a ZERO-BOUND nominal collision comparison. Reconstruct the
% test-only drone endpoint along the original raw endpoint direction.
% This is NOT an uncertainty tube, a safety margin, or physical guarantee.
% The unmodified original state is used everywhere else.
if ~isequal(size(pLoad),size(pDroneRaw),size(cableDirection)) || ...
        size(pLoad,1)~=3 || ~isscalar(L) || ~isfinite(L) || L<=0 || ...
        any(~isfinite([pLoad(:);pDroneRaw(:);cableDirection(:)]))
    error('V7:NominalBaseline','Invalid original nominal trace input');
end
segment=pLoad-pDroneRaw;
segmentLength=vecnorm(segment,2,1);
qLength=vecnorm(cableDirection,2,1);
if any(segmentLength<=eps) || any(qLength<=eps) || ...
    any(sum(segment.*cableDirection,1)<=0)
    error('V7:NominalBaseline','Original segment/cable direction is invalid');
end
% Minimum endpoint displacement needed to satisfy norm(pL-pD)==L.
% Preserve the original endpoint line direction exactly.
q=bsxfun(@rdivide,segment,segmentLength);
pDrone=pLoad-L*q;
delta=vecnorm(pDrone-pDroneRaw,2,1);
meta=struct('maxCorrection_m',max(delta), ...
    'maxDirectionMismatch_norm',max(vecnorm(q-bsxfun(@rdivide,cableDirection,qLength),2,1)), ...
    'maxOriginalTautResidual_m',max(abs(vecnorm(pLoad-pDroneRaw,2,1)-L)), ...
    'maxProjectedTautResidual_m',max(abs(vecnorm(pLoad-pDrone,2,1)-L)), ...
    'onlyNominalTest',true,'physicalBoundCertified',false);
end
