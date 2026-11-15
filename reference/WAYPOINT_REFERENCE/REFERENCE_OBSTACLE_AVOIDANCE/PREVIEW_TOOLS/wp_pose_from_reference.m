function [pD,pL,info]=wp_pose_from_reference(xd,cfg,opts)
% Reconstruct future nominal tether pose from desired LOAD flat output.
% The existing Flight sequence applies SUSPENDED_LOAD_REF_ADJUST:
% pL_des=raw_waypoint(1:3)-[0;0;L]; ddot(pL_des)=xd(9:11).
% Taut, point-load dynamics: q (drone->load) = -(aL+g*e3)/||...||;
% pDrone=pLoad-L*q. Neither this nor the vertical mode predicts tracking.
if nargin<3,opts=struct();end
if ~isfield(opts,'predictionMode'),opts.predictionMode='flatness';end
if ~isfield(opts,'gravity'),opts.gravity=9.81;end
if ~isfield(opts,'freefallThreshold'),opts.freefallThreshold=0.15;end
pL=xd(1:3);pL(3)=pL(3)-cfg.cableLength;
aL=xd(9:11);
gvec=[0;0;opts.gravity];
info=struct('valid',true,'reason','','predictionMode',char(opts.predictionMode), ...
    'nominalCableDirection',[0;0;-1],'effectiveAccelerationNorm',NaN);
switch lower(char(opts.predictionMode))
    case 'flatness'
        w=aL+gvec;
        info.effectiveAccelerationNorm=norm(w);
        if info.effectiveAccelerationNorm<opts.freefallThreshold
            pD=nan(3,1);info.valid=false;
            info.reason='flatness singular: load acceleration approaches -g';return;
        end
        q=-w/norm(w);
    case 'vertical'
        q=[0;0;-1];
        info.reason='vertical cable assumption, ignores nominal load swing';
    otherwise
        error('WPPreview:PredictionMode','Use flatness or vertical');
end
pD=pL-cfg.cableLength*q;
info.nominalCableDirection=q;
end
