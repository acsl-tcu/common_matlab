function jet=CBF_TRAJ_LOCAL_CORRECTION_JET(mode,j0,s,omega)
% Single mode-dependent local analytic correction, evaluated for diagnostics.
% No future plant input is assumed. The same j0 is sent to HLC at s=0.
if s==0,jet=j0;return;end
switch mode
    case 'CARRY'
        jet=[j0(:,1),zeros(3,6)];
    case 'RETURN'
        jet=CBF_TRAJ_RETURN_JET(j0,s,omega);
    case {'NOMINAL','DEGRADED'}
        jet=[j0(:,1),zeros(3,6)];
    otherwise
        error('CBFTraj:CorrectionMode','Unexpected correction mode');
end
end
