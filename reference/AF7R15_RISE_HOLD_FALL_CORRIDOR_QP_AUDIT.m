function OUT = AF7R15_RISE_HOLD_FALL_CORRIDOR_QP_AUDIT()
% AF7R15_RISE_HOLD_FALL_CORRIDOR_QP_AUDIT
% Read-only source audit for backend-1 R6 corridor-QP synthesis.

clc;
src=which('REPLANNING_SYSTEM');
assert(~isempty(src)&&isfile(src),'[AF7R15] REPLANNING_SYSTEM.m not found.');
txt=fileread(src);

hasRHF=contains(txt,'BEZIER_D15_C7 / RISE-HOLD-FALL CORRIDOR-QP PROPOSAL');
hasTail=contains(txt,'find_bezier_nominal_safe_tail_start');
hasQP=contains(txt,'solve_min_norm_halfspace_qp_3d');
hasHalf=contains(txt,'CONTINUOUS_HOLD_SUPPORT_HALFSPACES_PLUS_MIN_NORM_QP');
hasEigen=contains(txt,'LATERAL_ELLIPSOID_METRIC_PRINCIPAL_EIGENVECTOR');
hasThreeSplit=contains(txt,'Split exactly at the C7 rise/hold/fall joins.');
hasRHFDer=contains(txt,'bezier_c7_rise_hold_fall_derivatives');
oldAlpha=contains(txt,'alphaReq=');
oldDir=contains(txt,'dir=gw/ng');
oldDepth=contains(txt,'maxDepth=12;');
sampled=contains(txt,'sampled_acceptance'',true');

fprintf('\n');
fprintf('====================================================================================================\n');
fprintf(' AF7R15 | RISE-HOLD-FALL CORRIDOR-QP SOURCE AUDIT\n');
fprintf('====================================================================================================\n');
fprintf(' source                                  = %s\n',src);
fprintf(' R6 rise-hold-fall backend present       = %d\n',hasRHF);
fprintf(' continuous nominal safe-tail theorem    = %d\n',hasTail);
fprintf(' 3-D min-norm halfspace QP present       = %d\n',hasQP);
fprintf(' continuous hold halfspace theorem       = %d\n',hasHalf);
fprintf(' axis-free lateral metric eigenproblem   = %d\n',hasEigen);
fprintf(' certificate split rise/hold/fall       = %d\n',hasThreeSplit);
fprintf(' C7 rise-hold-fall derivative map        = %d\n',hasRHFDer);
fprintf(' old single-point alphaReq present       = %d\n',oldAlpha);
fprintf(' old trigger-gradient fixed dir present  = %d\n',oldDir);
fprintf(' old fixed maxDepth=12 authority         = %d\n',oldDepth);
fprintf(' sampled acceptance introduced           = %d\n',sampled);
closed=hasRHF&&hasTail&&hasQP&&hasHalf&&hasEigen&&hasThreeSplit&&hasRHFDer&& ...
    ~oldAlpha&&~oldDir&&~oldDepth&&~sampled;
fprintf(' backend-1 R6 source closed              = %d\n',closed);
fprintf('====================================================================================================\n');

OUT=struct();
OUT.schema='AF7R15_RISE_HOLD_FALL_CORRIDOR_QP_AUDIT_V1';
OUT.source=src;
OUT.closed=closed;
OUT.old_alpha_present=oldAlpha;
OUT.old_fixed_direction_present=oldDir;
OUT.old_fixed_depth_present=oldDepth;
OUT.sampled_acceptance_present=sampled;
assignin('base','AF7R15_RISE_HOLD_FALL_CORRIDOR_QP_AUDIT_OUT',OUT);
end
