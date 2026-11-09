function R=TRAJECTORY_C9_FAST_TOTAL_PIPELINE(guide,target_free,observed, ...
    radii,authority,geometry_options,force_options,corridor_options)
% TRAJECTORY_C9_FAST_TOTAL_PIPELINE
% Measure and execute the WHOLE certified-guide C9 subpipeline:
%  (1) joint all-time load/UAV/cable+tension+thrust proof,
%  (2) corridor planes from that SAME proof's time leaves,
%  (3) candidate ray in the coupled cone corridor,
%  (4) independent all-time joint proof of the resulting candidate.
%
% This is a CERTIFIED-GUIDE refinement, NOT an automatic detour search
% from a colliding original path. Torque, attitude/HLC tracking, and
% absolute WCET are NOT proved. Never authorizes HLC commit.
start=tic;
R=struct('status','UNKNOWN_NOT_CERTIFIED','candidate_available',false, ...
    'safe_to_commit',false,'scope', ...
    'IDEAL_TAUT_CERTIFIED_GUIDE_ONLY__NO_TORQUE_HLC', ...
    'corridor_ms',NaN,'ray_and_final_certificate_ms',NaN, ...
    'total_ms',NaN,'ray_fraction',NaN, ...
    'guide_joint_proof_ms',NaN,'corridor_plane_ms',NaN, ...
    'final_independent_joint_ms',NaN,'guide_proof_status','', ...
    'candidate_status','', 'corridor',struct(),'candidate',struct());
s=tic;
C=TRAJECTORY_C9_FAST_SHARED_CORRIDOR(guide,observed,radii, ...
    authority,geometry_options,force_options,corridor_options);
R.corridor_ms=toc(s)*1000;
R.guide_joint_proof_ms=C.joint_proof_ms;
R.corridor_plane_ms=C.plane_construction_ms;
R.guide_proof_status=C.joint_status;
if ~C.certified
    R.status=['UNKNOWN_CORRIDOR_' C.status];
    R.corridor=C;
    R.total_ms=toc(start)*1000;
    return
end
s=tic;
Q=TRAJECTORY_C9_TAUT_CORRIDOR_RAY(C,target_free,observed, ...
    radii,authority,geometry_options,force_options);
R.ray_and_final_certificate_ms=toc(s)*1000;
R.candidate_status=Q.status;
R.ray_fraction=Q.ray_fraction;
if isstruct(Q.fullbody_certificate)&& ...
        isfield(Q.fullbody_certificate,'elapsed_ms')
    R.final_independent_joint_ms=Q.fullbody_certificate.elapsed_ms;
end
R.corridor=C;
R.candidate=Q;
R.candidate_available=Q.candidate_available;
if Q.candidate_available
    R.status='CERTIFIED_C9_GUIDE_REFINEMENT_ALL_TIME_GEOMETRY_FORCE__NO_HLC';
else
    R.status=['UNKNOWN_RAY_' Q.status];
end
R.total_ms=toc(start)*1000;
end
