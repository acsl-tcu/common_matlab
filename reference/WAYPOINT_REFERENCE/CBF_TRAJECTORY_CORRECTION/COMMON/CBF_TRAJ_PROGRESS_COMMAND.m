function cmd=CBF_TRAJ_PROGRESS_COMMAND(e,slowdownStart,slowdownFull,minScale)
% Same smoothstep law as WAYPOINT_REFERENCE / REV37 governor_sample.
% Only the error target changes; the six-stage time filter is untouched.
if isfinite(e)
    u=max(0,min(1,(e-slowdownStart)/(slowdownFull-slowdownStart)));
    u=u*u*(3-2*u);
    cmd=1-(1-minScale)*u;
else
    cmd=1; % identical to original missing-estimate behavior
end
end
