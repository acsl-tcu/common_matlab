function B=BUILD_REFERENCE_AVOIDANCE_C_MEX(forceBuild)
%BUILD_REFERENCE_AVOIDANCE_C_MEX Optional manual rebuild/diagnostics.
% Normal use NEVER calls this: prepare_observed auto-builds if C selected.
if nargin<1,forceBuild=true;end
if forceBuild,action='force';else,action='ensure';end
B=reference_avoidance_mex_ensure(action);
end
