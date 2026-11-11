function R=TRAJECTORY_C9_FUSED_NATIVE_PREFLIGHT(rebuild)
% Explicit native build/check only BEFORE Flight. Never called by search.
if nargin<1,rebuild=false;end
R=struct('status','UNKNOWN_FUSED_NATIVE_NOT_BUILT','ready',false, ...
    'message','','source_path','','binary_path','','elapsed_ms',NaN);
tic0=tic;
try
 assert(isscalar(rebuild)&&any(double(rebuild)==[0 1]), ...
  'C9_FUSED:Flag','rebuild must be 0/1.');
 root=fileparts(mfilename('fullpath'));
 name='TRAJECTORY_C9_FUSED_ALLTIME_CORE';
 src=fullfile(root,[name '.cpp']);
 bin=fullfile(root,[name '.' mexext]);
 R.source_path=src;R.binary_path=bin;
 if ~isfile(src),R.status='UNKNOWN_FUSED_NATIVE_SOURCE_MISSING';return;end
 if logical(rebuild)
    clear TRAJECTORY_C9_FUSED_ALLTIME_CORE
    mex('-R2018a','-output',name,'-outdir',root,src);rehash;
 end
 if exist(bin,'file')~=3||~strcmpi(which(name),bin)
    R.status='UNKNOWN_FUSED_NATIVE_BINARY_MISSING_OR_SHADOWED';return
 end
 A=dir(src);B=dir(bin);
 if isempty(A)||isempty(B)||B.datenum<A.datenum
    R.status='UNKNOWN_FUSED_NATIVE_BINARY_STALE';return
 end
 if ~strcmp(feval(name),'C9_FUSED_V3_GENERAL_SEPARATOR_20261111')
    R.status='UNKNOWN_FUSED_NATIVE_SIGNATURE_MISMATCH';return
 end
 % The direct MEX smoke passes only actual PREPARED canonical shapes.
 % A source-level all-time proof is NOT a Flight/HLC certificate.
 p=[zeros(1,28);linspace(0,27,28);3*ones(1,28)];
 a=OBSTACLE_INFORMATION.define(912,'sphere',[50 50 50],1);
 ss=TRAJECTORY_OBSTACLE_SUPPORT.prepare(a);
 vec=[1.1,.14,2,0,0,20,0,.3,.01,.2,0,8,900,1];
 X=feval(name,p,27,[0;0;-9.80665],vec,ss, ...
    'C9_FUSED_V3_GENERAL_SEPARATOR_20261111');
 assert(X.certified && X.checked_node_count>=1 && ...
        ~X.safe_to_commit && ~X.numerically_directed_rounding_proven);
 a=OBSTACLE_INFORMATION.define(913,'sphere',[0 13 3],1);
 ss=TRAJECTORY_OBSTACLE_SUPPORT.prepare(a);
 Y=feval(name,p,27,[0;0;-9.80665],vec,ss, ...
    'C9_FUSED_V3_GENERAL_SEPARATOR_20261111');
 assert(~Y.certified,'C9_FUSED:NearCollision','Near crossing cannot PASS.');
 % The support-point Gilbert refinement must be reachable from a genuine
 % near obstacle, and must NOT turn an overlapping leaf into a proof.
 assert(isfield(Y,'generic_separator_calls') && ...
    Y.generic_separator_calls>0 && ...
    isfield(Y,'generic_separator_iterations') && ...
    Y.generic_separator_iterations>=0, ...
    'C9_FUSED:GenericSearch','Native general SO(3) support search missing.');
 assert(isfield(Y,'unresolved_phase_interval') && ...
    all(isfinite(Y.unresolved_phase_interval)) && ...
    Y.unresolved_phase_interval(2)>Y.unresolved_phase_interval(1) && ...
    isfield(Y,'unresolved_obstacle_ids') && ...
    any(Y.unresolved_obstacle_ids==913) && ...
    isfield(Y,'unresolved_body_indices') && ...
    ~isempty(Y.unresolved_body_indices), ...
    'C9_FUSED:Witness','Near crossing must report native proof-tree witness.');
 R.ready=true;R.status='PASS_FUSED_NATIVE_PINNED__MATLAB_AB_REQUIRED';
catch ME
 R.status=['UNKNOWN_FUSED_NATIVE_PREFLIGHT_' ME.identifier];
 R.message=ME.message;
end
R.elapsed_ms=1000*toc(tic0);
end
