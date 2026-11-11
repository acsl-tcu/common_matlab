function R=TRAJECTORY_C9_SEPARATOR_NATIVE_PREFLIGHT(build_requested)
% Compile and pin a shape-general Gilbert/Frank-Wolfe normal finder.
% This native kernel NEVER decides collision: the original MATLAB support
% function independently verifies each successful candidate normal.
% Run only before Flight; never compile from a trajectory proof call.
if nargin<1,build_requested=false;end
clock=tic;
R=struct('status','NOT_STARTED','ready',false,'compiled',false, ...
    'source_path','','binary_path','','message','', ...
    'safe_to_commit',false,'elapsed_ms',NaN);
try
    assert((isnumeric(build_requested)||islogical(build_requested))&& ...
        isscalar(build_requested)&&isreal(build_requested)&& ...
        isfinite(double(build_requested))&&any(double(build_requested)==[0 1]), ...
        'C9_SEP_PREFLIGHT:Flag','build_requested must be 0/1.');
    folder=fileparts(mfilename('fullpath'));
    name='TRAJECTORY_C9_SEPARATOR_NORMAL_NATIVE';
    cpp=fullfile(folder,[name '.cpp']);
    binary=fullfile(folder,[name '.' mexext]);
    R.source_path=cpp;R.binary_path=binary;
    if exist(cpp,'file')~=2
        R.status='UNKNOWN_SEPARATOR_CPP_MISSING';return
    end
    if logical(build_requested)
        mex('-R2018a','-output',name,'-outdir',folder,cpp);
        R.compiled=true;
        clear('TRAJECTORY_C9_SEPARATOR_NORMAL_NATIVE');rehash;
    end
    if exist(binary,'file')~=3||~strcmpi(which(name),binary)
        R.status='UNKNOWN_SEPARATOR_MEX_MISSING_OR_SHADOWED';return
    end
    a=dir(cpp);b=dir(binary);
    if isempty(a)||isempty(b)||b.datenum<a.datenum
        R.status='UNKNOWN_SEPARATOR_BINARY_STALE';return
    end
    if ~strcmp(TRAJECTORY_C9_SEPARATOR_NORMAL_NATIVE(), ...
            'C9_NATIVE_SEPARATOR_NORMAL_20261110_V2_EMPTY_VERTEX_COMPAT')
        R.status='UNKNOWN_SEPARATOR_SIGNATURE';return
    end
    % Test arbitrary REAL canonical shape, no encoded obstacle scenario.
    S=struct('id',1,'kind','sphere','dim',0.5, ...
        'center',[0;0;0],'R',eye(3),'local_vertices',zeros(0,3));
    H=[2 2.1 1.9;0 0.1 -0.1;0 0 0];
    [n,proposed,ncalls,iters]=TRAJECTORY_C9_SEPARATOR_NORMAL_NATIVE( ...
        H,S,0.1,mean(H,2),16);
    assert(proposed&&ncalls>=1&&iters>=0&& ...
        isequal(size(n),[3 1])&&all(isfinite(n))&& ...
        min(n.'*H)>TRAJECTORY_OBSTACLE_SUPPORT.value(S,n)+0.1, ...
        'C9_SEP_PREFLIGHT:Verification', ...
        'Candidate did not pass the independent canonical support test.');
    % The 12-shape audit previously supplied 0x0 [] for nonvertex bodies,
    % while the old MEX only accepted the canonical 0x3 empty vertex set.
    % Validate both equivalent empty representations before declaring ready.
    S.local_vertices=[];
    [n0,proposed0,ncalls0,iters0]=TRAJECTORY_C9_SEPARATOR_NORMAL_NATIVE( ...
        H,S,0.1,mean(H,2),16);
    assert(proposed0&&ncalls0>=1&&iters0>=0&& ...
        isequal(size(n0),[3 1])&&all(isfinite(n0))&& ...
        min(n0.'*H)>TRAJECTORY_OBSTACLE_SUPPORT.value(S,n0)+0.1, ...
        'C9_SEP_PREFLIGHT:EmptyVertexCompatibility', ...
        'MEX rejected MATLAB [] for an analytic shape without vertices.');
    R.ready=true;
    R.status='PASS_SEPARATOR_NORMAL_PINNED__MATLAB_AB_REQUIRED';
catch ME
    R.status=['UNKNOWN_SEPARATOR_PREFLIGHT_' ME.identifier];
    R.message=ME.message;
end
R.elapsed_ms=1000*toc(clock);
end
