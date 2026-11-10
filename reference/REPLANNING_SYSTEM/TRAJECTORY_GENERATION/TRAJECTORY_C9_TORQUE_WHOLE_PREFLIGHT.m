function R=TRAJECTORY_C9_TORQUE_WHOLE_PREFLIGHT(build_requested)
% Explicit pre-Flight compile/pin; never compile inside the Flight loop.
% No COMMIT is ever authorized by compiling or by this smoke test.
if nargin<1,build_requested=false;end
w=tic;
R=struct('status','NOT_STARTED','ready',false,'compiled',false, ...
    'native_path','','source_path','','message','', ...
    'safe_to_commit',false,'elapsed_ms',NaN);
try
    assert((islogical(build_requested)||isnumeric(build_requested))&& ...
        isscalar(build_requested)&&any(double(build_requested)==[0 1]), ...
        'C9_WHOLE_PREFLIGHT:Flag','build_requested must be 0/1');
    folder=fileparts(mfilename('fullpath'));
    cpp=fullfile(folder,'TRAJECTORY_C9_TORQUE_WHOLE_NATIVE.cpp');
    bin=fullfile(folder,['TRAJECTORY_C9_TORQUE_WHOLE_NATIVE.' mexext]);
    R.source_path=cpp;
    if exist(cpp,'file')~=2
        R.status='UNKNOWN_CPP_SOURCE_MISSING';return
    end
    if logical(build_requested)
        mex('-R2018a','-output','TRAJECTORY_C9_TORQUE_WHOLE_NATIVE', ...
            '-outdir',folder,cpp);
        R.compiled=true;
        clear('TRAJECTORY_C9_TORQUE_WHOLE_NATIVE');
        rehash;
    end
    R.native_path=which('TRAJECTORY_C9_TORQUE_WHOLE_NATIVE');
    if exist(bin,'file')~=3 || ~strcmpi(R.native_path,bin)
        R.status='UNKNOWN_MEX_MISSING_OR_PATH_SHADOWED';return
    end
    a=dir(cpp);b=dir(bin);
    if isempty(a)||isempty(b)||b.datenum<a.datenum
        R.status='UNKNOWN_MEX_STALE_REBUILD_BEFORE_FLIGHT';return
    end
    assert(strcmp(TRAJECTORY_C9_TORQUE_WHOLE_NATIVE(), ...
        'C9_DIRECTED_WHOLE_TREE_20261110_V1'), ...
        'C9_WHOLE_PREFLIGHT:Signature','Native signature mismatch');
    % Nontrivial linear flight: after exact C9 differentiation, torque is
    % bounded around zero. This is a test, NOT a Flight guarantee.
    U=linspace(0,1,24);
    C=[4*U;0.02*U;3+zeros(1,24)];
    phys=[1;0.1;0.1;2;0;0;-9.80665;0.01;0.01;0.02;0;0];
    flags=[1;1;1];lim=[-1 1;-1 1;-1 1];
    [M,yes,nn,ll,reason]=TRAJECTORY_C9_TORQUE_WHOLE_NATIVE( ...
        C,8,phys,flags,lim,zeros(3,1),[8;511]);
    assert(isequal(yes,true)&&reason==0&&nn>=1&&ll>=1&& ...
        isequal(size(M),[3 2])&&all(isfinite(M(:)))&& ...
        all(M(:,1)<=0)&&all(M(:,2)>=0), ...
        'C9_WHOLE_PREFLIGHT:Smoke','Nontrivial whole-tree smoke invalid');
    R.ready=true;
    R.status='PASS_WHOLE_NATIVE_COMPILED_PINNED__FULL_SCENE_AB_REQUIRED';
catch ME
    R.status=['UNKNOWN_WHOLE_NATIVE_PREFLIGHT_' ME.identifier];
    R.message=ME.message;
end
R.elapsed_ms=1000*toc(w);
end
