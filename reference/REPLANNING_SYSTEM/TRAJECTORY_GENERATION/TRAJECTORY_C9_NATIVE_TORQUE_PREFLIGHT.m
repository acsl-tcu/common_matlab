function R=TRAJECTORY_C9_NATIVE_TORQUE_PREFLIGHT(build_requested)
% Explicit compile-and-pin step BEFORE Flight; never inside HLC or Flight.
% The compiled binary is local to the production source directory. If a
% compiler is unavailable, all production math uses the original MATLAB
% proof. This test does NOT establish interval outward rounding.
if nargin<1,build_requested=false;end
clock=tic;
R=struct('status','NOT_STARTED','ready',false,'compiled',false, ...
    'native_path','','source_path','','message','', ...
    'safe_to_commit',false,'elapsed_ms',NaN);
try
    assert((islogical(build_requested)||isnumeric(build_requested))&& ...
        isscalar(build_requested)&&any(double(build_requested)==[0 1]), ...
        'C9_NATIVE_PREFLIGHT:Flag','build_requested must be 0/1');
    folder=fileparts(mfilename('fullpath'));
    cpp=fullfile(folder,'TRAJECTORY_C9_TORQUE_LEAF_NATIVE.cpp');
    native=fullfile(folder,['TRAJECTORY_C9_TORQUE_LEAF_NATIVE.' mexext]);
    R.source_path=cpp;
    if exist(cpp,'file')~=2
        R.status='UNKNOWN_CPP_SOURCE_MISSING';R.elapsed_ms=1000*toc(clock);return
    end
    if logical(build_requested)
        mex('-R2018a','-output','TRAJECTORY_C9_TORQUE_LEAF_NATIVE', ...
            '-outdir',folder,cpp);
        R.compiled=true;
        clear('TRAJECTORY_C9_TORQUE_LEAF_NATIVE');
        rehash;
    end
    R.native_path=which('TRAJECTORY_C9_TORQUE_LEAF_NATIVE');
    if exist(native,'file')~=3 || ~strcmpi(R.native_path,native)
        R.status='UNKNOWN_MEX_MISSING_OR_PATH_SHADOWED';R.elapsed_ms=1000*toc(clock);return
    end
    a=dir(cpp);b=dir(native);
    if isempty(a)||isempty(b)||b.datenum<a.datenum
        R.status='UNKNOWN_MEX_STALE__REBUILD_BEFORE_FLIGHT';R.elapsed_ms=1000*toc(clock);return
    end
    assert(strcmp(TRAJECTORY_C9_TORQUE_LEAF_NATIVE(), ...
        'C9_FUSED_TORQUE_LEAF_20261110_V1'), ...
        'C9_NATIVE_PREFLIGHT:Signature','Native source/build signature mismatch');
    % Pure software, finite exact-zero leaf smoke. Never Flight authority.
    lo=zeros(3,6);hi=lo;
    lo(:,1)=[0.8;0;0];hi(:,1)=lo(:,1);
    phy=[1;0.1;0.1;2;0;0;-9.80665;0.01;0.01;0.02;0;0];
    [box,ok]=TRAJECTORY_C9_TORQUE_LEAF_NATIVE(lo,hi,phy,[1;1;1]);
    assert(ok&&isequal(size(box),[3 2])&& ...
        all(isfinite(box(:)))&&all(box(:,1)<=box(:,2)), ...
        'C9_NATIVE_PREFLIGHT:Smoke','Native smoke leaf invalid');
    R.ready=true;
    R.status='PASS_NATIVE_COMPILED_PINNED__MATLAB_AB_STILL_REQUIRED';
catch ME
    R.status=['UNKNOWN_NATIVE_PREFLIGHT_' ME.identifier];
    R.message=ME.message;
end
R.elapsed_ms=1000*toc(clock);
end
