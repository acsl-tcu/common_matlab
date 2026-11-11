function R=TRAJECTORY_C9_25D_TORQUE_BATCH_PREFLIGHT(build_requested)
%TRAJECTORY_C9_25D_TORQUE_BATCH_PREFLIGHT  Explicit build/pin BEFORE Flight.
% Not a controller/obstacle/collision certificate. Never called from the
% Flight 25-ms loop. All dynamically changing parameters enter the MEX
% independently on EVERY local QP call.
if nargin<1,build_requested=false;end
clock=tic;
R=struct('status','NOT_STARTED','ready',false,'compiled',false, ...
    'source_path','','binary_path','','message','', ...
    'safe_to_commit',false,'elapsed_ms',NaN);
try
    assert((isnumeric(build_requested)||islogical(build_requested))&& ...
        isscalar(build_requested)&&isreal(build_requested)&& ...
        isfinite(double(build_requested))&&any(double(build_requested)==[0 1]), ...
        'C9_BATCH_PREFLIGHT:Flag','build_requested must be 0/1.');
    folder=fileparts(mfilename('fullpath'));
    name='TRAJECTORY_C9_25D_TORQUE_BATCH_NATIVE';
    cpp=fullfile(folder,[name '.cpp']);
    binary=fullfile(folder,[name '.' mexext]);
    R.source_path=cpp;R.binary_path=binary;
    if exist(cpp,'file')~=2
        R.status='UNKNOWN_NATIVE_BATCH_SOURCE_MISSING';return
    end
    if logical(build_requested)
        mex('-R2018a','-output',name,'-outdir',folder,cpp);
        R.compiled=true;
        clear('TRAJECTORY_C9_25D_TORQUE_BATCH_NATIVE');rehash;
    end
    actual=which(name);
    if exist(binary,'file')~=3||~strcmpi(actual,binary)
        R.status='UNKNOWN_NATIVE_BATCH_MISSING_OR_SHADOWED';return
    end
    a=dir(cpp);b=dir(binary);
    if isempty(a)||isempty(b)||b.datenum<a.datenum
        R.status='UNKNOWN_NATIVE_BATCH_STALE__BUILD_BEFORE_FLIGHT';return
    end
    if ~strcmp(TRAJECTORY_C9_25D_TORQUE_BATCH_NATIVE(), ...
            'C9_25D_TORQUE_BATCH_NATIVE_20261111_V1')
        R.status='UNKNOWN_NATIVE_BATCH_SIGNATURE';return
    end
    % Source-independent smoke: constant positive thrust, nonzero heading,
    % zero angular rate/torque. Synthetic values are NOT Flight fixtures.
    jets=cell(1,7);
    for k=1:7,jets{k}=zeros(78,3);end
    jets{2}(1,:)=1.5;jets{2}(2,:)=0.4;
    [tau,J,valid,head]=TRAJECTORY_C9_25D_TORQUE_BATCH_NATIVE( ...
        jets,[0;0;-9.80665],[1;0.15;2],[0.02;0.03;0.04],[0;0]);
    assert(isequal(size(tau),[3 3])&& ...
        isequal(size(J),[3 25 3])&& ...
        all(valid(:))&&all(isfinite(tau(:)))&& ...
        max(abs(tau(:)))<1e-12&&all(head>0), ...
        'C9_BATCH_PREFLIGHT:Smoke','Native local torque gradient smoke failed.');
    R.ready=true;
    R.status='PASS_25D_BATCH_NATIVE_PINNED__MATLAB_AB_STILL_REQUIRED';
catch ME
    R.status=['UNKNOWN_25D_BATCH_PREFLIGHT_' ME.identifier];
    R.message=ME.message;
end
R.elapsed_ms=1000*toc(clock);
end
