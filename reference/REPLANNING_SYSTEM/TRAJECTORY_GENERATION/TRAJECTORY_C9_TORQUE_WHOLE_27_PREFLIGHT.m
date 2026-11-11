function R=TRAJECTORY_C9_TORQUE_WHOLE_27_PREFLIGHT(rebuild)
% Explicit Flight-preflight MEX build for the new degree-27 whole-tree
% directed nominal torque proof; NEVER compile inside a Flight update.
if nargin<1,rebuild=false;end
R=struct('status','UNKNOWN_DEGREE27_TORQUE_MEX', ...
    'ready',false,'message','', ...
    'signature','C9_DIRECTED_WHOLE_TREE_27_20261110_V1');
folder=fileparts(mfilename('fullpath'));
cpp=fullfile(folder,'TRAJECTORY_C9_TORQUE_WHOLE_27_NATIVE.cpp');
bin=fullfile(folder,['TRAJECTORY_C9_TORQUE_WHOLE_27_NATIVE.' mexext]);
if exist(cpp,'file')~=2
    R.status='UNKNOWN_CPP_SOURCE_MISSING';return
end
if rebuild
    try
        clear TRAJECTORY_C9_TORQUE_WHOLE_27_NATIVE
        mex('-outdir',folder,cpp);
    catch ME
        R.status='UNKNOWN_DEGREE27_MEX_BUILD_FAILED';
        R.message=ME.message;return
    end
end
rehash
if exist(bin,'file')~=3 || ...
        ~strcmpi(which('TRAJECTORY_C9_TORQUE_WHOLE_27_NATIVE'),bin)
    R.status='UNKNOWN_DEGREE27_MEX_MISSING_OR_SHADOWED';return
end
infoCpp=dir(cpp);infoMex=dir(bin);
if isempty(infoCpp)||isempty(infoMex)||infoMex.datenum<infoCpp.datenum
    R.status='UNKNOWN_DEGREE27_MEX_STALE';return
end
try
    sig=TRAJECTORY_C9_TORQUE_WHOLE_27_NATIVE();
    R.ready=strcmp(sig,R.signature);
    if R.ready
        R.status='PASS_DEGREE27_DIRECTED_NOMINAL_TORQUE_MEX_PINNED';
    else
        R.status='UNKNOWN_DEGREE27_MEX_SIGNATURE';
    end
catch ME
    R.status='UNKNOWN_DEGREE27_MEX_CALL_EXCEPTION';
    R.message=ME.message;
end
end
