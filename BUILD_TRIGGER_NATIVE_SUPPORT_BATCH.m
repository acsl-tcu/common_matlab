function R=BUILD_TRIGGER_NATIVE_SUPPORT_BATCH()
% Temporary build helper: put at common_matlab/; compiled MEX is production.
% Does not touch HLC, EKF, plant, logger, sensor or flight runner.
R=struct('status','BUILD_NOT_STARTED','reason','', ...
    'mex_path','','source_path','');
try
    here=fileparts(mfilename('fullpath'));
    folder=fullfile(here,'reference','REPLANNING_SYSTEM', ...
        'FUTURE_COLLISION_TRIGGER');
    src=fullfile(folder,'TRIGGER_SUPPORT_UPPER_NATIVE.cpp');
    assert(exist(src,'file')==2,'C++ source not installed at expected reference path.');
    mex('-R2018a','-outdir',folder,src);
    clear TRIGGER_SUPPORT_UPPER_NATIVE
    rehash;
    R.mex_path=which('TRIGGER_SUPPORT_UPPER_NATIVE');
    R.source_path=src;
    assert(exist('TRIGGER_SUPPORT_UPPER_NATIVE','file')==3, ...
        'Compiled MEX not visible on MATLAB path; add the reference root.');
    R.status='PASS_MEX_COMPILED__NOT_GEOMETRY_PARITY';
catch ME
    R.status='FAIL_MEX_BUILD';
    R.reason=[ME.identifier ' | ' ME.message];
end
fprintf('%s\n%s\n',R.status,R.reason);
end
