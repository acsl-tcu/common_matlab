function R=BUILD_V8_NATIVE_MEX()
%BUILD_V8_NATIVE_MEX Compile optional standalone C broadphase MEX.
% Standard MATLAB mex compiler; MATLAB Coder license is NOT required.
R=struct('status','NOT_RUN','reason','', ...
    'fullSelected',struct('status','NOT_RUN'));
if exist('mex','file')~=2 && exist('mex','builtin')~=5
    R.status='SKIPPED_NO_MEX_COMPILER_INTERFACE';return
end
src=fullfile(fileparts(mfilename('fullpath')), ...
    'wp_six_trigger_C_chain_numeric_mex_v8.c');
out=fullfile(fileparts(mfilename('fullpath')),'MEX_BUILD');
if ~isfolder(out),mkdir(out);end
try
    mex('-R2018a','-outdir',out,'-output', ...
        'wp_six_trigger_C_chain_numeric_mex_v8',src);
    addpath(out);
    R.numericParity=TEST_TRIGGER_V8_MEX_NUMERIC_PARITY(250);
    R.fullSelected=BENCH_TRIGGER_V8_FULL_SELECTED_MEX(60);
    if R.fullSelected.classificationAgrees
        R.status='COMPILED_NATIVE_MEX_AND_FULL_ONE_MODE_PARITY';
    else
        R.status='COMPILED_BUT_FULL_ONE_MODE_PARITY_FAILED';
    end
catch ME
    R.status='BUILD_FAILED';
    R.reason=[ME.identifier ': ' ME.message];
end
end
