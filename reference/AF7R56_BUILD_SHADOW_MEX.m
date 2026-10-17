function OUT = AF7R56_BUILD_SHADOW_MEX()
clc;
fprintf('\n====================================================================================================\n');
fprintf(' AF7R56 | BUILD SINGLE-TOKEN + STAGE-B TRACE SHADOW MEX\n');
fprintf('====================================================================================================\n');
fprintf('[AUTHORITY] shadowOnly=1 | productionModified=0\n');

here=fileparts(mfilename('fullpath'));
c1=fullfile(here,'af7r55_chain_mex.cpp');
c2=fullfile(here,'af7r56_trace_mex.cpp');
c3=fullfile(here,'af7r54_interval_kkt_mex.cpp');
assert(isfile(c1)&&isfile(c2)&&isfile(c3));

try, eval('clear af7r56_chain_mex af7r56_trace_mex af7r56_interval_kkt_mex'); catch, end
if ispc
    mex('-R2018a','-O','COMPFLAGS="$COMPFLAGS /std:c++17"', ...
        '-output','af7r56_chain_mex','-outdir',here,c1);
    mex('-R2018a','-O','COMPFLAGS="$COMPFLAGS /std:c++17"', ...
        ['-I' here],'-output','af7r56_trace_mex','-outdir',here,c2);
    mex('-R2018a','-O','COMPFLAGS="$COMPFLAGS /std:c++17"', ...
        '-output','af7r56_interval_kkt_mex','-outdir',here,c3);
else
    mex('-R2018a','-O','CXXFLAGS="$CXXFLAGS -std=c++17"', ...
        '-output','af7r56_chain_mex','-outdir',here,c1);
    mex('-R2018a','-O','CXXFLAGS="$CXXFLAGS -std=c++17"', ...
        ['-I' here],'-output','af7r56_trace_mex','-outdir',here,c2);
    mex('-R2018a','-O','CXXFLAGS="$CXXFLAGS -std=c++17"', ...
        '-output','af7r56_interval_kkt_mex','-outdir',here,c3);
end
rehash;

assert(exist('af7r56_chain_mex','file')==3);
assert(exist('af7r56_trace_mex','file')==3);
assert(exist('af7r56_interval_kkt_mex','file')==3);

fprintf(' chain MEX                               = %s\n',which('af7r56_chain_mex'));
fprintf(' Stage-B trace MEX                       = %s\n',which('af7r56_trace_mex'));
fprintf(' interval KKT MEX                        = %s\n',which('af7r56_interval_kkt_mex'));
fprintf(' production source modified             = 0\n');
fprintf('====================================================================================================\n');

OUT=struct('schema','AF7R56_BUILD_SHADOW_MEX_V1','closed',true);
assignin('base','AF7R56_BUILD_OUT',OUT);
end
