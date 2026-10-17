function OUT = AF7R55_BUILD_CHAIN_MEX()
clc;
fprintf('\n====================================================================================================\n');
fprintf(' AF7R55A | BUILD IMMUTABLE CORRECTION-CHAIN SHADOW MEX\n');
fprintf('====================================================================================================\n');
here=fileparts(mfilename('fullpath'));cpp=fullfile(here,'af7r55_chain_mex.cpp');
assert(isfile(cpp));
try, eval('clear af7r55_chain_mex'); catch, end
if ispc
    mex('-R2018a','-O','COMPFLAGS="$COMPFLAGS /std:c++17"', ...
        '-output','af7r55_chain_mex','-outdir',here,cpp);
else
    mex('-R2018a','-O','CXXFLAGS="$CXXFLAGS -std=c++17"', ...
        '-output','af7r55_chain_mex','-outdir',here,cpp);
end
rehash;assert(exist('af7r55_chain_mex','file')==3);
fprintf(' chain MEX                               = %s\n',which('af7r55_chain_mex'));
fprintf(' production source modified             = 0\n');
fprintf('====================================================================================================\n');
OUT=struct('schema','AF7R55A_BUILD_V1','closed',true);
assignin('base','AF7R55A_BUILD_OUT',OUT);
end
