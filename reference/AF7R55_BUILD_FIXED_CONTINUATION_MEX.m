function OUT = AF7R55_BUILD_FIXED_CONTINUATION_MEX()
clc;
fprintf('\n====================================================================================================\n');
fprintf(' AF7R55B | BUILD FIXED-CORRIDOR CONTINUATION SHADOW MEX\n');
fprintf('====================================================================================================\n');
fprintf('[AUTHORITY] productionModified=0 | fixedEpisodeTailTheoremRequired=1 | sampledAcceptance=0\n');

here=fileparts(mfilename('fullpath'));
cpp=fullfile(here,'af7r55_fixed_continuation_mex.cpp');
assert(isfile(cpp));
try, eval('clear af7r55_fixed_continuation_mex'); catch, end
if ispc
    mex('-R2018a','-O','COMPFLAGS="$COMPFLAGS /std:c++17"', ...
        ['-I' here],'-output','af7r55_fixed_continuation_mex','-outdir',here,cpp);
else
    mex('-R2018a','-O','CXXFLAGS="$CXXFLAGS -std=c++17"', ...
        ['-I' here],'-output','af7r55_fixed_continuation_mex','-outdir',here,cpp);
end
rehash;assert(exist('af7r55_fixed_continuation_mex','file')==3);
fprintf(' fixed continuation MEX                  = %s\n',which('af7r55_fixed_continuation_mex'));
fprintf(' production source modified             = 0\n');
fprintf('====================================================================================================\n');
OUT=struct('schema','AF7R55B_BUILD_V1','closed',true);
assignin('base','AF7R55B_BUILD_OUT',OUT);
end
