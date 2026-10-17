function OUT = AF7R53_BUILD_INTERVAL_PROOF_MEX()
clc;
fprintf('\n====================================================================================================\n');
fprintf(' AF7R53B | BUILD ALL-CANDIDATE + INTERVAL-KKT SHADOW MEX\n');
fprintf('====================================================================================================\n');
here=fileparts(mfilename('fullpath'));
a=fullfile(here,'af7r53_allcandidate_mex.cpp');
b=fullfile(here,'af7r53_interval_kkt_mex.cpp');
assert(isfile(a)&&isfile(b));

try, eval('clear af7r53_allcandidate_mex af7r53_interval_kkt_mex'); catch, end
if ispc
    mex('-R2018a','-O','COMPFLAGS="$COMPFLAGS /std:c++17"', ...
        ['-I' here],'-output','af7r53_allcandidate_mex','-outdir',here,a);
    mex('-R2018a','-O','COMPFLAGS="$COMPFLAGS /std:c++17"', ...
        '-output','af7r53_interval_kkt_mex','-outdir',here,b);
else
    mex('-R2018a','-O','CXXFLAGS="$CXXFLAGS -std=c++17"', ...
        ['-I' here],'-output','af7r53_allcandidate_mex','-outdir',here,a);
    mex('-R2018a','-O','CXXFLAGS="$CXXFLAGS -std=c++17"', ...
        '-output','af7r53_interval_kkt_mex','-outdir',here,b);
end
rehash;
assert(exist('af7r53_allcandidate_mex','file')==3);
assert(exist('af7r53_interval_kkt_mex','file')==3);
fprintf(' all-candidate MEX                       = %s\n',which('af7r53_allcandidate_mex'));
fprintf(' interval-KKT MEX                        = %s\n',which('af7r53_interval_kkt_mex'));
fprintf(' production source modified             = 0\n');
fprintf('====================================================================================================\n');
OUT=struct('schema','AF7R53B_BUILD_V1','closed',true);
assignin('base','AF7R53B_BUILD_OUT',OUT);
end
