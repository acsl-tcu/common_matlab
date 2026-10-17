function OUT = AF7R53_BUILD_PREPARED_COMMIT_MEX()
clc;
fprintf('\n====================================================================================================\n');
fprintf(' AF7R53 | BUILD PREPARED-COMMIT MEX\n');
fprintf('====================================================================================================\n');
fprintf('[AUTHORITY] sameR52PreparedMath=1 | productionSourceModified=0\n');

here=fileparts(mfilename('fullpath'));
cpp=fullfile(here,'af7r53_prepared_commit_mex.cpp');
assert(isfile(cpp),'[AF7R53] prepared MEX source missing.');
out=fullfile(here,['af7r53_prepared_commit_mex.' mexext]);
try, eval('clear af7r53_prepared_commit_mex'); catch, end
if isfile(out), delete(out); end

if ispc
    mex('-R2018a','-O','COMPFLAGS="$COMPFLAGS /std:c++17"', ...
        '-output','af7r53_prepared_commit_mex','-outdir',here,cpp);
else
    mex('-R2018a','-O','CXXFLAGS="$CXXFLAGS -std=c++17"', ...
        '-output','af7r53_prepared_commit_mex','-outdir',here,cpp);
end
rehash;
assert(isfile(out),'[AF7R53] prepared MEX build failed.');

fprintf(' prepared MEX                            = %s\n',out);
fprintf(' production source modified             = 0\n');
fprintf('====================================================================================================\n');
OUT=struct('schema','AF7R53_BUILD_PREPARED_MEX_V1','built',true,'mex',out);
assignin('base','AF7R53_BUILD_PREPARED_MEX_OUT',OUT);
end
