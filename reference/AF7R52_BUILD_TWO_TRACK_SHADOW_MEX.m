function OUT = AF7R52_BUILD_TWO_TRACK_SHADOW_MEX()
clc;
fprintf('\n====================================================================================================\n');
fprintf(' AF7R52 | BUILD TWO-TRACK SHADOW MEX\n');
fprintf('====================================================================================================\n');
fprintf('[AUTHORITY] productionSourceModified=0 | branchMathSameAsAF7R36=1 | shadowOnly=1\n');

src=which('REPLANNING_SYSTEM');
assert(~isempty(src)&&isfile(src),'[AF7R52] REPLANNING_SYSTEM unresolved.');
here=fileparts(mfilename('fullpath'));
addpath(here,'-begin');

cpp1=fullfile(here,'af7r52_prepared_commit_mex.cpp');
cpp2=fullfile(here,'af7r52_branch_mex.cpp');
hpp =fullfile(here,'af7r52_branch_core.hpp');
assert(isfile(cpp1)&&isfile(cpp2)&&isfile(hpp));

out1=fullfile(here,['af7r52_prepared_commit_mex.' mexext]);
out2=fullfile(here,['af7r52_branch_mex.' mexext]);
try, eval('clear af7r52_prepared_commit_mex af7r52_branch_mex'); catch, end
if isfile(out1), delete(out1); end
if isfile(out2), delete(out2); end

if ispc
    mex('-R2018a','-O','COMPFLAGS="$COMPFLAGS /std:c++17"', ...
        ['-I' here],'-output','af7r52_prepared_commit_mex','-outdir',here,cpp1);
    mex('-R2018a','-O','COMPFLAGS="$COMPFLAGS /std:c++17"', ...
        ['-I' here],'-output','af7r52_branch_mex','-outdir',here,cpp2);
else
    mex('-R2018a','-O','CXXFLAGS="$CXXFLAGS -std=c++17"', ...
        ['-I' here],'-output','af7r52_prepared_commit_mex','-outdir',here,cpp1);
    mex('-R2018a','-O','CXXFLAGS="$CXXFLAGS -std=c++17"', ...
        ['-I' here],'-output','af7r52_branch_mex','-outdir',here,cpp2);
end
rehash;
assert(isfile(out1)&&isfile(out2),'[AF7R52] shadow MEX build incomplete.');

fprintf(' prepared-commit MEX                    = %s\n',out1);
fprintf(' branch/KKT MEX                          = %s\n',out2);
fprintf(' production REPLANNING_SYSTEM modified  = 0\n');
fprintf('====================================================================================================\n');

OUT=struct('schema','AF7R52_BUILD_TWO_TRACK_SHADOW_MEX_V1', ...
    'built',true,'prepared_mex',out1,'branch_mex',out2,'production_modified',false);
assignin('base','AF7R52_BUILD_OUT',OUT);
end
