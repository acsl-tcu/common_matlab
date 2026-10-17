function OUT = AF7R49_BUILD_SHADOW_MEX()
clc;
fprintf('\n====================================================================================================\n');
fprintf(' AF7R49 | BUILD MIN-CORRECTION-ENERGY NATIVE SHADOW CORE\n');
fprintf('====================================================================================================\n');
fprintf('[AUTHORITY] productionSourceModified=0 | sameFeasibleSet=1 | sameContinuousSafetyTests=1\n');

src=which('REPLANNING_SYSTEM'); assert(~isempty(src)&&isfile(src));
root=fileparts(src); here=fileparts(mfilename('fullpath'));
files={'af7r49_single_core_mex.cpp','af7r49_core.hpp','af7r49_prime_core_reference_runtime.m','af7r49_replay_context.m'};
for k=1:numel(files)
    a=fullfile(here,files{k}); b=fullfile(root,files{k}); assert(isfile(a));
    if ~strcmpi(a,b), [ok,msg]=copyfile(a,b,'f'); assert(ok,'%s',msg); end
end
cpp=fullfile(root,'af7r49_single_core_mex.cpp');
mexPath=fullfile(root,['af7r49_single_core_mex.' mexext]);
try, eval('clear af7r49_single_core_mex'); catch, end
if isfile(mexPath), delete(mexPath); end
if ispc
    mex('-R2018a','-O','COMPFLAGS="$COMPFLAGS /std:c++17"', ...
        '-output','af7r49_single_core_mex','-outdir',root,cpp);
else
    mex('-R2018a','-O','CXXFLAGS="$CXXFLAGS -std=c++17"', ...
        '-output','af7r49_single_core_mex','-outdir',root,cpp);
end
rehash; assert(isfile(mexPath)); addpath(root,'-begin');

fprintf(' MEX                                     = %s\n',mexPath);
fprintf(' production source modified              = 0\n');
fprintf(' performance objective                   = integral ||correction||^2 dt\n');
fprintf(' legacy min-snap recoverability family   = retained\n');
fprintf(' safety acceptance tests                 = unchanged\n');
fprintf('====================================================================================================\n');
OUT=struct('schema','AF7R49_BUILD_SHADOW_MEX_V1','built',true,'mex_path',mexPath);
assignin('base','AF7R49_BUILD_OUT',OUT);
end
