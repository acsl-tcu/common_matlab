function OUT = AF7R36_BUILD_CANONICAL_SINGLE_CORE_MEX()
clc;
fprintf('\n====================================================================================================\n');
fprintf(' AF7R36 | BUILD R14D CANONICAL-SEPARATOR SINGLE CORE\n');
fprintf('====================================================================================================\n');
fprintf('[AUTHORITY] shadowFirst=1 | canonicalDegenerateEigenspace=1 | productionModification=0\n');

src=which('REPLANNING_SYSTEM'); assert(~isempty(src)&&isfile(src));
root=fileparts(src); here=fileparts(mfilename('fullpath'));
cppSrc=fullfile(here,'af7r36_single_core_mex.cpp');
hppSrc=fullfile(here,'af7r36_core.hpp');
assert(isfile(cppSrc)&&isfile(hppSrc));

cppDst=fullfile(root,'af7r36_single_core_mex.cpp');
hppDst=fullfile(root,'af7r36_core.hpp');
if ~strcmpi(cppSrc,cppDst), [ok,msg]=copyfile(cppSrc,cppDst,'f'); assert(ok,'%s',msg); else, cppDst=cppSrc; end
if ~strcmpi(hppSrc,hppDst), [ok,msg]=copyfile(hppSrc,hppDst,'f'); assert(ok,'%s',msg); else, hppDst=hppSrc; end

mexPath=fullfile(root,['af7r36_single_core_mex.' mexext]);
if isfile(mexPath), delete(mexPath); end
if ispc
    mex('-R2018a','-O','COMPFLAGS="$COMPFLAGS /std:c++17"', ...
        '-output','af7r36_single_core_mex','-outdir',root,cppDst);
else
    mex('-R2018a','-O','CXXFLAGS="$CXXFLAGS -std=c++17"', ...
        '-output','af7r36_single_core_mex','-outdir',root,cppDst);
end
rehash; assert(isfile(mexPath));
addpath(root,'-begin');

OUT=struct('schema','AF7R36_BUILD_CANONICAL_SINGLE_CORE_MEX_V1', ...
    'built',true,'mex_path',mexPath,'production_modified',false);
assignin('base','AF7R36_BUILD_CANONICAL_SINGLE_CORE_MEX_OUT',OUT);
fprintf(' MEX                                     = %s\n',mexPath);
fprintf(' build PASS                              = 1\n');
fprintf(' production source modified              = 0\n');
fprintf('====================================================================================================\n');
end
