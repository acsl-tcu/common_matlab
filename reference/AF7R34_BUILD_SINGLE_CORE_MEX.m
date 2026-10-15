function OUT = AF7R34_BUILD_SINGLE_CORE_MEX()
clc;
fprintf('\n====================================================================================================\n');
fprintf(' AF7R34 | BUILD R14 SINGLE-CALL NATIVE REPLANNING CORE\n');
fprintf('====================================================================================================\n');
fprintf('[AUTHORITY] shadowFirst=1 | theoremContractFrozen=1 | productionModification=0\n');

src=which('REPLANNING_SYSTEM');
assert(~isempty(src)&&isfile(src),'[AF7R34] REPLANNING_SYSTEM unresolved.');
root=fileparts(src);
here=fileparts(mfilename('fullpath'));
cppSrc=fullfile(here,'af7r34_single_core_mex.cpp');
hppSrc=fullfile(here,'af7r34_core.hpp');
assert(isfile(cppSrc)&&isfile(hppSrc),'[AF7R34] C++ source/header missing.');

cppDst=fullfile(root,'af7r34_single_core_mex.cpp');
hppDst=fullfile(root,'af7r34_core.hpp');
if ~strcmpi(cppSrc,cppDst)
    [ok,msg]=copyfile(cppSrc,cppDst,'f'); assert(ok,'[AF7R34] cpp copy failed: %s',msg);
else
    cppDst=cppSrc;
end
if ~strcmpi(hppSrc,hppDst)
    [ok,msg]=copyfile(hppSrc,hppDst,'f'); assert(ok,'[AF7R34] hpp copy failed: %s',msg);
else
    hppDst=hppSrc;
end

mexPath=fullfile(root,['af7r34_single_core_mex.' mexext]);
if isfile(mexPath), delete(mexPath); end

if ispc
    mex('-R2018a','-O','COMPFLAGS="$COMPFLAGS /std:c++17"', ...
        '-outdir',root,cppDst);
else
    mex('-R2018a','-O','CXXFLAGS="$CXXFLAGS -std=c++17"', ...
        '-outdir',root,cppDst);
end
rehash;
assert(isfile(mexPath),'[AF7R34] single-core MEX build failed.');
addpath(root,'-begin');

OUT=struct('schema','AF7R34_BUILD_SINGLE_CORE_MEX_V1', ...
    'mex_path',mexPath,'built',true,'production_modified',false);
assignin('base','AF7R34_BUILD_SINGLE_CORE_MEX_OUT',OUT);

fprintf(' MEX                                     = %s\n',mexPath);
fprintf(' build PASS                              = 1\n');
fprintf(' production source modified              = 0\n');
fprintf('====================================================================================================\n');
end
