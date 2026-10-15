function OUT = AF7R30_BUILD_R12_NATIVE_MEX()
clc;
fprintf('\n====================================================================================================\n');
fprintf(' AF7R30 | BUILD R12 NATIVE SUPPORT/HULL MEX\n');
fprintf('====================================================================================================\n');

src=which('REPLANNING_SYSTEM');
assert(~isempty(src)&&isfile(src),'[AF7R30] REPLANNING_SYSTEM.m not found.');
root=fileparts(src);
here=fileparts(mfilename('fullpath'));
cppSrc=fullfile(here,'af7r30_support_hull_mex.cpp');
assert(isfile(cppSrc),'[AF7R30] C++ source missing next to builder.');

cppDst=fullfile(root,'af7r30_support_hull_mex.cpp');
copyfile(cppSrc,cppDst);
mex('-R2018a','-O','-outdir',root,cppDst);
rehash;

mexPath=fullfile(root,['af7r30_support_hull_mex.' mexext]);
assert(isfile(mexPath),'[AF7R30] MEX build failed.');
addpath(root,'-begin');

R=eye(3); c=[0;0;0]; a=[1;2;3];
af7r30_support_hull_mex(0,[4;0;0],c,R,a,0.1);
af7r30_support_hull_mex(1,[4;0;0],[4;1;0],c,R,a,0.1);
af7r30_support_hull_mex(2,[0;1;2],[1;0;1]);

OUT=struct('schema','AF7R30_BUILD_R12_NATIVE_MEX_V1', ...
    'mex_path',mexPath,'built',true);
assignin('base','AF7R30_BUILD_R12_NATIVE_MEX_OUT',OUT);
fprintf(' MEX                                     = %s\n',mexPath);
fprintf(' build/warmup PASS                       = 1\n');
fprintf('====================================================================================================\n');
end
