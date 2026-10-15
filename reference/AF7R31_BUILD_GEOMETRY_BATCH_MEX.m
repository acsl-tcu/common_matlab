function OUT = AF7R31_BUILD_GEOMETRY_BATCH_MEX()
clc;
fprintf('\n====================================================================================================\n');
fprintf(' AF7R31 | BUILD R13 BATCH GEOMETRY MEX\n');
fprintf('====================================================================================================\n');
src=which('REPLANNING_SYSTEM');
assert(~isempty(src)&&isfile(src),'[AF7R31] REPLANNING_SYSTEM source unresolved.');
root=fileparts(src);
here=fileparts(mfilename('fullpath'));
cppSrc=fullfile(here,'af7r31_geometry_batch_mex.cpp');
assert(isfile(cppSrc),'[AF7R31] C++ source missing.');
cppDst=fullfile(root,'af7r31_geometry_batch_mex.cpp');
if ~strcmpi(cppSrc,cppDst)
    [ok,msg]=copyfile(cppSrc,cppDst,'f');
    assert(ok,'[AF7R31] C++ copy failed: %s',msg);
else
    cppDst=cppSrc;
end
mexPath=fullfile(root,['af7r31_geometry_batch_mex.' mexext]);
if isfile(mexPath), delete(mexPath); end
mex('-R2018a','-O','-outdir',root,cppDst);
rehash;
assert(isfile(mexPath),'[AF7R31] geometry MEX build failed.');
addpath(root,'-begin');
pL=[3;0;0]; pQ=[3;0;1]; h=0.01; c=[0;0;0]; R=eye(3); a=[1;1;1];
af7r31_geometry_batch_mex(0,pL,pQ,h,c,R,a,0.2,0.2,0.1,1,1);
af7r31_geometry_batch_mex(1,pL,pQ,h,c,R,a,[1;0;0],0.2,0.2,0.1,1,1);
af7r31_geometry_batch_mex(2,pL,pQ,h,c,reshape(R,9,1),a,0.2,0.2,0.1,1,1);
OUT=struct('schema','AF7R31_BUILD_GEOMETRY_BATCH_MEX_V1','mex_path',mexPath,'built',true);
assignin('base','AF7R31_BUILD_GEOMETRY_BATCH_MEX_OUT',OUT);
fprintf(' MEX                                     = %s\n',mexPath);
fprintf(' build/warmup PASS                       = 1\n');
fprintf('====================================================================================================\n');
end
