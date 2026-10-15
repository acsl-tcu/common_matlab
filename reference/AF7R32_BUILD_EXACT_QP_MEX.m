function OUT = AF7R32_BUILD_EXACT_QP_MEX()
clc;
fprintf('\n====================================================================================================\n');
fprintf(' AF7R32 | BUILD R13B EXACT SMALL-QP SHADOW MEX\n');
fprintf('====================================================================================================\n');
fprintf('[AUTHORITY] synthesisShadowOnly=1 | noHeuristicNodeCap=1 | productionModification=0\n');

src=which('REPLANNING_SYSTEM');
assert(~isempty(src)&&isfile(src),'[AF7R32] REPLANNING_SYSTEM unresolved.');
root=fileparts(src);
here=fileparts(mfilename('fullpath'));
cppSrc=fullfile(here,'af7r32_exact_qp_mex.cpp');
assert(isfile(cppSrc),'[AF7R32] C++ source missing.');
cppDst=fullfile(root,'af7r32_exact_qp_mex.cpp');
if ~strcmpi(cppSrc,cppDst)
    [ok,msg]=copyfile(cppSrc,cppDst,'f');
    assert(ok,'[AF7R32] copy failed: %s',msg);
else
    cppDst=cppSrc;
end
mexPath=fullfile(root,['af7r32_exact_qp_mex.' mexext]);
if isfile(mexPath), delete(mexPath); end
mex('-R2018a','-O','-outdir',root,cppDst);
rehash;
assert(isfile(mexPath),'[AF7R32] QP MEX build failed.');
addpath(root,'-begin');

H=eye(2); A=[1 0;0 1]; b=[1;2];
[z,~,~,ok]=af7r32_exact_qp_mex(H,A,b);
assert(ok&&max(abs(z-[1;2]))<1e-12,'[AF7R32] QP warmup self-test failed.');

OUT=struct('schema','AF7R32_BUILD_EXACT_QP_MEX_V1', ...
    'mex_path',mexPath,'built',true,'production_modified',false);
assignin('base','AF7R32_BUILD_EXACT_QP_MEX_OUT',OUT);
fprintf(' MEX                                     = %s\n',mexPath);
fprintf(' build/self-test PASS                    = 1\n');
fprintf('====================================================================================================\n');
end
