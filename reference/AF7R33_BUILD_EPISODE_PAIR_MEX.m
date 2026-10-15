function OUT = AF7R33_BUILD_EPISODE_PAIR_MEX()
clc;
fprintf('\n====================================================================================================\n');
fprintf(' AF7R33 | BUILD ALL-OBSTACLE PAIRED EPISODE MEX\n');
fprintf('====================================================================================================\n');
src=which('REPLANNING_SYSTEM'); assert(~isempty(src)&&isfile(src));
root=fileparts(src); here=fileparts(mfilename('fullpath'));
cppSrc=fullfile(here,'af7r33_episode_pair_mex.cpp');
assert(isfile(cppSrc));
cppDst=fullfile(root,'af7r33_episode_pair_mex.cpp');
if ~strcmpi(cppSrc,cppDst)
    [ok,msg]=copyfile(cppSrc,cppDst,'f'); assert(ok,'%s',msg);
else
    cppDst=cppSrc;
end
mexPath=fullfile(root,['af7r33_episode_pair_mex.' mexext]);
if isfile(mexPath), delete(mexPath); end
mex('-R2018a','-O','-outdir',root,cppDst);
rehash; assert(isfile(mexPath));
addpath(root,'-begin');

pL=[3 4;0 0;0 0]; pQ=[3 4;0 0;1 1]; h=[.01 .01];
C=[0 10;0 0;0 0]; RF=[reshape(eye(3),9,1),reshape(eye(3),9,1)];
AX=[1 1;1 1;1 1]; oi=[1 2];
af7r33_episode_pair_mex(pL,pQ,h,C,RF,AX,oi,.2,.2,.1,1,1);

OUT=struct('schema','AF7R33_BUILD_EPISODE_PAIR_MEX_V1', ...
    'mex_path',mexPath,'built',true);
assignin('base','AF7R33_BUILD_EPISODE_PAIR_MEX_OUT',OUT);
fprintf(' MEX                                     = %s\n',mexPath);
fprintf(' build/warmup PASS                       = 1\n');
fprintf('====================================================================================================\n');
end
