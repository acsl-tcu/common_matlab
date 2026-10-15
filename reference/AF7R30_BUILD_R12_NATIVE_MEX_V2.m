function OUT = AF7R30_BUILD_R12_NATIVE_MEX_V2()
% AF7R30_BUILD_R12_NATIVE_MEX_V2
% Robust builder when this package is already placed in the same directory
% as REPLANNING_SYSTEM.m. In that case cppSrc == cppDst and copyfile must be
% skipped (MATLAB rejects source==destination).

clc;
fprintf('\n====================================================================================================\n');
fprintf(' AF7R30 V2 | BUILD R12 NATIVE SUPPORT/HULL MEX\n');
fprintf('====================================================================================================\n');

src=which('REPLANNING_SYSTEM');
assert(~isempty(src)&&isfile(src),'[AF7R30V2] REPLANNING_SYSTEM.m not found.');
root=fileparts(src);

here=fileparts(mfilename('fullpath'));
cppSrc=fullfile(here,'af7r30_support_hull_mex.cpp');
assert(isfile(cppSrc),'[AF7R30V2] C++ source missing next to builder.');

cppDst=fullfile(root,'af7r30_support_hull_mex.cpp');

sameLocation=strcmpi(char(cppSrc),char(cppDst));
if ~sameLocation
    [ok,msg]=copyfile(cppSrc,cppDst,'f');
    assert(ok,'[AF7R30V2] copy C++ source failed: %s',msg);
else
    % Package is already in the production reference directory.
    % Use the existing source in-place; no copy is required.
    cppDst=cppSrc;
end

% Remove an older binary so existence after mex proves this build generated it.
mexPath=fullfile(root,['af7r30_support_hull_mex.' mexext]);
if isfile(mexPath)
    delete(mexPath);
end

mex('-R2018a','-O','-outdir',root,cppDst);
rehash;
assert(isfile(mexPath),'[AF7R30V2] MEX build failed.');

addpath(root,'-begin');

% Warm exact code paths. This is implementation warmup only, not safety
% acceptance; full MATLAB/MEX equivalence is checked separately by R30A.
R=eye(3);
c=[0;0;0];
a=[1;2;3];
af7r30_support_hull_mex(0,[4;0;0],c,R,a,0.1);
af7r30_support_hull_mex(1,[4;0;0],[4;1;0],c,R,a,0.1);
af7r30_support_hull_mex(2,[0;1;2],[1;0;1]);

OUT=struct();
OUT.schema='AF7R30_BUILD_R12_NATIVE_MEX_V2';
OUT.source_cpp=cppDst;
OUT.mex_path=mexPath;
OUT.same_source_destination=logical(sameLocation);
OUT.built=true;
assignin('base','AF7R30_BUILD_R12_NATIVE_MEX_OUT',OUT);

fprintf(' production root                         = %s\n',root);
fprintf(' C++ source                              = %s\n',cppDst);
fprintf(' source already in production root       = %d\n',sameLocation);
fprintf(' MEX                                     = %s\n',mexPath);
fprintf(' build/warmup PASS                       = 1\n');
fprintf('====================================================================================================\n');
end
