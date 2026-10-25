function S=AF7R58R114_SOURCE_CHECK()
% Compare exact production sources, not old R113 scripts.
root=fileparts(mfilename('fullpath'));
S=struct('status','NOT_CHECKED','installable',false, ...
  'already_installed',false,'main_source_unchanged',false, ...
  'source_hpp_sha','','source_cpp_sha','', ...
  'patch_hpp_sha','','patch_cpp_sha','');
main=fullfile(root,'REPLANNING_SYSTEM.m');
hpp=fullfile(root,'af7r36_core.hpp');
cpp=fullfile(root,'af7r36_single_core_mex.cpp');
newH=fullfile(root,'AF7R58R114_PATCHED_CORE.hpp');
newC=fullfile(root,'AF7R58R114_PATCHED_MEX.cpp');
assert(isfile(main) && isfile(hpp) && isfile(cpp) && ...
       isfile(newH) && isfile(newC), 'R114:MissingSource', ...
       'Production files or the R114 patch files are missing.');
Hmain=AF7R58R114_HASH_FILE(main);
Horig='8ba9621932ab35c775f0d0fd8468edf100cb7f44dc1f78fff86c658490ee3676';
HbaseH='f599e2b769e4279c7938337474ed81dac2cc760399f86ee2c775915104b46669';
HbaseC='94438bf9245297613e8acdd06e674eb142d16448cbf409292e770e15e408f8ce';
Hpatch='b1ad728e77e0aa48616c746d5466eb85b5f0c1fa72f00a7bb9095ef7495a3df7';
Cpatch='21562ab49734c8c76ac98fdf94b7f3981063b2e929b484f763259866dce1154e';
S.source_hpp_sha=AF7R58R114_HASH_FILE(hpp);
S.source_cpp_sha=AF7R58R114_HASH_FILE(cpp);
S.patch_hpp_sha=AF7R58R114_HASH_FILE(newH);
S.patch_cpp_sha=AF7R58R114_HASH_FILE(newC);
S.main_source_unchanged=strcmp(Hmain,Horig);
assert(S.main_source_unchanged, 'R114:MainChanged', ...
       'The production REPLANNING_SYSTEM.m differs from the uploaded R111. Nothing is modified.');
assert(strcmp(S.patch_hpp_sha,Hpatch) && strcmp(S.patch_cpp_sha,Cpatch), ...
       'R114:PatchTampered','The bundled R114 source hashes differ.');
S.already_installed=strcmp(S.source_hpp_sha,Hpatch) && strcmp(S.source_cpp_sha,Cpatch);
S.installable=strcmp(S.source_hpp_sha,HbaseH) && strcmp(S.source_cpp_sha,HbaseC);
if S.already_installed
    S.status='PASS_R114_PRODUCTION_SOURCES_PATCHED';
elseif S.installable
    S.status='PASS_R114_R111_UPLOADED_SOURCE_EXACT_PINNED';
else
    S.status='R114_SOURCE_DRIFT__NOT_INSTALLABLE';
end
end
