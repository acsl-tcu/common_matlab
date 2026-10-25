function S=AF7R58R115_SOURCE_CHECK()
% Exact original R111+R114 verification; no file is modified here.
root=fileparts(mfilename('fullpath'));
mp=fullfile(root,'REPLANNING_SYSTEM.m');
cp=fullfile(root,'af7r36_single_core_mex.cpp');
hp=fullfile(root,'af7r36_core.hpp');
assert(isfile(mp)&&isfile(cp)&&isfile(hp), ...
    'R115:RequiredSourceMissing','Production MATLAB/C++ sources are required.');
S=struct('status','SOURCE_MISMATCH','installable',false, ...
    'already_installed',false,'main_sha256','', 'cpp_sha256','', 'header_sha256','');
S.main_sha256=AF7R58R115_SHA256(mp);
S.cpp_sha256=AF7R58R115_SHA256(cp);
S.header_sha256=AF7R58R115_SHA256(hp);
assert(strcmp(S.header_sha256,'b1ad728e77e0aa48616c746d5466eb85b5f0c1fa72f00a7bb9095ef7495a3df7'), ...
    'R115:HeaderDrift','The active core header must match tested R114 exactly.');
S.installable=strcmp(S.main_sha256,'8ba9621932ab35c775f0d0fd8468edf100cb7f44dc1f78fff86c658490ee3676') && ...
    strcmp(S.cpp_sha256,'21562ab49734c8c76ac98fdf94b7f3981063b2e929b484f763259866dce1154e');
S.already_installed=strcmp(S.main_sha256,'5068b2138d5e6829b2bc6cbb300d99d65d5920b039c01cb79b57144237e6cc8b') && ...
    strcmp(S.cpp_sha256,'9c84ba1fe9185503167b3b3845a2cac893a647b1147f5690b4b8d4cd268cf0d5');
assert(S.installable||S.already_installed, ...
    'R115:SourceDrift','Exact R111+R114 pair is required. No automatic fallback.');
if S.installable
    S.status='PASS_R115_R111_AND_R114_EXACT_PINNED';
else
    S.status='PASS_R115_PRODUCTION_SOURCES_INSTALLED';
end
end
