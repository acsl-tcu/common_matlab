function S=AF7R58R110_SOURCE_CHECK()
% Read-only exact-source guard. R109 must have been installed.
root=fileparts(mfilename('fullpath'));
a=fullfile(root,'REPLANNING_SYSTEM.m');
b=fullfile(root,'REPLANNING_SYSTEM_R110_PATCHED_SOURCE.txt');
S=struct('status','NOT_READY','source_exists',isfile(a), ...
    'patch_exists',isfile(b),'r109_sha256','ae826711a6d79e9ed4c25e39d294f55308b436dc935b93ae6e8cf8666693c82a', ...
    'r110_sha256','599cc724bafdc3a8b2b99953cededdbcbc37f896d1a2fbabde2ff1c033d7df76','active_sha256','', ...
    'patch_sha256','','installable',false,'already_installed',false);
if ~S.source_exists || ~S.patch_exists
    S.status='R110_REQUIRED_SOURCE_MISSING';return;
end
S.active_sha256=AF7R58R110_HASH_FILE(a);
S.patch_sha256=AF7R58R110_HASH_FILE(b);
if ~strcmp(S.patch_sha256,S.r110_sha256)
    S.status='R110_PATCH_SOURCE_CHANGED';return;
end
S.already_installed=strcmp(S.active_sha256,S.r110_sha256);
S.installable=strcmp(S.active_sha256,S.r109_sha256);
if S.already_installed
    S.status='R110_ALREADY_INSTALLED';
elseif S.installable
    S.status='PASS_R110_R109_SOURCE_PINNED';
else
    S.status='R110_UNEXPECTED_ACTIVE_SOURCE__NOT_INSTALLED';
end
end
