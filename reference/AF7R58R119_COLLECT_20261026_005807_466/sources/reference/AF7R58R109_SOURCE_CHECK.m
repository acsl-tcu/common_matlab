function S=AF7R58R109_SOURCE_CHECK()
% Source-pinned upgrade from the installed R108 main class. No Flight work.
root=fileparts(mfilename('fullpath'));
a=fullfile(root,'REPLANNING_SYSTEM.m');
b=fullfile(root,'REPLANNING_SYSTEM_R109_PATCHED_SOURCE.txt');
S=struct('status','SOURCE_NOT_MATCHED','source_exists',isfile(a), ...
    'patch_exists',isfile(b),'r108_sha256','09025f4842af7f540ce24bbc286296e11048c6eae1948aa71135607c808f3720', ...
    'r109_sha256','ae826711a6d79e9ed4c25e39d294f55308b436dc935b93ae6e8cf8666693c82a', ...
    'active_sha256','','patch_sha256','', ...
    'installable',false,'already_installed',false);
if ~(S.source_exists && S.patch_exists)
    S.status='SOURCE_OR_PATCH_MISSING';return;
end
S.active_sha256=AF7R58R109_HASH_FILE(a);
S.patch_sha256=AF7R58R109_HASH_FILE(b);
if ~strcmp(S.patch_sha256,S.r109_sha256)
    S.status='PATCH_BYTES_MODIFIED';return;
end
S.already_installed=strcmp(S.active_sha256,S.r109_sha256);
S.installable=strcmp(S.active_sha256,S.r108_sha256);
if S.already_installed
    S.status='R109_ALREADY_INSTALLED';
elseif S.installable
    S.status='PASS_R109_R108_SOURCE_PINNED';
else
    S.status='ACTIVE_CLASS_SOURCE_DRIFT';
end
end
