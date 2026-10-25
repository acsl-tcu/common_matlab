function S=AF7R58R111_SOURCE_CHECK()
% R110 can be untouched or have ONLY the user's R17 flag 0->1 edit.
% Refuse unknown local modifications instead of silently overwriting them.
root=fileparts(mfilename('fullpath'));
active=fullfile(root,'REPLANNING_SYSTEM.m');
files={...
    fullfile(root,'REPLANNING_SYSTEM_R111_FROM_R110_FLAG0.txt'),...
    fullfile(root,'REPLANNING_SYSTEM_R111_FROM_R110_FLAG1.txt')};
baseHash={...
    '599cc724bafdc3a8b2b99953cededdbcbc37f896d1a2fbabde2ff1c033d7df76',...
    'd7f11058c6fde3279e9b307c11ab9132be01fda4ecc8936b934ccdb7bccc5621'};
patchHash={...
    'e5d732c16dad14f2bc095a09a32d444c8a5e101d8120c338efa16c2b591a8faf',...
    '8ba9621932ab35c775f0d0fd8468edf100cb7f44dc1f78fff86c658490ee3676'};
S=struct('status','R111_REQUIRED_FILES_MISSING','installable',false,...
    'already_installed',false,'source_variant',0,'active_sha256','',...
    'expected_patch_sha256','','patch_path','',...
    'user_flag_preserved',false,'production_modified',false);
if ~isfile(active)||~all(cellfun(@isfile,files))
    return;
end
for k=1:2
    if ~strcmp(AF7R58R111_HASH_FILE(files{k}),patchHash{k})
        S.status='R111_PATCH_CONTENT_CHANGED';return;
    end
end
S.active_sha256=AF7R58R111_HASH_FILE(active);
for k=1:2
    if strcmp(S.active_sha256,baseHash{k}) || strcmp(S.active_sha256,patchHash{k})
        S.source_variant=k;
        S.expected_patch_sha256=patchHash{k};
        S.patch_path=files{k};
        S.installable=strcmp(S.active_sha256,baseHash{k});
        S.already_installed=strcmp(S.active_sha256,patchHash{k});
        S.user_flag_preserved=true;
        if S.already_installed
            S.status='R111_ALREADY_INSTALLED';
        elseif k==1
            S.status='PASS_R111_R110_PINNED_FLAG0';
        else
            S.status='PASS_R111_R110_PINNED_USER_FLAG1';
        end
        return;
    end
end
S.status='R111_UNKNOWN_LOCAL_SOURCE__NOT_INSTALLED';
end
