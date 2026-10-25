function S=AF7R58R106_SOURCE_CHECK()
% Fail closed unless exact R88R2 production source or audited R106 source.
root=fileparts(mfilename('fullpath'));
original='7d9502ca5d8db06f089c534bcdffb07d184b731a29266ffc52a7859a0c3c8621';
patched='6d2e806a06d93265f771b2850d6911cd5106208a2c3fd40d1191e2906a6abedb';
need={'AF7R58R106_SOURCE_CHECK','AF7R58R106_SHA256_FILE', ...
    'AF7R58R106_INSTALL','AF7R58R106_SELFTEST', ...
    'AF7R58R106_PACK_EKF','AF7R58R106_CAPTURE_STATUS', ...
    'AF7R58R106_WRITE_REPORT', ...
    'AF7R58R106_SOURCE_EVENT','AF7R58R106_STAGE_REPLAY', ...
    'AF7R58R105_CLOSE_PARTIAL','AF7R58R103_EKF_HLC_TRIGGER'};
missing={};different={};
for j=1:numel(need)
    p=which(need{j});
    if isempty(p)
        missing{end+1}=need{j}; %#ok<AGROW>
    elseif ~strcmpi(char(p),fullfile(root,[need{j} '.m']))
        different{end+1}=need{j}; %#ok<AGROW>
    end
end
src=fullfile(root,'REPLANNING_SYSTEM_R106_PATCHED_SOURCE.txt');
target=fullfile(root,'REPLANNING_SYSTEM.m');
srcHash='';currentHash='';
if isfile(src),srcHash=AF7R58R106_SHA256_FILE(src);end
if isfile(target),currentHash=AF7R58R106_SHA256_FILE(target);end
okSrc=strcmp(srcHash,patched);
status='R106_BLOCKED_MISSING_OR_SHADOW_DEPENDENCIES';
if isempty(missing)&&isempty(different)&&okSrc
    if strcmp(currentHash,original)
        status='PASS_R106_PINNED_INSTALL_READY';
    elseif strcmp(currentHash,patched)
        status='PASS_R106_PINNED_ALREADY_INSTALLED';
    else
        status='R106_BLOCKED_UNEXPECTED_LIVE_CLASS_SHA';
    end
elseif ~okSrc
    status='R106_BLOCKED_PATCHED_CLASS_SOURCE_SHA';
end
S=struct('status',status,'missing_dependencies',{missing}, ...
    'different_source_directory',{different}, ...
    'baseline_sha256',original,'patched_sha256',patched, ...
    'class_current_sha256',currentHash, ...
    'patch_source_sha256',srcHash, ...
    'patch_bytes_verified',okSrc, ...
    'already_installed',strcmp(currentHash,patched), ...
    'installable',strcmp(currentHash,original)&& ...
        isempty(missing)&&isempty(different)&&okSrc, ...
    'trigger_switched',false,'generator_switched',false, ...
    'closed_loop_certified',false);
end
