function OUT=AF7R58ZZY_FINAL_ARCHIVE_LEGACY(apply)
% Consolidate historical generated Rxx artifacts after FINAL has been run.
% Default is dry-run. Runtime helpers af7r*.m/cpp/mex* are NEVER moved.
if nargin<1, apply=false; end
root=fileparts(mfilename('fullpath'));
archiveRoot=fullfile(root,'archive','AF7R58ZZY_R19_R33_HISTORY');

patterns={ ...
    'REPLANNING_SYSTEM_R*_*.m', ...
    'AF7R58ZZY_R*_INSTALL.m', ...
    'AF7R58ZZY_R*_SELFTEST.m', ...
    'AF7R58ZZY_R*_REPORT.m', ...
    'AF7R58ZZY_R*_RUN_CANONICAL.m', ...
    'AF7R58ZZY_R*_OUT.mat', ...
    'AF7R58ZZY_R*_REPORT.txt', ...
    'AF7R58ZZY_R*_PACKAGE.zip'};

files=struct('name',{},'folder',{});
for p=1:numel(patterns)
    q=dir(fullfile(root,patterns{p}));
    for k=1:numel(q)
        if q(k).isdir, continue; end
        files(end+1)=q(k); %#ok<AGROW>
    end
end
% unique full paths
full=cellfun(@(a,b)fullfile(a,b),{files.folder},{files.name},'UniformOutput',false);
[~,ia]=unique(lower(string(full)),'stable');
files=files(sort(ia));

fprintf('\n====================================================================================================\n');
fprintf(' AF7R58ZZY FINAL | LEGACY ARCHIVE %s\n',ternary(apply,'APPLY','DRY RUN'));
fprintf('====================================================================================================\n');
for k=1:numel(files)
    fprintf(' %s\n',fullfile(files(k).folder,files(k).name));
end
fprintf(' count = %d\n',numel(files));

OUT=struct('applied',logical(apply),'count',numel(files), ...
    'archive_root',archiveRoot,'files',{full});
if ~apply
    fprintf('No files moved. Run AF7R58ZZY_FINAL_ARCHIVE_LEGACY(true) after FINAL validation.\n');
    return;
end

% Require the final integrated source and a successful final report before move.
assert(isfile(fullfile(root,'reference','REPLANNING_SYSTEM.m')), ...
    '[FINAL ARCHIVE] production source missing.');
assert(evalin('base','exist(''AF7R58ZZY_FINAL_REPORT_OUT'',''var'')==1'), ...
    '[FINAL ARCHIVE] run FINAL_REPORT successfully before archiving.');
RR=evalin('base','AF7R58ZZY_FINAL_REPORT_OUT');
assert(isstruct(RR)&&isfield(RR,'closed')&&logical(RR.closed), ...
    '[FINAL ARCHIVE] final report is not closed.');

if ~isfolder(archiveRoot), mkdir(archiveRoot); end
for k=1:numel(files)
    src=fullfile(files(k).folder,files(k).name);
    dst=fullfile(archiveRoot,files(k).name);
    if isfile(dst)
        [~,n,e]=fileparts(files(k).name);
        dst=fullfile(archiveRoot,[n '_' char(datetime('now','Format','yyyyMMdd_HHmmss_SSS')) e]);
    end
    movefile(src,dst,'f');
end
fprintf('Moved %d historical files to %s\n',numel(files),archiveRoot);
end

function s=ternary(q,a,b), if q,s=a;else,s=b;end,end
