function R=PACK_TRIGGER_AUDIT_RESULTS()
% Consolidate already saved *.mat and *.txt results into ONE upload ZIP.
% Nothing is read or written under reference/; no inputs are deleted.
% The archive includes result files only (no production MATLAB sources).
root=fileparts(mfilename('fullpath'));
archive=fullfile(root,'TRIGGER_AUDIT_RESULTS_FOR_CHATGPT.zip');
R=struct('status','FAIL_PACK_TRIGGER_RESULTS', ...
    'archive_path',archive,'file_count',0,'files',{{}},'reason','');
try
    patterns={'TRIGGER_*_RESULT.mat','TRIGGER_*_RESULT.txt', ...
        'TRIGGER_DIAG_*.mat','TRIGGER_DIAG_*.txt', ...
        'TRIGGER_THREAD_RUN_*.mat','TRIGGER_THREAD_RUN_*.txt'};
    names=cell(1,0);
    for k=1:numel(patterns)
        D=dir(fullfile(root,patterns{k}));
        for j=1:numel(D)
            if ~D(j).isdir && ~any(strcmp(names,D(j).name))
                names{end+1}=D(j).name; %#ok<AGROW>
            end
        end
    end
    assert(~isempty(names), ...
        'PACK_TRIGGER_AUDIT_RESULTS:NoFiles', ...
        'No saved trigger MAT/TXT results exist at common_matlab root.');
    names=sort(names);
    % MATLAB zip supports relative file names with explicit rootfolder.
    % An existing ZIP is not included in names; zip overwrites its output.
    zip(archive,names,root);
    assert(exist(archive,'file')==2, ...
        'PACK_TRIGGER_AUDIT_RESULTS:Missing','ZIP archive was not created.');
    R.file_count=numel(names);
    R.files=names;
    R.status='PASS_RESULT_ZIP_READY';
    R.reason='Attach only TRIGGER_AUDIT_RESULTS_FOR_CHATGPT.zip next time.';
catch ME
    R.reason=[ME.identifier ' | ' ME.message];
end
end
