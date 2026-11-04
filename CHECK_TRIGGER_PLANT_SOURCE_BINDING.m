function R=CHECK_TRIGGER_PLANT_SOURCE_BINDING()
% Temporary audit in common_matlab/ (delete after verification).
% The production TRIGGER_ORIGINAL_PLANT_SOURCE_BINDING remains in reference/.
% No output is written in reference/ or the trigger source directory.
R=TRIGGER_ORIGINAL_PLANT_SOURCE_BINDING();
try
    audit_root=fileparts(mfilename('fullpath'));
    txt_path=fullfile(audit_root,'TRIGGER_PLANT_SOURCE_BINDING_RESULT.txt');
    mat_path=fullfile(audit_root,'TRIGGER_PLANT_SOURCE_BINDING_RESULT.mat');
    fid=fopen(txt_path,'wt');
    if fid<0
        error('CHECK_TRIGGER_PLANT_SOURCE_BINDING:Output', ...
            'Cannot create audit TXT in common_matlab root.');
    end
    close_file=onCleanup(@()fclose(fid));
    fprintf(fid,'status=%s\n',R.status);
    fprintf(fid,'source_verified=%d\n',logical(R.source_verified));
    fprintf(fid,'source_path=%s\n',R.source_path);
    fprintf(fid,'source_sha256=%s\n',R.source_sha256);
    fprintf(fid,'safe_to_commit=%d\n',logical(R.safe_to_commit));
    fprintf(fid,'reason=%s\n',R.reason);
    clear close_file
    save(mat_path,'R');
catch ME_output
    warning('CHECK_TRIGGER_PLANT_SOURCE_BINDING:ArtifactWrite', ...
        'Audit result could not be saved to common_matlab: %s',ME_output.message);
end
end
