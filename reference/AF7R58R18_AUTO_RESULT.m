function R18 = AF7R58R18_AUTO_RESULT()
% AF7R58R18: read existing R17 diagnostic MAT, extract exact native error.
% Research-only report. Never instantiates a replanner or changes a gate.
R18=AF7R58R17_AUTO_RESULT();
R18.native_exception_captured= ...
    isfield(R18,'native_exception_captured') && ...
    logical(R18.native_exception_captured);
if ~isfield(R18,'native_exception_identifier')
    R18.native_exception_identifier='';
end
if ~isfield(R18,'native_exception_message')
    R18.native_exception_message='';
end
if ~isfield(R18,'native_exception_time_s')
    R18.native_exception_time_s=NaN;
end
if ~isfield(R18,'native_exception_stack')
    R18.native_exception_stack=struct('file',{},'name',{},'line',{});
end
folder=fileparts(mfilename('fullpath'));
report=fullfile(folder,'AF7R58R18_RESULT.txt');
fid=fopen(report,'w','n','UTF-8');
assert(fid>=0,'[R18] Cannot write native-exception report.');
c=onCleanup(@()fclose(fid));
fprintf(fid,'AF7R58R18 | FULL MEX EXCEPTION PROVENANCE (DIAGNOSTIC ONLY)\n');
fprintf(fid,'R17 classification = %s\n',R18.classification);
fprintf(fid,'native_exception_captured = %d\n',R18.native_exception_captured);
if R18.native_exception_captured
    fprintf(fid,'native_exception_identifier = %s\n',R18.native_exception_identifier);
    fprintf(fid,'native_exception_time_s = %.17g\n',R18.native_exception_time_s);
    fprintf(fid,'NATIVE EXCEPTION TEXT BEGIN\n%s\nNATIVE EXCEPTION TEXT END\n', ...
        R18.native_exception_message);
    fprintf(fid,'MATLAB CALL STACK\n');
    for k=1:numel(R18.native_exception_stack)
        st=R18.native_exception_stack(k);
        fprintf(fid,'%d | %s | line %d | %s\n',k, ...
            char(st.name),double(st.line),char(st.file));
    end
else
    fprintf(fid,['No exact exception message in this capture. ', ...
        'A fresh R18 simulation run is necessary.\n']);
end
fprintf(fid,'stage1_ms = %.9g\n',R18.stage1_ms);
fprintf(fid,'stage1_generated = %d\n',R18.stage1_generated);
fprintf(fid,'stage2_entered = %d\n',R18.stage2_entered);
fprintf(fid,'stage2_prepared = %d\n',R18.stage2_prepared);
fprintf(fid,'stage3_readonly_entered = %d\n',R18.stage3_readonly_entered);
save(fullfile(folder,'AF7R58R18_RESULT.mat'),'R18');
fprintf('========== R18 FULL NATIVE EXCEPTION ==========\n');
fprintf('classification: %s\n',R18.classification);
fprintf('captured: %d\n',R18.native_exception_captured);
if R18.native_exception_captured
    fprintf('identifier: %s\n',R18.native_exception_identifier);
    fprintf('message: %s\n',R18.native_exception_message);
    for k=1:numel(R18.native_exception_stack)
        st=R18.native_exception_stack(k);
        fprintf('stack %d: %s line %d\n',k,char(st.name),double(st.line));
    end
else
    fprintf('Run the same simulation again after installing R18.\n');
end
fprintf('report: %s\n',report);
end
