function R43 = AF7R43_NATIVE_SPEED_WITNESS_AUDIT()
fprintf('\n====================================================================================================\n');
fprintf(' AF7R43 NATIVE PRIMARY SPEED-WITNESS AUDIT (DIAGNOSTIC ONLY)\n');
fprintf(' acceptance/safety semantics modified by this audit = 0\n');
fprintf('====================================================================================================\n');

R43=struct();
R43.timestamp=char(datetime('now','Format','yyyy-MM-dd HH:mm:ss.SSS'));
R43.candidate=double(af7r36_single_core_mex(20));
R43.segment=double(af7r36_single_core_mex(21));
R43.component=double(af7r36_single_core_mex(22));
R43.signs=double(af7r36_single_core_mex(23));

auditDir=fileparts(mfilename('fullpath'));if isempty(auditDir),auditDir=pwd;end
reportFile=fullfile(auditDir,'AF7R43_NATIVE_SPEED_WITNESS_REPORT.txt');
captureFile=fullfile(auditDir,'AF7R43_NATIVE_SPEED_WITNESS_CAPTURE.mat');
R43.report_file=reportFile;R43.capture_file=captureFile;

fid=fopen(reportFile,'wt','n','UTF-8');assert(fid>=0,'Could not open R43 report.');
cl=onCleanup(@()fclose(fid)); %#ok<NASGU>
fprintf(fid,'AF7R43 NATIVE PRIMARY SPEED-WITNESS AUDIT\n');
fprintf(fid,'timestamp=%s\n',R43.timestamp);
fprintf(fid,'candidate_rows=%d\nsegment_rows=%d\ncomponent_rows=%d\nsign_rows=%d\n\n', ...
    size(R43.candidate,1),size(R43.segment,1),size(R43.component,1),size(R43.signs,1));

fprintf('\n[A. TRACE COUNTS]\n');
fprintf('candidate rows                              = %d\n',size(R43.candidate,1));
fprintf('segment rows                                = %d\n',size(R43.segment,1));

if isempty(R43.candidate)
    R43.classification='NO_PRIMARY_TRACE__RERUN_SCENARIO_AFTER_R43_REBUILD';
else
    C=R43.candidate;
    wit=C(:,26)>0;
    R43.finite_witness_rows=sum(wit);
    if any(wit)
        ids=find(wit); [~,k]=max(C(ids,33)); rw=ids(k);
        R43.worst_candidate_row=rw;
        R43.worst_candidate_seq=C(rw,1);
        R43.worst_witness_seg=C(rw,28);
        R43.worst_witness_class=C(rw,29);
        R43.worst_witness_ta=C(rw,30);R43.worst_witness_tb=C(rw,31);
        R43.worst_witness_t=C(rw,32);R43.worst_witness_vq=C(rw,33);
        R43.worst_leaf_vl=C(rw,34);R43.worst_leaf_accel=C(rw,35);
        R43.worst_leaf_jerk=C(rw,36);R43.worst_leaf_sigma=C(rw,37);
        R43.worst_leaf_qdot=C(rw,38);R43.worst_leaf_vq_ub=C(rw,39);
        R43.classification=['FINITE_PRIMARY_SPEED_WITNESS_IN_' local_class_name(C(rw,29))];
    else
        R43.classification='PRIMARY_TRACE_PRESENT_BUT_NO_FINITE_WITNESS';
    end

    fprintf('\n[B. PRIMARY CANDIDATES]\n');
    for i=1:size(C,1)
        fprintf('cand %d seq=%.0f | U=[%.6g %.6g %.6g] sigma=%.6g VQ_A=%.6g | Aimp=%d Bwit=%d\n', ...
            i,C(i,1),C(i,16),C(i,17),C(i,18),C(i,22),C(i,23),round(C(i,13)),round(C(i,26)));
        fprintf(fid,'CAND %d %s\n',i,mat2str(C(i,:),17));
    end
end

fprintf('\n[C. CLASSIFICATION]\n');
fprintf('classification                              = %s\n',R43.classification);
if isfield(R43,'worst_witness_vq')
    fprintf('candidate seq                               = %.0f\n',R43.worst_candidate_seq);
    fprintf('segment / class                             = %.0f / %s\n',R43.worst_witness_seg,local_class_name(R43.worst_witness_class));
    fprintf('leaf interval [s]                           = [%.12g, %.12g]\n',R43.worst_witness_ta,R43.worst_witness_tb);
    fprintf('finite witness t / VQ                       = %.12g s / %.12g m/s\n',R43.worst_witness_t,R43.worst_witness_vq);
    fprintf('leaf VL / A / J                             = %.12g / %.12g / %.12g\n',R43.worst_leaf_vl,R43.worst_leaf_accel,R43.worst_leaf_jerk);
    fprintf('leaf sigma / qdot / VQ_UB                   = %.12g / %.12g / %.12g\n',R43.worst_leaf_sigma,R43.worst_leaf_qdot,R43.worst_leaf_vq_ub);
end
fprintf(fid,'classification=%s\n',R43.classification);

fprintf(fid,'\nSEGMENT_MATRIX\n');
for i=1:size(R43.segment,1),fprintf(fid,'SEG %d %s\n',i,mat2str(R43.segment(i,:),17));end
fprintf(fid,'\nCOMPONENT_MATRIX\n');
for i=1:size(R43.component,1),fprintf(fid,'COMP %d %s\n',i,mat2str(R43.component(i,:),17));end
fprintf(fid,'\nSIGN_MATRIX\n');
for i=1:size(R43.signs,1),fprintf(fid,'SIGN %d %s\n',i,mat2str(R43.signs(i,:),17));end

save(captureFile,'R43','-v7.3');
assignin('base','R43',R43);
fprintf('\n[D. OUTPUT]\nreport = %s\ncapture = %s\n',reportFile,captureFile);
fprintf('====================================================================================================\n');
end

function s=local_class_name(c)
switch round(c)
    case 1, s='ENTRY_C7';
    case 2, s='UNSAFE_AFFINE';
    case 3, s='INTER_C7';
    case 4, s='EXIT_REJOIN_C7';
    otherwise, s='UNKNOWN';
end
end
