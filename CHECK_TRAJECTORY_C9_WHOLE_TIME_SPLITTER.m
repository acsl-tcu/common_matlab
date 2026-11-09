function R=CHECK_TRAJECTORY_C9_WHOLE_TIME_SPLITTER()
% CHECK_TRAJECTORY_C9_WHOLE_TIME_SPLITTER  DEVELOPMENT-ONLY, NEVER FLIGHT.
% One command tests the exact midpoint-control transform against the
% baseline, measures its execution cost, then runs the EXISTING whole
% C9 integrated CHECK (no duplicate physical fixtures or fake Flight
% authority). Optionally exports the original HLC/controller source once
% for derivation of REAL closed-loop reachability, not for runtime use.
root=fileparts(mfilename('fullpath'));
module=fullfile(root,'reference','REPLANNING_SYSTEM','TRAJECTORY_GENERATION');
R=struct('status','FAIL_C9_WHOLE_TIME_SPLITTER','stage','INIT', ...
    'error_identifier','','error_message','','error_stack',{{}}, ...
    'real_arithmetic_equivalence_pass',false, ...
    'degree_coverage',double([1 2 8 19 20 21 22 23 24 26]), ...
    'tested_polynomials',0, 'max_relative_control_error',Inf, ...
    'baseline_split_median_us',NaN,'matrix_split_median_us',NaN, ...
    'split_speedup_ratio',NaN, 'integrated_check_status','NOT_RUN', ...
    'integrated_case_status',{{}},'integrated_repeated_median_ms',[], ...
    'integrated_repeated_p95_ms',[],'integrated_repeated_max_ms',[], ...
    'integrated_safe_to_commit',false, ...
    'source_export_status','NOT_ATTEMPTED', ...
    'original_hlc_source_names',{{}}, ...
    'source_export_txt',fullfile(root,'TRAJECTORY_C9_CONTROLLER_EQUATIONS_FOR_RESEARCH.txt'), ...
    'result_zip',fullfile(root,'TRAJECTORY_C9_WHOLE_TIME_SPLITTER_RESULT_FOR_CHATGPT.zip'));
try
    R.stage='VERIFY_MODULE';
    assert(isfolder(module),'C9_FAST_SPLIT:Path','Production module missing');
    addpath(module,'-begin');
    R.stage='BERNSTEIN_MIDPOINT_EQUIVALENCE';
    d=R.degree_coverage;
    maxerr=0;count=0;
    for k=1:numel(d)
        n=d(k);
        for trial=1:80
            v=1+mod(trial,4);  % float scale across degrees and 3D axes
            a=(1:n)*(trial/17+0.03);
            P=10^v*[sin(a);cos(a*0.7);sin(a*0.2).*cos(a*0.31)];
            [Lold,Rold]=reference_split(P);
            [Lnew,Rnew]=TRAJECTORY_C9_BERNSTEIN_MIDPOINT_SPLIT(P);
            err=max(abs([Lold(:)-Lnew(:);Rold(:)-Rnew(:)]));
            rel=err/max(1,max(abs(P(:))));
            maxerr=max(maxerr,rel);
            count=count+1;
            assert(isequal(size(Lold),size(Lnew))&& ...
                isequal(size(Rold),size(Rnew))&& ...
                all(isfinite([Lnew(:);Rnew(:)]))&&rel<1e-12, ...
                'C9_FAST_SPLIT:Parity', ...
                'Matrix subdivision differs from de Casteljau controls');
            if n<=25
                assert(all(Lnew(:,1)==P(:,1))&& ...
                    all(Rnew(:,end)==P(:,end)), ...
                    'C9_FAST_SPLIT:Endpoint','Subdivision lost endpoint');
            end
        end
    end
    R.tested_polynomials=count;
    R.max_relative_control_error=maxerr;
    R.real_arithmetic_equivalence_pass=true;
    R.stage='MEASURE_MIDPOINT_COST';
    % Measure same degree-23 input for both methods, alternating order.
    n=24; P=100*[sin((1:n)*0.05);cos((1:n)*0.23);sin((1:n)*0.33)];
    for warm=1:15
        reference_split(P);
        TRAJECTORY_C9_BERNSTEIN_MIDPOINT_SPLIT(P);
    end
    reps=500; batches=9; tnew=zeros(1,batches);told=zeros(1,batches);
    for batch=1:batches
        if mod(batch,2)==0
            tic1=tic;
            for j=1:reps,TRAJECTORY_C9_BERNSTEIN_MIDPOINT_SPLIT(P);end
            tnew(batch)=1e6*toc(tic1)/reps;
            tic2=tic;
            for j=1:reps,reference_split(P);end
            told(batch)=1e6*toc(tic2)/reps;
        else
            tic1=tic;
            for j=1:reps,reference_split(P);end
            told(batch)=1e6*toc(tic1)/reps;
            tic2=tic;
            for j=1:reps,TRAJECTORY_C9_BERNSTEIN_MIDPOINT_SPLIT(P);end
            tnew(batch)=1e6*toc(tic2)/reps;
        end
    end
    R.baseline_split_median_us=median(told);
    R.matrix_split_median_us=median(tnew);
    R.split_speedup_ratio=R.baseline_split_median_us/R.matrix_split_median_us;
    R.stage='EXISTING_FULL_PIPELINE_REGRESSION';
    % A single established integrated audit is the authority for actual
    % per-scenario status and end-to-end timing; no duplicate fixtures.
    if exist('CHECK_TRAJECTORY_C9_INTEGRATED_PIPELINE','file')==2
        G=CHECK_TRAJECTORY_C9_INTEGRATED_PIPELINE();
        R.integrated_check_status=G.status;
        if isfield(G,'case_status'),R.integrated_case_status=G.case_status;end
        if isfield(G,'repeated_case_median_ms')
            R.integrated_repeated_median_ms=G.repeated_case_median_ms;
            R.integrated_repeated_p95_ms=G.repeated_case_p95_ms;
            R.integrated_repeated_max_ms=G.repeated_case_max_ms;
        end
        if ~startsWith(G.status,'PASS_')
            error('C9_FAST_SPLIT:ExistingFullPipelineFailed', ...
                'The existing full integrated audit did not PASS: %s',G.status);
        end
    else
        error('C9_FAST_SPLIT:ExistingFullPipelineMissing', ...
            'Run the existing CHECK_TRAJECTORY_C9_INTEGRATED_PIPELINE.');
    end
    R.stage='COMPLETE';
    R.status='PASS_MIDPOINT_ALGEBRA_AND_EXISTING_FULL_NOMINAL_PIPELINE__HLC_PENDING';
catch ME
    R.error_identifier=ME.identifier;
    R.error_message=ME.message;
    R.error_stack=arrayfun(@(s)sprintf('%s:%d',s.name,s.line), ...
        ME.stack,'UniformOutput',false);
end
% Development-only source capture. Not executed in any Flight or
% replan event. It is needed because the HLC's *actual control equations*
% are not recoverable from nominal torque/headroom alone. Missing source
% never turns a failed numerical certification into a PASS.
try
    export_source_once();
catch ME
    R.source_export_status=['NOT_AVAILABLE_' ME.identifier];
end
matfile=fullfile(root,'TRAJECTORY_C9_WHOLE_TIME_SPLITTER_RESULT.mat');
txtfile=fullfile(root,'TRAJECTORY_C9_WHOLE_TIME_SPLITTER_RESULT.txt');
save(matfile,'R');
fid=fopen(txtfile,'wt');
if fid>=0
    c=onCleanup(@()fclose(fid));
    fprintf(fid,'status=%s\nstage=%s\nerror=%s | %s\n', ...
        R.status,R.stage,R.error_identifier,R.error_message);
    fprintf(fid,'parity=%d | polynomials=%d | max relative difference=%.12g\n', ...
        R.real_arithmetic_equivalence_pass,R.tested_polynomials,R.max_relative_control_error);
    fprintf(fid,'baseline_split_median_us=%.6f | matrix_split_median_us=%.6f | ratio=%.6f\n', ...
        R.baseline_split_median_us,R.matrix_split_median_us,R.split_speedup_ratio);
    fprintf(fid,'integrated_status=%s\n',R.integrated_check_status);
    nCases=min([numel(R.integrated_case_status), ...
        numel(R.integrated_repeated_median_ms), ...
        numel(R.integrated_repeated_p95_ms), ...
        numel(R.integrated_repeated_max_ms)]);
    for k=1:nCases
        fprintf(fid,'case%d=%s | median_ms=%.6f | P95_ms=%.6f | max_ms=%.6f\n', ...
            k,R.integrated_case_status{k},R.integrated_repeated_median_ms(k), ...
            R.integrated_repeated_p95_ms(k),R.integrated_repeated_max_ms(k));
    end
    fprintf(fid,'original_HLC_source_export=%s\n',R.source_export_status);
    fprintf(fid,'actual_closed_loop_certification=NOT_PROVEN\nsafe_to_commit=0\n');
    clear c
end
items={'TRAJECTORY_C9_WHOLE_TIME_SPLITTER_RESULT.mat', ...
    'TRAJECTORY_C9_WHOLE_TIME_SPLITTER_RESULT.txt'};
if isfile(R.source_export_txt)
    items{end+1}='TRAJECTORY_C9_CONTROLLER_EQUATIONS_FOR_RESEARCH.txt';
end
try
    zip(R.result_zip,items,root);
catch ME
    fprintf('[C9 FAST SPLIT] Result ZIP creation failed: %s\n',ME.message);
end
fprintf('[C9 FAST SPLIT] %s | %s\n',R.status,R.result_zip);

    function export_source_once()
        % No user-supplied source is invented. Capture only original
        % active MATLAB HLC/controller class files, read-only. Never
        % export outside this development CHECK or on each Flight.
        names={'HLC_SUSPENDED_LOAD','Controller_HL_Suspended_Load'};
        output='';found=0; paths={};
        for j=1:numel(names)
            f=which(names{j});
            if isempty(f)||~isfile(f),continue;end
            src=fileread(f);
            paths{end+1}=f; %#ok<AGROW>
            output=[output sprintf('\n=========== %s ===========\n',f) src newline]; %#ok<AGROW>
            found=found+1;
        end
        if found>0
            f=fopen(R.source_export_txt,'wt');
            if f>=0
                c=onCleanup(@()fclose(f));
                fwrite(f,output,'char');
                clear c
                R.source_export_status=sprintf('EXPORTED_%d_ORIGINAL_FILES_DEVELOPMENT_ONLY',found);
                R.original_hlc_source_names=paths;
            else
                R.source_export_status='FILE_OPEN_FAILED';
            end
        else
            R.source_export_status='ORIGINAL_CONTROLLER_CLASSES_NOT_RESOLVED';
        end
    end
end

function [left,right]=reference_split(P)
n=size(P,2);Q=P;left=zeros(size(P));right=zeros(size(P));
left(:,1)=P(:,1);right(:,n)=P(:,n);
for k=2:n
    Q=0.5*(Q(:,1:end-1)+Q(:,2:end));
    left(:,k)=Q(:,1);right(:,n-k+1)=Q(:,end);
end
end
