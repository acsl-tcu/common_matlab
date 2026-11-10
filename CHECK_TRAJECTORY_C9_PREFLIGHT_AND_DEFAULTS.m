function R=CHECK_TRAJECTORY_C9_PREFLIGHT_AND_DEFAULTS()
% Development-only static-preflight and code integration audit.
root=fileparts(mfilename('fullpath'));
base='TRAJECTORY_C9_PREFLIGHT_AND_DEFAULTS_RESULT';
R=struct('status','NOT_RUN','error_message','', ...
    'preflight_status','NOT_RUN','preflight_ms',NaN, ...
    'gram_cache_hit',false,'gram_original_formula_max_abs',NaN, ...
    'split_parity_max_abs',NaN,'vectorized_normsq_default_enabled',false, ...
    'original_scalar_mode_retained',false, ...
    'source_resolution_pass',false,'safe_to_commit',false, ...
    'result_zip',fullfile(root,[base '_FOR_CHATGPT.zip']));
try
    moddir=fullfile(root,'reference','REPLANNING_SYSTEM','TRAJECTORY_GENERATION');
    addpath(moddir,'-begin');
    fnames={'TRAJECTORY_C9_NOMINAL_TORQUE_INTERVAL', ...
            'TRAJECTORY_C9_13D_NATURAL_QP', ...
            'TRAJECTORY_C9_13D_STATIC_GRAM', ...
            'TRAJECTORY_C9_STATIC_KERNEL_PREFLIGHT', ...
            'TRAJECTORY_C9_BERNSTEIN_MIDPOINT_SPLIT'};
    for j=1:numel(fnames)
        assert(strcmp(which(fnames{j}),fullfile(moddir,[fnames{j} '.m'])), ...
            'C9_PREFLIGHT:SourceShadowed', ...
            'Unexpected/missing MATLAB module: %s',fnames{j});
    end
    R.source_resolution_pass=true;
    P=TRAJECTORY_C9_STATIC_KERNEL_PREFLIGHT();
    R.preflight_status=P.status;
    R.preflight_ms=P.duration_ms;
    R.gram_cache_hit=P.gram_cache_hit_on_second_call;
    R.split_parity_max_abs=P.split_max_absolute_parity;
    assert(P.ready,'C9_PREFLIGHT:Preparation','Static warmup failed.');
    % Reference recomputation is EXACTLY the old local QP Gram algorithm.
    [G,M]=TRAJECTORY_C9_13D_STATIC_GRAM(21,true);
    assert(M.cache_hit,'C9_PREFLIGHT:ColdGram','Preflight did not warm the Gram.');
    G0=zeros(22);
    for i=0:21
        for j=0:i
            logv=gammaln(22)-gammaln(i+1)-gammaln(22-i) + ...
                 gammaln(22)-gammaln(j+1)-gammaln(22-j) - ...
                 log(43)-gammaln(43)+gammaln(i+j+1)+gammaln(43-i-j);
            G0(i+1,j+1)=exp(logv);
            G0(j+1,i+1)=G0(i+1,j+1);
        end
    end
    R.gram_original_formula_max_abs=max(abs(G(:)-G0(:)));
    assert(isequal(G,G0),'C9_PREFLIGHT:GramMismatch', ...
        'Shared Gram differs from the old QP expression.');
    textcode=fileread(fullfile(moddir,'TRAJECTORY_C9_NOMINAL_TORQUE_INTERVAL.m'));
    R.vectorized_normsq_default_enabled=contains(textcode, ...
        sprintf('    fast_normsq=true;'));
    R.original_scalar_mode_retained=contains(textcode, ...
        'use_vectorized_normsq_kernel')&&contains(textcode,'fast_normsq=logical(v)');
    assert(R.vectorized_normsq_default_enabled && ...
        R.original_scalar_mode_retained, ...
        'C9_PREFLIGHT:Default','Validated norm-square switch or fallback missing.');
    R.status='PASS_PREFLIGHT_AND_DEFAULTS__REALTIME_AND_HLC_PENDING';
catch ME
    R.status=['FAIL_PREFLIGHT_OR_INTEGRATION_' ME.identifier];
    R.error_message=ME.message;
end
try
    save(fullfile(root,[base '.mat']),'R');
    f=fopen(fullfile(root,[base '.txt']),'wt');
    if f>=0
        fprintf(f,'status=%s\nerror=%s\npreflight_ms=%.8f\ngram_cache_hit=%d\n', ...
            R.status,R.error_message,R.preflight_ms,R.gram_cache_hit);
        fprintf(f,'gram_formula_delta=%.17g\nsplit_delta=%.17g\n', ...
            R.gram_original_formula_max_abs,R.split_parity_max_abs);
        fclose(f);
    end
    zip(R.result_zip,{fullfile(root,[base '.mat']),fullfile(root,[base '.txt'])});
catch ME
    R.error_message=[R.error_message ' RESULT_ZIP: ' ME.message];
end
end
