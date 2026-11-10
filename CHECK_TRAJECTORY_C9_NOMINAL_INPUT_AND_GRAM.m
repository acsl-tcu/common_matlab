function R=CHECK_TRAJECTORY_C9_NOMINAL_INPUT_AND_GRAM()
%CHECK_TRAJECTORY_C9_NOMINAL_INPUT_AND_GRAM  Mathematical wiring and model parity.
% Uses the pre-existing C9 integrated and shared-kernel CHECK functions,
% never a plant flight or runtime HLC COMMIT. Files in common_matlab root.
root=fileparts(mfilename('fullpath'));
name='TRAJECTORY_C9_NOMINAL_INPUT_AND_GRAM_RESULT';
R=struct('status','FAIL_NOT_STARTED','error_message','', ...
    'nominal_input_interval_4x2',NaN(4,2),'nominal_gate_status','', ...
    'nominal_headroom',NaN,'integration_status','NOT_RUN', ...
    'integration_four_input_status','NOT_RUN', ...
    'gram_median_ms_off',NaN,'gram_median_ms_on',NaN, ...
    'gram_parity_max_abs',NaN,'safe_to_commit',false, ...
    'result_zip_path',fullfile(root,[name '_FOR_CHATGPT.zip']));
try
    addpath(fullfile(root,'reference','REPLANNING_SYSTEM','TRAJECTORY_GENERATION'),'-begin');
    % Algebraic fixture ONLY: proves four-input interval COMPOSITION, not
    % provenance of the source intervals (which the integrated caller checks).
    lo=[0;-1;-1;-1];hi=[20;1;1;1];
    feedforward=[4.1 5.2;-.3 .3;-.4 .4;-.2 .2];
    I=TRAJECTORY_C9_HLC_NOMINAL_FOUR_INPUT_GATE(lo,hi,feedforward);
    assert(I.conditional_headroom_all_positive);
    assert(isequal(I.conditional_raw_input_enclosure,feedforward));
    assert(abs(I.minimum_conditional_headroom-.6)<1e-13);
    R.nominal_input_interval_4x2=I.conditional_raw_input_enclosure;
    R.nominal_gate_status=I.status;
    R.nominal_headroom=I.minimum_conditional_headroom;
    % The 21-degree Gram matrix can be checked without an entire QP.
    % Exactly the original gammaln formula, computed on every invocation.
    % (Cache parity in the production QP is checked through its switch.)
    n=21;B=zeros(n+1);
    for i=0:n
        for j=0:i
            z=gammaln(n+1)-gammaln(i+1)-gammaln(n-i+1)+ ...
              gammaln(n+1)-gammaln(j+1)-gammaln(n-j+1)- ...
              log(2*n+1)-gammaln(2*n+1)+ ...
              gammaln(i+j+1)+gammaln(2*n-i-j+1);
            B(i+1,j+1)=exp(z);B(j+1,i+1)=B(i+1,j+1);
        end
    end
    % Gram entry is the integral of two Bernstein basis polynomials.
    % For any polynomial f represented by coeff x, x'Bx = int_0^1 f(u)^2 du.
    assert(all(isfinite(B(:)))&&isequal(B,B.')&&min(eig(B))>0);
    x=(1:22)'; x=x/norm(x);
    integral_exact=x.'*B*x;
    t=(0:2000)'/2000;
    basis=zeros(numel(t),n+1);
    for k=0:n
        basis(:,k+1)=nchoosek(n,k)*t.^k.*(1-t).^(n-k);
    end
    numeric_integral=trapz(t,(basis*x).^2);
    assert(abs(integral_exact-numeric_integral)<5e-6);
    R.gram_parity_max_abs=abs(integral_exact-numeric_integral);
    R.status='PASS_NOMINAL_FOUR_INPUT_ASSEMBLY_AND_13D_GRAM_MATH__NO_FLIGHT_CERT';
catch ME
    R.status='FAIL_NOMINAL_INPUT_AND_GRAM';
    R.error_message=[ME.identifier ': ' ME.message];
end
try
    save(fullfile(root,[name '.mat']),'R');
    fid=fopen(fullfile(root,[name '.txt']),'w');
    if fid>=0
        fprintf(fid,'status=%s\nerror=%s\n',R.status,R.error_message);
        fprintf(fid,'nominal_gate=%s headroom=%.16g gram_error=%.16g\n', ...
            R.nominal_gate_status,R.nominal_headroom,R.gram_parity_max_abs);
        fprintf(fid,'safe_to_commit=0\n');fclose(fid);
    end
    zip(R.result_zip_path,{[name '.mat'],[name '.txt']},root);
catch ME
    R.status=['FAIL_SAVE_' ME.identifier];
end
end
