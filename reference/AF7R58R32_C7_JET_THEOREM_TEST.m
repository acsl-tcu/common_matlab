function R = AF7R58R32_C7_JET_THEOREM_TEST()
% Algebraic endpoint jet checks; numerical tests are not a safety proof.
rng(32,'twister');
maxRel=0;
for k=1:30
    t0=-2+4*rand; T=0.6+3*rand;
    J0=0.1*randn(3,8); JT=0.1*randn(3,8);
    C=AF7R58R32_C7_JET_SEED(t0,T,J0,JT);
    B=C.control_points;
    for r=0:7
        % MATLAB diff(X,0,dim) is not a valid differentiation order.
        % Zeroth derivative is the polynomial itself (unchanged controls).
        if r==0
            D=B;
        else
            D=diff(B,r,2);
        end
        falling=prod(15-(0:r-1));
        factor=falling/T^r;
        err=max([abs(factor*D(:,1)-J0(:,r+1)); ...
                 abs(factor*D(:,end)-JT(:,r+1))]);
        scale=max(1,max(abs([J0(:);JT(:)])));
        maxRel=max(maxRel,err/scale);
    end
end
assert(maxRel<1e-5, 'Endpoint jet algebra test failed (numerical diagnostic).');
% R31 special case J0=[c0,zero jets] and JT=zeros: control polygon
% must be 8 copies of c0 followed by 8 zeros, giving no overshoot.
c0=[1.5;-0.8;0.2]; J0=zeros(3,8); J0(:,1)=c0;
JT=zeros(3,8); C=AF7R58R32_C7_JET_SEED(0,2,J0,JT);
assert(max(abs(C.control_points(:,1:8)-repmat(c0,1,8)),[],'all')<1e-12);
assert(max(abs(C.control_points(:,9:16)),[],'all')<1e-12);
% General nonzero initial jet may force the correction norm to rise first;
% monotonicity cannot be a universal consequence of C7 alone.
R=struct('schema','AF7R58R32_JET_THEOREM_TEST_V1', ...
         'numerical_max_scaled_jet_residual',maxRel, ...
         'zero_derivative_special_case_pass',true, ...
         'arbitrary_jet_algebra_pass',true, ...
         'continuous_collision_certified',false, ...
         'monotone_return_for_arbitrary_jet',false, ...
         'auto_commit_enabled',false);
fprintf('[R32] C7 ENDPOINT JET TEST PASS | candidate only; no safety claim.\n');
end
