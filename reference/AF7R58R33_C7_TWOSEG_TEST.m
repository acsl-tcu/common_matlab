function R = AF7R58R33_C7_TWOSEG_TEST()
% Independent algebra/optimization check, NOT a safety certificate.
q=rng; reset=onCleanup(@()rng(q)); %#ok<NASGU>
rng(33,'twister');
maxres=0; maxrelative_obj_error=0;
for k=1:15
    t0=rand; T=2.5+2*rand;
    J0=0.02*randn(3,8); JT=0.02*randn(3,8);
    Rk=AF7R58R33_C7_TWOSEG_SEED(t0,T,J0,JT,2.0,9.81);
    assert(~Rk.committable && ~Rk.obstacle_wholebody_certified);
    maxres=max(maxres,Rk.max_endpoint_jet_residual);
    assert(Rk.t_mid==Rk.first.t_end && ...
        Rk.t_mid==Rk.second.t_start && Rk.t_end==Rk.second.t_end, ...
        'The global knots must match the actual segment knots exactly.');
    Jstart=AF7R58R33_C7_SEED_EVAL(Rk,Rk.t_start);
    JL=AF7R58R33_C7_SEED_EVAL(Rk,Rk.t_mid);
    Jend=AF7R58R33_C7_SEED_EVAL(Rk,Rk.t_end);
    assert(max(abs(Jstart(:)-J0(:)))<1e-5 && ...
        max(abs(Jend(:)-JT(:)))<1e-5, ...
        'C7 two-segment outer endpoint residual too large.');
    assert(max(abs(JL(:)-Rk.mid_jets(:)))<1e-5, ...
        'C7 two-segment midpoint evaluation residual too large.');
    % Boundary checks must not admit extrapolation, even by one ULP.
    outLo=Rk.t_start-eps(Rk.t_start);
    outHi=Rk.t_end+eps(Rk.t_end);
    assert(outLo<Rk.t_start && outHi>Rk.t_end, ...
        'Cannot represent neighboring out-of-domain test times.');
    rejectedLo=false; rejectedHi=false;
    try
        AF7R58R33_C7_SEED_EVAL(Rk,outLo);
    catch ME
        rejectedLo=strcmp(ME.message, ...
            'Outside the two-segment candidate interval.');
    end
    try
        AF7R58R33_C7_SEED_EVAL(Rk,outHi);
    catch ME
        rejectedHi=strcmp(ME.message, ...
            'Outside the two-segment candidate interval.');
    end
    assert(rejectedLo && rejectedHi, ...
        'Extrapolation outside the two-segment interval must be rejected.');
    relative_obj_error=(Rk.integral_cost-Rk.zero_mid_jet_cost) / ...
        max(1,abs(Rk.zero_mid_jet_cost));
    maxrelative_obj_error=max(maxrelative_obj_error,relative_obj_error);
    assert(relative_obj_error<1e-7, ...
        'Free middle jets did not improve the exact integral objective.');
end
% If the entire correction is a positive constant direction at entry,
% and all derivatives zero at endpoints, the optimizer may reduce amplitude;
% one-sidedness is checked instead of assumed.
J0=zeros(3,8);JT=zeros(3,8); J0(:,1)=[1;0;0];
Rk=AF7R58R33_C7_TWOSEG_SEED(0,4,J0,JT,2.0,9.81);
R=struct('schema','AF7R58R33_TWOSEG_TEST_V1', ...
 'trial_count',15,'maximum_endpoint_jet_residual',maxres, ...
 'maximum_relative_objective_excess',maxrelative_obj_error, ...
 'sample_one_sided_projection_certified',Rk.one_sided_projection_certified, ...
 'safety_proof_completed',false, 'auto_commit_enabled',false);
fprintf('[R33] two-segment C7 jet/algebra/energy audit PASS; NO SAFETY CLAIM.\n');
end
