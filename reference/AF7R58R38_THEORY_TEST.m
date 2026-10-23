function T=AF7R58R38_THEORY_TEST()
% No new vehicle safety guarantees. Tests analytic endpoint jet and shape.
t0=3.5;Tdur=4.0;L=2.0;g=9.81;
J0=zeros(3,8);JT=zeros(3,8);
J0(1,1)=2; JT(1,1)=0;
U=AF7R58R33_C7_TWOSEG_SEED(t0,Tdur,J0,JT,L,g);
assert(U.c7_algebraic_structure && U.max_endpoint_jet_residual<1e-6, ...
    '[R38] two-piece jet seed failed this numeric example.');
for r=0:7
    if r==0
        A=U.first.control_points;B=U.second.control_points;
    else
        A=diff(U.first.control_points,r,2);
        B=diff(U.second.control_points,r,2);
    end
    falling=prod(15-(0:r-1));
    lhs=(falling/U.first.duration^r)*A(:,end);
    rhs=(falling/U.second.duration^r)*B(:,1);
    assert(norm(lhs-rhs,inf)<1e-6,'[R38] interior C7 mismatch.');
end
T=struct('schema','R38_TEST_NO_SAFETY_CLAIM', ...
    'example_C7_jet_numerical_pass',true,'max_residual',U.max_endpoint_jet_residual, ...
    'free_duration_is_not_optimized',true, ...
    'collision_safety_certified',false,'trigger_dynamics_certified',false);
disp('[R38] THEORY SANITY PASS: C7 connection example; NOT collision certification.');
end
