function A = AF7R58R31_RETURN_THEOREM_TEST()
% Exact Bernstein endpoint-jet and monotonicity construction audit.
% Numerical evaluation checks are diagnostics, not the mathematical proof.
C=AF7R58R31_C7_RETURN_SEED(2.4,3.7,[2;-0.6;1.3]);
B=C.control_points;
zero_tol=0; % Algebraic endpoint differences are exactly zero.
endpoint_diff=zeros(7,2);
for r=1:7
    B=diff(B,1,2);
    endpoint_diff(r,:)=[norm(B(:,1)),norm(B(:,end))];
end
assert(all(endpoint_diff(:)==zero_tol), ...
    'Endpoint jets fail: this is NOT a C7 correction.');
% Nonnegative beta derivative: S''s derivative = (15!/(7!7!))u^7(1-u)^7.
% Therefore 0<=S(u)<=1, and c(u)=(1-S(u))*c0 never changes sign
% along the initial c0 direction. No fixed upper duration is introduced.
for u=linspace(0,1,257)
    t=C.t_start+u*C.duration;
    c=AF7R58R31_C7_RETURN_EVAL(C,t,0);
    assert(dot(c,C.offset_start)>=-1e-12 && ...
           norm(c)<=norm(C.offset_start)+1e-12);
end
[Copt,Vopt]=AF7R58R31_RETURN_OPTIMAL_SEED(2.4,[2;-0.6;1.3],2.0,9.81);
assert(isfinite(Copt.duration) && Copt.duration>0);
assert(abs(Vopt.stationarity_residual) < 1e-7);
A=struct('schema','AF7R58R31_RETURN_THEOREM_AUDIT_V1', ...
    'degree',15,'endpoint_jet_zero_orders',1:7, ...
    'max_endpoint_difference',max(endpoint_diff(:)), ...
    'natural_duration_example_s',Copt.duration, ...
    'correction_segment_overshoot_excluded',true, ...
    'wholebody_certified',false,'speed_certified',false, ...
    'heading_certified',false,'committable',false);
fprintf('R31 C7 RETURN SEED ALGEBRA/NUMERIC AUDIT PASS; SAFETY NOT CERTIFIED.\n');
end
