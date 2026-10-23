function [C,A] = AF7R58R31_RETURN_OPTIMAL_SEED(t_start,offset,cableL,gravity)
% C7 geometric correction seed with an analytically selected FREE duration.
% No arbitrary duration cap or same-clock return is a planner constraint here.
% This is NOT a reference trajectory: the spatial-path phase must remain free;
% full speed/yaw/UAV/cable/payload continuous-time certificates are missing.
validateattributes(cableL,{'double'},{'real','finite','scalar','positive'});
validateattributes(gravity,{'double'},{'real','finite','scalar','positive'});
validateattributes(offset,{'double'},{'real','finite','numel',3});
assert(norm(offset)>0, 'Zero terminal correction: no return candidate needed.');
B = [ones(1,8),zeros(1,8)];
a=zeros(1,4);
for r=0:3
    n=numel(B)-1;
    % Gram(B_i^n,B_j^n) = C(n,i)*C(n,j) / ((2n+1)*C(2n,i+j)).
    G=zeros(n+1,n+1);
    for i=0:n
        for j=0:n
            G(i+1,j+1)=nchoosek(n,i)*nchoosek(n,j) / ...
                ((2*n+1)*nchoosek(2*n,i+j));
        end
    end
    a(r+1)=B*G*B.';
    if r<3
        B=n*diff(B);
    end
end
assert(all(isfinite(a))&&all(a>0), ...
    'Positive analytic squared derivative integrals expected.');
% J = ||c0||^2/tau * (a0*x + a1/x + a2/x^3 + a3/x^5)
% with x=T/tau >0; unique minimizer solves
% a0*z^3 - a1*z^2 -3*a2*z -5*a3=0, z=x^2.
rr=roots([a(1),-a(2),-3*a(3),-5*a(4)]);
positive=real(rr(abs(imag(rr))<=1e-10*max(1,abs(real(rr))) & real(rr)>0));
assert(numel(positive)==1 && isfinite(positive), ...
    'Unable to establish unique positive natural-time minimizer.');
tau=sqrt(cableL/gravity);
T=tau*sqrt(positive);
C=AF7R58R31_C7_RETURN_SEED(t_start,T,offset);
C.duration_selection='UNCONSTRAINED_MINIMUM_NATURAL_TIME_DEVIATION_COST';
C.spatial_phase='MUST_BE_OPTIMIZED_SEPARATELY_NOT_SAME_CLOCK';
C.duration_is_not_safety_certified=true;
A=struct('tau_s',tau,'normalized_time_opt',sqrt(positive), ...
    'candidate_duration_s',T,'integral_coefficients',a, ...
    'stationarity_residual',polyval([a(1),-a(2),-3*a(3),-5*a(4)],positive), ...
    'safe_or_feasible',false,'committable',false);
end
