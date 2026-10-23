function J = AF7R58R33_C7_SEED_EVAL(R,t)
% Physical-time jets 0..7 of the two-segment CORRECTION, not entire flight.
% Evaluates only the certified algebraic segment support; no extrapolation.
validateattributes(t,{'double'},{'real','finite','scalar'});
assert(isstruct(R) && isfield(R,'schema') && ...
    strcmp(R.schema,'AF7R58R33_TWOSEG_C7_SEED_V1'), ...
    'An R33 two-segment seed is required.');
assert(t>=R.t_start && t<=R.t_end, ...
    'Outside the two-segment candidate interval.');
assert(R.t_mid==R.first.t_end && R.t_mid==R.second.t_start && ...
    R.t_start==R.first.t_start && R.t_end==R.second.t_end, ...
    'Two-segment interval boundaries are inconsistent.');
if t<=R.t_mid
    C=R.first;
else
    C=R.second;
end
h=C.duration;
% An exact stored segment endpoint must be evaluated at the exact Bernstein
% endpoint. Subtracting and dividing t_end-t_start by h can round above 1
% (e.g. 1+eps), although the requested time is exactly the valid knot.
% Do NOT clamp arbitrary outside times: the enclosing interval assertion
% above remains strict, and interior u values remain checked.
if t==C.t_start
    u=0;
elseif t==C.t_end
    u=1;
else
    u=(t-C.t_start)/h;
end
assert(isfinite(u) && u>=0 && u<=1, ...
    'Normalized time out of range.');
B=C.control_points;
J=zeros(3,8);
for r=0:7
    if r==0, D=B;
    else, D=diff(B,r,2);
    end
    k=15-r;
    for j=1:k
        D=(1-u)*D(:,1:end-1)+u*D(:,2:end);
    end
    J(:,r+1)=D(:,1)*(prod(15-(0:r-1))/h^r);
end
end
