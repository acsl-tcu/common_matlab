function R=AF7R58R65_RETURN_BERNSTEIN_SCREEN(C)
% Geometry-independent necessary quality gate for an OFFSET-return Bezier.
% E(u)=sum_{i=0}^n C_i B_i^n(u), u in [0,1], in R^3.
% The sufficient condition e dot e' <= 0 is certified via the complete
% Bernstein coefficients of that polynomial (degree 2n-1).  This proves
% non-increase of norm(e)^2, NOT obstacle/cable/controller safety.
% No straight-line/circle/saddle model, shape, or maneuver-time assumption.
% No controller/EKF robust tube.  No acceptance or commit side effects.
%
% All arithmetic is MATLAB double; this finite test includes a deliberately
% conservative rounding guard but is NOT an outward-rounded interval proof.
% Fail closed on invalid input or any unresolved sign. No tolerance is used
% to silently forgive a positive radial derivative.
R=struct('valid',false,'nonexpanding',false, ...
    'no_crossing',false,'terminal_zero',false, ...
    'max_radial_bernstein_upper',Inf, ...
    'min_projection',NaN,'max_control_norm',NaN, ...
    'status','INVALID_OFFSET_POLYGON', ...
    'commit_allowed',false,'wholebody_proved',false);
if ~isnumeric(C) || ~isreal(C) || size(C,1)~=3 || ...
        size(C,2)<2 || ~all(isfinite(C(:)))
    return;
end
C=double(C);n=size(C,2)-1;
if n>28
    R.status='UNSUPPORTED_POLYNOMIAL_DEGREE__FAIL_CLOSED';return;
end
v0=C(:,1);nv=norm(v0);
if ~isfinite(nv) || nv<=0
    R.status='ZERO_INITIAL_OFFSET__NO_RETURN_DIRECTION';return;
end
D=n*diff(C,1,2);   % derivative w.r.t normalized Bezier parameter u
% Bernstein product identity: B_i^n B_j^(n-1) =
% choose(n,i)*choose(n-1,j)/choose(2n-1,i+j) * B_(i+j)^(2n-1).
maxUpper=-Inf;
gamma=(4096*eps)/(1-4096*eps);
for k=0:(2*n-1)
    total=0;absTotal=0;
    for i=max(0,k-(n-1)):min(n,k)
        j=k-i;
        w=(nchoosek(n,i)*nchoosek(n-1,j))/nchoosek(2*n-1,k);
        term=w*dot(C(:,i+1),D(:,j+1));
        total=total+term;
        absTotal=absTotal+abs(term);
    end
    upper=total+gamma*absTotal;
    maxUpper=max(maxUpper,upper);
end
proj=v0.'*C;
cpNorm=sqrt(sum(C.^2,1));
R.max_radial_bernstein_upper=maxUpper;
R.min_projection=min(proj);
R.max_control_norm=max(cpNorm);
R.nonexpanding=logical(maxUpper<=0);
R.no_crossing=logical(all(proj>=0));
R.terminal_zero=logical(all(C(:,end)==0));
R.valid=logical(R.nonexpanding && R.no_crossing && R.terminal_zero);
if R.valid
    R.status='SUFFICIENT_OFFSET_NORM_MONOTONE__NOT_WORLD_SAFETY';
else
    R.status='RETURN_POLYGON_NOT_CERTIFIED__DO_NOT_COMMIT';
end
end
