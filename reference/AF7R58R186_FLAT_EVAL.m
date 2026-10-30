function xd=AF7R58R186_FLAT_EVAL(C,tGlobal)
% 15th-degree piecewise Bernstein pL trajectory: exact analytic d0..d7.
% Heading psi = continuous lift of arg(vx+i*vy), and analytic d0..d6.
% Reference-only evaluator; it cannot certify execution or actuator forces.
% C.times_s absolute, control_points 3x16 per segment, yaw_lift_rad
% anchored at the start of each segment. Outputs HLC 28x1 [XYZ yaw] x7.
assert(isstruct(C)&&isscalar(C)&&all(isfield(C,...
 {'times_s','control_points','yaw_lift_rad'})), 'R186:InvalidCandidate');
T=double(C.times_s(:).'); P=C.control_points;
assert(iscell(P)&&numel(P)==numel(T)-1&&numel(P)>=1&&...
 isscalar(tGlobal)&&isfinite(tGlobal)&&all(isfinite(T))&&...
 all(diff(T)>0)&&numel(C.yaw_lift_rad)==numel(P), 'R186:InvalidDomain');
t=double(tGlobal);
tol=64*eps(max([1;abs(T(:));abs(t)]));
assert(t>=T(1)-tol&&t<=T(end)+tol,'R186:OutsideCertifiedCandidateDomain');
t=min(T(end),max(T(1),t));
s=find(t<T(2:end),1,'first');if isempty(s),s=numel(P);end
h=T(s+1)-T(s);u=max(0,min(1,(t-T(s))/h));
X=double(P{s});if isequal(size(X),[16 3]),X=X';end
assert(isequal(size(X),[3 16])&&all(isfinite(X(:))), 'R186:InvalidControlPoints');
D=zeros(3,8);B=X;
for k=0:7
 n=size(B,2)-1; w=local_bernstein_weights(n,u);
 factor=prod(16-k:15)/h^k;
 D(:,k+1)=factor*(B*w(:));
 if k<7,B=diff(B,1,2);end
end
% Differentiate log(vx + i vy) as a truncated power series:
% log(z)_n = (n*z_n - sum_{k=1}^{n-1} k*log(z)_k*z_{n-k})/(n*z_0)
% where z_n = (p_x^(n+1) + i p_y^(n+1))/n! .
z=complex(zeros(1,7));
for k=0:6
 z(k+1)=complex(D(1,k+2),D(2,k+2))/factorial(k);
end
assert(abs(z(1))>0&&isfinite(abs(z(1))), 'R186:UndefinedPathTangentYaw');
Y=zeros(1,7);
anchor=double(C.yaw_lift_rad(s));raw=angle(z(1));
% In each segment the Bernstein velocity convex hull must be in an open
% halfplane; then theta remains within pi of its start angle.
start=complex(X(1,2)-X(1,1),X(2,2)-X(2,1));
assert(abs(start)>0,'R186:ZeroSegmentStartVelocity');
theta0=angle(start);
thetaRel=atan2(sin(raw-theta0),cos(raw-theta0));
Y(1)=anchor+thetaRel;
logCoeffs=complex(zeros(1,7));
for n=1:6
 rhs=n*z(n+1);
 for k=1:n-1
  rhs=rhs-k*logCoeffs(k+1)*z(n-k+1);
 end
 logCoeffs(n+1)=rhs/(n*z(1));
 Y(n+1)=factorial(n)*imag(logCoeffs(n+1));
end
assert(all(isfinite([D(:);Y(:)])),'R186:NonfiniteJet');
xd=zeros(28,1);
for k=0:6,xd(4*k+(1:4))=[D(:,k+1);Y(k+1)];end
end
function w=local_bernstein_weights(n,u)
% Stable de Casteljau basis via recursion, including endpoints.
w=1;
for j=1:n
 w=[w*(1-u),0]+[0,w*u]; %#ok<AGROW>
end
end
