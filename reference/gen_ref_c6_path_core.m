function ref = gen_ref_c6_path_core(method, points, maxSpeed, rampTime, yaw, durationOverride)
% Analytic 0..6 time derivatives for TWO generic 3-D path representations.
% No finite differencing, no symbolic piecewise expression, no CBF bypass.
% Each result uses the pre-differentiated reference convention:
%   [x;y;z;yaw; dx;dy;dz;dyaw; ... ; d6x;d6y;d6z;d6yaw] (28x1).
% The path parameter s runs [0,1]. No waypoint arrival times are set.
% The nominal completion time is computed from a user-chosen approximate
% maximum path speed (unless durationOverride is explicitly supplied).
% A smooth endpoint time law makes s(t) C^7 at its piecewise joins.
points = double(points);
validateattributes(points, {'double'}, {'2d','ncols',3,'finite','real'});
validateattributes(maxSpeed, {'double'}, {'scalar','finite','positive'});
if nargin<6, durationOverride=NaN; end
if ~(isnumeric(durationOverride) && isscalar(durationOverride) && ...
        (isnan(durationOverride) || (isfinite(durationOverride) && durationOverride>0)))
    error('CBF:C6Duration','durationOverride must be NaN or positive scalar.');
end
validateattributes(rampTime, {'double'}, {'scalar','finite','positive'});
validateattributes(yaw, {'double'}, {'scalar','finite','real'});
if size(points,1)<2
    error('CBF:C6PathPoints','At least two 3-D points are required.');
end
switch lower(char(method))
    case 'polynomial'
        data = localMinimumSeventhPath(points);
    case 'bspline'
        if size(points,1)<8
            error('CBF:C6BsplinePoints', ...
                'A degree-7 B-spline needs at least 8 control points.');
        end
        data.degree=7;
        data.points=points;
        n=size(points,1);
        internal=n-7-1;
        data.knots=[zeros(1,8), (1:internal)/(internal+1), ones(1,8)];
    otherwise
        error('CBF:C6PathMethod','Unknown C6 path representation.');
end
% Estimate the maximum ||d path/ds|| without choosing waypoint times.
% Sampling is for nominal time allocation, NOT a certified dynamic bound.
s_grid=linspace(0,1,2001);
maxDs=0; lengthEstimate=0; oldPoint=[];
for i=1:numel(s_grid)
    si=s_grid(i);
    switch lower(char(method))
        case 'polynomial', pj=localPolyJet(si,data);
        case 'bspline',    pj=localSplineJet(si,data);
    end
    maxDs=max(maxDs,norm(pj(2,:)));
    if ~isempty(oldPoint)
        lengthEstimate=lengthEstimate+norm(pj(1,:)-oldPoint);
    end
    oldPoint=pj(1,:);
end
if maxDs<1e-12
    error('CBF:C6DegeneratePath','Path points describe a zero-length curve.');
end
if isnan(durationOverride)
    % The plateau speed satisfies ||dp/dt|| <= maxSpeed on the S-grid.
    % The extra rampTime accounts for smooth start/stop without assigning
    % *any* intermediate-point arrival times.
    duration=rampTime+max(maxDs/maxSpeed, rampTime+1e-3);
    timingMode='speed_based';
else
    duration=durationOverride;
    timingMode='duration_override';
end
if 2*rampTime >= duration
    error('CBF:C6Ramp','Require 2*rampTime < duration.');
end
ref.pYaw = @(t) localStack(t,method,data,duration,rampTime,yaw);
ref.progress = @(t) localTimeJet(t,duration,rampTime);
ref.pathJet = @(s) localPathJet(s,method,data);
ref.q = @(t) localEulerStack(t,yaw);
ref.order = 6;
ref.info = struct('method',char(method),'duration',duration, ...
    'rampTime',rampTime,'points',points, 'maxSpeedRequested',maxSpeed, ...
    'maxDsSampled',maxDs,'arcLengthApprox',lengthEstimate, ...
    'timingMode',timingMode,'yaw',yaw, ...
    'interpolatesInternalPoints',strcmpi(method,'polynomial'));
% An analytic 6th-derivative stack is returned by .pYaw. Changes to
% maxSpeed made by RECREATING the reference are NOT C6-online replans.
end

function full = localStack(t,method,data,T,R,yaw)
validateattributes(t,{'double'},{'scalar','finite','real'});
sjet = localTimeJet(t,T,R);
s = min(max(sjet(1),0),1);
pjet=localPathJet(s,method,data);
% Bell polynomials: B(n,k)(s',...,s^(n-k+1)), with B(0,0)=1.
B=zeros(7,7); B(1,1)=1;
for n=1:6
    for k=1:n
        v=0;
        for j=1:(n-k+1)
            v=v+nchoosek(n-1,j-1)*sjet(j+1)*B(n-j+1,k);
        end
        B(n+1,k+1)=v;
    end
end
full=zeros(28,1);
for n=0:6
    if n==0
        value=pjet(1,:).';
    else
        value=zeros(3,1);
        for k=1:n
            value=value+pjet(k+1,:).'*B(n+1,k+1);
        end
    end
    full((4*n+1):(4*n+3))=value;
end
full(4)=yaw; % fixed yaw is independent of vertical/tangent direction
if ~all(isfinite(full))
    error('CBF:C6PathNonfinite','Nonfinite analytic derivative stack.');
end
end

function pjet=localPathJet(s,method,data)
validateattributes(s,{'double'},{'scalar','finite','real'});
s=min(max(s,0),1);
switch lower(char(method))
    case 'polynomial', pjet=localPolyJet(s,data);
    case 'bspline',    pjet=localSplineJet(s,data);
    otherwise, error('CBF:C6PathMethod','Unknown C6 method');
end
end

function qstack=localEulerStack(~,yaw)
qstack=zeros(21,1);
qstack(3)=yaw;
end

function sj=localTimeJet(t,T,R)
% Normalized progress s(t), with start/end velocity ramp using the
% degree-13 smoothstep S(u)=integral_0^u C*v^6*(1-v)^6 dv.
% All time derivatives 1..6 of the velocity ramp vanish at its ends;
% therefore position derivatives 0..6 join continuously.
persistent smoothCoeff integralCoeff
if isempty(smoothCoeff)
    smoothCoeff=zeros(1,14); % ascending powers, c(p+1)*u^p
    C=factorial(13)/(factorial(6)*factorial(6));
    for j=0:6
        smoothCoeff(8+j)=C*(-1)^j*nchoosek(6,j)/(7+j);
    end
    integralCoeff=[0, smoothCoeff./(1:14)];
end
sj=zeros(1,7);
invLength=1/(T-R);
if t<=0
    return
elseif t<R
    u=t/R;
    sj(1)=invLength*R*localPolyEval(integralCoeff,u);
    for k=1:6
        sj(k+1)=invLength*R^(1-k)*localPolyDerivEval(integralCoeff,u,k);
    end
elseif t<T-R
    sj(1)=invLength*(R/2+t-R);
    sj(2)=invLength;
elseif t<T
    u=(t-(T-R))/R;
    sj(1)=invLength*(T-3*R/2+R*(u-localPolyEval(integralCoeff,u)));
    sj(2)=invLength*(1-localPolyDerivEval(integralCoeff,u,1));
    for k=2:6
        sj(k+1)=-invLength*R^(1-k)*localPolyDerivEval(integralCoeff,u,k);
    end
else
    sj(1)=1;
end
end

function data=localMinimumSeventhPath(P)
% C6 Hermite waypoint interpolation with a degree-13 polynomial per segment.
% Endpoint 0..6 derivatives at each knot are prescribed consistently:
%   position = waypoint, d/ds = chord-based tangent, d^2..d^6/ds=0.
% With these boundary jets fixed, the unique degree-13 polynomial on each
% segment minimizes the integral of squared 7th derivative on that segment.
% Not a GLOBAL minimum-7th-derivative optimization over free knot jets.
m=size(P,1)-1;
% Chord-length parametrization prevents unevenly spaced points from
% creating artificial high-speed segments. It does not assign timestamps.
segmentLength=vecnorm(diff(P,1,1),2,2);
if any(segmentLength<1e-8)
    error('CBF:C6DuplicatePoints', ...
        'Polynomial waypoints must be consecutive distinct positions.');
end
knots=[0; cumsum(segmentLength)];
knots=knots/knots(end);
tangent=zeros(m+1,3);
tangent(1,:)=(P(2,:)-P(1,:))/(knots(2)-knots(1));
tangent(end,:)=(P(end,:)-P(end-1,:))/(knots(end)-knots(end-1));
for i=2:m
    tangent(i,:)=(P(i+1,:)-P(i-1,:))/(knots(i+1)-knots(i-1));
end
% Build one common 14x14 Hermite matrix for all segments, using local
% coordinate u=(s-s_k)/h. Higher derivatives at both ends vanish.
A=zeros(14,14);
for k=0:6
    A(k+1,k+1)=factorial(k); % derivative at u=0
    for j=k:13
        A(8+k,j+1)=prod((j-k+1):j); % derivative at u=1
    end
end
% Normalize row constraints before solving (6th derivative ~= position).
rowScale=max(sqrt(sum(A.^2,2)),realmin);
As=A./rowScale;
coeff=zeros(3,14,m);
for seg=1:m
    h=knots(seg+1)-knots(seg);
    values=zeros(14,3);
    values(1,:)=P(seg,:);
    values(2,:)=h*tangent(seg,:);
    values(8,:)=P(seg+1,:);
    values(9,:)=h*tangent(seg+1,:);
    c=As\(values./rowScale);
    residual=A*c-values;
    if any(~isfinite(c(:))) || max(abs(residual(:)))>1e-5
        error('CBF:C6PolynomialSolve', ...
            'C6 Hermite solve failed: residual %g.',max(abs(residual(:))));
    end
    coeff(:,:,seg)=c.';
end
data.coeff=coeff;
data.m=m;
data.knots=knots;
end

function pj=localPolyJet(s,data)
knots=data.knots;
m=data.m;
if s>=1
    seg=m; u=1;
else
    seg=find(knots(1:m)<=s & s<knots(2:m+1),1,'last');
    if isempty(seg),seg=1;end
    u=(s-knots(seg))/(knots(seg+1)-knots(seg));
end
h=knots(seg+1)-knots(seg);
coeff=squeeze(data.coeff(:,:,seg)).'; % 14x3 ascending power
pj=zeros(7,3);
for k=0:6
    for axis=1:3
        pj(k+1,axis)=h^(-k)*localPolyDerivEval(coeff(:,axis).',u,k);
    end
end
end

function pj=localSplineJet(s,data)
% Differentiate a B-spline by recursive control point differencing; then
% evaluate each derivative exactly using de Boor, no finite differences.
U=data.knots;
P=data.points;
p=data.degree;
pj=zeros(7,3);
for k=0:6
    pj(k+1,:)=localDeBoor(s,U,P,p);
    if k==6, break; end
    n=size(P,1);
    NP=zeros(n-1,3);
    for i=1:n-1
        denom=U(i+p+1)-U(i+1);
        if denom>0
            NP(i,:)=(p/denom)*(P(i+1,:)-P(i,:));
        end
    end
    P=NP;
    U=U(2:end-1);
    p=p-1;
end
end

function point=localDeBoor(s,U,P,p)
n=size(P,1);
if s>=U(n+1)
    span=n;
else
    span=find(U(1:n)<=s & s<U(2:n+1),1,'last');
end
if isempty(span)
    error('CBF:C6SplineSpan','Invalid spline parameter.');
end
d=P(span-p:span,:);
for r=1:p
    for j=p:-1:r
        i=span-p+j;
        denom=U(i+p-r+1)-U(i);
        alpha=0;
        if denom>0, alpha=(s-U(i))/denom; end
        d(j+1,:)=(1-alpha)*d(j,:)+alpha*d(j+1,:);
    end
end
point=d(p+1,:);
end

function val=localPolyEval(coeff,x)
val=0;
for j=numel(coeff):-1:1
    val=val*x+coeff(j);
end
end

function val=localPolyDerivEval(coeff,x,k)
if k==0
    val=localPolyEval(coeff,x);
    return
end
val=0;
for j=numel(coeff)-1:-1:k
    ff=1;
    for l=0:k-1, ff=ff*(j-l); end
    val=val*x+coeff(j+1)*ff;
end
end
