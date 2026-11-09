function ref = gen_ref_c6_bspline(param)
% GEN_REF_C6_BSPLINE  Clamped degree-7 B-spline C6 reference.
% CONTROL points set shape and are not generally passed through. With
% simple internal knots, position and its first 6 derivatives are continuous.
% No intermediate point arrival times. Duration is selected from maxSpeed
% by default; duration override is available for controlled comparisons.
% Output: ref.pYaw(t) is the 28x1 analytic time-derivative stack.
arguments
    param.controlPoints (:,3) double = [0 0 3; 0.3 0 7.5; 1.1 0.4 16.5; ...
        1.8 0.8 25.5; 1.6 1.4 34.5; 0.7 1.8 43.5; -0.2 1.2 52.5; ...
        0 0.5 61.5; 0 0 66]
    param.maxSpeed (1,1) double {mustBePositive} = 1.65
    param.rampTime (1,1) double {mustBePositive} = 3
    param.duration (1,1) double = NaN
    param.yaw (1,1) double = 0
end
% A complete, independent path generator. No shared C6 path core.
% Path geometry comes from the supplied points, not fixed waypoint times.
% The time law is chosen once from maxSpeed (not yet online path following).
if isnan(param.duration)
    timingMode = 'speed_based';
else
    timingMode = 'duration_override';
end
P = double(param.controlPoints);
validateattributes(P,{'double'},{'2d','ncols',3,'finite','real'});
if size(P,1)<8
    error('CBF:C6PathPoints','Not enough points for this path.');
end
if ~(isnan(param.duration) || (isfinite(param.duration) && param.duration>0))
    error('CBF:C6Duration','duration must be NaN or positive.');
end
data.degree=7;
data.points=P;
n=size(P,1); internal=n-7-1;
data.knots=[zeros(1,8),(1:internal)/(internal+1),ones(1,8)];
s_grid=linspace(0,1,2001);
maxDs=0; lengthEstimate=0; oldPoint=[];
for i=1:numel(s_grid)
    jet = localSplineJet(s_grid(i),data);
    maxDs=max(maxDs,norm(jet(2,:)));
    if ~isempty(oldPoint)
        lengthEstimate=lengthEstimate+norm(jet(1,:)-oldPoint);
    end
    oldPoint=jet(1,:);
end
if maxDs<1e-12
    error('CBF:C6DegeneratePath','Path length must be nonzero.');
end
if isnan(param.duration)
    duration=param.rampTime+max(maxDs/param.maxSpeed,param.rampTime+1e-3);
else
    duration=param.duration;
end
if 2*param.rampTime>=duration
    error('CBF:C6Ramp','Require 2*rampTime < duration.');
end
ref.pYaw=@(t) localStack(t,data,duration,param.rampTime,param.yaw);
ref.progress=@(t) localTimeJet(t,duration,param.rampTime);
ref.pathJet=@(s) localSplineJet(min(max(s,0),1),data);
ref.q=@(t) localEulerStack(t,param.yaw);
ref.order=6;
ref.info=struct('method','bspline','duration',duration, ...
    'rampTime',param.rampTime,'points',P, ...
    'maxSpeedRequested',param.maxSpeed,'maxDsSampled',maxDs, ...
    'arcLengthApprox',lengthEstimate,'timingMode',timingMode, ...
    'yaw',param.yaw,'interpolatesInternalPoints',false);
end

function full = localStack(t,data,T,R,yaw)
validateattributes(t,{'double'},{'scalar','finite','real'});
sjet = localTimeJet(t,T,R);
s = min(max(sjet(1),0),1);
pjet=localSplineJet(s,data);
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
