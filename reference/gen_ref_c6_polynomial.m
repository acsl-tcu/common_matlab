function ref = gen_ref_c6_polynomial(param)
% GEN_REF_C6_POLYNOMIAL  Waypoint-interpolating C6 reference.
% Segment curves are 13th-degree Hermite polynomials (0..6 jets match).
% No intermediate waypoint arrival times. Completion time is selected
% automatically from maxSpeed, unless duration is explicitly overridden.
% Output: ref.pYaw(t) = 28x1 [position,yaw,...,6th time derivative].
% Do not send this pre-differentiated result to symbolic differentiation.
arguments
    param.waypoints (:,3) double = [0 0 3; 0.3 0 7.5; 1.1 0.4 16.5; ...
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
P = double(param.waypoints);
validateattributes(P,{'double'},{'2d','ncols',3,'finite','real'});
if size(P,1)<2
    error('CBF:C6PathPoints','Not enough points for this path.');
end
if ~(isnan(param.duration) || (isfinite(param.duration) && param.duration>0))
    error('CBF:C6Duration','duration must be NaN or positive.');
end
data=localMinimumSeventhPath(P);
s_grid=linspace(0,1,2001);
maxDs=0; lengthEstimate=0; oldPoint=[];
for i=1:numel(s_grid)
    jet = localPolyJet(s_grid(i),data);
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
ref.pathJet=@(s) localPolyJet(min(max(s,0),1),data);
ref.q=@(t) localEulerStack(t,param.yaw);
ref.order=6;
ref.info=struct('method','polynomial','duration',duration, ...
    'rampTime',param.rampTime,'points',P, ...
    'maxSpeedRequested',param.maxSpeed,'maxDsSampled',maxDs, ...
    'arcLengthApprox',lengthEstimate,'timingMode',timingMode, ...
    'yaw',param.yaw,'interpolatesInternalPoints',true);
end

function full = localStack(t,data,T,R,yaw)
validateattributes(t,{'double'},{'scalar','finite','real'});
sjet = localTimeJet(t,T,R);
s = min(max(sjet(1),0),1);
pjet=localPolyJet(s,data);
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
