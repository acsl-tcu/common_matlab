function out=CBF_V16_R2_CascadeEllipsoid(x23,s,opts)
%CBF_V16_R2_CASCADEELLIPSOID Backstepping-inspired cascade test.
% This is an EXACT invariant-ellipsoid CBF for an explicitly stated
% SIX-STATE VERTICAL LINEAR SURROGATE with taut vertical cable:
% hdot=v, vdot=T/mTot-g, Tdot=T1, T1dot=T2,T2dot=T3,T3dot=j.
% NO TORQUE DYNAMIC EXTENSION. Not claimed as a CBF for full 3D model.
% Uses Lyapunov equation for Acl of known pole-placed vertical chain.
if nargin<3,opts=struct();end
if ~isfield(opts,'omega'),opts.omega=2;end
if ~isfield(opts,'hReference'),opts.hReference=0.20;end
if ~isfield(opts,'gain'),opts.gain=1.2;end
assert(numel(x23)==23 && opts.omega>0 && opts.hReference>0);
m=s.physical.massDrone+s.physical.massLoad;
T0=m*s.physical.gravity;
h=x23(10)-s.ground.height-s.shape.loadRadius-s.cbf.safetyMargin;
y=[h-opts.hReference;x23(13);x23(20)-T0;x23(21:23)];
A=zeros(6);A(1,2)=1;A(2,3)=1/m;A(3,4)=1;A(4,5)=1;A(5,6)=1;
B=[0;0;0;0;0;1];w=opts.omega;
K=[m*w^6,6*m*w^5,15*w^4,20*w^3,15*w^2,6*w];
Acl=A-B*K;
% Acl'*P+P*Acl=-eye(6): exact algebraic Lyapunov solve, no toolbox.
n=6;mat=kron(eye(n),Acl')+kron(Acl',eye(n));
Q=-eye(n);P=reshape(mat\Q(:),n,n);
P=(P+P')/2;
if min(eig(P))<=0,error('CBF:V16PNotPD','Lyapunov P not PD.');end
Pinv=P\eye(6);
Troom=min(T0-s.input.thrustMin,s.input.thrustMax-T0);
rho=min((opts.hReference^2)/Pinv(1,1),Troom^2/Pinv(3,3))/4;
H=rho-y'*P*y;
Lf=-2*y'*P*A*y;
Lg=-2*y'*P*B;
jNom=-K*y;
resNom=Lf+Lg*jNom+opts.gain*H;
% Minimal correction for 1D affine inequality; NO clip / slack.
if abs(Lg)<1e-12
    if Lf+opts.gain*H>=-1e-10,j=jNom;flag='AFFINE_ROW_INACTIVE';
    else,j=NaN;flag='ZERO_AUTHORITY_VIOLATION';end
else
    minVal=-(Lf+opts.gain*H)/Lg;
    if Lg>0,j=max(jNom,minVal);else,j=min(jNom,minVal);end
    flag='SURROGATE_POINTWISE_SATISFIED';
end
% Check tangent dynamics of ORIGINAL 3D model, relative to surrogate.
J=CBF_V11_RealTaylor('R2',x23,[0;0;0;0],s,1);
% Original physical load vertical acceleration uses x(13) derivative.
actualA=J(13,2);
surrogateA=x23(20)/m-s.physical.gravity;
res=Lf+Lg*j+opts.gain*H;
out=struct('hGround',h,'Hsurrogate',H,'rho',rho,...
 'insideSurrogateSet',H>=0,'nominalJ',jNom,'chosenJ',j,...
 'cbfResidual',res,'nominalCbfResidual',resNom,...
 'modelAccelerationMismatch',actualA-surrogateA,...
 'verticalSurrogateValidAtState',abs(actualA-surrogateA)<=1e-7,...
 'thrustInsideBox',x23(20)>=s.input.thrustMin && x23(20)<=s.input.thrustMax,...
 'PcheckNorm',norm(Acl'*P+P*Acl+eye(6),Inf),...
 'status',flag,...
 'scope','INVARIANT_ELLIPSOID_ONLY_FOR_VERTICALLY_TAUT_LINEAR_SURROGATE');
end
