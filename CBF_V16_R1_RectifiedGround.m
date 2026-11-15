function out=CBF_V16_R1_RectifiedGround(shape,x19,s,opts)
%CBF_V16_R1_RECTIFIEDGROUND Ong et al. ReCBF ground candidate, R1 physical u.
% hR=h-c*min(hdot+k*h,0)^2, c>0 fixed [s^2/m].
% Lf hR+Lg hR u+kR*hR >=0, with exact 2nd order R1 geometry jets.
% INPUT BOX HARD, no slack. The theorem's weak-degree/zero-effect hypotheses
% still require verification on a neighborhood, not just this sample.
if nargin<4,opts=struct();end
if ~isfield(opts,'k'),opts.k=1.2;end
if ~isfield(opts,'kR'),opts.kR=opts.k;end
if ~isfield(opts,'rectCoeff'),opts.rectCoeff=0.5;end
assert(opts.k>0 && opts.kR>0 && opts.rectCoeff>0);
x19=x19(:); assert(numel(x19)==19);
obs=struct('type','sphere','center',[100;0;max(x19(10),0)],'radius',0.25);
[H,labels]=CBF_V11_JetRows('R1',shape,x19,zeros(4,1),s,obs,2);
keep=strncmp(labels,'ground_',7);H=H(keep,:);labels=labels(keep);
N=size(H,1);h=H(:,1);v=H(:,2);a0=2*H(:,3);
A2=zeros(N,4);maxNL=0;maxEarly=0;
for j=1:4
 e=zeros(4,1);e(j)=1;
 HP=CBF_V11_JetRows('R1',shape,x19,e,s,obs,2);
 HM=CBF_V11_JetRows('R1',shape,x19,-e,s,obs,2);
 HP=HP(keep,:);HM=HM(keep,:);
 A2(:,j)=HP(:,3)-HM(:,3); % 2!*central difference /2
 maxNL=max(maxNL,max(abs(2*(HP(:,3)+HM(:,3)-2*H(:,3)))));
 maxEarly=max(maxEarly,max(abs(HP(:,1:2)-HM(:,1:2)),[],'all'));
end
w=v+opts.k*h;
pen=opts.rectCoeff*min(w,0).^2;
hRect=h-pen;
fac=-2*opts.rectCoeff*min(w,0);
AR=fac.*A2;
bR=-(v+fac.*(a0+opts.k*v)+opts.kR*hRect);
AH=A2;bH=-(a0+2*opts.k*v+opts.k^2*h);
nom=[(s.physical.massDrone+s.physical.massLoad)*s.physical.gravity;0;0;0];
lb=[s.input.thrustMin;s.input.torqueMin(:)];ub=[s.input.thrustMax;s.input.torqueMax(:)];
valid=maxNL<=1e-7 && maxEarly<=1e-8;
[qR,resR,efR]=localQP(AR,bR,nom,lb,ub,valid);
[qH,resH,efH]=localQP(AH,bH,nom,lb,ub,valid);
% This is only a sampled weak relative degree check, NOT a theorem proof.
weakViolation=any(vecnorm(A2,2,2)<1e-9 & w < -1e-9);
out=struct('shape',shape,'minGeometricH',min(h),'minHOCBFPsi1',min(w),...
 'minRectifiedH',min(hRect),'rectifiedInitialValid',all(hRect>=-1e-9),...
 'hocbfInitialValid',all(h>=-1e-9 & w>=-1e-9),...
 'sampledWeakDegreeConditionViolated',weakViolation,...
 'maxEarlyInput',maxEarly,'maxNonaffine',maxNL,'affinePass',valid,...
 'nRows',N,'rectifiedQPExit',efR,'rectifiedQPResidual',resR,...
 'hocbfQPExit',efH,'hocbfQPResidual',resH,...
 'rectifiedQPInput',qR,'hocbfQPInput',qH,...
 'labels',{labels},'scope','POINTWISE_R1_GROUND_ONLY_NOT_GLOBAL_RECBF_PROOF');
end
function [u,res,ef]=localQP(A,b,nom,lb,ub,valid)
u=nan(4,1);res=NaN;ef=-7;
if ~valid,return;end
try
 opt=optimoptions('quadprog','Display','off');
 W=diag([1,10,10,10]);
 [ansU,~,ef]=quadprog(2*W,-2*W*nom,-A,-b,[],[],lb,ub,[],opt);
 if ~isempty(ansU)&&all(isfinite(ansU))
   u=ansU;res=min([A*u-b;u-lb;ub-u]);
 end
catch,ef=-9;
end
end
