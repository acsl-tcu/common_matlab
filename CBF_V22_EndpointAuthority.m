function T=CBF_V22_EndpointAuthority(s,states)
% Physical R1 T -> p(lambda)'' and physical R2_TWO Tddot -> p(lambda)''''.
% Ideal flat VIRTUAL nu_xyz -> p(lambda)''''.
% lambda=0 drone, lambda=1 load. NOT geometry CBF rows, diagnostics only.
P=cbf_parameter_vector(s);L=s.physical.cableLength;
rows={};
for ii=1:numel(states)
 x=states{ii}(:);q=x(14:16);m=s.physical.massDrone;mL=s.physical.massLoad;
 T0=(m+mL)*s.physical.gravity;
 qrot=x(1:4);b=[2*(qrot(2)*qrot(4)+qrot(1)*qrot(3));...
  2*(qrot(3)*qrot(4)-qrot(1)*qrot(2));...
  qrot(1)^2-qrot(2)^2-qrot(3)^2+qrot(4)^2];
 xv=toVirtual(x,s);
 for lambda=[0,.25,.75,1]
  mu=1-lambda;proj=eye(3)-q*q.';
  physicalG=q*q.'/(m+mL)+mu*proj/m;
  expectedPhys=physicalG*b;
  Jp=physDerivative('R1',x,[T0;0;0;0],lambda,L,s,2,.1);
  Jp4=physDerivative('R2_TWO',[x;T0;0],zeros(4,1),lambda,L,s,4,.1);
  % Ideal flat cable q=-normalize(a), a=-g*q, so only transverse nu.
  expectedVirtual=mu*L/s.physical.gravity*proj;
  Jv=virtualDerivative(xv,P,lambda,L,.1);
  rows(end+1,:)={ii,lambda, norm(Jp-expectedPhys),...
   norm(Jp4-expectedPhys),norm(Jv-expectedVirtual,'fro'),...
   norm(expectedPhys),norm(expectedVirtual,'fro'),...
   'FULL3D_PHYSICAL_TAYLOR_VS_ANALYTIC_AND_FLATMAP_GEOMETRY'}; %#ok<AGROW>
 end
end
T=cell2table(rows,'VariableNames',{'stateId','lambda','R1_d2Error',...
 'R2_21_d4Error','virtual_d4Error','physAuthorityNorm',...
 'virtualAuthorityFro','scope'});
end
function J=physDerivative(mode,x,u,lambda,L,s,K,step)
J=zeros(3,1);e=zeros(4,1);e(1)=step;
p=physPoint(mode,x,u+e,lambda,L,s,K);
m=physPoint(mode,x,u-e,lambda,L,s,K);
J=(p-m)/(2*step);
end
function p=physPoint(mode,x,u,lambda,L,s,K)
if strcmp(mode,'R1'),Z=CBF_V11_RealTaylor('R1',x,u,s,K);
else,Z=CBF_V22_R2TwoTaylor(x,u,s,K);end
% K-th derivative, not Taylor coefficient:
p=factorial(K)*(Z(8:10,K+1)-(1-lambda)*L*Z(14:16,K+1));
end
function J=virtualDerivative(xv,P,lambda,L,step)
J=zeros(3,3);
for k=1:3
 e=zeros(4,1);e(k)=step;
 [pp,qp]=CBF_V12_VirtualJetFixed(xv,e,P,4);
 [pm,qm]=CBF_V12_VirtualJetFixed(xv,-e,P,4);
 vplus=24*(pp(:,5)-(1-lambda)*L*qp(:,5));
 vminus=24*(pm(:,5)-(1-lambda)*L*qm(:,5));
 J(:,k)=(vplus-vminus)/(2*step);
end
end
function xv=toVirtual(x19,s)
xv=zeros(20,1);xv(1:3)=x19(8:10);xv(4:6)=x19(11:13);
q=x19(14:16);g=s.physical.gravity;
xv(7:9)=-g*q-g*[0;0;1]+[s.physical.disturbanceXY(:);0];
end
