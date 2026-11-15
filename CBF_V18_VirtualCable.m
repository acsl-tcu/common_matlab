function out=CBF_V18_VirtualCable(x19,xv,nuNom,s,obs,opts)
% Virtual flat-output, corrected cable q^(4) from pL^(6), ONE obstacle row.
% Uses exact generated HLC inverse for physical torque/thrust audit.
% No geometric rows other than the suspended cable outer ellipsoid.
if nargin<6,opts=struct();end
if ~isfield(opts,'gain'),opts.gain=1.2;end
assert(numel(x19)==19&&numel(xv)==20&&numel(nuNom)==4);
k=opts.gain;H=CBF_V18_CableOnly('VIRTUAL',xv,nuNom,s,obs,4);
H0=CBF_V18_CableOnly('VIRTUAL',xv,zeros(4,1),s,obs,4);
A=zeros(1,4);nonaff=0;early=0;
for j=1:4
  e=zeros(4,1);e(j)=.1;
  HP=CBF_V18_CableOnly('VIRTUAL',xv,e,s,obs,4);
  HM=CBF_V18_CableOnly('VIRTUAL',xv,-e,s,obs,4);
  A(j)=24*(HP(5)-HM(5))/.2;
  nonaff=max(nonaff,abs(24*(HP(5)+HM(5)-2*H0(5))));
  early=max(early,max(abs((HP(1:4)-HM(1:4)).*factorial(0:3))));
end
D=factorial(0:4).*H0;
psi=zeros(4,1);
for i=0:3
  val=0;
  for j=0:i,val=val+nchoosek(i,j)*k^(i-j)*D(j+1);end
  psi(i+1)=val;
end
base=0;
for j=0:4,base=base+nchoosek(4,j)*k^(4-j)*D(j+1);end
rhs=-base;
status='';nu=nan(4,1);minres=NaN;hardPhysical=NaN;map=[];
if any(psi < -1e-9)
  status='INVALID_INITIAL_VIRTUAL_AUXILIARY_SET';
elseif early>1e-7 || nonaff>1e-5
  status='EARLY_INPUT_OR_NONAFFINE';
elseif norm(A,2)<1e-10 && rhs>1e-8
  status='ZERO_AUTHORITY_POSITIVE_RHS';
else
  try
   map=CBF_V12_VirtualPhysicalMap(x19,xv,nuNom,s,opts);
   if map.affineRelative>1e-6 || map.thrustVariationInNu>1e-7
      status='PHYSICAL_MAP_NOT_AFFINE_IN_VIRTUAL_INPUT';
   elseif map.thrustWithinLimits==0
      status='PHYSICAL_THRUST_OUT_OF_LIMITS';
   else
     AA=[A;map.A];bb=[rhs;map.b];
     [vv,~,ef]=quadprog(2*eye(4),-2*nuNom(:),-AA,-bb,[],[],[],[],[],...
         optimoptions('quadprog','Display','off'));
     if ef>0&&~isempty(vv)&&all(isfinite(vv))
       nu=vv;minres=min(AA*nu-bb);
       actual=CBF_V12_VirtualPhysicalMap(x19,xv,nu,s,opts);
       hardPhysical=min([actual.torque-s.input.torqueMin(:);...
           s.input.torqueMax(:)-actual.torque;...
           actual.thrust-s.input.thrustMin;s.input.thrustMax-actual.thrust]);
       if minres>=-1e-7 && hardPhysical>=-1e-7
         status='POINTWISE_VIRTUAL_FEASIBLE_NOT_CLOSED_LOOP_CERTIFIED';
       else,status='QP_OR_INVERSE_RESIDUAL_FAILURE';end
     else,status='QP_FAILED';end
   end
  catch ME
   status='HLC_OR_QP_ERROR';
   out=struct('status',status,'errorText',[ME.identifier ': ' ME.message],...
       'h',H(1),'psi',psi,'inputCoefficient',A,'inputRHS',rhs,'nu',nu,...
       'earlyInput',early,'nonAffine',nonaff,'physicalResidual',hardPhysical,...
       'scope','ONE_CABLE_ONLY_HLC_INVERSE_REQUIRED');return;
  end
end
out=struct('status',status,'errorText','','h',H(1),'psi',psi,...
 'inputCoefficient',A,'inputRHS',rhs,'nu',nu,'minQPResidual',minres,...
 'earlyInput',early,'nonAffine',nonaff,'physicalResidual',hardPhysical,...
 'scope','ONE_CABLE_ONLY_VIRTUAL_ORDER4_POINTWISE_NOT_CLOSED_LOOP_CERTIFIED');
end
