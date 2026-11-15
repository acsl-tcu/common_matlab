function T=CBF_V22_ModelAudit(s,states,obs,opts)
% One ellipsoid cable vs one rotated ellipsoid obstacle, full nonlinear jets.
% Audit candidate input influence 1..4 for R1, R2_TWO and virtual system.
% Not a safety controller; not a guarantee. No hidden changes to base code.
if nargin<4,opts=struct();end
if ~isfield(opts,'amplitude'),opts.amplitude=[.1; .05;.05;.05];end
if ~isfield(opts,'probeTolerance'),opts.probeTolerance=1e-7;end
rows={};
labels={'T_or_Tddot_or_nu1','tauX_or_nu2','tauY_or_nu3','tauZ_or_nu4'};
for i=1:numel(states)
 x19=states{i}(:);T0=(s.physical.massDrone+s.physical.massLoad)*s.physical.gravity;
 x21=[x19;T0;0];
 xVirtual=flatState(x19,s);
 for mode={'R1','R2_TWO','VIRTUAL'}
  name=mode{1};
  switch name
   case 'R1',x=x19;u0=[T0;0;0;0];
   case 'R2_TWO',x=x21;u0=zeros(4,1);
   case 'VIRTUAL',x=xVirtual;u0=zeros(4,1);
  end
  try
   J0=evalJet(name,x,u0,s,obs);
   for component=1:4
    e=zeros(4,1);amp=opts.amplitude(component);e(component)=amp;
    plus=evalJet(name,x,u0+e,s,obs);
    minus=evalJet(name,x,u0-e,s,obs);
    coeff=(plus-minus)/(2*amp);
    nonaff=max(abs(plus+minus-2*J0));
    first=find(abs(coeff)>opts.probeTolerance,1,'first');
    if isempty(first),first=NaN;end
    rows(end+1,:)={i,name,labels{component},J0(1),first, ...
       coeff(2),coeff(3),coeff(4),coeff(5),nonaff, ...
       'TAYLOR_FIXED_INPUT_CHECK_EACH_DIRECTION_LOCAL_NOT_GLOBAL_RELATIVE_DEGREE'}; %#ok<AGROW>
   end
  catch ME
   for component=1:4
    rows(end+1,:)={i,name,labels{component},NaN,NaN,NaN,NaN,NaN,NaN,NaN,...
       ['ERROR_' ME.identifier]}; %#ok<AGROW>
   end
  end
 end
end
T=cell2table(rows,'VariableNames',{'stateId','model','inputComponent','h0',...
 'firstDetectedOrder','dH1','dH2','dH3','dH4','maxNonAffineSecondDifference','status'});
end
function d=evalJet(name,x,u,s,obs)
if strcmp(name,'R2_TWO')
 H=CBF_V22_CableJet21(x,u,s,obs,4);
elseif strcmp(name,'R1')
 H=CBF_V18_CableOnly('R1',x,u,s,obs,4);
else
 H=CBF_V18_CableOnly('VIRTUAL',x,u,s,obs,4);
end
d=H(1:5).*factorial(0:4);
end
function xv=flatState(x19,s)
xv=zeros(20,1);xv(1:3)=x19(8:10);
xv(4:6)=x19(11:13);
qc=x19(14:16);g=s.physical.gravity;
% Flat-map q=-normalize(pLdd+g*e3-dist), choosing nonzero tension.
dist=[s.physical.disturbanceXY(:);0];
xv(7:9)=-g*qc-g*[0;0;1]+dist;
% pL 3rd..5th are zero diagnostic data, not exact real-state matching.
xv(19:20)=[0;0];
end
