function T=CBF_V22_VirtualMapAudit(s,states,obs)
% Exercises EXISTING V18 virtual cable 4th-order / generated HLC inverse
% with the same load-mass ESTIMATE. Only a pointwise audit, not rollout.
rows={};
for i=1:numel(states)
 x19=states{i};xv=zeros(20,1);
 xv(1:3)=x19(8:10);xv(4:6)=x19(11:13);
 q=x19(14:16);g=s.physical.gravity;
 xv(7:9)=-g*q-g*[0;0;1]+[s.physical.disturbanceXY(:);0];
 try
  r=CBF_V18_VirtualCable(x19,xv,zeros(4,1),s,obs,struct());
  residual=NaN;if isfield(r,'physicalResidual'),residual=r.physicalResidual;end
  coeff=NaN;if isfield(r,'inputCoefficient'),coeff=norm(r.inputCoefficient);end
  rows(end+1,:)={i,s.physical.massLoad,r.h,coeff,residual,char(r.status),...
    'POINTWISE_ONLY_HLC_INVERSE_NOT_CLOSED_LOOP'}; %#ok<AGROW>
 catch ME
  rows(end+1,:)={i,s.physical.massLoad,NaN,NaN,NaN,...
    ['ERROR_' ME.identifier],'HLC_INVERSE_UNAVAILABLE_OR_FAILED'}; %#ok<AGROW>
 end
end
T=cell2table(rows,'VariableNames',{'stateId','estimatedLoadMass_kg','h','virtualRowNorm',...
    'physicalResidual','status','scope'});
end
