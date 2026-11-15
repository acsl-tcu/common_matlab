function T=CBF_V22_ModelParity(s,states,obs)
% Identical physical thrust T and torque tau, Tdot=Tddot=0:
% original 19-state full nonlinear Taylor jet MUST match the 21-state
% two-integrator model's first 19 states, to all tested orders.
rows={};
for j=1:numel(states)
 x=states{j};T0=(s.physical.massDrone+s.physical.massLoad)*s.physical.gravity;
 tau=[.01;-.02;.015];
 Z1=CBF_V11_RealTaylor('R1',x,[T0;tau],s,4);
 Z2=CBF_V22_R2TwoTaylor([x;T0;0],[0;tau],s,4);
 H1=CBF_V18_CableOnly('R1',x,[T0;tau],s,obs,4);
 H2=CBF_V22_CableJet21([x;T0;0],[0;tau],s,obs,4);
 stateDifference=max(abs(Z1(:)-reshape(Z2(1:19,:),[],1)));
 barrierDifference=max(abs(H1(:)-H2(:)));
 rows(end+1,:)={j,stateDifference,barrierDifference, ...
    stateDifference<1e-9 && barrierDifference<1e-9,...
    'STATIC_T_EQUIVALENCE_UP_TO_FOURTH_JET_NOT_CLOSED_LOOP'}; %#ok<AGROW>
end
T=cell2table(rows,'VariableNames',{'stateId','maxStateJetDifference',...
  'maxBarrierJetDifference','parityPass','scope'});
end
