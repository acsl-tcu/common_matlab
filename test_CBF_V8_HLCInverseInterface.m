function report=test_CBF_V8_HLCInverseInterface(options)
% 既存HLCがMATLABパス上にある場合のみ、その本物の生成関数を呼び出して比較。
% 注意：現在の6次仮想QP決定入力を実推力に直接変換する試験ではない。
% v1,v1の微分、v2～v4の関係が一致した後で初めて完全逆変換制約を作れる。
if nargin<1,options=struct();end
paths=test_CBF_SetupPaths();s=test_CBF_Fixture();
if isfield(options,'settings'),s=options.settings;end
folder=fullfile(paths.generator,'CBF','test_results');if ~exist(folder,'dir'),mkdir(folder);end
available=(exist('HLC_SUSPENDED_LOAD','class')==8);
status='SKIPPED_HLC_CLASS_NOT_ON_PATH';reason='';
T=NaN;torque=nan(3,1);rcondBeta=NaN;inputPhysical=false;
hoverThrust=(s.physical.massDrone+s.physical.massLoad)*s.physical.gravity;
if available
 try
  hlc=HLC_SUSPENDED_LOAD([],struct());
  xr=CBF_Audit_State('real',s,1.5,0,0,0,0,0,0);
  xCompact=xr(1:19);
  XD=zeros(1,28);XD(3)=1.5;
  V1=zeros(1,6);
  VS=zeros(3,1);
  P=[s.physical.massDrone; s.physical.inertia(:); s.physical.gravity;...
     s.physical.massLoad; s.physical.cableLength; s.physical.disturbanceXY(:)].';
  uf=hlc.Uf_SuspendedLoadxyDst(xCompact,XD,V1,P);
  beta=hlc.Beta2_SuspendedLoadxyDst(xCompact,XD,V1,P);
  rhs=hlc.V2_alpha2_SuspendedLoadxyDst(xCompact,XD,V1,VS,P);
  rcondBeta=rcond(beta);
  if ~isfinite(rcondBeta) || rcondBeta<1e-12
      status='HLC_BETA2_SINGULAR';
  else
      u=[uf(1);beta\rhs(:)];T=u(1);torque=u(2:4);
      inputPhysical=all(isfinite(u)) && T>=s.input.thrustMin &&...
        T<=s.input.thrustMax && all(torque>=s.input.torqueMin(:)) &&...
        all(torque<=s.input.torqueMax(:));
      if inputPhysical,status='HLC_HOVER_INVERSE_WITHIN_INPUT_LIMITS';
      else,status='HLC_HOVER_INVERSE_OUTSIDE_INPUT_LIMITS';end
  end
 catch ME
  status='HLC_INTERFACE_ERROR';reason=[ME.identifier ' ' ME.message];
 end
end
report=table({status},{reason},available,hoverThrust,T,torque(1),torque(2),...
    torque(3),rcondBeta,inputPhysical, ...
   'VariableNames',{'status','errorText','HLCclassAvailable','expectedHoverThrust',...
   'mappedThrust','mappedTauX','mappedTauY','mappedTauZ','rcondBeta2',...
   'withinRealInputLimits'});
writetable(report,fullfile(folder,'v8_hlc_inverse_interface_status.csv'));
fprintf('[V8 HLC逆変換] %s（実際の6階決定入力との接続は別途必要）\n',status);
end
