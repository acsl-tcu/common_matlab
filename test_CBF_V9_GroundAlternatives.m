function report=test_CBF_V9_GroundAlternatives(options)
% [4] 現行一律6次、段別6次、停止距離・入力応答の診断比較。
% 地面/入力限界のハード制約は不変。停止距離は保証付きCBFとは呼ばない。
if nargin<1,options=struct();end
s=CBF_V9_Util('options',options,'settings',test_CBF_Fixture());
full=CBF_V9_Util('options',options,'full',false);
if full,alts=[.2 .3 .5 1.2];speeds=[-.65,-.25,0];
else,alts=[.2 .3 .5];speeds=[-.65,0];end
patterns={[1.2 1.2 1.2 1.2 1.2 1.2],...
          [3 3 3 3 3 3],...
          [1.2 1.6 2 2.4 2.8 3.2],...
          [3.2 2.8 2.4 2 1.6 1.2]};
names={'uniform_1_2','uniform_3','increasing','decreasing'};
rows=cell(0,17);id=0;
for z=alts
 for vz=speeds
  for tilt=[0 .12]
   for k=1:numel(patterns)
    id=id+1;shape='B';name=names{k};err='';
    gpsi=false;gp=false;inpsi=false;minpsi=NaN;minh=NaN;
    optimistic=NaN;predictedByRamp=false;rampMargin=NaN;
    origErr=NaN;
    try
      pattern=patterns{k};st=s;st.cbf.groundGain=pattern(1);
      x=CBF_Audit_State('real',st,z,vz,0,0,tilt,0,0);
      obs=CBF_Audit_State('obstacle','sphere',x(8:10),.75,'side',0);
      orig=cbf_hocbf_scene('real',shape,x,st,{obs});
      [remod,check]=CBF_Audit_StageGainRebuild(orig,shape,x,st,obs,pattern);
      origErr=check.coefficientError;
      if all(pattern==st.cbf.groundGain)
        assert(norm(remod.b-orig.b,Inf)<1e-6*(1+norm(orig.b,Inf)),...
               'CBF:V9EqualGainCheck','元の等ゲイン式と一致しません');
      end
      [gm,im]=CBF_V9_Util('masks',remod);
      gpsi=all(remod.psiInitiallyNonnegative(gm));
      inpsi=all(remod.psiInitiallyNonnegative(im));
      minpsi=min(cellfun(@min,remod.psi(gm)));
      lp=CBF_Audit_LP(remod,gm|im);gp=lp.feasible;
      margins=CBF_V9_Util('groundmargin',x,st);minh=min(margins);
      amax=st.input.thrustMax/(st.physical.massDrone+st.physical.massLoad)-st.physical.gravity;
      if amax>0
        optimistic=minh-max(0,-vz)^2/(2*amax);
      end
      % 一定の入力上限まで滑らかに変化する具体的な候補入力を検証。
      brake=CBF_V9_BrakeRollout(x,st,.15,options);
      rampMargin=min(brake.minMargins);
      predictedByRamp=brake.stopped && ~brake.groundViolated && ~brake.modelInvalid;
    catch ME
      err=[ME.identifier ': ' ME.message];
    end
    rows(end+1,:)={id,name,z,vz,tilt,pattern(1),pattern(6),...
      gpsi,inpsi,gp,minpsi,minh,optimistic,rampMargin,...
      predictedByRamp,origErr,err}; %#ok<AGROW>
   end
  end
 end
end
report=cell2table(rows,'VariableNames',{'caseId','gainPattern','loadAltitude','loadVz',...
 'cableTilt','firstStageGain','lastStageGain','groundAuxiliaryOK',...
 'inputAuxiliaryOK','groundAndInputHardLPFeasible','minimumGroundPsi',...
 'minimumPhysicalGroundMargin','optimisticInstantStopMargin',...
 'nonlinearRampStopMargin','rampSafeStopObserved','inputCoefficientDifference',...
 'errorText'});
writetable(report,fullfile(CBF_V9_Util('folder'),'v9_ground_alternative_comparison.csv'));
fprintf('[V9地面候補] %d条件。段別ゲインは診断用、停止距離は保証CBFではありません。\n',height(report));
end
