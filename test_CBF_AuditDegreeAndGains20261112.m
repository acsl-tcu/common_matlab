function reports=test_CBF_AuditDegreeAndGains20261112(options)
% 6階を前提にせず、0～6階の入力作用を各状態・各チャネルで確認。
% 段別ゲインは診断用のqpコピーにだけ適用。既存CBFコード変更なし。
if nargin<1,options=struct();end
paths=test_CBF_SetupPaths();s=test_CBF_Fixture();
if isfield(options,'settings'),s=options.settings;end
full=getopt(options,'full',true);
if full
   configs=[1.5 0 0 0 0;1.2 -.55 0 0 0;2 .3 .18 .12 .15;...
       1.4 -.35 -.22 .12 .15;2.5 .6 .3 -.10 .25];
else
   configs=[1.5 0 0 0 0;1.2 -.55 .12 .10 .1];
end
shapes={'C','B','D'}; kinds={'sphere','ellipsoid'};
rows=cell(0,13);idx=0;
P=cbf_parameter_vector(s);plan=cbf_choose_cable_spheres(s);
for i=1:size(configs,1)
 c=configs(i,:);
 xr=CBF_Audit_State('real',s,c(1),c(2),0,c(3),c(4),c(5),0);
 xv=CBF_Audit_State('virtualfromreal',xr,s);
 for modeCell={'real','virtual'}
    mode=modeCell{1};if strcmp(mode,'real'),x=xr;else,x=xv;end
    cache=cbf_precompute_jets(mode,x,s);
    for shapeCell=shapes
       shape=shapeCell{1};
       for kindCell=kinds
           kind=kindCell{1};
           obs=CBF_Audit_State('obstacle',kind,xr(8:10),1.1,'front',.6);
           O=cbf_obstacle_vector(obs,kind);
           [~,info]=cbf_evaluate_case(mode,shape,kind,x,s,obs);
           H=cell(5,1);
           for v=1:5
              J=cache.jets{v};
              H{v}=cbf_geometry_taylor(shape,kind,J.pLoad,J.qCable,J.pDrone,P,O,plan,'all');
           end
           for j=1:numel(info.labels)
              if strcmp(mode,'real'),r=6;else,r=info.orderHint(j);end
              first=zeros(1,4);coeffAtDesign=zeros(1,4);maxEarly=0;
              for v=1:4
                  effect=zeros(7,1);
                  for k=0:6
                     effect(k+1)=factorial(k)*(H{v+1}(j,k+1)-H{1}(j,k+1));
                  end
                  tol=1e-8*(1+max(abs(effect)));
                  e=find(abs(effect)>tol,1);
                  if ~isempty(e),first(v)=e-1;else,first(v)=NaN;end
                  coeffAtDesign(v)=effect(r+1);
                  maxEarly=max(maxEarly,max(abs(effect(1:r))));
              end
              idx=idx+1;
              rows(end+1,:)={idx,mode,shape,kind,i,info.labels{j},r,...
                  mat2str(first,8),norm(coeffAtDesign),...
                  maxEarly,norm(coeffAtDesign)<=s.cbf.degeneracyTolerance,...
                  double(any(first<r)),mat2str(coeffAtDesign,8)}; %#ok<AGROW>
           end
       end
    end
 end
end
reports.relativeDegree=cell2table(rows,'VariableNames',...
  {'caseId','mode','systemShape','obstacleType','stateId','barrierLabel',...
   'designOrder','firstEffectOrderPerInput','effectNormAtDesign',...
   'maxEffectBelowDesignOrder','degenerateAtDesignOrder',...
   'inputAppearsBeforeDesignOrder','designInputCoefficients'});
folder=fullfile(paths.generator,'CBF','test_results');if ~exist(folder,'dir'),mkdir(folder);end
writetable(reports.relativeDegree,fullfile(folder,'audit4_relative_degree_channels.csv'));
% 段ごとゲインの効果。力学/地面制約のAと実入力限界CBFは保持する。
patterns={repmat(1.2,1,6),repmat(2,1,6),repmat(3,1,6),...
    [1.2 1.6 2 2.4 2.8 3.2],[3.2 2.8 2.4 2 1.6 1.2]};
patternNames={'uniform_1p2','uniform_2','uniform_3',...
    'ascending','descending'};
scenes=[1.2 -.65 0 0;1.5 0 0 0;1.5 -.4 .15 .1;2 .5 .2 .12];
if ~full,scenes=scenes(1:2,:);end
rows=cell(0,15); idx=0;
for sh=shapes
 shape=sh{1};
 for j=1:size(scenes,1)
   c=scenes(j,:);x=CBF_Audit_State('real',s,c(1),c(2),0,c(3),c(4),0,0);
   obs=CBF_Audit_State('obstacle','sphere',x(8:10),100,'front',0);
   % 基準を共通ゲイン1.2に固定。これは試験のためで本番設定は不変。
   st=s;st.cbf.groundGain=1.2;
   qp=cbf_hocbf_scene('real',shape,x,st,{obs});
   for p=1:numel(patterns)
      [alt,chk]=CBF_Audit_StageGainRebuild(qp,shape,x,st,obs,patterns{p});
      if p==1
          % 等ゲインの再構成は既存HOCBFと数値一致しなければならない。
          errPsi=max(cellfun(@(a,b)norm(a-b,Inf),qp.psi,alt.psi));
          if norm(qp.b-alt.b,Inf)>1e-6*(1+norm(qp.b,Inf)) || errPsi>1e-6
              error('CBF:AuditStageMismatch','段別ゲインの1.2統一が既存CBFと不一致');
          end
      end
      gg=contains(alt.labels,'ground_');
      ii=contains(alt.labels,'_thrust_')|contains(alt.labels,'_tau_');
      rG=CBF_Audit_LP(alt,gg);rI=CBF_Audit_LP(alt,ii);
      rGI=CBF_Audit_LP(alt,gg|ii);
      idx=idx+1;
      rows(end+1,:)={idx,shape,j,patternNames{p},...
          mat2str(patterns{p},5),all(alt.psiInitiallyNonnegative(gg)),...
          all(alt.psiInitiallyNonnegative(ii)),rG.feasible,rI.feasible,rGI.feasible,...
          min(cellfun(@min,alt.psi(gg))),chk.coefficientError,...
          chk.maxBChange,chk.groundRowsModified, rGI.exitflag}; %#ok<AGROW>
   end
 end
end
reports.gainSweep=cell2table(rows,'VariableNames',...
 {'caseId','systemShape','stateId','gainPattern','stageGains',...
 'initialGroundPsiOK','initialInputPsiOK','groundOnlyFeasible',...
 'inputOnlyFeasible','groundPlusInputFeasible','minGroundPsi',...
 'differenceInA','changeInBFromBaseline','changedGroundRows','LPexitflag'});
writetable(reports.gainSweep,fullfile(folder,'audit4_stage_gain_comparison.csv'));
fprintf('[次数/ゲイン] 微分行%d、早期入力作用%d、退化%d、段別ゲイン試験%d\n',...
    size(reports.relativeDegree,1),sum(reports.relativeDegree.inputAppearsBeforeDesignOrder),...
    sum(reports.relativeDegree.degenerateAtDesignOrder),size(reports.gainSweep,1));
end
function v=getopt(o,k,d),if isfield(o,k)&&~isempty(o.(k)),v=o.(k);else,v=d;end,end
