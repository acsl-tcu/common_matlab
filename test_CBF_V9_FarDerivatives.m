function report=test_CBF_V9_FarDerivatives(options)
% [1] 遠方障害物の6階CBFを階ごとに分解。相対次数退化/ゼロ行を診断。
% 100mの行を除外しない。入力0の残差、LP、QPと生微分の内訳を保存。
if nargin<1,options=struct();end
s=CBF_V9_Util('options',options,'settings',test_CBF_Fixture());
full=CBF_V9_Util('options',options,'full',false);
if full,postures=[0,.12,.2];tilts=[0,.12];distances=[1,3,10,100];
else,postures=[0,.2];tilts=0;distances=[1,100];end
folder=CBF_V9_Util('folder');
rows=cell(0,29); terms=cell(0,19); qprows=cell(0,14); failures=cell(0,7);id=0;tid=0;
for pitch=postures
 for tilt=tilts
  for shapeCell={'B','D'}
   shape=shapeCell{1};
   for kindCell={'sphere','ellipsoid'}
    kind=kindCell{1};
    for dist=distances
     id=id+1;err='';
     
     try
      x=CBF_Audit_State('real',s,1.5,-.25,0,pitch,tilt,.2,0);
      obs=CBF_Audit_State('obstacle',kind,x(8:10),dist,'front',.7);
      qp=cbf_hocbf_scene('real',shape,x,s,{obs});
      [gm,im,om]=CBF_V9_Util('masks',qp);
      lpBase=CBF_Audit_LP(qp,gm|im);lpFull=CBF_Audit_LP(qp);
      zlp=nan(4,1);if lpBase.feasible,zlp=lpBase.nu;end
      solFlag=NaN;solResidual=NaN;solNorm=NaN;solStatus='not_run';
      try
       sol=cbf_solve_hocbf_qp(qp,zeros(4,1),eye(4));
       solFlag=sol.exitflag;
       if sol.exitflag>0 && ~isempty(sol.nu)
        solResidual=min((qp.A*sol.nu-qp.b)./(1+abs(qp.b)));
        solNorm=norm(sol.nu,Inf);
        if solResidual>=-1e-8,solStatus='solved';else,solStatus='negative_residual';end
       else,solStatus='QP_no_solution';end
      catch MEs
       solStatus=['QP_MATLAB_ERROR: ' MEs.identifier];
      end
      bad=(qp.effectNorm<=s.cbf.degeneracyTolerance)&(qp.b>0);
      maxBadB=0;if any(bad),maxBadB=max(qp.b(bad));end
      qprows(end+1,:)={id,pitch,tilt,shape,kind,dist,...
           lpBase.feasible,lpFull.feasible,solFlag,solResidual,solNorm,...
           sum(bad),maxBadB,solStatus}; %#ok<AGROW>
      P=cbf_parameter_vector(s);O=cbf_obstacle_vector(obs,kind);
      cablePlan=cbf_choose_cable_spheres(s);
      z0=cbf_real_state_taylor(x,zeros(4,1),P,6);
      p0=z0(8:10,:);q0=z0(14:16,:);d0=p0-P(3)*q0;
      H0=cbf_geometry_taylor(shape,kind,p0,q0,d0,P,O,cablePlan,'all');
      [~,ginfo]=cbf_evaluate_case('real',shape,kind,x,s,obs);
      for j=find(om(:)).'
        lab=qp.labels{j}; rawLabel=regexprep(lab,'^obs1_','');
        local=find(strcmp(ginfo.labels,rawLabel),1);
        if isempty(local),error('CBF:V9Label','行ラベルが一致しません: %s',lab);end
        r=qp.order(j); d=H0(local,1:r+1).*factorial(0:r);
        gain=s.cbf.hocbfGain;
        part=zeros(1,r+1);
        for k=0:r,part(k+1)=nchoosek(r,k)*gain^(r-k)*d(k+1);end
        reconstructed=sum(part);
        rowResidualAtBase=NaN;
        if lpBase.feasible,rowResidualAtBase=qp.A(j,:)*zlp-qp.b(j);end
        qprow=qp.A(j,:);aNorm=norm(qprow);
        deg=aNorm<=s.cbf.degeneracyTolerance;
        % 第1～6階の入力依存を実験的に数える。
        % 同じモデルの基底テイラー差分であり独立した相対次数の証明ではない。
        inputAtOrder=zeros(1,7);
        for inp=1:4
          nu=zeros(4,1);nu(inp)=1;
          z=cbf_real_state_taylor(x,nu,P,6);
          pl=z(8:10,:);qc=z(14:16,:);pd=pl-P(3)*qc;
          Hb=cbf_geometry_taylor(shape,kind,pl,qc,pd,P,O,cablePlan,'all');
          inputAtOrder=max(inputAtOrder,abs((Hb(local,:)-H0(local,:)).*factorial(0:6)));
        end
        observed=find(inputAtOrder>1e-8,1,'first')-1;
        if isempty(observed),observed=NaN;end
        rows(end+1,:)={id,pitch,tilt,shape,kind,dist,lab,r,...
           d(1),d(2),d(3),d(4),d(5),d(6),d(7),...
           part(1),part(2),part(3),part(4),part(5),part(6),part(7),...
           reconstructed,-qp.b(j),max(abs(qprow)),deg,observed,...
           lpBase.feasible,lpFull.feasible}; %#ok<AGROW>
        for k=0:r
           tid=tid+1;
           terms(end+1,:)={tid,id,lab,shape,kind,dist,pitch,tilt,k,...
             d(k+1),part(k+1),gain,nchoosek(r,k),r-k,...
             aNorm,qp.b(j),-qp.b(j),reconstructed,rowResidualAtBase}; %#ok<AGROW>
        end
      end
     catch ME
      err=[ME.identifier ': ' ME.message];
      failures(end+1,:)={id,pitch,tilt,shape,kind,dist,err}; %#ok<AGROW>
      warning('CBF:V9FarCase','case %d: %s',id,err);
     end
    end
   end
  end
 end
end
report=cell2table(rows,'VariableNames',{'caseId','pitch','tilt','shape','obstacleType',...
 'distance','label','order','h0','h1','h2','h3','h4','h5','h6',...
 'term0','term1','term2','term3','term4','term5','term6',...
 'hocbfAtZero','negativeB','maxAbsA','degenerate','firstInputDerivativeOrder',...
 'groundInputLPFeasible','fullLPFeasible'});
byTerm=cell2table(terms,'VariableNames',{'termId','caseId','label','shape','obstacleType','distance',...
 'pitch','tilt','derivativeOrder','rawDerivative','hocbfContribution',...
 'gain','binomial','gainExponent','inputEffectNorm','b','zeroInputResidual',...
 'sumAllContributions','groundInputWitnessResidual'});
writetable(report,fullfile(folder,'v9_far_derivatives.csv'));
writetable(byTerm,fullfile(folder,'v9_far_derivative_terms.csv'));
qptable=cell2table(qprows,'VariableNames',{'caseId','pitch','tilt','shape',...
 'obstacleType','distance','groundInputLPFeasible','fullLPFeasible',...
 'QPexitflag','minimumScaledQPResidual','maximumQPDecision','uncontrollableRows',...
 'maxImpossiblePositiveB','QPstatus'});
writetable(qptable,fullfile(folder,'v9_far_qp_status.csv'));
if ~isempty(failures)
   failed=cell2table(failures,'VariableNames',{'caseId','pitch','tilt',...
     'shape','obstacleType','distance','errorText'});
   writetable(failed,fullfile(folder,'v9_far_failures.csv'));
end
if isempty(rows),error('CBF:V9FarNoResults','遠方障害物の有効な解析結果がありません');end
if ~isempty(report)
  faulty=report.degenerate & report.negativeB<0;
  fprintf('[V9遠方] %d行、退化かつA=0,b>0に相当: %d行\n',height(report),sum(faulty));
end
end
