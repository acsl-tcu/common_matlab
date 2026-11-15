function report=test_CBF_V9_LowHover(options)
% [2] 0.15～0.50mの幾何高度、HOCBF初期集合、地面+入力QPを分離。
% ダミー障害物行は厳格に除外し、地面/入力だけの診断を実施。
if nargin<1,options=struct();end
s=CBF_V9_Util('options',options,'settings',test_CBF_Fixture());
full=CBF_V9_Util('options',options,'full',false);
if full,alts=[.15 .18 .20 .25 .30 .40 .50];vzs=[0,-.1,-.35,-.65];
 pitches=[0 .15];tilts=[0 .12];gains=[1.2 2 3];shapes={'B','D'};
else,alts=[.15 .20 .30 .50];vzs=[0,-.25];
 pitches=[0 .15];tilts=0;gains=[1.2 3];shapes={'B','D'};end
rows=cell(0,19);id=0;
for sh=shapes
 shape=sh{1};
 for gain=gains
  for z=alts
   for vz=vzs
    for pitch=pitches
     for tilt=tilts
      id=id+1;errorText='';gmin=NaN;psimin=NaN;ipsimin=NaN;
      gpsi=false;ipsi=false;gf=false;inputFeasible=false;both=false;zero=false;
      tReq=NaN;positiveT=false;
      try
       st=s;st.cbf.groundGain=gain;
       x=CBF_Audit_State('real',st,z,vz,0,pitch,tilt,0,0);
       q=CBF_V9_Util('groundqp','real',shape,x,st);
       [gm,im]=CBF_V9_Util('masks',q);
       gmin=min(CBF_V9_Util('groundmargin',x,st));
       psimin=min(cellfun(@min,q.psi(gm)));ipsimin=min(cellfun(@min,q.psi(im)));
       gpsi=all(q.psiInitiallyNonnegative(gm));
       ipsi=all(q.psiInitiallyNonnegative(im));
       lpG=CBF_Audit_LP(q,gm);gf=lpG.feasible;
       lpI=CBF_Audit_LP(q,im);inputFeasible=lpI.feasible;
       lpBoth=CBF_Audit_LP(q,gm|im);both=lpBoth.feasible;
       zero=CBF_V9_Util('feasiblezero',q);
       d=cbf_model_diagnostics('real',x,st);
       tReq=x(20);positiveT=d.tension>0;
      catch ME
       errorText=[ME.identifier ': ' ME.message];
      end
      rows(end+1,:)={id,shape,gain,z,vz,pitch,tilt,gmin,psimin,ipsimin,...
       gpsi,ipsi,gf,inputFeasible,both,zero,tReq,positiveT,errorText}; %#ok<AGROW>
     end
    end
   end
  end
 end
end
report=cell2table(rows,'VariableNames',{'caseId','shape','groundGain','loadAltitude',...
 'loadVz','pitch','cableTilt','minPhysicalGroundMargin','minimumGroundPsi',...
 'minimumInputPsi','groundPsiOK','inputPsiOK','groundOnlyLPFeasible',...
 'inputOnlyLPFeasible','bothHardLPFeasible','zeroDecisionFeasible',...
 'requiredInitialThrust','positiveTension','errorText'});
writetable(report,fullfile(CBF_V9_Util('folder'),'v9_low_hover_grid.csv'));
fprintf('[V9低高度] %d条件、地面/入力ハード両立 %d、補助初期集合と両立 %d\n',...
 height(report),sum(report.bothHardLPFeasible),...
 sum(report.bothHardLPFeasible & report.groundPsiOK & report.inputPsiOK));
end
