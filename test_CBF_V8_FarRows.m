function report=test_CBF_V8_FarRows(options)
% V8: 遠い障害物を無条件に無視せず、100m時のQP追加行を生の残差で調べる。
% 地面＋入力制限のLP witnessが障害物行を満たすかも記録する。
if nargin<1,options=struct();end
paths=test_CBF_SetupPaths();s=test_CBF_Fixture();
if isfield(options,'settings'),s=options.settings;end
folder=fullfile(paths.generator,'CBF','test_results');if ~exist(folder,'dir'),mkdir(folder);end
full=getOpt(options,'full',false);
if full,distances=[1 3 10 100];else,distances=[3 100];end
rows=cell(0,19);rid=0;
for gain=[1.2 3]
 for posture=[0 .2]
 for tilt=[0 .12]
  for kindCell={'sphere','ellipsoid'}
   kind=kindCell{1};
   for shapeCell={'B','D'}
    shape=shapeCell{1};
    for dist=distances
     st=s;st.cbf.groundGain=gain;
     x=CBF_Audit_State('real',st,1.5,-.25,0,posture,tilt,.2,0);
     obs=CBF_Audit_State('obstacle',kind,x(8:10),dist,'front',.7);
     qp=cbf_hocbf_scene('real',shape,x,st,{obs});
     gm=contains(qp.labels,'ground_');
     im=contains(qp.labels,'_thrust_')|contains(qp.labels,'_tau_');
     om=~(gm|im);
     lpBase=CBF_Audit_LP(qp,gm|im);
     lpFull=CBF_Audit_LP(qp);
     for i=find(om(:)).'
       rid=rid+1;
       baseWitnessRes=NaN;
       if lpBase.feasible,baseWitnessRes=qp.A(i,:)*lpBase.nu-qp.b(i);end
       rows(end+1,:)={rid,gain,posture,tilt,shape,kind,dist,qp.labels{i},...
         qp.order(i),qp.A(i,1),qp.A(i,2),qp.A(i,3),qp.A(i,4),...
         qp.b(i),norm(qp.A(i,:)),min(qp.psi{i}),baseWitnessRes,...
         lpBase.feasible,lpFull.feasible}; %#ok<AGROW>
     end
    end
   end
  end
 end
 end
end
report=cell2table(rows,'VariableNames',{'rowId','groundGain','pitch','cableTilt',...
 'systemShape','obstacleType','obstacleDistance','label','order',...
 'A1','A2','A3','A4','b','effectNorm','minimumPsi',...
 'groundAndInputWitnessResidual','groundAndInputFeasible','fullQPFeasible'});
writetable(report,fullfile(folder,'v8_far_obstacle_row_audit.csv'));
fprintf('[V8 遠方障害物] %d行。100mでも障害物のCBFを消さず解析しました。\n',height(report));
end
function v=getOpt(a,f,d)
if isfield(a,f)&&~isempty(a.(f)),v=a.(f);else,v=d;end
end
