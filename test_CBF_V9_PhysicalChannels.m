function report=test_CBF_V9_PhysicalChannels(options)
% [1b] 実入力T,tau（拡張決定入力nuとは別）が障害物hの生微分に及ぼす効果。
% 同じ状態のT/tauを中央差分で摂動しh^(0..6)を比較。
% ゼロの判定は数値検証であり、全状態の相対次数の証明ではない。
if nargin<1,options=struct();end
s=CBF_V9_Util('options',options,'settings',test_CBF_Fixture());
full=CBF_V9_Util('options',options,'full',false);
if full,pitches=[0 .12 .2];tilts=[0 .12];
else,pitches=[0 .2];tilts=0;end
P=cbf_parameter_vector(s);plan=cbf_choose_cable_spheres(s);
rows=cell(0,16);caseId=0;
for pitch=pitches
 for tilt=tilts
  x=CBF_Audit_State('real',s,1.5,-.25,0,pitch,tilt,.2,0);
  for dist=[1 100]
   for shapeCell={'B','D'}
    shape=shapeCell{1};
    for kindCell={'sphere','ellipsoid'}
     kind=kindCell{1};
     obs=CBF_Audit_State('obstacle',kind,x(8:10),dist,'front',.7);
     O=cbf_obstacle_vector(obs,kind);
     [~,meta]=cbf_evaluate_case('real',shape,kind,x,s,obs);
     rid=find(strcmp(meta.labels,'obstacle_load'),1);
     if isempty(rid),error('CBF:V9PhysicalLabel','吊り荷行がありません');end
     base=localRaw(x,P,plan,O,shape,kind,rid);
     for component=1:4
       caseId=caseId+1;
       if component==1,loc=20;name='thrust_T';
       else,loc=22+component;name=sprintf('torque_%d',component-1);end
       delta=1e-4*max(1,abs(x(loc)));
       xp=x;xm=x;xp(loc)=xp(loc)+delta;xm(loc)=xm(loc)-delta;
       hp=localRaw(xp,P,plan,O,shape,kind,rid);
       hm=localRaw(xm,P,plan,O,shape,kind,rid);
       sensitivity=(hp-hm)/(2*delta);
       % 未正規化であるh^(j)の物理状態T/tauへの偏微分。
       first=find(abs(sensitivity)>1e-7*(1+abs(base)),1);
       if isempty(first),first=NaN;else,first=first-1;end
       rows(end+1,:)={caseId,pitch,tilt,dist,shape,kind,name,delta,...
          first,sensitivity(1),sensitivity(2),sensitivity(3),...
          sensitivity(4),sensitivity(5),sensitivity(6),sensitivity(7)}; %#ok<AGROW>
     end
    end
   end
  end
 end
end
report=cell2table(rows,'VariableNames',{'caseId','pitch','cableTilt','distance',...
  'shape','obstacleType','physicalInput','perturbation','firstObservedOrder',...
  'sensitivityH0','sensitivityH1','sensitivityH2','sensitivityH3',...
  'sensitivityH4','sensitivityH5','sensitivityH6'});
writetable(report,fullfile(CBF_V9_Util('folder'),'v9_far_physical_channels.csv'));
fprintf('[V9物理作用] %d行（T/tauの状態摂動。nuRと混同禁止）\n',height(report));
end
function d=localRaw(x,P,plan,O,shape,kind,rid)
Z=cbf_real_state_taylor(x,zeros(4,1),P,6);
pl=Z(8:10,:);qc=Z(14:16,:);pd=pl-P(3)*qc;
H=cbf_geometry_taylor(shape,kind,pl,qc,pd,P,O,plan,'all');
d=H(rid,:).*factorial(0:6);
end
