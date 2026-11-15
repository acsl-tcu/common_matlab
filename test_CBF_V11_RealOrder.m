function report=test_CBF_V11_RealOrder(opts)
%TEST_CBF_V11_REALORDER  R1/R2の部位別・入力別の初回作用階数を数値監査。
% 「初めて非ゼロになる次数」は局所状態での実測であり、大域的な証明ではない。
if nargin<1,opts=struct();end
test_CBF_SetupPaths();s=test_CBF_Fixture();
if isfield(opts,'full') && opts.full, distances=[0.8 3 100]; cases=1:4;
else,distances=[1 100]; cases=1:3;end
shapes={'C','B','D'};kinds={'sphere','ellipsoid'};
rows=cell(0,10);irow=0;
% 候補のパリティ：R1/R2を元の29状態Taylorと突き合わせる。
P=cbf_parameter_vector(s);
for cid=cases
 [x19,u,~]=CBF_V11_Examples(cid,s,'sphere',1);
 x23=[x19;u(1);zeros(3,1)];
 x29=[x23;u(2:4);zeros(3,1)];
 Z1=CBF_V11_RealTaylor('R1',x19,u,s,6);
 Z2=CBF_V11_RealTaylor('R2',x23,[0;u(2:4)],s,6);
 Z29=cbf_real_state_taylor(x29,zeros(4,1),P,6);
 tol=1e-8*(1+norm(Z29(:),Inf));
 assert(norm(Z1-Z29(1:19,:),Inf)<tol,'R1と元の29状態モデルが異なります。');
 assert(norm(Z2-Z29(1:23,:),Inf)<tol,'R2と元の29状態モデルが異なります。');
end
fprintf('[V11] R1/R2の物理モデルは元の29状態Taylorと一致しました。\n');
for cid=cases
 for si=1:numel(shapes)
  for ki=1:numel(kinds)
   for distance=distances
    shape=shapes{si};kind=kinds{ki};
    [x19,u,obs]=CBF_V11_Examples(cid,s,kind,distance);
    for model={'R1','R2'}
     mode=model{1};
     if strcmp(mode,'R1'),xx=x19; u0=u;else,xx=[x19;u(1);zeros(3,1)];u0=[0;u(2:4)];end
     [H0,labels]=CBF_V11_JetRows(mode,shape,xx,u0,s,obs,6);
     sensitivity=zeros(size(H0,1),7,4);
     for channel=1:4
       step=1e-3;if channel==1,step=1e-2;end
       direction=zeros(4,1);direction(channel)=step;
       Hp=CBF_V11_JetRows(mode,shape,xx,u0+direction,s,obs,6);
       Hm=CBF_V11_JetRows(mode,shape,xx,u0-direction,s,obs,6);
       sensitivity(:,:,channel)=(Hp-Hm)/(2*step).*factorial(0:6);
     end
     for idx=1:numel(labels)
       for channel=1:4
         vals=reshape(sensitivity(idx,:,channel),1,[]);
         scale=max(abs(vals));
         % 単位の異なる階数で一律絶対閾値は使わない。検出閾値はCSVへ記録。
         threshold=max(1e-9,1e-8*scale);
         k=find(abs(vals(2:end))>threshold,1,'first');
         if isempty(k),first=NaN;else,first=k;end
         irow=irow+1;
         rows(irow,:)={mode,shape,kind,cid,distance,labels{idx},channel,first,scale,threshold};
       end
     end
    end
   end
  end
 end
end
report=cell2table(rows,'VariableNames', ...
 {'model','shape','obstacleType','stateId','distance','barrier','channel', ...
  'firstNonzeroOrder','maxAbsSensitivity','detectionThreshold'});
output=CBF_V11_OutputFolder();
writetable(report,fullfile(output,'v11_real_component_action_orders.csv'));
% ここで「4/6の大域証明」と判定しない。退化した行もすべてCSVへ出す。
fprintf('[V11] 実入力R1/R2の作用階数：%d行を保存\n',height(report));
end
