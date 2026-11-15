function report=test_CBF_V10_FarNormalization(options)
%TEST_CBF_V10_FARNORMALIZATION 旧hとh/sqrt(||c-p||^2+eps^2)を同一状態で比較。
% HOCBF階数6、gain同一。QP本体は変更しない。球（機体/荷）行だけ評価。
% 両者のh>=0は同じだが高階初期集合・QP制約は異なるので安全性未保証。
if nargin<1,options=struct();end
s=localOption(options,'settings',test_CBF_Fixture());
P=cbf_parameter_vector(s);plan=cbf_choose_cable_spheres(s);
rows=cell(0,18);id=0;
for pitch=[0 .2]
 for distance=[1 3 10 100]
  for shapeCell={'B','D'}
   shape=shapeCell{1};
   for kindCell={'sphere','ellipsoid'}
    kind=kindCell{1};
    x=CBF_Audit_State('real',s,1.5,-.25,0,pitch,0,.2,0);
    obs=CBF_Audit_State('obstacle',kind,x(8:10),distance,'front',.7);
    O=cbf_obstacle_vector(obs,kind);
    qp=cbf_hocbf_scene('real',shape,x,s,{obs});
    z=cell(5,1);geom=cell(5,1);
    for inp=0:4
     e=zeros(4,1);if inp>0,e(inp)=1;end
     z{inp+1}=cbf_real_state_taylor(x,e,P,6);
     pl=z{inp+1}(8:10,:);qc=z{inp+1}(14:16,:);
     pd=pl-P(3)*qc;
     geom{inp+1}=cbf_geometry_taylor(shape,kind,pl,qc,pd,P,O,plan,'all');
    end
    [~,info]=cbf_evaluate_case('real',shape,kind,x,s,obs);
    for target={'obstacle_drone','obstacle_load'}
     name=target{1};
     local=find(strcmp(info.labels,name),1);
     globalidx=find(strcmp(qp.labels,['obs1_' name]),1);
     if isempty(local)||isempty(globalidx),error('CBF:V10FarLabel','%sがありません',name);end
     normalized=cell(5,1);
     for inp=1:5
       if strcmp(name,'obstacle_drone')
         ps=z{inp}(8:10,:)-P(3)*z{inp}(14:16,:);
       else
         ps=z{inp}(8:10,:);
       end
       normalized{inp}=localNormalizedJet(geom{inp}(local,:),ps,obs.center(:),s.cbf.smoothEpsilon);
     end
     drift=normalized{1}.*factorial(0:6);
     gain=s.cbf.hocbfGain;
     lower=0;
     for j=0:6
       lower=lower+nchoosek(6,j)*gain^(6-j)*drift(j+1);
     end
     a=zeros(1,4);
     for inp=1:4
       a(inp)=factorial(6)*(normalized{inp+1}(7)-normalized{1}(7));
     end
     old=qp.A(globalidx,:);bOld=qp.b(globalidx);
     oldPsi=qp.psi{globalidx};
     newPsi=zeros(6,1);
     for r=0:5
      for j=0:r
       newPsi(r+1)=newPsi(r+1)+nchoosek(r,j)*gain^(r-j)*drift(j+1);
      end
     end
     id=id+1;
     rows(end+1,:)={id,shape,kind,pitch,distance,name, ...
       geom{1}(local,1),normalized{1}(1),norm(old),norm(a),...
       bOld,-lower,min(oldPsi),min(newPsi),...
       norm(old)<=s.cbf.degeneracyTolerance && bOld>0, ...
       norm(a)<=s.cbf.degeneracyTolerance && -lower>0,...
       -bOld,lower}; %#ok<AGROW>
    end
   end
  end
 end
end
report=cell2table(rows,'VariableNames',{'caseId','shape','obstacleType','pitch',...
 'distance','rowLabel','oldH','normalizedH','oldANorm','normalizedANorm',...
 'oldB','normalizedB','oldMinPsi','normalizedMinPsi',...
 'oldZeroApositiveB','normalizedZeroApositiveB','oldAtZeroResidual',...
 'normalizedAtZeroResidual'});
writetable(report,fullfile(CBF_V9_Util('folder'),'v10_far_normalization_comparison.csv'));
fprintf('[V10遠方正規化候補] %d行、旧/新の高階初期集合とQP係数を比較。CBF本体変更なし。\n',size(report,1));
end
function H=localNormalizedJet(Hraw,p,center,epsv)
% Taylor係数の演算: sqrt(Σ n_j*n_{k-j}+eps^2)、その後Hraw/d。
N=-p;N(:,1)=center-p(:,1);
S=zeros(1,7);D=zeros(1,7);H=zeros(1,7);
for k=0:6
 for i=0:k,S(k+1)=S(k+1)+N(:,i+1).'*N(:,k-i+1);end
end
S(1)=S(1)+epsv^2;
D(1)=sqrt(S(1));
if D(1)<=0,error('CBF:V10NormDenom','距離分母がゼロです');end
for k=1:6
 q=0;
 for i=1:k-1,q=q+D(i+1)*D(k-i+1);end
 D(k+1)=(S(k+1)-q)/(2*D(1));
end
for k=0:6
 q=0;
 for i=1:k,q=q+D(i+1)*H(k-i+1);end
 H(k+1)=(Hraw(k+1)-q)/D(1);
end
end
function v=localOption(s,name,def)
if isfield(s,name)&&~isempty(s.(name)),v=s.(name);else,v=def;end
end
