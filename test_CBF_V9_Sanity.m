function report=test_CBF_V9_Sanity(options)
% 実験前の自己検算。生成器を再生成しない。
if nargin<1,options=struct();end
s=CBF_V9_Util('options',options,'settings',test_CBF_Fixture());
q=CBF_Audit_State('real',s,.45,0,0,0,0,0,0);
assert(numel(q)==29 && abs(q(20)-(s.physical.massDrone+s.physical.massLoad)*s.physical.gravity)<1e-9,...
 'CBF:V9HoverThrust','ホバリングの初期推力が平衡値ではありません');
model=cbf_model_diagnostics('real',q,s);
assert(model.tension>0 && model.thrustInBounds && model.torqueInBounds,...
 'CBF:V9HoverValidity','初期平衡モデルが不正');
f=test_CBF_IndependentPlantRHS(q,zeros(4,1),s);
assert(norm(f,Inf)<1e-8,'CBF:V9HoverEquilibrium','元モデルのホバリング平衡ではありません');
phase=CBF_V9_Util('options',options,'phaseDuration',1.2);
maxErr=0;
for k=0:6
 a=CBF_V9_LowReference(0,k,options);
 b=CBF_V9_LowReference(3*phase,k,options);
 if k==0
    zH=CBF_V9_Util('options',options,'zHigh',.45);
    dx=CBF_V9_Util('options',options,'travelX',.50);
    targetA=[0;0;zH];targetB=[dx;0;zH];
 else,targetA=zeros(3,1);targetB=zeros(3,1);end
 maxErr=max([maxErr;norm(a-targetA,Inf);norm(b-targetB,Inf)]);
end
assert(maxErr<1e-7,'CBF:V9RefEndpoint','参照軌道の端点微分が不整合');
% 0..6階までの有限差分検証: 6次入力そのものは正確な解析式を前提とする。
h=1e-4;t=.3*phase;derivErr=0;
for k=0:4
 a=CBF_V9_LowReference(t+h,k,options);
 b=CBF_V9_LowReference(t-h,k,options);
 analytic=CBF_V9_LowReference(t,k+1,options);
 derivErr=max(derivErr,norm((a-b)/(2*h)-analytic,Inf)/(1+norm(analytic,Inf)));
end
assert(derivErr<1e-4,'CBF:V9RefDifferentiation','参照微分が不整合');
report=table(maxErr,derivErr,model.tension,...
 'VariableNames',{'maxEndpointDerivativeError','maxFiniteDifferenceRelativeError','hoverTension'});
writetable(report,fullfile(CBF_V9_Util('folder'),'v9_sanity.csv'));
fprintf('[V9 自己検算] ホバリング平衡と参照微分の検証に合格\n');
end
