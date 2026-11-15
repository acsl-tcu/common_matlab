function result=test_CBF_V10_Smoke(options)
%TEST_CBF_V10_SMOKE 添字修正・基準物理モデル・参照高次微分を事前確認。
if nargin<1,options=struct();end
test_CBF_SetupPaths();
if isfield(options,'settings'),s=options.settings;else,s=test_CBF_Fixture();end
x=CBF_Audit_State('real',s,.45,0,0,0,0,0,0);
q=CBF_V9_Util('groundqp','real','B',x,s);
assert(size(q.A,1)==numel(q.b) && size(q.A,1)==numel(q.labels) ...
       && size(q.A,1)==numel(q.psi),'CBF:V10SmokeRows','QP行が不整合');
assert(numel(q.earlyInputResidual)==sum(~contains(string(q.labels),'thrust_') ...
       & ~contains(string(q.labels),'tau_')),'CBF:V10SmokeEarly','早期入力行数不整合');
assert(all(isfinite(q.A(:))) && all(isfinite(q.b)),'CBF:V10SmokeFinite','QPに非有限値');
f=test_CBF_IndependentPlantRHS(x,zeros(4,1),s);
assert(norm(f,Inf)<1e-8,'CBF:V10SmokeHover','物理ホバリング微分がゼロでない');
for k=1:6
 assert(norm(CBF_V10_Reference('lowcycle',0,k,options),Inf)<1e-9, ...
  'CBF:V10SmokeReference','初期の高次参照微分がゼロでない');
end
result=table(size(q.A,1),numel(q.earlyInputResidual),norm(f,Inf),...
 'VariableNames',{'groundAndInputRows','earlyInputResidualRows','hoverRhsNorm'});
writetable(result,fullfile(CBF_V9_Util('folder'),'v10_smoke.csv'));
fprintf('[V10 smoke] QP抽出/ホバリング/参照高次導関数 PASS\n');
end
