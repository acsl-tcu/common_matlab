function result=test_CBF_V11_Smoke()
%TEST_CBF_V11_SMOKE インストール直後の簡易試験（モデル一致・幾何一致）。
test_CBF_SetupPaths();s=test_CBF_Fixture();
[x19,u,obs]=CBF_V11_Examples(1,s,'sphere',1);
x23=[x19;u(1);zeros(3,1)];x29=[x23;u(2:4);zeros(3,1)];
P=cbf_parameter_vector(s);
Z0=cbf_real_state_taylor(x29,zeros(4,1),P,6);
Z1=CBF_V11_RealTaylor('R1',x19,u,s,6);
Z2=CBF_V11_RealTaylor('R2',x23,[0;u(2:4)],s,6);
assert(norm(Z1-Z0(1:19,:),Inf)<1e-7, 'R1のモデル不一致');
assert(norm(Z2-Z0(1:23,:),Inf)<1e-7, 'R2のモデル不一致');
for sh={'C','B','D'}
 [H,labels]=CBF_V11_JetRows('R2',sh{1},x23,[0;0;0;0],s,obs,6);
 [h,~]=cbf_evaluate_case('real',sh{1},obs.type,x29,s,obs);
 assert(numel(labels)==numel(h));
 assert(norm(H(:,1)-h,Inf)<1e-7, '幾何h不一致 %s',sh{1});
end
result=struct('status','PASS','modelEquality',true,'geometryEquality',true);
fprintf('[PASS] V11 smoke：R1/R2運動方程式＋C/B/D幾何一致。\n');
end
