function test_CBF_IndependentDynamics()
%TEST_CBF_INDEPENDENTDYNAMICS Taylor一次微分と別実装の運動方程式を比較。
test_CBF_SetupPaths();s=test_CBF_Fixture();P=cbf_parameter_vector(s);
rng(119);
for k=1:30
    x=zeros(29,1);
    q=randn(4,1);q=q/norm(q);x(1:4)=q;
    x(5:7)=0.3*randn(3,1);
    x(8:10)=[0;0;1.5]+0.3*randn(3,1);
    x(11:13)=randn(3,1)*0.1;
    % 今回の試験は正張力領域に限定する。機体推力方向のほぼ反対側に紐を置く。
    q0=q(1);v=q(2:4);
    skew=[0 -v(3) v(2);v(3) 0 -v(1);-v(2) v(1) 0];
    rot=(q0^2-v.'*v)*eye(3)+2*(v*v.')+2*q0*skew;
    qc=-rot(:,3)+0.08*randn(3,1);qc=qc/norm(qc);x(14:16)=qc;
    w=0.2*randn(3,1);x(17:19)=w-qc*(qc.'*w);
    x(20)=7+6*rand();x(21:23)=randn(3,1)*0.05;
    x(24:26)=randn(3,1)*0.1;x(27:29)=randn(3,1)*0.05;
    info=cbf_model_diagnostics('real',x,s);
    assert(info.tension>0,'内部テストの正張力条件が破れました。');
    nu=randn(4,1);
    jet=cbf_real_state_taylor(x,nu,P,1);
    independent=test_CBF_IndependentPlantRHS(x,nu,s);
    scale=1+norm(independent,Inf);
    assert(norm(jet(:,2)-independent,Inf)<1e-10*scale, ...
       '別実装の運動方程式とTaylor一次微分に差があります。ケース%d',k);
end
fprintf(' [PASS] 非平衡・任意姿勢の30状態で元の運動方程式と一致\n');
end
