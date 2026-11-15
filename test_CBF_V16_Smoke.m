function test_CBF_V16_Smoke()
%TEST_CBF_V16_SMOKE  independent algebraic invariants and model structure.
test_CBF_SetupPaths();s=test_CBF_Fixture();
[x,u]=CBF_V12_PureHover(s);
% Set hover exactly at the center of the low-altitude vertical funnel.
x(10)=s.ground.height+s.shape.loadRadius+s.cbf.safetyMargin+0.20;
assert(numel(x)==19);
xx=[x;u(1);zeros(3,1)];
Z=CBF_V11_RealTaylor('R2',xx,[2;0.1;-0.2;0.1],s,1);
assert(norm(Z(20:23,2)-[0;0;0;2])<1e-11,...
  'R2 must augment thrust only; torque directly drives angular acceleration.');
b=CBF_V16_R2_CascadeEllipsoid(xx,s);
assert(b.PcheckNorm<1e-8,'Lyapunov identity broken');
assert(b.verticalSurrogateValidAtState,...
    'At hover vertical surrogate should match original acceleration');
assert(b.insideSurrogateSet && b.cbfResidual>-1e-8,...
    'Surrogate CBF should be valid at hover');
high=CBF_V16_R2_HighOrderRectified(xx,s);
assert(high.hRectified>0 && high.QPResidual>=-1e-8,...
    'Higher-degree ReCBF on vertical surrogate should pass hover');
r=CBF_V16_R1_RectifiedGround('B',x,s);
assert(r.affinePass && r.minRectifiedH>0,...
    'R1 ground ReCBF at safe hover should be valid');
assert(r.rectifiedQPResidual>=-1e-6,'R1 rectified QP failed at hover');
ob=struct('type','sphere','center',[100;0;x(10)],'radius',0.25);
z=CBF_V16_ZOCBFStep('R2','B',xx,s,ob,struct('dt',0.025,'gamma',0.2,...
 'substeps',3,'maxIterations',15));
assert(strncmp(z.status,'NOMINAL_SAMPLED_PASS',20),...
 'R2 sampled ZOCBF nominal hovering should pass without optimizer');
fprintf('[PASS] V16 smoke: R2 thrust-only chain, exact vertical surrogate, R1 ReCBF, sampled ZOCBF.\n');
end
