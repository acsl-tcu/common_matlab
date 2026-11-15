function result=test_CBF_V14_Smoke()
%TEST_CBF_V14_SMOKE Exercise row-guard logic on the known V13 anomaly.
% Does not claim collision avoidance or QP/physical feasibility.
test_CBF_SetupPaths();s=test_CBF_Fixture();
[x,u,ob]=CBF_V11_Examples(2,s,'sphere',100);
Z=CBF_V11_RealTaylor('R1',x,u,s,6);
xv=zeros(20,1);
for j=0:5,xv(3*j+1:3*j+3)=factorial(j)*Z(8:10,j+1);end
D=CBF_V13_VirtualCoefficientAudit('B',xv,s,ob);
G=CBF_V14_RowGuard(D.A,D.b);
k=find(strcmp(D.labels,'ground_drone'));
assert(numel(k)==1,'Expected exactly one ground_drone barrier.');
assert(G.unresolvedPositiveRhs(k),...
 'Known V13 nearly-zero control row must be flagged, not solved with huge input.');
assert(D.b(k)>0 && G.rowNormInf(k)<1e-10);
% Pure algebra: positive-RHS exactly-zero row is impossible; row scaling
% cannot make it feasible. Diagnostic threshold is not a proof of exact zero.
z=CBF_V14_RowGuard([0 0 0 0],2.7342);
assert(z.hasUnresolved && isinf(z.minRequiredDerivativeNorm2(1)));
gok=CBF_V14_RowGuard([1 0 0 0],-1);
assert(~gok.hasUnresolved);
result=struct('status','PASS','groundDroneRHS',D.b(k),...
    'groundDroneRowInf',G.rowNormInf(k),...
    'requiredNormLowerBound',G.minRequiredDerivativeNorm2(k));
fprintf('[PASS] V14 numerical-degeneracy smoke: ground_drone A_inf=%.5g, b=%.5g.\n',...
 G.rowNormInf(k),D.b(k));
end
