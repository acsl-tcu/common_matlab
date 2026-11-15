function R=test_CBF_V13_Smoke()
%TEST_CBF_V13_SMOKE Source/shape regression. No safety guarantee.
a=test_CBF_V12_Smoke();
assert(isstruct(a) && strcmp(a.status,'PASS'));
test_CBF_SetupPaths();s=test_CBF_Fixture();
[x,u]=CBF_V12_PureHover(s);
Z=CBF_V11_RealTaylor('R1',x,u,s,6);xv=zeros(20,1);
for k=0:5
  xv(3*k+1:3*k+3)=factorial(k)*Z(8:10,k+1);
end
ob=struct('type','sphere','center',[1;0;1.5],'radius',0.25);
for shape={'C','B','D'}
    D=CBF_V13_VirtualCoefficientAudit(shape{1},xv,s,ob);
    assert(size(D.A,2)==4 && numel(D.labels)==size(D.A,1));
    assert(all(isfinite([D.A(:);D.b(:);D.H0(:)])));
end
R=struct('status','PASS','fixedTaylor',true,'virtualCoefficientDimensions',true);
fprintf('[PASS] V13 smoke: corrected virtual Taylor + C/B/D row assembly.\n');
end
