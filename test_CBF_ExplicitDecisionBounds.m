function test_CBF_ExplicitDecisionBounds()
%TEST_CBF_EXPLICITDECISIONBOUNDS QPの明示的な入力制限を検査。
% 仮想入力の試験上限は物理推力・トルク制限とは異なる。
test_CBF_SetupPaths();
qp=struct('A',[1 0 0 0;-1 0 0 0],'b',[1;-2], ...
    'psiInitiallyNonnegative',true(2,1),'inputLimitIncluded',false);
box=struct('lower',[-1;-1;-1;-1],'upper',[2;1;1;1]);
out=cbf_solve_hocbf_qp(qp,[100;0;0;0],eye(4),box);
assert(out.exitflag>0 && out.minResidual>=-1e-7);
assert(abs(out.nu(1)-2)<1e-6 && out.decisionBoundsUsed);
box.upper(1)=.9;
out=cbf_solve_hocbf_qp(qp,[100;0;0;0],eye(4),box);
assert(out.exitflag<=0,'CBF:BoundsNotApplied','不可能な上下限を無視しました。');
fprintf('[PASS] 決定入力上限で大入力を制限し、不可行は不可行として返しました。\n');
end
