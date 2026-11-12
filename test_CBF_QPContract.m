function test_CBF_QPContract()
%TEST_CBF_QPCONTRACT 可行/不可行を区別し、ハード制約緩和しないことを検証。
test_CBF_SetupPaths();W=diag([1 2 3 4]);
% 可行問題: 0.2 <= u1 <= 0.6
q=struct('A',[1 0 0 0;-1 0 0 0],'b',[0.2;-0.6], ...
    'psiInitiallyNonnegative',[true;true], 'inputLimitIncluded',true);
out=cbf_solve_hocbf_qp(q,zeros(4,1),W);
assert(out.exitflag>0 && abs(out.nu(1)-0.2)<1e-7);
assert(out.minResidual>=-1e-8);
% 不可行問題: u1>=1 かつ u1<=-1
q.b=[1;1];out=cbf_solve_hocbf_qp(q,zeros(4,1),W);
assert(out.exitflag<=0,'QP実行不能を黙って可行扱いしてはいけません。');
% 既存12構成: 遠方障害物・初期ホバリングでQPが可行になること
s=test_CBF_Fixture();obsS=struct('center',[8;0;2],'radius',0.20);
obsE=struct('center',[8;0;2],'axes',[0.18;0.30;0.4],'R',eye(3));
xR=zeros(29,1);xR(1)=1;xR(8:10)=[0;0;1.5];xR(14:16)=[0;0;-1];
xR(20)=(s.physical.massDrone+s.physical.massLoad)*s.physical.gravity;
xV=zeros(20,1);xV(1:3)=[0;0;1.5];
for im=1:2
    if im==1,mode='real';x=xR;else,mode='virtual';x=xV;end
    for sh={'C','B','D'}
        for t={'sphere','ellipsoid'}
            if strcmp(t{1},'sphere'),obs=obsS;else,obs=obsE;end
            fn=str2func(sprintf('CBF_%s_%s_%s',mode,sh{1},t{1}));
            [h,~,qp]=fn(x,s,obs);
            assert(all(h>0) && all(qp.psiInitiallyNonnegative), ...
             '初期高次集合が成立していない状態を可行例に使っています。');
            out=cbf_solve_hocbf_qp(qp,zeros(4,1),W);
            assert(out.exitflag>0,'可行なホバリング条件でQPが解けません: %s',func2str(fn));
            assert(out.minResidual>=-1e-7);
        end
    end
end
fprintf(' [PASS] 手作り可行/不可行＋12構成のホバリングQP\n');
end
