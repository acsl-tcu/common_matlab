function report = test_CBF_ProfilingEquivalence()
%TEST_CBF_PROFILINGEQUIVALENCE  計時の有無でCBF-QPの係数が変わらないことを確認。
% 計時用の第2出力を追加しただけで、V5の式とQPは変更していない。
%
% common_matlab直下で実行:
%   test_CBF_ProfilingEquivalence();

%% 1. 試験用状態と設定（生成済みCBFには固定値を埋め込まない）
test_CBF_SetupPaths();
s=test_CBF_Fixture(); W=eye(4);
xReal=zeros(29,1); xReal(1)=1;
xReal(8:10)=[0;0;1.5]; xReal(14:16)=[0;0;-1];
xReal(20)=(s.physical.massDrone+s.physical.massLoad)*s.physical.gravity;
xVirtual=zeros(20,1); xVirtual(1:3)=[0;0;1.5];

%% 2. 実/仮想 × C/B/D × 球/楕円体 × 1/3障害物
numberChecked=0;
for modeCell={'real','virtual'}
    mode=modeCell{1};
    if strcmp(mode,'real'), x=xReal; else, x=xVirtual; end
    for shapeCell={'C','B','D'}
        shape=shapeCell{1};
        for kindCell={'sphere','ellipsoid'}
            kind=kindCell{1};
            for n=[1 3]
                obs=makeProfileObstacles(n,kind);
                qpOld=cbf_hocbf_scene(mode,shape,x,s,obs);
                [qpTimed,tCoef]=cbf_hocbf_scene(mode,shape,x,s,obs);
                assert(isequal(qpOld.labels,qpTimed.labels), 'CBF:ProfileLabels', ...
                    'ラベルが変化しました。');
                assert(isequaln(qpOld.A,qpTimed.A) && isequaln(qpOld.b,qpTimed.b), ...
                    'CBF:ProfileCoefficients','タイミング機能でAまたはbが変化しました。');
                assert(isequaln(qpOld.order,qpTimed.order) && ...
                       isequaln(qpOld.psi,qpTimed.psi), 'CBF:ProfileAuxiliary', ...
                    '高次CBF補助条件が変化しました。');
                assert(tCoef.totalMs>=0 && tCoef.jetMs>=0 && ...
                       tCoef.zeroTaylorMs>=0 && tCoef.fourBasisTaylorMs>=0);
                solOld=cbf_solve_hocbf_qp(qpOld,zeros(4,1),W);
                [solTimed,tQP]=cbf_solve_hocbf_qp(qpTimed,zeros(4,1),W);
                assert(solOld.exitflag==solTimed.exitflag,'CBF:ProfileSolver', ...
                    'QP結果が異なります。');
                if solOld.exitflag>0
                    assert(norm(solOld.nu-solTimed.nu,Inf)<1e-6*(1+norm(solOld.nu,Inf)), ...
                        'CBF:ProfileSolution','QP解が異なります。');
                end
                assert(tQP.totalMs>=0 && tQP.quadprogMs>=0);
                numberChecked=numberChecked+1;
            end
            fprintf('[PASS] %-7s %s %-9s : 1/3障害物のA,b,psi,QPを照合\n',mode,shape,kind);
        end
    end
end
report=struct('passed',true,'nCases',numberChecked);
fprintf('=== 計測機能の非侵襲性テスト: %dケース合格 ===\n',numberChecked);
end
