function report=test_CBF_AuditVirtualD(options)
%TEST_CBF_AUDITVIRTUALD 仮想入力Dの幾何・状態変換・QPの整合性試験。
% 巨大入力をクリップせず、原因を記録する。実HLC逆変換は未検証。
if nargin<1,options=struct();end
paths=test_CBF_SetupPaths();s=test_CBF_Fixture();
if isfield(options,'settings'),s=options.settings;end
kinds={'sphere','ellipsoid'};distances=[.38 .55 .9 1.5];
orientations=[0 .12 -.18];velocities=[-.8 0 .8];
rows=cell(0,17);idx=0;
for theta=orientations
    for vx=velocities
        xR=CBF_Audit_State('real',s,1.6,-.1,.08,theta,.12,vx,.10,.35);
        % 実入力モデルから求めた吊り荷0～5階微分で仮想状態を構成する。
        % 実状態と独立に高階微分を0と置く不整合を避ける。
        xV=CBF_Audit_State('virtualfromreal',xR,s);
        P=cbf_parameter_vector(s);
        a=xV(7:9)+s.physical.gravity*[0;0;1]- ...
            [s.physical.disturbanceXY(:);0];
        qReconstructed=-a/norm(a);
        qError=norm(qReconstructed-xR(14:16));
        if qError>1e-8,error('CBF:AuditFlatMismatch', ...
                '張りケーブルに対する実→仮想変換が不一致');end
        diag=cbf_model_diagnostics('virtual',xV,s);
        for kindCell=kinds
            kind=kindCell{1};
            for distance=distances
                ob=CBF_Audit_State('obstacle',kind,xR(8:10), ...
                    distance,'front',.62);
                hDr=cbf_evaluate_case('real','D',kind,xR,s,ob);
                hDv=cbf_evaluate_case('virtual','D',kind,xV,s,ob);
                hBv=cbf_evaluate_case('virtual','B',kind,xV,s,ob);
                geometryError=norm(hDr-hDv,Inf);
                if geometryError>1e-7*(1+norm(hDr,Inf))
                    error('CBF:AuditDGeometry','Dの実/仮想幾何が一致しません。');
                end
                % Dのケーブル楕円体はBのカプセルを包含する設計。
                % 3行目がケーブル衝突条件（機体と吊り荷が先にある）。
                containmentGap=hBv(3)-hDv(3);
                % 平滑化epsilonのため、比較はわずかな許容誤差つき。
                if containmentGap<-1e-5*(1+abs(hBv(3)))
                    error('CBF:AuditContainment','楕円体DがカプセルBより緩い可能性があります。');
                end
                qp=cbf_hocbf_scene('virtual','D',xV,s,{ob});
                initialOK=all(qp.psiInitiallyNonnegative);
                p=CBF_Audit_LP(qp);
                minEffect=min(qp.effectNorm);
                % 診断のため無制限の仮想決定入力を計算。実入力とは別。
                if p.feasible
                    sol=cbf_solve_hocbf_qp(qp,zeros(4,1),eye(4));
                    if sol.exitflag>0
                        nuInf=norm(sol.nu,Inf);
                        minResid=sol.minResidual;
                    else
                        nuInf=NaN;minResid=NaN;
                    end
                else
                    nuInf=NaN;minResid=NaN;
                end
                idx=idx+1;
                rows(end+1,:)={idx,kind,theta,vx,distance, ...
                    qError,geometryError,containmentGap,diag.flatMapDenominator, ...
                    min(hDv),min(cellfun(@min,qp.psi)),initialOK, ...
                    p.feasible,minEffect,sum(qp.degenerate),nuInf,minResid}; %#ok<AGROW>
            end
        end
    end
end
report=cell2table(rows,'VariableNames',{'caseId','obstacleType','pitch', ...
 'loadVelocityX','obstacleCenterDistance','qCableParityError', ...
 'DRealVirtualGeometryError','BMinusDCableBarrier','flatMapDenominator', ...
 'minH','minPsi','initialPsiOK','qpFeasible', ...
 'minDecisionEffectNorm','degenerateRows', ...
 'unboundedVirtualQPNuInf','qpResidual'});
folder=fullfile(paths.generator,'CBF','test_results');
if ~exist(folder,'dir'),mkdir(folder);end
writetable(report,fullfile(folder,'audit_virtual_D_consistency.csv'));
fprintf('[仮想D] %d条件、初期集合OK=%d、QP可行=%d、有限QP入力=%d。物理入力保証は別。\n', ...
    size(report,1),sum(report.initialPsiOK),sum(report.qpFeasible), ...
    sum(isfinite(report.unboundedVirtualQPNuInf)));
end
