function report=test_CBF_AuditGroundGrid(options)
%TEST_CBF_AUDITGROUNDGRID 高度・下降速度・姿勢・紐角度・ゲインの地面単独試験。
% 障害物CBF行はQPに含めない。遠方球は既存APIで地面の係数を得るだけ。
% 現行のHOCBF及び実入力制約はハードのまま。既存生成器は変更しない。
if nargin<1,options=struct();end
paths=test_CBF_SetupPaths();s0=test_CBF_Fixture();
if isfield(options,'settings'),s0=options.settings;end
hvals=getopt(options,'altitudes',[.75 1.2 1.8 3.0]);
vvals=getopt(options,'verticalSpeeds',[-1.0 -.4 0 .4]);
pvals=getopt(options,'pitchAngles',[0 .12 .28]);
cvals=getopt(options,'cableTilts',[0 .14]);
gvals=getopt(options,'groundGains',[.8 1.2 2.0 3.0]);
igvals=getopt(options,'inputGains',[1.2]);
rows=cell(0,27);detail=cell(0,15);ext=cell(0,14);
caseId=0;
for ig=igvals(:).'
    for kg=gvals(:).'
        for loadAltitude=hvals(:).'
            for vz=vvals(:).'
                for pitch=pvals(:).'
                    for tilt=cvals(:).'
                        caseId=caseId+1;
                        s=s0;s.cbf.groundGain=kg;s.cbf.inputGain=ig;
                        x=CBF_Audit_State('real',s,loadAltitude,vz,0,pitch,tilt,0,0);
                        % 地面高度とは十分に無関係な遠方障害物を仮置き。
                        ob=CBF_Audit_State('obstacle','sphere',x(8:10),100,'front',0);
                        qp=cbf_hocbf_scene('real','B',x,s,{ob});
                        labels=qp.labels;
                        ground=contains(labels,'ground_');
                        input=contains(labels,'_thrust_') | contains(labels,'_tau_');
                        pG=CBF_Audit_LP(qp,ground);
                        pI=CBF_Audit_LP(qp,input);
                        pGI=CBF_Audit_LP(qp,ground|input);
                        % 真に全制約も計算し、遠方障害物も除いた場合との差を記録。
                        pAll=CBF_Audit_LP(qp);
                        p=cbf_parameter_vector(s);
                        jet=cbf_real_state_taylor(x,zeros(4,1),p,6);
                        diag=cbf_model_diagnostics('real',x,s);
                        minGP=Inf;minGH=Inf;worstDef=-Inf;worstName='';
                        for row=find(ground).'
                            label=labels{row};
                            % groundのh,各階の生微分は幾何位置のTaylorから独立算出。
                            if contains(label,'ground_drone')
                                zz=jet(8:10,:)-s.physical.cableLength*jet(14:16,:);
                                radius=s.shape.droneRadius;
                            elseif contains(label,'ground_load')
                                zz=jet(8:10,:);radius=s.shape.loadRadius;
                            elseif contains(label,'ground_cable_drone_end')
                                zz=jet(8:10,:)-s.physical.cableLength*jet(14:16,:);
                                radius=s.shape.cableRadius;
                            elseif contains(label,'ground_cable_load_end')
                                zz=jet(8:10,:);radius=s.shape.cableRadius;
                            else
                                % B形状の地面のみを対象。未知のラベルは明示記録。
                                zz=nan(3,7);radius=NaN;
                            end
                            raw=zz(3,:).*factorial(0:6);
                            raw(1)=raw(1)-radius-s.ground.height-s.cbf.safetyMargin;
                            phi=qp.psi{row};minGP=min(minGP,min(phi));
                            % 保存されたA,b,psiを、生のh～h^(6)から独立に再構成。
                            % r=6の実入力地面条件について係数生成を監査。
                            r=qp.order(row); psiCheck=zeros(r,1);
                            for j=0:r-1
                                for m=0:j
                                    psiCheck(j+1)=psiCheck(j+1)+ ...
                                        nchoosek(j,m)*kg^(j-m)*raw(m+1);
                                end
                            end
                            expectedDrift=raw(r+1);
                            for j=0:r-1
                                expectedDrift=expectedDrift+ ...
                                    nchoosek(r,j)*kg^(r-j)*raw(j+1);
                            end
                            bError=abs(qp.b(row)+expectedDrift);
                            psiError=norm(phi-psiCheck,Inf);
                            minGH=min(minGH,raw(1));
                            % input CBFで許されたuの中で、当該地面行のlhsを最大化。
                            feasibleRow=false;maxLHS=NaN;deficit=NaN;
                            if pI.feasible
                                [~,fval,fl]=linprog(-qp.A(row,:).', ...
                                    -qp.A(input,:),-qp.b(input),[],[],[],[], ...
                                    optimoptions('linprog','Display','off'));
                                if fl>0,feasibleRow=true;maxLHS=-fval;
                                    deficit=qp.b(row)-maxLHS;end
                            end
                            if isfinite(deficit) && deficit>worstDef
                                worstDef=deficit;worstName=label;
                            end
                            detail(end+1,:)={caseId,kg,ig,loadAltitude,vz,pitch,tilt, ...
                                label,raw(1),raw(2),raw(3),raw(4),raw(5),raw(6),raw(7)}; %#ok<AGROW> 
                            % 入力係数とpsiの全要素は別の詳細テーブルへ後で出力。
                            ext(end+1,:)={caseId,label,qp.order(row), ...
                                qp.A(row,1),qp.A(row,2),qp.A(row,3),qp.A(row,4), ...
                                qp.b(row),mat2str(phi(:).',8),feasibleRow, ...
                                maxLHS,deficit,bError,psiError}; %#ok<AGROW>
                        end
                        minH=min(qp.psi{find(ground,1)});
                        rows(end+1,:)={caseId,kg,ig,loadAltitude,vz,pitch,tilt, ...
                            x(20),diag.tension,all(qp.psiInitiallyNonnegative(ground)), ...
                            all(qp.psiInitiallyNonnegative(input)), ...
                            pG.feasible,pI.feasible,pGI.feasible,pAll.feasible, ...
                            minGH,minGP,worstDef,worstName, ...
                            pGI.exitflag,pAll.exitflag, ...
                            norm(qp.A(ground,:),Inf),max(qp.b(ground)), ...
                            minH,numel(find(ground)),numel(find(input)), ...
                            max(qp.earlyInputResidual)}; %#ok<AGROW>
                    end
                end
            end
        end
    end
end
report=cell2table(rows,'VariableNames',{'caseId','groundGain','inputGain', ...
 'loadAltitude','verticalSpeed','pitch','cableTilt','thrust', ...
 'tension','initialGroundPsiOK','initialInputPsiOK','groundFeasible', ...
 'inputFeasible','groundInputFeasible','allIncludingFarObstacleFeasible', ...
 'minGroundH','minGroundPsi','largestGroundInputDeficit','worstGroundRow', ...
 'groundInputLPFlag','allLPFlag','groundAInfNorm','maxGroundRHS', ...
 'oneGroundPsiMin','groundRows','inputRows','maxEarlyInputResidual'});
% 明細はcaseIdで接続可能。列名を静的に定義する。
rawTable=cell2table(detail,'VariableNames',{'caseId','groundGain','inputGain', ...
    'loadAltitude','verticalSpeed','pitch','cableTilt','groundRow', ...
    'h','h1','h2','h3','h4','h5','h6'});
extraTable=cell2table(ext,'VariableNames',{'caseId','groundRow','designOrder', ...
    'nuTcoef','nuTauXcoef','nuTauYcoef','nuTauZcoef','requiredRHS', ...
    'allAuxiliaryPsi','inputLPFeasible','maxLHSWithinInputCBF','deficit', ...
    'bReconstructionError','psiReconstructionError'});
folder=fullfile(paths.generator,'CBF','test_results');
if ~exist(folder,'dir'),mkdir(folder);end
writetable(report,fullfile(folder,'audit_ground_grid.csv'));
writetable(rawTable,fullfile(folder,'audit_ground_derivatives.csv'));
writetable(extraTable,fullfile(folder,'audit_ground_input_conflict.csv'));
fprintf('[地面単独] %d状態、初期地面psi成立=%d、地面+入力可行=%d、双方成立=%d\n', ...
    size(report,1),sum(report.initialGroundPsiOK), ...
    sum(report.groundInputFeasible), ...
    sum(report.initialGroundPsiOK & report.initialInputPsiOK & report.groundInputFeasible));
end
function v=getopt(o,key,default)
if isfield(o,key)&&~isempty(o.(key)),v=o.(key);else,v=default;end
end
