function report = test_CBF_DiagnoseRealRollout()
%TEST_CBF_DIAGNOSEREALROLLOUT  実入力側QPが解けなくなった原因を調べる専用試験。
%
% common_matlab直下で以下を実行:
%   test_CBF_All();
%   report = test_CBF_DiagnoseRealRollout();
%
% ・元のtest_CBF_StandaloneRolloutと同一の初期状態、障害物、名目入力を使う。
% ・失敗を勝手に成功扱いしない。制約を緩めた解を制御には使わない。
% ・失敗時の状態、全QP行、初期psi、ソルバ出力をMATに保存。
% ・別の線形計画で「数式上の実行不能か数値解法の問題か」を診断。
% ・診断用スラックは原因分析専用。CBFの運用には使用しない。

paths=test_CBF_SetupPaths();
s=test_CBF_Fixture();
obs=struct('center',[0.8;0;1.40],'radius',0.14); % 元のストレス試験と一致
nuNom=[0;20;0;0]; W=eye(4);
x=zeros(29,1); x(1)=1; x(8:10)=[0;0;1.40]; x(14:16)=[0;0;-1];
x(20)=(s.physical.massDrone+s.physical.massLoad)*s.physical.gravity;
dt=0.01; steps=25;
folder=fullfile(paths.generator,'CBF','test_results');
if exist(folder,'dir')~=7,mkdir(folder);end
hist=nan(steps,10);
report=struct('status','not_started','failureStep',NaN,'reason','', ...
    'outputFolder',folder,'diagnosticLP',struct());

fprintf('\n=== 実入力CBF-QP：失敗ステップ診断 ===\n');
fprintf('条件はtest_CBF_StandaloneRolloutと同じ（名目nu=[0;20;0;0]）\n');

for k=1:steps
    t=(k-1)*dt;
    [h,~,qp]=CBF_real_B_sphere(x,s,obs);
    psi=vertcat(qp.psi{:});
    minPsi=min(psi);
    initialOK=all(qp.psiInitiallyNonnegative);
    sol=cbf_solve_hocbf_qp(qp,nuNom,W);
    residual=NaN;
    if sol.exitflag>0 && isfinite(sol.minResidual)
        residual=sol.minResidual;
    end
    hist(k,:)=[k,t,min(h),minPsi,double(initialOK),sol.exitflag, ...
                residual,x(20),max(abs(x(24:26))),sum(qp.degenerate)];
    fprintf('step=%2d t=%.3f hMin=% .3e psiMin=% .3e QPexit=%2d res=% .3e\n', ...
        k,t,min(h),minPsi,sol.exitflag,residual);

    % HOCBF初期集合が破れている場合、QPが可行でも数学的な安全の主張はできない。
    if ~initialOK || sol.exitflag<=0 || ~isfinite(residual) || residual < -1e-7
        if ~initialOK
            reason='high_order_initial_set_violation';
        elseif sol.exitflag<=0
            reason='solver_failed_or_qp_infeasible';
        else
            reason='negative_qp_residual';
        end
        report.status='failure'; report.failureStep=k; report.reason=reason;
        report=saveDiagnostic(report,folder,k,x,qp,h,psi,sol,s,obs,nuNom,hist(1:k,:));
        fprintf(2,'\n[DIAG] step %dの原因を記録しました。\n',k);
        fprintf(2,'[DIAG] MATLAB記録: %s\n',fullfile(folder,'real_B_sphere_failure_snapshot.mat'));
        fprintf(2,'[DIAG] CSV記録: %s\n',fullfile(folder,'real_B_sphere_diagnosis.csv'));
        return;
    end

    % 元試験と同一のRK4で積分（入力を変更せず適用）
    x=localRK4(x,sol.nu,dt,s);
    if abs(norm(x(1:4))-1)>=1e-6 || abs(norm(x(14:16))-1)>=1e-6
        report.status='failure';report.failureStep=k;report.reason='state_constraint_drift';
        report=saveDiagnostic(report,folder,k,x,qp,h,psi,sol,s,obs,nuNom,hist(1:k,:));
        return;
    end
    if x(20)<s.input.thrustMin-1e-7 || x(20)>s.input.thrustMax+1e-7 || ...
       any(x(24:26)<s.input.torqueMin(:)-1e-7) || ...
       any(x(24:26)>s.input.torqueMax(:)+1e-7)
        report.status='failure';report.failureStep=k;report.reason='actual_input_limit_violation';
        report=saveDiagnostic(report,folder,k,x,qp,h,psi,sol,s,obs,nuNom,hist(1:k,:));
        return;
    end
    phys=cbf_model_diagnostics('real',x,s);
    if ~isfinite(phys.tension) || phys.tension<=0
        report.status='failure';report.failureStep=k;report.reason='tension_not_positive';
        report=saveDiagnostic(report,folder,k,x,qp,h,psi,sol,s,obs,nuNom,hist(1:k,:));
        return;
    end
    hNext=CBF_real_B_sphere(x,s,obs);
    if any(hNext < -1e-7)
        report.status='failure';report.failureStep=k;report.reason='geometry_collision';
        report=saveDiagnostic(report,folder,k,x,qp,h,psi,sol,s,obs,nuNom,hist(1:k,:));
        return;
    end
end
report.status='pass';report.failureStep=NaN;report.reason='none';
writeLog(fullfile(folder,'real_B_sphere_diagnosis.csv'),hist);
fprintf('[PASS] %dステップを完走。診断ログを保存しました。\n',steps);
end

function report=saveDiagnostic(report,folder,k,x,qp,h,psi,sol,s,obs,nuNom,hist)
% 失敗が数式上の矛盾かquadprogの数値的な失敗かを診断する。
A=qp.A;b=qp.b;
n=size(A,1);
scale=max([ones(n,1),sqrt(sum(A.^2,2)),abs(b)],[],2);
if exist('linprog','file')==2
    try
        % 診断用の最小共通緩和量 t >=0:
        % A*nu + scale*t >= b 。tは適用入力へは絶対に使わない。
        f=[zeros(4,1);1];
        Aineq=[-A,-scale];bineq=-b;
        lb=[-inf(4,1);0];
        opts=optimoptions('linprog','Display','off');
        [z,tmin,flag,linOutput]=linprog(f,Aineq,bineq,[],[],lb,[],opts);
        report.diagnosticLP=struct('exitflag',flag,'minRelaxation',tmin, ...
            'candidate',z,'output',linOutput);
        if flag>0
            fprintf('  [LP] 最小正規化スラック = %.6e （診断専用）\n',tmin);
            if tmin>1e-7
                fprintf(2,'  [LP] 元のハード制約が同時に成立しない可能性が高いです。\n');
            else
                fprintf('  [LP] LPでは可行。quadprogの数値条件を優先して調べてください。\n');
            end
            report.diagnosticLP.residualAtCandidate=A*z(1:4)-b;
            resnorm=report.diagnosticLP.residualAtCandidate./scale;
            [~,idx]=sort(resnorm,'ascend');
            fprintf('  [LP] 代表的な制約（LP候補の小さい正規化余裕順）：\n');
            for t=1:min(8,n)
                j=idx(t);
                fprintf('    row=%2d %-32s A_norm=% .3e b=% .3e residual_scaled=% .3e\n', ...
                    j,qp.labels{j},norm(A(j,:)),b(j),resnorm(j));
            end
        else
            fprintf('  [LP] linprogも解を返しませんでした（exitflag=%d）。\n',flag);
        end
    catch ME
        report.diagnosticLP=struct('error',ME.message);
        fprintf('  [LP] 診断自体が失敗: %s\n',ME.message);
    end
else
    report.diagnosticLP=struct('error','linprogが見つかりません');
    fprintf('  [LP] linprogが使えないため、スラック診断はスキップします。\n');
end

[sortedPsi,idxPsi]=sort(cellfun(@(v)min(v),qp.psi),'ascend');
fprintf('  [HOCBF] 最小補助条件の行：\n');
for i=1:min(8,n)
    j=idxPsi(i);
    fprintf('    row=%2d %-32s min(psi)=% .3e degenerate=%d\n', ...
       j,qp.labels{j},sortedPsi(i),qp.degenerate(j));
end
fprintf('  [STATE] T=%.8g, dotT=%.8g, ddotT=%.8g, T3=%.8g\n',x(20:23));
fprintf('  [STATE] tau=[% .6g % .6g % .6g], dtau=[% .6g % .6g % .6g]\n', ...
    x(24:26),x(27:29));
fprintf('  [STATE] degenerate rows=%d, uncontrollableViolation=%d\n', ...
    sum(qp.degenerate),sum(qp.uncontrollableViolation));
if isfield(sol.output,'message'),fprintf('  [QP] %s\n',sol.output.message);end
report.qpExitflag=sol.exitflag;
report.state=x; report.activeLabels=qp.labels;
writeLog(fullfile(folder,'real_B_sphere_diagnosis.csv'),hist);
save(fullfile(folder,'real_B_sphere_failure_snapshot.mat'), ...
    'k','x','qp','h','psi','sol','s','obs','nuNom','report');
end

function xn=localRK4(x,nu,dt,s)
k1=test_CBF_IndependentPlantRHS(x,nu,s);
k2=test_CBF_IndependentPlantRHS(x+dt*k1/2,nu,s);
k3=test_CBF_IndependentPlantRHS(x+dt*k2/2,nu,s);
k4=test_CBF_IndependentPlantRHS(x+dt*k3,nu,s);
xn=x+dt*(k1+2*k2+2*k3+k4)/6;
end

function writeLog(file,data)
fp=fopen(file,'w');
if fp<0,error('CBF:DiagnosticLog','ログに書き込めません: %s',file);end
cleanup=onCleanup(@()fclose(fp));
fprintf(fp,'step,t_s,min_h,min_psi,initial_psi_ok,qp_exitflag,min_qp_residual,thrust_N,max_abs_tau_Nm,degenerate_rows\n');
for i=1:size(data,1)
    fprintf(fp,'%.15g,%.15g,%.15g,%.15g,%.15g,%.15g,%.15g,%.15g,%.15g,%.15g\n',data(i,:));
end
end
