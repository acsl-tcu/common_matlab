function report=test_CBF_RealGroundPolicyRollout(gains,steps)
%TEST_CBF_REALGROUNDPOLICYROLLOUT 同じストレス入力で地面CBFゲインを比較。
% 失敗時は入力を適用せず終了。実行不能をPASSとして扱わない。
% 呼出し: report=test_CBF_RealGroundPolicyRollout([1.2 1.5 2],25);
% 注意: この実入力拡張試験はHLC・軌道生成を使用しない。
if nargin<1 || isempty(gains),gains=[1.2 1.5 2];end
if nargin<2 || isempty(steps),steps=25;end
paths=test_CBF_SetupPaths();
s0=test_CBF_Fixture();
obs=struct('type','sphere','center',[.8;0;1.4],'radius',.14);
folder=fullfile(paths.generator,'CBF','test_results');
if ~exist(folder,'dir'),mkdir(folder);end
rows=cell(numel(gains),7);
for ig=1:numel(gains)
    s=s0; s.cbf.groundGain=gains(ig);
    x=zeros(29,1);x(1)=1;x(8:10)=[0;0;1.4];x(14:16)=[0;0;-1];
    x(20)=(s.physical.massDrone+s.physical.massLoad)*s.physical.gravity;
    nuNom=[0;20;0;0];dt=.01;minH=inf;minPsi=inf;
    nGood=0;status='completed';maxAngle=0;corrections=0;
    for k=1:steps
        [h,~,qp]=CBF_real_B_sphere(x,s,obs);
        minH=min(minH,min(h));
        minPsi=min(minPsi,min(cellfun(@min,qp.psi)));
        if ~all(qp.psiInitiallyNonnegative)
            status='initial_HOCBF_set_violation';break;
        end
        sol=cbf_solve_hocbf_qp(qp,nuNom,eye(4));
        if sol.exitflag<=0 || ~isfinite(sol.minResidual) || sol.minResidual<-1e-7
            status='qp_infeasible';break;
        end
        corrections=corrections+(norm(sol.nu-nuNom)>1e-8);
        xn=realRK4(x,sol.nu,dt,s);
        if abs(norm(xn(1:4))-1)>1e-5 || abs(norm(xn(14:16))-1)>1e-5
            status='model_constraint_drift';break;
        end
        if xn(20)<s.input.thrustMin-1e-7 || xn(20)>s.input.thrustMax+1e-7 || ...
           any(xn(24:26)<s.input.torqueMin(:)-1e-7) || ...
           any(xn(24:26)>s.input.torqueMax(:)+1e-7)
            status='physical_input_violation';break;
        end
        diag=cbf_model_diagnostics('real',xn,s);
        if ~isfinite(diag.tension) || diag.tension<=0
            status='cable_tension_nonpositive';break;
        end
        hNext=CBF_real_B_sphere(xn,s,obs);
        minH=min(minH,min(hNext));
        if any(hNext<-1e-7)
            status='geometry_violation';break;
        end
        x=xn;nGood=k;
        maxAngle=max(maxAngle,2*acos(min(1,abs(x(1)))));
    end
    rows(ig,:)={gains(ig),nGood,steps,status,minH,minPsi,corrections};
    fprintf('地面ゲイン %.2f: %d/%d ステップ %s, minH=%.5g, minPsi=%.5g, QP修正=%d\n', ...
        gains(ig),nGood,steps,status,minH,minPsi,corrections);
end
report=cell2table(rows,'VariableNames', ...
    {'groundGain','completedSteps','requestedSteps','status','minH','minPsi','corrections'});
writetable(report,fullfile(folder,'ground_gain_rollout.csv'));
fprintf('※ 全ステップ完走でも将来の安全保証・一般初期状態への保証ではありません。\n');
end
function xn=realRK4(x,nu,dt,s)
a=test_CBF_IndependentPlantRHS(x,nu,s);
b=test_CBF_IndependentPlantRHS(x+dt*a/2,nu,s);
c=test_CBF_IndependentPlantRHS(x+dt*b/2,nu,s);
d=test_CBF_IndependentPlantRHS(x+dt*c,nu,s);
xn=x+dt*(a+2*b+2*c+d)/6;
end
