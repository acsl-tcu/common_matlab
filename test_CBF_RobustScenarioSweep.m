function report = test_CBF_RobustScenarioSweep(steps)
%TEST_CBF_ROBUSTSCENARIOSWEEP HLC接続前の多姿勢・多軌道・多障害物試験。
% 1) 実入力: 姿勢・ケーブル傾斜・速度を変更、正張力と入力上下限を監視
% 2) 仮想入力: 吊り荷の3D速度/加速度/jerkを変更
% 3) 3形状×2障害物形状(任意回転楕円体)×実/仮想
% 4) 閉ループでは入力を無理にクリップせず、QP不可行・初期psi違反を記録
% 5) 仮想入力は「試験用に外部設定した決定入力限界」を使う。
%    これは実入力限界の代替ではなく、D方式の異常巨大入力を隠さず停止するため。
% 6) 回避目標に到達する制御器の試験ではない。
%
% 実行: report=test_CBF_RobustScenarioSweep(12);

if nargin<1 || isempty(steps),steps=12;end
assert(isscalar(steps)&&steps>=1&&steps==floor(steps));
paths=test_CBF_SetupPaths();s0=test_CBF_Fixture();
scenes={'sphere','ellipsoid'};shapes={'C','B','D'};modes={'real','virtual'};
% この数値はテスト専用で、生成されるCBFには書き込みません。
% 観測されたD方式の10^12～10^15の入力を「成功」と誤判定しない。
virtualTestLimits=struct('lower',[-30;-30;-30;-5], ...
                         'upper',[ 30; 30; 30; 5]);
W=eye(4);dt=.01;
rows=cell(2*3*2*5,17);idx=0;
for im=1:numel(modes)
    mode=modes{im};
    for shapeCell=shapes
        shape=shapeCell{1};
        for kindCell=scenes
            kind=kindCell{1};
            for scenario=1:5
                s=s0;
                x=initialState(mode,s,scenario);
                obstacles=makeObstacles(kind,scenario);
                status='completed';good=0;minH=inf;minPsi=inf;
                minResidual=inf;maxInput=0;maxRealTorque=0;minTension=inf;
                solveCount=0;maxDegenerate=0;maxEarlyResidual=0;diagMessage='';
                for k=1:steps
                    try
                        qp=cbf_hocbf_scene(mode,shape,x,s,obstacles);
                    catch err
                        status='coefficient_error';diagMessage=err.message;break;
                    end
                    maxDegenerate=max(maxDegenerate,sum(qp.degenerate));
                    maxEarlyResidual=max(maxEarlyResidual,max(qp.earlyInputResidual));
                    psiVals=vertcat(qp.psi{:});
                    minPsi=min(minPsi,min(psiVals));
                    if ~all(isfinite(psiVals)) || ~all(qp.psiInitiallyNonnegative)
                        status='initial_HOCBF_set_violation';break;
                    end
                    if ~all(isfinite(qp.A(:))) || ~all(isfinite(qp.b))
                        status='nonfinite_coefficient';break;
                    end
                    t=(k-1)*dt;
                    nuNom=nominalInput(mode,scenario,t);
                    if strcmp(mode,'virtual')
                        sol=cbf_solve_hocbf_qp(qp,nuNom,W,virtualTestLimits);
                    else
                        sol=cbf_solve_hocbf_qp(qp,nuNom,W);
                    end
                    if sol.exitflag<=0 || ~isfinite(sol.minResidual) || sol.minResidual<-1e-7
                        status='qp_infeasible_or_numerical';break;
                    end
                    solveCount=solveCount+1;
                    maxInput=max(maxInput,norm(sol.nu,Inf));
                    minResidual=min(minResidual,sol.minResidual);
                    try
                        if strcmp(mode,'real')
                            xn=realStep(x,sol.nu,dt,s);
                            diag=cbf_model_diagnostics('real',xn,s);
                            if ~all(isfinite(xn))
                                status='nonfinite_real_state';break;
                            end
                            if diag.quaternionNormError>1e-5 || ...
                               diag.cableDirectionNormError>1e-5 || ...
                               ~diag.thrustInBounds || ~diag.torqueInBounds
                                status='model_or_actual_input_violation';break;
                            end
                            if ~isfinite(diag.tension) || diag.tension<=0
                                status='cable_tension_nonpositive';break;
                            end
                            minTension=min(minTension,diag.tension);
                            maxRealTorque=max(maxRealTorque,max(abs(xn(24:26))));
                        else
                            xn=virtualStep(x,sol.nu,dt);
                            diag=cbf_model_diagnostics('virtual',xn,s);
                            if ~all(isfinite(xn)) || ...
                               ~diag.flatMapValid || ...
                               norm(xn(1:18),Inf)>1000
                                status='virtual_state_or_flat_map_invalid';break;
                            end
                        end
                        hNext=cbf_evaluate_scene(mode,shape,xn,s,obstacles);
                    catch err
                        status='integration_or_geometry_error';diagMessage=err.message;break;
                    end
                    minH=min(minH,min(hNext));
                    if any(~isfinite(hNext)) || any(hNext<-1e-7)
                        status='geometric_safety_violation';break;
                    end
                    % 離散実装のため連続時間の不変性保証はない。次ステップは再検証。
                    x=xn;good=k;
                end
                if good==steps && strcmp(status,'completed')
                    % 最終積分点も次周期の高次安全集合内かチェック。
                    try
                        qpEnd=cbf_hocbf_scene(mode,shape,x,s,obstacles);
                        if ~all(qpEnd.psiInitiallyNonnegative)
                            status='final_HOCBF_set_violation';
                        end
                    catch err
                        status='final_coefficient_error';diagMessage=err.message;
                    end
                end
                if isinf(minH)
                    try
                        h0=cbf_evaluate_scene(mode,shape,x,s,obstacles);
                        minH=min(h0);
                    catch
                        minH=NaN;
                    end
                end
                idx=idx+1;
                rows(idx,:)={mode,shape,kind,scenario,numel(obstacles),steps,good, ...
                    status,solveCount,minH,minPsi,minResidual,maxInput, ...
                    maxRealTorque,minTension,maxDegenerate,maxEarlyResidual};
                fprintf('%-7s %s %-9s ケース%d : %d/%d, %-39s, max|nu|=%.4g\n', ...
                    mode,shape,kind,scenario,good,steps,status,maxInput);
                if ~isempty(diagMessage)
                    fprintf('    診断: %s\n',diagMessage);
                end
            end
        end
    end
end
report=cell2table(rows,'VariableNames',{'mode','shape','obstacleType', ...
    'scenario','nObstacles','requestedSteps','completedSteps','status', ...
    'nSolves','minH','minPsi','minResidual','maxDecisionInput', ...
    'maxAbsRealTorque','minCableTension','maxDegenerateRows','maxEarlyInputResidual'});
folder=fullfile(paths.generator,'CBF','test_results');
if ~exist(folder,'dir'),mkdir(folder);end
writetable(report,fullfile(folder,'robust_scenario_sweep.csv'));
fprintf('完走=%d/%d (未完走は原因として分類。QP可行率を保証するものではありません)\n', ...
    sum(strcmp(report.status,'completed')),height(report));
end

function x=initialState(mode,s,i)
if strcmp(mode,'real')
    x=zeros(29,1);
    angles=[0 0 0; .10 -.07 .2; -.08 .11 -.17; .15 .12 .28; -.16 -.13 -.25];
    a=angles(i,:); x(1:4)=yprQuat(a(3),a(2),a(1));
    x(8:10)=[0;0;1.5];
    x(11:13)=[0.05*(i-3);0.025*(i-3);0.02*(i-3)];
    q=[0.035*(i-3);-0.025*(i-3);-1];q=q/norm(q);
    x(14:16)=q;x(5:7)=[.01;-.015;.02]*(i-3);
    w=[0.03;-0.02;0.01]*(i-3);x(17:19)=w-q*(q.'*w);
    x(20)=(s.physical.massDrone+s.physical.massLoad)*s.physical.gravity;
else
    x=zeros(20,1);x(1:3)=[0;0;1.5];
    x(4:6)=[0.07*(i-3);0.04*(i-3);0.02*(i-3)];
    x(7:9)=[0.025*(i-3);-0.015*(i-3);0.01*(i-3)];
    x(10:12)=[0.01*(i-3);0.01*(i-3);0.005*(i-3)];
    x(13:15)=[0.005;-0.006;0.003]*(i-3);
    x(16:18)=[0.003;0.004;0.002]*(i-3);
    x(19)=0.1*(i-3);x(20)=0.02*(i-3);
end
end
function u=nominalInput(mode,scenario,t)
if strcmp(mode,'real')
    % 拡張入力 [T^(4);tau''x;tau''y;tau''z] の穏当な試験信号。
    % 以前の [0;20;0;0] は別途ストレス試験として保存する。
    u=[0;0.20*sin(2*t+scenario);0.15*cos(t+scenario);0.07*sin(t)];
else
    u=[0.6*sin(t+scenario);0.5*cos(1.3*t+scenario);0.25*sin(t);0.1*cos(t)];
end
end
function obs=makeObstacles(kind,scenario)
% 近接/遠方、複数、傾いた楕円体を共通中心の周囲に配置。
num=1+mod(scenario,3);obs=cell(1,num);
for k=1:num
    cen=[0.92+0.2*k;(-1)^k*0.18*k;1.5+.08*sin(scenario+k)];
    if strcmp(kind,'sphere')
        obs{k}=struct('type','sphere','center',cen,'radius',.12+.02*k);
    else
        yaw=.19*scenario;pitch=-.13*k;roll=.21*(scenario-k);
        R=rotZ(yaw)*rotY(pitch)*rotX(roll);
        obs{k}=struct('type','ellipsoid','center',cen, ...
            'axes',[.12+.01*k;.17+.01*k;.23+.01*k],'R',R);
    end
end
end
function q=yprQuat(y,p,r)
cy=cos(y/2);sy=sin(y/2);cp=cos(p/2);sp=sin(p/2);cr=cos(r/2);sr=sin(r/2);
q=[cr*cp*cy+sr*sp*sy;sr*cp*cy-cr*sp*sy;cr*sp*cy+sr*cp*sy;cr*cp*sy-sr*sp*cy];
end
function R=rotX(a)
R=[1 0 0;0 cos(a) -sin(a);0 sin(a) cos(a)];
end
function R=rotY(a)
R=[cos(a) 0 sin(a);0 1 0;-sin(a) 0 cos(a)];
end
function R=rotZ(a)
R=[cos(a) -sin(a) 0;sin(a) cos(a) 0;0 0 1];
end
function xn=realStep(x,u,dt,s)
a=test_CBF_IndependentPlantRHS(x,u,s);
b=test_CBF_IndependentPlantRHS(x+dt*a/2,u,s);
c=test_CBF_IndependentPlantRHS(x+dt*b/2,u,s);
d=test_CBF_IndependentPlantRHS(x+dt*c,u,s);
xn=x+dt*(a+2*b+2*c+d)/6;
end
function xn=virtualStep(x,u,dt)
xn=x;
for j=0:5
    val=zeros(3,1);
    for ell=j:5
        val=val+x(3*ell+1:3*ell+3)*dt^(ell-j)/factorial(ell-j);
    end
    xn(3*j+1:3*j+3)=val+u(1:3)*dt^(6-j)/factorial(6-j);
end
xn(19)=x(19)+dt*x(20)+.5*dt^2*u(4);xn(20)=x(20)+dt*u(4);
end
