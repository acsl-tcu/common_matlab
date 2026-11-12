function results=test_CBF_MultiObstacleRollout(shapes,steps)
%TEST_CBF_MULTIOBSTACLEROLLOUT HLCを使わない複数障害物の閉ループ試験。
% 例: results=test_CBF_MultiObstacleRollout({'B','D'},40);
% 何かが失敗したら無理な入力を適用せず、その時点で停止して原因を記録。
% 名目入力はストレス試験用でありHLCの模擬ではない。
% 初期状態から保証された安全回避軌道が存在するとは仮定しない。
if nargin<1 || isempty(shapes),shapes={'B'};end
if ischar(shapes),shapes={shapes};end
if nargin<2 || isempty(steps),steps=40;end
assert(steps>=1 && steps==floor(steps));
paths=test_CBF_SetupPaths();s=test_CBF_Fixture();
W=eye(4);scenes={'front','three','corridor','five'};
folder=fullfile(paths.generator,'CBF','test_results');
if ~exist(folder,'dir'),mkdir(folder);end
rows=cell(2*numel(shapes)*numel(scenes),11);idx=0;
for modeCell={'real','virtual'}
    mode=modeCell{1};
    for shapeCell=shapes(:).'
        shape=char(shapeCell{1});
        for sceneCell=scenes
            scene=sceneCell{1}; obstacles=localScene(scene);
            [x,dt,nominal]=initialState(mode,s);
            h0=cbf_evaluate_scene(mode,shape,x,s,obstacles);
            minH=min(h0);minPsi=inf;minResidual=inf;
            correction=0;completed=0;status='completed';
            positions=zeros(steps,3);logs=zeros(steps,8);
            % テスト専用の診断域。制御入力の飽和・変更には使わない。
            % QPの完走と、状態の数値発散を同じ成功扱いにしないため。
            testMaxDistance=50;  % m, 初期位置からの移動量（このテストだけ）
            testMaxAbsState=1e7; % 各状態成分（このテストだけ）
            maxNu=0;
            startX=localLoadX(mode,x);
            for k=1:steps
                qp=cbf_hocbf_scene(mode,shape,x,s,obstacles);
                minPsi=min(minPsi,min(cellfun(@min,qp.psi)));
                if ~all(qp.psiInitiallyNonnegative)
                    status='initial_HOCBF_set_violation';break;
                end
                sol=cbf_solve_hocbf_qp(qp,nominal,W);
                if sol.exitflag<=0 || ~isfinite(sol.minResidual) || sol.minResidual<-1e-7
                    status='qp_infeasible_or_solver_failure';break;
                end
                correction=correction+(norm(sol.nu-nominal)>1e-7);
                maxNu=max(maxNu,norm(sol.nu));
                minResidual=min(minResidual,sol.minResidual);
                if strcmp(mode,'real')
                    xn=realStep(x,sol.nu,dt,s);
                    valid=abs(norm(xn(1:4))-1)<1e-5 && ...
                          abs(norm(xn(14:16))-1)<1e-5 && ...
                          xn(20)>=s.input.thrustMin-1e-7 && xn(20)<=s.input.thrustMax+1e-7 && ...
                          all(xn(24:26)>=s.input.torqueMin(:)-1e-7) && ...
                          all(xn(24:26)<=s.input.torqueMax(:)+1e-7);
                    if ~valid,status='extended_input_or_state_violation';break;end
                    d=cbf_model_diagnostics('real',xn,s);
                    if ~isfinite(d.tension) || d.tension<=0
                        status='tension_model_invalid';break;
                    end
                else
                    xn=virtualStep(x,sol.nu,dt);
                end
                if any(~isfinite(xn)) || ...
                   abs(localLoadX(mode,xn)-startX)>testMaxDistance || ...
                   max(abs(xn))>testMaxAbsState
                    status='numerical_state_growth';break;
                end
                % 高次補助条件も次状態で確認する（QPが解けても必要）。
                qpNext=cbf_hocbf_scene(mode,shape,xn,s,obstacles);
                if ~all(qpNext.psiInitiallyNonnegative)
                    status='post_step_HOCBF_set_violation';break;
                end
                hNext=cbf_evaluate_scene(mode,shape,xn,s,obstacles);
                minH=min(minH,min(hNext));
                if any(hNext<-1e-7)
                    status='geometric_safety_violation';break;
                end
                x=xn;completed=k;
                positions(k,:)=xLoad(mode,x).';
                logs(k,:)=[k*dt,min(hNext),minPsi,sol.minResidual, ...
                    norm(sol.nu-nominal),localLoadX(mode,x), ...
                    sum(qp.degenerate),norm(sol.nu)];
            end
            if completed==0, minResidual=NaN; end
            idx=idx+1;
            rows(idx,:)={mode,shape,scene,numel(obstacles),steps,completed, ...
                         status,correction,minH,minPsi,localLoadX(mode,x)-startX};
            tag=sprintf('multi_rollout_%s_%s_%s',mode,shape,scene);
            header={'timeStep','minH','minPsi','qpResidual','correctionNorm', ...
                    'loadX','degenerateRows','nuNorm'};
            if completed>0
                writetable(array2table(logs(1:completed,:),'VariableNames',header), ...
                    fullfile(folder,[tag '.csv']));
            end
            fprintf('%7s %s %-8s: %2d/%2d step, status=%s, QP修正=%d, minH=%.5g, dx=%.4g m, max|nu|=%.3g\n', ...
                mode,shape,scene,completed,steps,status,correction,minH,localLoadX(mode,x)-startX,maxNu);
        end
    end
end
results=cell2table(rows,'VariableNames',{'mode','shape','scene','nObstacles', ...
 'requestedSteps','completedSteps','status','nCorrections','minH','minPsi','loadDeltaX'});
writetable(results,fullfile(folder,'multi_rollout_summary.csv'));
fprintf('注意: 完走は回避目標への到達や将来可行性の証明ではありません。\n');
end
function [x,dt,nominal]=initialState(mode,s)
if strcmp(mode,'virtual')
    x=zeros(20,1);x(1:3)=[0;0;1.4];
    dt=.025;nominal=[100;0;0;0];
else
    x=zeros(29,1);x(1)=1;x(8:10)=[0;0;1.4];x(14:16)=[0;0;-1];
    x(20)=(s.physical.massDrone+s.physical.massLoad)*s.physical.gravity;
    dt=.01;nominal=[0;3;0;0];
end
end
function o=localScene(name)
front=struct('type','sphere','center',[.8;0;1.4],'radius',.14);
three={front, struct('type','sphere','center',[1.1; .55;1.5],'radius',.18), ...
       struct('type','sphere','center',[1.1;-.55;1.5],'radius',.18)};
a=.25;b=.32;
Rz=[cos(a) -sin(a) 0;sin(a) cos(a) 0;0 0 1];
Rx=[1 0 0;0 cos(b) -sin(b);0 sin(b) cos(b)];
e=struct('type','ellipsoid','center',[1.7;.4;1.6], ...
         'axes',[.12;.22;.26],'R',Rz*Rx);
if strcmp(name,'front')
    o={front};
elseif strcmp(name,'three')
    o=three;
elseif strcmp(name,'corridor')
    o={struct('type','sphere','center',[.88;.55;1.4],'radius',.18), ...
       struct('type','sphere','center',[.88;-.55;1.4],'radius',.18),e};
else
    o=[three,{e,struct('type','sphere','center',[2;-.6;1.45],'radius',.19)}];
end
end
function x=realStep(x,nu,dt,s)
a=test_CBF_IndependentPlantRHS(x,nu,s);
b=test_CBF_IndependentPlantRHS(x+dt*a/2,nu,s);
c=test_CBF_IndependentPlantRHS(x+dt*b/2,nu,s);
d=test_CBF_IndependentPlantRHS(x+dt*c,nu,s);
x=x+dt/6*(a+2*b+2*c+d);
end
function x=virtualStep(x,nu,dt)
old=x;
for j=0:5
    val=zeros(3,1);
    for l=j:5
        val=val+old(3*l+1:3*l+3)*dt^(l-j)/factorial(l-j);
    end
    x(3*j+1:3*j+3)=val+nu(1:3)*dt^(6-j)/factorial(6-j);
end
x(19)=old(19)+dt*old(20)+dt^2*nu(4)/2;
x(20)=old(20)+dt*nu(4);
end
function x=xLoad(mode,s)
if strcmp(mode,'real'),x=s(8:10);else,x=s(1:3);end
end
function x=localLoadX(mode,s)
if strcmp(mode,'real'),x=s(8);else,x=s(1);end
end
