function report=test_CBF_ReferenceTrajectoryCases(steps)
%TEST_CBF_REFERENCETRAJECTORYCASES HLCを接続しない仮想系の4種参照軌道試験。
% 直線・円・上昇・8の字。参照位置から6階微分まで解析的に設定する。
% 実入力での追従を保証するものではない。物理入力制限は未接続。
% 6階フィードフォワード値を名目決定入力とし、CBF-QPで修正する。
% 決定入力上下限は数値暴走検出用の試験上限。
if nargin<1||isempty(steps),steps=25;end
paths=test_CBF_SetupPaths();s=test_CBF_Fixture();
trajectories={'line','circle','ascent','figure8'};
shapes={'B','D'};types={'sphere','ellipsoid'};dt=.02;
% テスト時の明示的な決定入力制限。現実の推力/トルクではない。
limits=struct('lower',[-30;-30;-30;-5],'upper',[30;30;30;5]);
rows=cell(numel(trajectories)*numel(shapes)*numel(types),10);idx=0;
for trajectory=trajectories
    name=trajectory{1};
    for sh=shapes
        shape=sh{1};
        for ty=types
            kind=ty{1};
            ob=makeObstacle(kind);
            x=initialFromReference(name);
            status='completed';count=0;minH=inf;maxInput=0;minPsi=inf;
            for k=1:steps
                t=(k-1)*dt;
                qp=cbf_hocbf_scene('virtual',shape,x,s,{ob});
                psi=vertcat(qp.psi{:});minPsi=min(minPsi,min(psi));
                if ~all(qp.psiInitiallyNonnegative)
                    status='initial_HOCBF_set_violation';break;
                end
                nuNom=[referenceDerivative(name,t,6);0];
                sol=cbf_solve_hocbf_qp(qp,nuNom,eye(4),limits);
                if sol.exitflag<=0 || ~isfinite(sol.minResidual) || sol.minResidual<-1e-7
                    status='qp_infeasible_with_explicit_bounds';break;
                end
                maxInput=max(maxInput,norm(sol.nu,Inf));
                xn=stepVirtual(x,sol.nu,dt);
                diag=cbf_model_diagnostics('virtual',xn,s);
                if ~all(isfinite(xn)) || ~diag.flatMapValid || norm(xn(1:18),Inf)>1000
                    status='flat_map_or_state_invalid';break;
                end
                h=cbf_evaluate_scene('virtual',shape,xn,s,{ob});
                minH=min(minH,min(h));
                if any(~isfinite(h)) || any(h<-1e-7)
                    status='geometric_safety_violation';break;
                end
                x=xn;count=k;
            end
            if count==steps && strcmp(status,'completed')
                try
                    qpEnd=cbf_hocbf_scene('virtual',shape,x,s,{ob});
                    if ~all(qpEnd.psiInitiallyNonnegative)
                        status='final_HOCBF_set_violation';
                    end
                catch
                    status='final_coefficient_error';
                end
            end
            idx=idx+1;
            rows(idx,:)={name,shape,kind,steps,count,status,minH, ...
                minPsi,maxInput,norm(x(1:3)-referenceDerivative(name,count*dt,0))};
            fprintf('軌道 %-7s %s %-9s %2d/%2d %s, max|nu|=%.4g\n', ...
                name,shape,kind,count,steps,status,maxInput);
        end
    end
end
report=cell2table(rows,'VariableNames',{'trajectory','shape','obstacleType', ...
    'requestedSteps','completedSteps','status','minH','minPsi', ...
    'maxInput','endReferenceError'});
folder=fullfile(paths.generator,'CBF','test_results');
if ~exist(folder,'dir'),mkdir(folder);end
writetable(report,fullfile(folder,'virtual_reference_trajectory_cases.csv'));
end
function x=initialFromReference(name)
x=zeros(20,1);
for j=0:5,x(3*j+1:3*j+3)=referenceDerivative(name,0,j);end
end
function v=referenceDerivative(name,t,k)
% 試験用のみ。生成関数へ座標・速度・周波数は固定しない。
w=1.0;
switch name
    case 'line'
        if k==0,v=[0;0;1.5];
        elseif k==1,v=[.25;0;0];
        else,v=zeros(3,1);end
    case 'circle'
        v=[.3*w^k*cos(w*t+k*pi/2); ...
           .3*w^k*sin(w*t+k*pi/2);double(k==0)*1.5];
    case 'ascent'
        v=[0;0;.14*w^k*sin(w*t+k*pi/2)+double(k==0)*1.5];
    case 'figure8'
        v=[.30*w^k*sin(w*t+k*pi/2); ...
           .15*(2*w)^k*sin(2*w*t+k*pi/2);double(k==0)*1.5];
    otherwise
        error('CBF:Trajectory','不明な参照軌道');
end
end
function o=makeObstacle(kind)
if strcmp(kind,'sphere')
    o=struct('type','sphere','center',[1.25;.18;1.5],'radius',.15);
else
    a=.41;b=-.25;c=.23;
    Rx=[1 0 0;0 cos(a) -sin(a);0 sin(a) cos(a)];
    Ry=[cos(b) 0 sin(b);0 1 0;-sin(b) 0 cos(b)];
    Rz=[cos(c) -sin(c) 0;sin(c) cos(c) 0;0 0 1];
    o=struct('type','ellipsoid','center',[1.25;.18;1.5], ...
        'axes',[.13;.19;.24],'R',Rz*Ry*Rx);
end
end
function xn=stepVirtual(x,u,dt)
xn=x;
for j=0:5
    v=zeros(3,1);
    for ell=j:5
        v=v+x(3*ell+1:3*ell+3)*dt^(ell-j)/factorial(ell-j);
    end
    xn(3*j+1:3*j+3)=v+u(1:3)*dt^(6-j)/factorial(6-j);
end
xn(19)=x(19)+dt*x(20)+.5*dt^2*u(4);xn(20)=x(20)+dt*u(4);
end
