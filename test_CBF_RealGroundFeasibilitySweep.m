function report = test_CBF_RealGroundFeasibilitySweep(gains)
%TEST_CBF_REALGROUNDFEASIBILITYSWEEP 実入力の地面・入力制約競合の原因を分離。
% 保存ステップ12の矛盾は「元の6階地面CBF」と「高次入力上下限」由来。
% ゲインを変えても一般状態での可行性を保証できないため、
% 幅広い姿勢と高度で比較し、将来のCBF設計へ診断結果を渡す。
% ここでは制約を緩めない。地面制約を本番で削除しない。
if nargin<1 || isempty(gains),gains=[0.8 1.2 1.5 2.0 3.0];end
paths=test_CBF_SetupPaths();s0=test_CBF_Fixture();
obs=struct('type','ellipsoid','center',[1.35;0.24;1.75], ...
    'axes',[.2;.25;.34],'R',rotZ(.30)*rotY(-.23)*rotX(.17));
options=optimoptions('linprog','Display','off');
rows=cell(numel(gains)*5,10);idx=0;
for g=gains(:).'
    for trial=1:5
        s=s0;s.cbf.groundGain=g;
        x=zeros(29,1);x(1:4)=yprQuat(.1*(trial-3),-.06*(trial-3),.08*(trial-3));
        x(8:10)=[0;0;1.15+.12*trial];
        x(11:13)=[0;0;-.08*(trial-3)];
        q=[.04*(trial-3);.03*(trial-3);-1];x(14:16)=q/norm(q);
        x(20)=(s.physical.massDrone+s.physical.massLoad)*s.physical.gravity;
        x(5:7)=[.02;-.015;.01]*(trial-3);
        x(17:19)=[-.01;.02;0]*(trial-3);
        qp=cbf_hocbf_scene('real','B',x,s,{obs});
        labels=qp.labels;
        isG=~cellfun('isempty',strfind(labels,'ground_')); %#ok<STREMP>
        isI=~cellfun('isempty',strfind(labels,'thrust_')) | ...
            ~cellfun('isempty',strfind(labels,'tau_')); %#ok<STREMP>
        h=cbf_evaluate_scene('real','B',x,s,{obs});
        idx=idx+1;
        rows(idx,:)={g,trial,min(h),all(qp.psiInitiallyNonnegative), ...
            feasible(qp,true(size(isG)),options), ...
            feasible(qp,isG|isI,options), ...
            feasible(qp,isG,options), ...
            feasible(qp,isI,options), ...
            min(qp.b(isG)),max(qp.b(isG))};
    end
end
report=cell2table(rows,'VariableNames',{'groundGain','scenario', ...
    'minH','initialPsiOK','allFeasible','groundInputFeasible', ...
    'groundOnlyFeasible','inputOnlyFeasible','groundBMin','groundBMax'});
folder=fullfile(paths.generator,'CBF','test_results');
if ~exist(folder,'dir'),mkdir(folder);end
writetable(report,fullfile(folder,'real_ground_feasibility_sweep.csv'));
disp(report(:,{'groundGain','scenario','minH','initialPsiOK', ...
    'allFeasible','groundInputFeasible'}));
fprintf('注: 一時点の線形制約可行性のみ。衝突回避成功や将来可行性の証明ではありません。\n');
end
function tf=feasible(qp,keep,options)
A=qp.A(keep,:);b=qp.b(keep);
if isempty(A),tf=true;return;end
[nu,~,flag]=linprog(zeros(4,1),-A,-b,[],[],[],[],options);
tf=flag>0 && ~isempty(nu) && all(A*nu>=b-1e-7);
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
