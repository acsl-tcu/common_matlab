function summary = inspect_CBF_QPConflict(snapshotPath)
%INSPECT_CBF_QPCONFLICT ステップ12の保存QPを、実装変更なしに原因分類する。
%
% 配置: common_matlab/inspect_CBF_QPConflict.m
% 使用:
%   summary=inspect_CBF_QPConflict();
%   summary=inspect_CBF_QPConflict('C:\path\to\real_B_sphere_failure_snapshot.mat');
%
% 元の障害物を除いても可行性が戻らないかをlinprogで再検査する。
% 確認用線形計画の解は制御へ適用しない。
if nargin<1 || isempty(snapshotPath)
    paths=test_CBF_SetupPaths();
    snapshotPath=fullfile(paths.generator,'CBF','test_results', ...
        'real_B_sphere_failure_snapshot.mat');
end
if exist(snapshotPath,'file')~=2,error('CBF:NoSnapshot','MATがありません: %s',snapshotPath);end
if exist('linprog','file')~=2,error('CBF:NeedsLinprog','linprogが必要です。');end
D=load(snapshotPath);
q=D.qp;A=q.A;b=q.b;L=string(q.labels(:));
obsRows=startsWith(L,'obstacle_') | startsWith(L,'cable_');
groundRows=startsWith(L,'ground_');
inputRows=startsWith(L,'thrust_') | startsWith(L,'tau_');
groups={'全制約','障害物なし','地面なし','地面と入力のみ','入力のみ','地面のみ'};
maskList={true(size(b)),~obsRows,~groundRows,groundRows|inputRows,inputRows,groundRows};
summary=struct();summary.path=snapshotPath;
summary.status=cell(numel(groups),2);
opts=optimoptions('linprog','Display','off');
fprintf('\n=== 保存QPの制約グループ別可行性 ===\n');
for k=1:numel(groups)
    mask=maskList{k};
    [~,~,flag]=linprog(zeros(4,1),-A(mask,:),-b(mask),[],[], ...
        -inf(4,1),inf(4,1),opts);
    summary.status{k,1}=groups{k};summary.status{k,2}=flag;
    fprintf('%-20s: 行%3d、linprog exitflag=%d\n',groups{k},sum(mask),flag);
end

% 失敗した一部の不等式を、数値として人が確認できるように表示。
required={'ground_drone','thrust_max','tau_x_min','tau_y_max'};
ix=zeros(4,1);
for k=1:4
    found=find(L==required{k},1);
    if isempty(found)
        fprintf('必要なラベル%sが見つからないため、上限証明を省略します。\n',required{k});
        return;
    end
    ix(k)=found;
end
ag=A(ix(1),:);
if ag(1)>0 && ag(2)<0 && ag(3)>=0 && abs(ag(4))<1e-12
    % 保存ケースでは入力限界の行が各決定変数に直接作用する。
    nuTmax=-b(ix(2));tauXmin=b(ix(3));tauYmax=-b(ix(4));
    maximumLHS=ag(1)*nuTmax+ag(2)*tauXmin+ag(3)*tauYmax;
    fprintf('\n[説明] 地面CBF: %.6f*nuT %+.6f*nuTauX %+ .6g*nuTauY >= %.6f\n', ...
        ag(1),ag(2),ag(3),b(ix(1)));
    fprintf('[説明] 入力制約内の左辺最大値=%.6f（要求値=%.6f、差=%.6f）\n', ...
        maximumLHS,b(ix(1)),b(ix(1))-maximumLHS);
    summary.proofBound=maximumLHS;summary.required=b(ix(1));
else
    fprintf('入力符号が想定した状態と異なります。数値的証明を省略します。\n');
end
fprintf('\nこれは現行のHOCBF不等式の競合を示すもので、現実の回避不能を証明するものではありません。\n');
end
