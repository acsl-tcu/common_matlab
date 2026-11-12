function summary=test_CBF_GroundGainDiagnostics(gains)
%TEST_CBF_GROUNDGAINDIAGNOSTICS HOCBFゲインに対する地面/入力の可行性を調査。
% 例: test_CBF_GroundGainDiagnostics([1.2 1.5 2 3]);
% 実行中の制御器を変更しない。安全保証・自動ゲイン調整ではない。
% 保存された step12 の状態を再利用し、各ゲインごとに全CBFを再計算。
if nargin<1 || isempty(gains), gains=[1.2 1.4 1.5 2 3]; end
paths=test_CBF_SetupPaths();
filename=fullfile(paths.generator,'CBF','test_results','real_B_sphere_failure_snapshot.mat');
assert(exist(filename,'file')==2, '元の失敗記録がありません: %s',filename);
raw=load(filename,'x','s','obs');x=raw.x;s=raw.s;obs=raw.obs;
assert(numel(x)==29,'状態数が異なります。');
% 古いスナップショットはサンプルの構造体フィールドを含む。
if ~isfield(obs,'type'),obs.type='sphere';end
rows=cell(numel(gains),8);
for i=1:numel(gains)
    s.cbf.groundGain=gains(i);
    [h,info]=cbf_evaluate_case('real','B',obs.type,x,s,obs);
    qp=cbf_hocbf_evaluate('real','B',obs.type,x,s,obs,h,info);
    [okAll,~]=localLP(qp.A,qp.b);
    isGround=~cellfun('isempty',strfind(qp.labels,'ground_')); %#ok<STREMP>
    isInput=~cellfun('isempty',strfind(qp.labels,'thrust_')) | ...
            ~cellfun('isempty',strfind(qp.labels,'tau_')); %#ok<STREMP>
    [okGroundInput,~]=localLP(qp.A(isGround|isInput,:),qp.b(isGround|isInput));
    rows(i,:)={gains(i),okAll,okGroundInput, ...
        all(qp.psiInitiallyNonnegative),min(h), ...
        min(cellfun(@min,qp.psi)),min(qp.b(isGround)),max(qp.b(isGround))};
    fprintf('groundGain=%6.2f  全制約=%d 地面+入力=%d 初期psi=%d minH=%.4g\n', ...
        gains(i),okAll,okGroundInput,rows{i,4},min(h));
end
summary=cell2table(rows,'VariableNames', ...
 {'groundGain','allFeasible','groundInputFeasible','initialPsiOK', ...
  'minH','minPsi','minGroundB','maxGroundB'});
folder=fullfile(paths.generator,'CBF','test_results');
writetable(summary,fullfile(folder,'ground_gain_sweep.csv'));
fprintf('※ 別ゲインで一時点が可行でも、閉ループ将来可行性や安全性は証明されません。\n');
end
function [ok,flag]=localLP(A,b)
opts=optimoptions('linprog','Display','off');
[~,~,flag]=linprog(zeros(4,1),-A,-b,[],[],[],[],opts);
ok=flag>0;
end
