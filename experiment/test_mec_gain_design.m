% 近似的に重複する観測量を持つ既知の系で、同定と座標変換を検証する。
repo_root = fileparts(fileparts(mfilename('fullpath')));
addpath(genpath(repo_root));
setting = mec_gain_settings();
stream = RandStream('mt19937ar','Seed',219);
Atrue = [0.85,0.04,0;0,0.75,0.02;0,0,0.65];
Btrue = [0.1,0.02,0,0;0,0.05,0.03,0;0,0,0.02,0.04];
observable = randn(stream,26,3);
observable(16,:) = 0;
trials = cell(1,10);
for trial_index = 1:10
    U = randn(stream,4,300).*[0.2;0.03;0.03;0.02];
    X = zeros(3,301);
    X(:,1) = randn(stream,3,1)*0.1;
    for sample = 1:300
        X(:,sample+1) = Atrue*X(:,sample)+Btrue*U(:,sample);
    end
    E = observable*X(:,1:end-1);
    Y = observable*X(:,2:end);
    trials{trial_index} = struct('E',E,'E_next',Y,'dU',U,'T',(0:299)*0.025, ...
        'metadata',struct('dt',0.025,'error_definition','nominal_input_one_step_v1'), ...
        'source_file',sprintf('synthetic_seed_%d',trial_index));
end
[est,report] = fit_mec_error_model(trials,setting);
assert(report.effective_rank==3,'重複した観測方向が除去されていません。');
assert(report.test.rmse<1e-10,'既知の線形系を再現できません。');
assert(isempty(intersect(report.train_ids,report.test_ids)));
assert(isempty(intersect(report.tune_ids,report.test_ids)));
assert(norm(est.encode*est.decode-eye(3),'fro')<1e-10);
gain = design_mec_error_gain(est,setting);
assert(isequal(size(gain.K_full),[4,26]));
assert(all(gain.K_full(:,16)==0));
assert(max(abs(eig(gain.A-gain.B*gain.K_full)))<1);
assert(gain.riccati_residual<1e-8);
% 正規化座標での制御則と元の26次元での制御則が一致することを確認する。
assert(norm(gain.K_full*trials{end}.E- ...
    (est.scale_u.*gain.K_reduced)*(est.encode*trials{end}.E),'fro')<1e-10);
fprintf('MEC_GAIN_DESIGN_TEST_PASSED\n');
