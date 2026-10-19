function data = build_mec_error_dataset(logger,metadata)
% e[k+1] ≈ A e[k]+B(u_plant[k]-u_nominal[k]) の同定用データを抽出する。
% 飛行中の隣接ステップだけを使い、初期化・停止・試行間を接続しない。
arguments
    logger
    metadata struct
end
results = logger.Data.agent(1).controller.result;
count = min([logger.k,numel(results),numel(logger.Data.t)]);
indices = [];
for sample_index = 1:count-1
    current = results{sample_index};
    following = results{sample_index+1};
    if logger.Data.phase(sample_index)~='f' || logger.Data.phase(sample_index+1)~='f'
        continue;
    end
    if ~isfield(current,'e_valid') || ~current.e_valid || ~following.e_valid
        continue;
    end
    interval = logger.Data.t(sample_index+1)-logger.Data.t(sample_index);
    if abs(interval-metadata.dt)>1e-8
        continue;
    end
    assert(strcmp(current.error_definition,metadata.error_definition), ...
        'MEC:ErrorDefinition','誤差の定義が一致しないデータは結合できません。');
    values = [current.e(:);following.e(:);current.input_plant(:);current.input_nominal(:)];
    assert(all(isfinite(values)),'MEC:NonfiniteDataset','データにNaNまたはInfが含まれています。');
    indices(end+1) = sample_index; %#ok<AGROW>
end
assert(~isempty(indices),'MEC:EmptyDataset','飛行中の有効な隣接データがありません。');
data.E = cell2mat(cellfun(@(r) r.e(:),results(indices),'UniformOutput',false));
data.E_next = cell2mat(cellfun(@(r) r.e(:),results(indices+1),'UniformOutput',false));
data.input_nominal = cell2mat(cellfun(@(r) r.input_nominal(:),results(indices),'UniformOutput',false));
data.input_plant = cell2mat(cellfun(@(r) r.input_plant(:),results(indices),'UniformOutput',false));
data.dU = data.input_plant-data.input_nominal;
data.excitation = cell2mat(cellfun(@(r) r.excitation(:),results(indices),'UniformOutput',false));
data.delta_u_mec = cell2mat(cellfun(@(r) r.delta_u_mec(:),results(indices),'UniformOutput',false));
data.X = cell2mat(cellfun(@(r) r.x_estimated(:),results(indices),'UniformOutput',false));
data.Y = cell2mat(cellfun(@(r) r.x_estimated(:),results(indices+1),'UniformOutput',false));
data.U = data.input_plant;
data.T = reshape(logger.Data.t(indices),1,[]);
data.T_next = reshape(logger.Data.t(indices+1),1,[]);
data.sample_index = indices;
data.N = numel(indices);
data.uN = 4;
data.fExp = logger.fExp;
data.metadata = metadata;
% 既存の対話型読み込み処理でも、そのまま使用できる選択情報。
data.datarange = 3;
data.range = 0;
data.IDX = 0;
data.phase2 = 0;
data.vxyz = 2;
assert(size(data.E,1)==26 && size(data.E_next,1)==26 && size(data.dU,1)==4, ...
    'MEC:DatasetDimension','E/E_nextは26行、dUは4行である必要があります。');
assert(max(abs(data.E(16,:)))<1e-12 && max(abs(data.E_next(16,:)))<1e-12, ...
    'MEC:ConstantObservable','定数観測量の差は0である必要があります。');
% 単位の違いを除いた行列で、4入力の独立性と回帰行列のランクを確認する。
regressor = [data.E([1:15,17:26],:);data.dU];
scale = sqrt(mean(regressor.^2,2));
scale(scale==0) = 1;
data.diagnostics.input_rank = rank(data.dU./scale(end-3:end));
data.diagnostics.regressor_rank = rank(regressor./scale);
data.diagnostics.regressor_rows = size(regressor,1);
data.diagnostics.input_rms = sqrt(mean(data.dU.^2,2));
assert(data.diagnostics.input_rank==4, ...
    'MEC:InputRank','4入力が独立に励振されていません。振幅・収集時間を確認してください。');
end
