function trials = load_mec_training_trials(folder)
% MATファイルを改名せず読み、飛行中の連続した(e,delta_u,e_next)だけを取り出す。
files = dir(fullfile(folder,'*.mat'));
assert(numel(files)>=5,'MEC:TooFewTrials','学習・調整・評価を分けるため5試行以上が必要です。');
trials = cell(1,numel(files));
for file_index = 1:numel(files)
    file = fullfile(files(file_index).folder,files(file_index).name);
    saved = load(file);
    if isfield(saved,'Dataset')
        data = saved.Dataset;
    elseif isfield(saved,'log')
        log = saved.log;
        results = log.Data.agent(1).controller.result;
        first = find(~cellfun(@isempty,results),1);
        assert(~isempty(first) && isfield(results{first},'error_definition'), ...
            'MEC:LegacyLog','誤差定義が記録されていない旧ログは使用できません：%s',file);
        steps = diff(unique(log.Data.t(log.Data.t>0)));
        metadata = struct('dt',median(steps),'error_definition',results{first}.error_definition);
        data = build_mec_error_dataset(log,metadata);
    else
        error('MEC:InvalidDataset','Datasetまたはlogがありません：%s',file);
    end
    assert(strcmp(data.metadata.error_definition,'nominal_input_one_step_v1'), ...
        'MEC:ErrorDefinition','MECKCの誤差定義と一致しません：%s',file);
    assert(size(data.E,1)==26 && size(data.dU,1)==4 && ...
        isequal(size(data.E),size(data.E_next)) && size(data.dU,2)==size(data.E,2), ...
        'MEC:DatasetSize','誤差と入力のサイズが不正です：%s',file);
    assert(all(isfinite([data.E(:);data.E_next(:);data.dU(:)])), ...
        'MEC:NonfiniteDataset','非有限値を含むデータです：%s',file);
    assert(max(abs([data.E(16,:),data.E_next(16,:)]))<1e-12, ...
        'MEC:ConstantObservable','定数観測量の誤差が0ではありません。');
    data.source_file = file;
    if file_index>1
        assert(abs(data.metadata.dt-trials{1}.metadata.dt)<1e-10, ...
            'MEC:SampleTime','異なる周期のデータは混ぜられません。');
        % 同じ試行のコピーを学習側と検証側へ分けない。
        for previous = 1:file_index-1
            assert(~isequal(data.E,trials{previous}.E) || ~isequal(data.dU,trials{previous}.dU), ...
                'MEC:DuplicateTrial','同じ試行の重複ファイルがあります：%s',file);
            if isfield(data.metadata,'trajectory_seed') && isfield(trials{previous}.metadata,'trajectory_seed')
                assert(data.metadata.trajectory_seed~=trials{previous}.metadata.trajectory_seed, ...
                    'MEC:RepeatedSeed','同じ軌道シードは別の検証試行として扱えません：%s',file);
            end
        end
    end
    trials{file_index} = data;
end
end
