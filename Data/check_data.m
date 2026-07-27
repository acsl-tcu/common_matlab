clear;
clc;

%% 複数のMATファイルを選択
[fileNames, filePath] = uigetfile( ...
    {'*.mat', 'MAT-files (*.mat)'; ...
     '*.*',   'すべてのファイル (*.*)'}, ...
    '確認したいMATファイルを選択', ...
    'MultiSelect', 'on');

%% キャンセルした場合
if isequal(fileNames, 0)
    disp('ファイル選択がキャンセルされました');
    return;
end

%% 1ファイルだけ選択した場合もセル配列に変換
if ischar(fileNames)
    fileNames = {fileNames};
end

%% kの合計を初期化
kTotal = 0;

%% 選択したファイルを順番に確認
for fileIdx = 1:length(fileNames)

    fileName = fileNames{fileIdx};
    fullFileName = fullfile(filePath, fileName);

    %% ファイル読み込み
    data = load(fullFileName);

    %% ファイル名表示
    fprintf('\n\n');
    fprintf('========================================\n');
    fprintf('ファイル名：%s\n', fileName);
    fprintf('========================================\n');

    %% 変数一覧
    info = whos('-file', fullFileName);

    fprintf('\n=== 変数一覧 ===\n');

    for i = 1:length(info)
        fprintf('%s    size: %s    class: %s\n', ...
            info(i).name, ...
            mat2str(info(i).size), ...
            info(i).class);
    end

    %% 各変数の中身
    fprintf('\n=== 変数の中身 ===\n');

    variableNames = fieldnames(data);

    for i = 1:length(variableNames)

        variableName = variableNames{i};

        fprintf('\n----- %s -----\n', variableName);
        disp(data.(variableName));

    end

    %% kを合計
%% logger.k を合計
%% kを合計
%% log.k を合計
if isfield(data, 'log') && isstruct(data.log) && isfield(data.log, 'k')

    currentK = data.log.k;

    if isnumeric(currentK) && isscalar(currentK)
        kTotal = kTotal + currentK;
        fprintf('このファイルの log.k：%g\n', currentK);
    else
        warning('%s の log.k は数値スカラーではありません。', fileName);
    end

else
    warning('%s には log.k がありません。', fileName);
end

end

%% 全ファイルのkの合計を表示
fprintf('\n\n');
fprintf('========================================\n');
fprintf('全ファイルの k の合計：%g\n', kTotal);
fprintf('========================================\n');