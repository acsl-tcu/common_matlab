%% 2025.07.21
% このセクションを実行するだけで良い
% LOGGERのdataメソッドを使用している

phase = 'f';

folder_path = 'Data/Sim_data/'; % 参照したいデータが保存されているフォルダのパス
learning_folder = 'Data/Learning_data/'; % 学習データ保存用フォルダのパス
if ~exist(learning_folder, "dir"), mkdir(learning_folder); end

matfiles = dir(fullfile(folder_path, '*.mat'));
file_names = {matfiles.name};
variables = ["input", "p", "q", "v", "w"];
attributes = ["", "e", "e", "e", "e"];

fprintf('Selected phase: %s', phase)

for i=1:length(file_names)
    file_name = fullfile(folder_path, file_names{i});
    logger = LOGGER(file_name);
    for j=1:length(variables)
        tmp = logger.data(1, variables(j), attributes(j), "phase",phase);
        switch variables(j)
            case "input", input = tmp';
            case "p", p = tmp';
            case "q", q = tmp';
            case "v", v = tmp';
            case "w", w = tmp';
        end
    end
    save(fullfile(learning_folder, file_names{i}), "input", "p", "q", "v", "w")
end

logger.plot({1,"p","e"}, "fig_num",1, "phase",phase) % for check
