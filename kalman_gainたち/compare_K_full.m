%% compare_A_mat_files.m
% 複数のMATファイルに保存された A 行列を比較するスクリプト
% 基準ファイルの A から、各比較ファイルの A をそれぞれ引き算します。
%
% 実行後、以下の変数がワークスペースに残ります。
%   A_ref          : 基準ファイルの A
%   A_compare{k}   : k番目の比較ファイルの A
%   A_diff{k}      : A_ref - A_compare{k}
%   ref_file       : 基準ファイルのフルパス
%   compare_files  : 比較ファイルのフルパス一覧
%
% ※関数ではなくスクリプトなので、結果はMATLABのワークスペースに残ります。

%% 1. 基準のMATファイルを選ぶ
[ref_name, ref_folder] = uigetfile('*.mat', ...
    '【1/2】基準ファイル（引かれる側）を1つ選択');

if isequal(ref_name, 0)
    disp('基準ファイルの選択がキャンセルされました。');
    return;
end

ref_file = fullfile(ref_folder, ref_name);
ref_data = load(ref_file, 'K_full');

if ~isfield(ref_data, 'K_full')
    error('基準ファイルに変数 K_full がありません: %s', ref_file);
end
K_full_ref = ref_data.K_full;

if ~isnumeric(K_full_ref)
    error('基準ファイルの K_full は数値配列ではありません。');
end

%% 2. 比較するMATファイルを複数選ぶ
[other_names, other_folder] = uigetfile(fullfile(ref_folder, '*.mat'), ...
    '【2/2】基準から引く比較ファイルを選択（複数選択可）', ...
    'MultiSelect', 'on');

if isequal(other_names, 0)
    disp('比較ファイルの選択がキャンセルされました。');
    return;
end

if ischar(other_names)
    other_names = {other_names};
end

% 基準ファイルが比較対象にも選ばれていたら除外
compare_files = cell(0, 1);
for k = 1:numel(other_names)
    file_path = fullfile(other_folder, other_names{k});
    if strcmpi(file_path, ref_file)
        warning('基準ファイルは比較対象から除外します: %s', other_names{k});
    else
        compare_files{end+1,1} = file_path; %#ok<SAGROW>
    end
end

if isempty(compare_files)
    disp('比較できるファイルがありません。');
    return;
end

%% 3. A 行列を読み込み、引き算
n = numel(compare_files);
K_full_compare = cell(n, 1);
K_full_diff = cell(n, 1);

fprintf('\n基準: %s\n', ref_name);
fprintf('------------------------------\n');

for k = 1:n
    file_data = load(compare_files{k}, 'K_full');
    if ~isfield(file_data, 'K_full')
        error('比較ファイルに変数 A がありません: %s', compare_files{k});
    end

    K_full_compare{k} = file_data.K_full;

    if ~isnumeric(K_full_compare{k})
        error('数値配列でない K_full が含まれています: %s', compare_files{k});
    end
    if ~isequal(size(K_full_ref), size(K_full_compare{k}))
        error('K_full のサイズが違うため引き算できません。\n基準: %s\n比較: %s', ...
            mat2str(size(K_full_ref)), mat2str(size(K_full_compare{k})));
    end

    K_full_diff{k} = K_full_ref - K_full_compare{k};

    [~, short_name, ext] = fileparts(compare_files{k});
    fprintf('A_diff{%d} = 基準A - %s%s のA\n', k, short_name, ext);
end

fprintf('------------------------------\n');
disp('完了。結果はワークスペースの A_diff に保存されています。');
disp('例: A_diff{1}, A_diff{2}');
