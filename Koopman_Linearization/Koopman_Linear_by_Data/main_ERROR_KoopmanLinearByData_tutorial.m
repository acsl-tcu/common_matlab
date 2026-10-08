%% Koopman Linear by Data %%
%--------------------------------------------------------------------------------
%初心者用のクープマン線形化プログラム
%コマンドウィンドウの案内に従えばできるようになっているはず．．．
%慣れてきたら，main_KoopmanLinearByDataのほうでやるといいかも(設定を自分でできる)
%--------------------------------------------------------------------------------
clc
clear

%--------------------------------------------------------------
% 先に main.m の Initialize settings を実行すること(※必ず行う)
%--------------------------------------------------------------
initialize = input('＜main.mのInitialize settingsを実行しましたか？＞\n はい:1，いいえ:0：','s');
initialize = str2double(initialize);
if initialize == 0
    error('main.m の Initialize settings を実行してください')
end
clear all
clc
%---------------------------------------------
flg.bilinear = 0; %1:双線形モデルへの切り替え 木山は実機のデータではうまくいかなかった
flg.normalize = 0; %正規化するかどうか
flg.without_pos = 0; %位置無観測量
flg.weight = 0; %重み付き最小2乗法
setting = 0; %この値はいじらない
%---------------------------------------------

%%

method = input('KL または LYKL を入力してください：', 's');

if strcmpi(method, 'KL')
    % KLの場合の処理
    disp('KLを実行します');

elseif strcmpi(method, 'LYKL')
    % LYKLの場合の処理
    disp('LYKLを実行します');

else
    error('入力は KL または LYKL にしてください。');
end

%%
%データ保存先ファイル名(逐次変更しないと，上書きされる)
FileName = input('保存するファイル名を入力してください(※ ～.matを付ける): ', 's');
% [~, baseName, ~] = fileparts(FileName);
[~, baseName, ext] = fileparts(FileName);

folderPath = 'datasets'; %データセットに使用するデータはデータセットフォルダにいれておく main.mの階層
files = dir(folderPath);
% folderPath = 'KMPCSimデータセット'
fileList = dir(fullfile(folderPath,'*.mat')); %対象のファイルを取得
fprintf('\n＜データセットに使用するファイル名の統一を行います＞\n')

nFiles = length(fileList);
% 元の名前と変更後の名前を記録
originalFilePaths = cell(nFiles, 1);
renamedFilePaths  = cell(nFiles, 1);
% 読み込むデータファイル名は同じにする必要がある：学習データ
% loading_filename_1 みたいな感じになる
% loading_filename = input('\n統一するファイル名を入力してください(※ .matは含まない):','s');

% FileName = input('保存するファイル名を入力してください(※ ～.matを付ける): ', 's');
loading_filename = FileName;

% for i = 1:length(fileList)
%     oldFileName = fullfile(folderPath,fileList(i).name);
%     newFileName = fullfile(folderPath,[append(loading_filename,'_',num2str(i),'.mat')]);
%     movefile(oldFileName, newFileName); %名前の変更
% end

% % % % % % for i = 1:nFiles
% % % % % %
% % % % % %     oldFileName = fullfile(folderPath, fileList(i).name);
% % % % % %     newFileName = fullfile( ...
% % % % % %         folderPath, sprintf('%s_%d.mat', baseName, i));
% % % % % %
% % % % % %     % ここが必須
% % % % % %     originalFilePaths{i} = oldFileName;
% % % % % %     renamedFilePaths{i}  = newFileName;
% % % % % %     % fprintf('src: %s\n', src);
% % % % % %     % fprintf('dst: %s\n', dst);
% % % % % %
% % % % % %     movefile(oldFileName, newFileName);
% % % % % % end

% for seed = 1:10
%
%     % データ読み込み・処理
%
%     fprintf("seed=%d : X NaN=%d, U NaN=%d\n", ...
%         seed, nnz(isnan(X)), nnz(isnan(U)));
%
% end

%% ファイル名変更
for i = 1:nFiles
    oldFileName = fullfile(folderPath, fileList(i).name);
    newFileName = fullfile(folderPath, sprintf('%s_%d.mat', baseName, i));

    originalFilePaths{i} = oldFileName;
    renamedFilePaths{i}  = newFileName;

    movefile(oldFileName, newFileName);
end

try
    %%


    Data.HowmanyDataset = numel(fileList); %読み込むデータ数

    if Data.HowmanyDataset > 0
        fprintf('\n＜ファイル名の統一が完了しました＞\n')
        fprintf('\n読み込むファイル数：%d\n',Data.HowmanyDataset)
    else
        error('データセットフォルダ内にファイルが存在しません') %データセットフォルダ内にファイルがない場合はエラー
    end

    %データ保存用,現在のファイルパスを取得,保存先を指定
    activeFile = matlab.desktop.editor.getActive;
    % nowFolder = fileparts(activeFile.Filename);
    nowFolder = fileparts(activeFile.Filename);
    % targetPath=append(nowFolder,'\',FileName);

    %% Defining Koopman Operator
    %<使用している観測量>
    F = @quaternions_all;
    % F = @quaternions_all_old;
    fprintf('\n選択されている観測量：%s\n',func2str(F))

    % load data
    % 実験データから必要なものを抜き出す処理,↓状態,→データ番号(同一番号のデータが対応関係にある)
    % Data.X 入力前の対象の状態
    % Data.U 対象への入力
    % Data.Y 入力後の対象の状態

    fprintf('\n＜データセットの結合を行います＞\n')

    for i = 1:Data.HowmanyDataset
        if contains(loading_filename,'.mat')
            Dataset = ImportFromExpData_ERROR_tutorial(loading_filename); %ImportFromExpData_tutorial:データセットをくっつけるための関数
        else
            if i == 1 %66 ~ 78はコマンドウィンドウから入力するのに必要(クープマン線形化には関係ない)
                setting = 1;
                Dataset = ImportFromExpData_ERROR_tutorial(append(loading_filename,'_',num2str(i),'.mat'),setting);
                datarange = Dataset.datarange;
                range = Dataset.range;
                IDX = Dataset.IDX;
                phase2 = Dataset.phase2;
                % vz_z = Dataset.vz_z;
                vz_z = Dataset.vxyz;
                fprintf('\n')
            else
                setting = 0;
                Dataset = ImportFromExpData_ERROR_tutorial(append(loading_filename,'_',num2str(i),'.mat'),setting,datarange,range,IDX,phase2,vz_z);
            end
        end
        if i==1
            Data.X = [Dataset.X];
            Data.U = [Dataset.U];
            Data.Y = [Dataset.Y];

            Data.E = [Dataset.E]; %追加
            Data.E_next = [Dataset.E_next];
            Data.dU = Dataset.dU;
        else
            Data.X = [Data.X, Dataset.X];
            Data.U = [Data.U, Dataset.U];
            Data.Y = [Data.Y, Dataset.Y];

            Data.E = [Data.E, Dataset.E];
            Data.E_next = [Data.E_next, Dataset.E_next];
            Data.dU = [Data.dU, Dataset.dU];
        end
        disp(append('loading data number: ',num2str(i),', now data:',num2str(Dataset.N),', all data: ',num2str(size(Data.X,2))))
    end

    fprintf('\n＜データセットの結合が完了しました＞\n')

    flg.normalize = input('\n＜正規化を行います＞\n はい:1，いいえ:0：','s');
    % if str2double(flg.normalize) == 1 %正規化を行うか(正規化については自分で調べて！)
    Ndata = Normalization(Data);
    Data.X = Ndata.x;
    Data.Y = Ndata.y;
    Data.U = Ndata.u;
    disp('正規化が完了しました')
    % end



    %% クォータニオンのノルムをチェック(クォータニオンのノルムは1にならなければいけないという制約がある)
    % 閾値を下回った or 上回った場合注意文を提示
    % attitude_norm 各時間におけるクォータニオンのノルム
    if size(Data.X,1)==13 %特に気にしなくていい
        thre = 0.01;
        attitude_norm = checkQuaternionNorm(Dataset.est.q',thre);
    end

    %% ここから始めるとき
    % clear; clc;
    % flg.bilinear = 0;
    % flg.normalize = 0;
    % F = @quaternions_all; % 改造用
    % FileName_common = strcat('EstimationResult_', string(datetime('now'), 'yyyy-MM-dd'), '_code03_');
    % FileName = strcat(FileName_common, 'Exp_Kiyama_code03_2');
    % activeFile = matlab.desktop.editor.getActive;
    % nowFolder = fileparts(activeFile.Filename);
    % % targetpath=append(nowFolder,'\',FileName);
    % targetpath=append(nowFolder,'\..\EstimationResult\',FileName);
    % load('Koopman_Linearization\Integration_Dataset\Kiyama_Exp_Dataset.mat');

    %% Koopman linearization
    % 12/12 関数化(双線形であるかどかの切り替え，flg.bilinear==1:双線形)
    fprintf('\n＜クープマン線形化を実行＞\n')
    if flg.bilinear == 1
        est = KL_biLinear(Data.X,Data.U,Data.Y,F);
    else
        if strcmpi(method, 'KL')

            disp('KL_err');
            est = KL_error(Data.E,Data.dU,Data.E_next,flg);


        elseif strcmpi(method, 'LYKL')
            % LYKLの場合の処理
            disp('LYKL_err');
            tic
            [~, ~, ~, ~, est] = rensyuuKLLY_err(Data.X,Data.U,Data.Y,F,flg); %クープマン線形化の具体的な計算をしてる部分
            calT = toc;

        end

    end


    % est.observable = F;
    fprintf('\n＜クープマン線形化が完了しました＞\n')

    %     %% 線形化が正常終了したのでファイル名を戻す
    %     restoreFileNames(originalFilePaths, renamedFilePaths);
    %
    %     fprintf('\n＜ファイル名を元に戻しました＞\n');
    %
    % fprintf('\n＜クープマン線形化が完了しました＞\n')

    fprintf('\n＜クープマン線形化が完了しました＞\n')

catch ME

    fprintf('\n========================================\n');
    fprintf('線形化中にエラーが発生しました。\n');
    fprintf('ファイル名を元に戻します。\n');
    fprintf('========================================\n');

    restoreFileNames(originalFilePaths, renamedFilePaths);

    fprintf('ファイル名の復元が完了しました。\n\n');

    rethrow(ME);

end


%% ここに来た = 正常終了
restoreFileNames(originalFilePaths, renamedFilePaths);

fprintf('\n＜ファイル名を元に戻しました＞\n');



%% Simulation by Estimated model
%% Simulation by Estimated model
% 誤差モデルの1ステップ予測精度検証

if ~strcmpi(method, 'KL') || flg.bilinear == 1
    error('この精度検証処理は現在KL_error専用です');
end

fprintf('\n＜誤差モデルの推定精度検証＞\n');

% 検証用データの読み込み
[fileName, filePath] = uigetfile('*.mat');

if isequal(fileName, 0)
    error('検証用データが選択されませんでした');
end

verification_data = fullfile(filePath, fileName);

simResult.reference = ...
    ImportFromExpData_ERROR_estimation_tutorial(verification_data);

%% 検証用データ

E      = simResult.reference.E;
E_next = simResult.reference.E_next;
dU     = simResult.reference.dU;

N = size(E,2);

% データ数と次元を確認
assert(isequal(size(E),size(E_next)), ...
    'EとE_nextのサイズが一致しません');

assert(size(dU,2) == N, ...
    'EとdUのデータ数が一致しません');

assert(size(E,1) == size(est.A,2), ...
    'EとAの次元が一致しません');

assert(size(E_next,1) == size(est.A,1), ...
    'E_nextとAの次元が一致しません');

assert(size(dU,1) == size(est.B,2), ...
    'dUとBの次元が一致しません');

%% 1ステップ予測

if flg.weight
    % 重み付き同定時は元の誤差座標に戻す
    simResult.Ehat_next = est.Q \ ...
        (est.A * (est.Q * E) + est.B * dU);
else
    simResult.Ehat_next = est.A * E + est.B * dU;
end

% 実際の次時刻の誤差
simResult.Etrue_next = E_next;

%% 予測誤差

simResult.residual = ...
    simResult.Etrue_next - simResult.Ehat_next;

% 全26次元のRMSE
simResult.RMSE = ...
    sqrt(mean(simResult.residual(:).^2));

% 各状態ごとのRMSE
simResult.RMSE_each = ...
    sqrt(mean(simResult.residual.^2,2));

fprintf('\n===== 1-step prediction =====\n');
fprintf('Data : %d samples\n',N);
fprintf('RMSE : %.6g\n',simResult.RMSE);

fprintf('\n＜推定精度検証が完了しました＞\n');
% 


if strcmp(method, 'KL')

    saveFileName = [baseName, '_KL_err', ext];

elseif strcmp(method, 'LYKL')

    saveFileName = [baseName, '_LYKL_err', ext];
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%実験メモ追加%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% 保存先
targetPath = fullfile(nowFolder,'refult_of_KL_miya', saveFileName);

%% メモファイル名
[saveFolder, name, ~] = fileparts(targetPath);
memoFilePath = fullfile(saveFolder, [name '_memo.txt']);

%% メモファイル作成

fid = fopen(memoFilePath, 'w');

if fid == -1
    error('メモファイルを作成できませんでした。');
end

%% 作成日時を記録
today = datetime('now','Format','yyyy/MM/dd HH:mm:ss');

fprintf(fid, '========== 実験メモ ==========\n');
fprintf(fid, '作成日時 : %s\n', char(today));
fprintf(fid, 'データセット : %s\n\n', folderPath);

fprintf(fid, '=== %s 内のファイル一覧 ===\n\n', folderPath);

for i = 1:length(files)
    if ~files(i).isdir
        fprintf(fid, '%s\n', files(i).name);
    end
end

if strcmpi(method, 'KL')

    disp('KL');
    fprintf(fid, 'KLでした');


elseif strcmpi(method, 'LYKL')
    % LYKLの場合の処理
    disp('LYKL');
    fprintf(fid, 'LYKL回数 : %d\n', est.iteration);
    fprintf("実行時間: %.3f 秒\n", calT);
end

fclose(fid);

fprintf('メモファイル "%s" を保存しました。\n', memoFilePath);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%




% 保存先のフルパス
% targetPath = fullfile(nowFolder, saveFileName);
% 保存
save(targetPath, 'est', 'Data', 'simResult', 'F');

% save(targetpath,'est','Data','simResult','F')

disp('Saved to')
disp(targetPath)

%%


%% データセット内のファイルの移動 Dataの中のなんというフォルダに入るか
% parentFolderPath = 'Data';
% newFolderName = input('データセットフォルダ内のファイルを移動します．\n移動先のフォルダ名を入力してください：','s');
% newFolderPath = fullfile(parentFolderPath,newFolderName);
% mkdir(newFolderPath)
%
% sourceFolderPath = 'データセット';
% destinationFolderPath = newFolderPath;
% files = dir(fullfile(sourceFolderPath,'*.mat'));
% for i = 1:length(files)
%     filePath = fullfile(sourceFolderPath,files(i).name);
%     movefile(filePath,destinationFolderPath);
% end
%
% fprintf('\n＜ファイルの移動が完了しました＞\n')



