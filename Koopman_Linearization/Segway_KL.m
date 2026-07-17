%% Koopman Linear by Data %%
%--------------------------------------------------------------------------------
%初心者用のクープマン線形化プログラム
%コマンドウィンドウの案内に従えばできるようになっているはず．．．
%慣れてきたら，main_KoopmanLinearByDataのほうでやるといいかも(設定を自分でできる)
%--------------------------------------------------------------------------------
clc
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
%データ保存先ファイル名(逐次変更しないと，上書きされる)
% FileName = input('保存するファイル名を入力してください(※ ～.matを付ける): ', 's');
% 
% folderPath = 'datasets'; %データセットに使用するデータはデータセットフォルダにいれておく main.mの階層
% % folderPath = 'KMPCSimデータセット'
% fileList = dir(fullfile(folderPath,'*.mat')); %対象のファイルを取得
% fprintf('\n＜データセットに使用するファイル名の統一を行います＞\n')
% 
% % 読み込むデータファイル名は同じにする必要がある：学習データ
% % loading_filename_1 みたいな感じになる
% loading_filename = input('\n統一するファイル名を入力してください(※ .matは含まない):','s');
% 
% for i = 1:length(fileList)
%     oldFileName = fullfile(folderPath,fileList(i).name);
%     newFileName = fullfile(folderPath,[append(loading_filename,'_',num2str(i),'.mat')]);
%     movefile(oldFileName, newFileName); %名前の変更
% end
% 
% Data.HowmanyDataset = numel(fileList); %読み込むデータ数
% if Data.HowmanyDataset > 0
%     fprintf('\n＜ファイル名の統一が完了しました＞\n')
%     fprintf('\n読み込むファイル数：%d\n',Data.HowmanyDataset)
% else
%     error('データセットフォルダ内にファイルが存在しません') %データセットフォルダ内にファイルがない場合はエラー
% end
% 
% %データ保存用,現在のファイルパスを取得,保存先を指定
% activeFile = matlab.desktop.editor.getActive;
% nowFolder = fileparts(activeFile.Filename);
% targetpath=append(nowFolder,'\',FileName);
% 
% %% Defining Koopman Operator
% %<使用している観測量>
% F = @quaternions_all; 
% % F = @quaternions_all_old; 
% fprintf('\n選択されている観測量：%s\n',func2str(F))
% 
% % load data
% % 実験データから必要なものを抜き出す処理,↓状態,→データ番号(同一番号のデータが対応関係にある)
% % Data.X 入力前の対象の状態
% % Data.U 対象への入力
% % Data.Y 入力後の対象の状態

fprintf('\n＜データセットの読み込みを行います＞\n')

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% ============================================================
% セグウェイデータの読み込み
% ============================================================
loaded_data = load("segway_koopman_train.mat");

% ============================================================
% KLコード用のデータ構造体
% ============================================================
data = struct();

% 入力前の状態 x[k]
data.X = loaded_data.X;

% 入力後の状態 x[k+1]
data.Y = loaded_data.X_next;

% 実際にプラントへ入力された u[k]
data.U = loaded_data.U;

% ============================================================
% 各遷移に対応する時刻
% ============================================================

% 1試行あたりの遷移時刻
% Nサンプルなら、遷移はN-1個
time_one_run = loaded_data.t(1:end-1).';

% 試行数
number_of_runs = double(loaded_data.N_runs);

% 全試行分を横方向に並べる
data.T = repmat( ...
    time_one_run, ...
    1, ...
    number_of_runs ...
);

% ============================================================
% サイズ確認
% ============================================================
fprintf("data.X : %d × %d\n", ...
    size(data.X, 1), size(data.X, 2));

fprintf("data.Y : %d × %d\n", ...
    size(data.Y, 1), size(data.Y, 2));

fprintf("data.U : %d × %d\n", ...
    size(data.U, 1), size(data.U, 2));

fprintf("data.T : %d × %d\n", ...
    size(data.T, 1), size(data.T, 2));

% 全データの列数が一致することを確認
number_of_data = size(data.X, 2);

assert( ...
    size(data.Y, 2) == number_of_data, ...
    "data.Xとdata.Yの列数が一致していません。" ...
);

assert( ...
    size(data.U, 2) == number_of_data, ...
    "data.Xとdata.Uの列数が一致していません。" ...
);

assert( ...
    size(data.T, 2) == number_of_data, ...
    "data.Xとdata.Tの列数が一致していません。" ...
);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

fprintf('\n＜データセットの結合が完了しました＞\n')

flg.normalize = input('\n＜正規化を行いますか＞\n はい:1，いいえ:0：','s');
if str2double(flg.normalize) == 1 %正規化を行うか(正規化については自分で調べて！)
    Ndata = Normalization(data);
    data.X = Ndata.x;
    data.Y = Ndata.y;
    data.U = Ndata.U;
    disp('正規化が完了しました')
end

%% クォータニオンのノルムをチェック(クォータニオンのノルムは1にならなければいけないという制約がある)
% 閾値を下回った or 上回った場合注意文を提示
% attitude_norm 各時間におけるクォータニオンのノルム
if size(data.X,1)==13 %特に気にしなくていい
    thre = 0.01;
    attitude_norm = checkQuaternionNorm(Dataset.est.q',thre);
end

%% ここから始めるとき
% clear; clc;
% flg.bilinear = 0;
% flg.normalize = 0;
F = @quaternions_all; % 改造用
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
    est = KL_biLinear(data.X,data.U,data.Y,F);
else
     [~, ~, ~, ~, est] = rensyuuKLLY(data.X,data.U,data.Y,F,flg); %クープマン線形化の具体的な計算をしてる部分
     % est = KL(Data.X,Data.U,Data.Y,F,flg);
end



% est.observable = F;
fprintf('\n＜クープマン線形化が完了しました＞\n')

%% Simulation by Estimated model(構築したモデルでシミュレーション)
%推定精度検証シミュレーション
%構築したクープマンモデルがどの程度正確かを確認する部分
fprintf('\n＜推定精度検証用データに設定するファイル名を選択してください＞\n')
[fileName, filePath] = uigetfile('*.mat');
verification_data = fileName;
simResult.reference = ImportFromExpData_estimation_tutorial(verification_data); %検証用データを格納

%arming時の実験データがうまく取れていないのを強引に解消
if simResult.reference.fExp == 1
    takeoff_idx = find(simResult.reference.T,1,'first');
    simResult.reference.X = simResult.reference.X(:,takeoff_idx:end);
    simResult.reference.Y = simResult.reference.Y(:,takeoff_idx:end);
    simResult.reference.U = simResult.reference.U(:,takeoff_idx:end);
    simResult.reference.T = simResult.reference.T(takeoff_idx:end);
    simResult.reference.T = simResult.reference.T - simResult.reference.T(1);
    simResult.reference.N = simResult.reference.N - takeoff_idx;
end

simResult.Z(:,1) = F(simResult.reference.X(:,1)); %検証用データの初期値を観測量に通して次元を合わせてる
simResult.Xhat(:,1) = simResult.reference.X(:,1);
simResult.U = simResult.reference.U(:,1:end);
simResult.T = simResult.reference.T(1:end);

if flg.normalize == 1 %推定精度検証用データの正規化
    for i  = 1:12
        simResult.Z(i,1) = (simResult.Z(i,1)-Ndata.meanValue.x(i))/Ndata.stdValue.x(i); %状態の正規化
    end
    for i = 1:4
        simResult.U(i,:) = (simResult.U(i,:)-Ndata.meanValue.u(i))/Ndata.stdValue.u(i); %入力の正規化
    end
end

%方程式を用いて計算を行う部分
if flg.bilinear == 1  %　flg.bilinear == 1:双線形
    for i = 1:1:simResult.reference.N-2
        simResult.Z(:,i+1) = est.ABE'*[simResult.Z(:,i);simResult.U(:,i);reshape(kron(simResult.Z(:,i),simResult.U(:,i)),[],1)];
    end
else
    for i = 1:1:simResult.reference.N-2
        simResult.Z(:,i+1) = est.A * simResult.Z(:,i) + est.B * simResult.U(:,i); %状態方程式 z[k+1] = Az[k]+BU
    end
end
simResult.Xhat = est.C * simResult.Z; %出力方程式 x[k] = Cz[k]，次元を元の12状態に戻してる

%正規化した場合には逆変換を行う必要がある
if str2double(flg.normalize) == 1 %逆変換
    for i = 1:size(simResult.Xhat,1)
        simResult.Xhat(i,:) = (simResult.Xhat(i,:) * Ndata.stdValue.x(i)) + Ndata.meanValue.x(i);
    end
    simResult.Xhat = cat(2,simResult.reference.X(:,1),simResult.Xhat);
end

fprintf('\n＜推定精度検証が完了しました(推定したA,B,C行列を用いた状態推定)＞\n')

%% Save Estimation Result(結果保存場所)
if size(data.X,1)==13
    simResult.state.p = simResult.Xhat(1:3,:);
    simResult.state.q = simResult.Xhat(4:7,:);
    simResult.state.v = simResult.Xhat(8:10,:);
    simResult.state.w = simResult.Xhat(11:13,:);
else
    simResult.state.p = simResult.Xhat(1:3,:);
    simResult.state.q = simResult.Xhat(4:6,:);
    simResult.state.v = simResult.Xhat(7:9,:);
    simResult.state.w = simResult.Xhat(10:12,:);
end
simResult.state.N = simResult.reference.N-1;

save(targetpath,'est','data','simResult','F')
disp('Saved to')
disp(targetpath)

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


