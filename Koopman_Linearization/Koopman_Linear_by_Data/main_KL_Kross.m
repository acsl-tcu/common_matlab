%クロスバリデーションで精度検証
clc
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
FileName = input('保存するファイル名を入力してください(※ ～.matを付ける): ', 's');

folderPath = 'データセット'; %データセットに使用するデータはデータセットフォルダにいれておく main.mの階層
fileList = dir(fullfile(folderPath,'*.mat')); %対象のファイルを取得
fprintf('\n＜データセットに使用するファイル名の統一を行います＞\n')

% 読み込むデータファイル名は同じにする必要がある：学習データ
% loading_filename_1 みたいな感じになる
loading_filename = input('\n統一するファイル名を入力してください(※ .matは含まない):','s');

for i = 1:length(fileList)
    oldFileName = fullfile(folderPath,fileList(i).name);
    newFileName = fullfile(folderPath,[append(loading_filename,'_',num2str(i),'.mat')]);
    movefile(oldFileName, newFileName); %名前の変更
end

Data.HowmanyDataset = numel(fileList); %読み込むデータ数
if Data.HowmanyDataset > 0
    fprintf('\n＜ファイル名の統一が完了しました＞\n')
    fprintf('\n読み込むファイル数：%d\n',Data.HowmanyDataset)
else
    error('データセットフォルダ内にファイルが存在しません') %データセットフォルダ内にファイルがない場合はエラー
end

%データ保存用,現在のファイルパスを取得,保存先を指定
activeFile = matlab.desktop.editor.getActive;
nowFolder = fileparts(activeFile.Filename);
targetpath=append(nowFolder,'\',FileName);

%% Defining Koopman Operator
%<使用している観測量>
F = @quaternions_all; 
fprintf('\n選択されている観測量：%s\n',func2str(F))

% load data
% 実験データから必要なものを抜き出す処理,↓状態,→データ番号(同一番号のデータが対応関係にある)
% Data.X 入力前の対象の状態
% Data.U 対象への入力
% Data.Y 入力後の対象の状態
fprintf("\nデータの読み込み開始")
for i = 1:Data.HowmanyDataset
    if contains(loading_filename,'.mat')
        Dataset = ImportFromExpData_tutorial(loading_filename); %ImportFromExpData_tutorial:データセットをくっつけるための関数
    else
        if i == 1 %66 ~ 78はコマンドウィンドウから入力するのに必要(クープマン線形化には関係ない)
            setting = 1;
            Dataset = ImportFromExpData_tutorial(append(loading_filename,'_',num2str(i),'.mat'),setting);
            datarange = Dataset.datarange;
            range = Dataset.range;
            IDX = Dataset.IDX;
            phase2 = Dataset.phase2;
            % vz_z = Dataset.vz_z;
            vz_z = Dataset.vxyz;
            fprintf('\n')
        else
            setting = 0;
            Dataset = ImportFromExpData_tutorial(append(loading_filename,'_',num2str(i),'.mat'),setting,datarange,range,IDX,phase2,vz_z);
        end
    end
    if i==1
        Data.all_X{1} = [Dataset.X];
        Data.all_U{1} = [Dataset.U];
        Data.all_Y{1} = [Dataset.Y];        
    else
        Data.all_X{i} = [Dataset.X];
        Data.all_U{i} = [Dataset.U];
        Data.all_Y{i} = [Dataset.Y];
    end
    disp(append('loading data number: ',num2str(i),', now data:',num2str(Dataset.N),', データ番号: ',num2str(size(Data.all_X,2))))
end


flg.normalize = input('\n＜正規化を行いますか＞\n はい:1，いいえ:0：','s');
k=length(Data.all_X);%データの個数と同じ
for j = 1:k
fprintf('\n＜%d回目のデータセットの結合を行います＞\n',j)
Data.X=[];
Data.U=[];
Data.Y=[];
for i = 1:k
    if i==j
        Data.test_X = [Data.all_X{i}];
        Data.test_U = [Data.all_U{i}];
        Data.test_Y = [Data.all_Y{i}];        
    else
        Data.X = [Data.X,Data.all_X{i}];
        Data.U = [Data.U,Data.all_U{i}];
        Data.Y = [Data.Y,Data.all_Y{i}];
    end
    disp(append('loading data number: ',num2str(i),', now data:',num2str(Dataset.N),', all data: ',num2str(size(Data.X,2))))
end
fprintf('\n＜%d回目のデータセットの結合が完了しました＞\n',j)

if str2double(flg.normalize) == 1 %正規化を行うか(正規化については自分で調べて！)
    Ndata = Normalization(Data);
    Data.X = Ndata.x;
    Data.Y = Ndata.y;
    Data.U = Ndata.u;
    disp('正規化が完了しました')
end
%% クォータニオンのノルムをチェック(クォータニオンのノルムは1にならなければいけないという制約がある)
% 閾値を下回った or 上回った場合注意文を提示
% attitude_norm 各時間におけるクォータニオンのノルム
if size(Data.X,1)==13 %特に気にしなくていい
    thre = 0.01;
    attitude_norm = checkQuaternionNorm(Dataset.est.q',thre);
end
%% Koopman linearization
fprintf('\n＜%d回目のクープマン線形化を実行＞\n',j)
if flg.bilinear == 1
    est{j} = KL_biLinear(Data.X,Data.U,Data.Y,F);
else
    est{j} = KL(Data.X,Data.U,Data.Y,F,flg); %クープマン線形化の具体的な計算をしてる部分
end
fprintf('\n＜%d回目のクープマン線形化が完了しました＞\n',j)

%% Simulation by Estimated model(構築したモデルでシミュレーション)
fprintf('\n＜推定精度検証用データに設定するファイル名を選択してください＞\n')
simResult.reference.X = Data.test_X;
simResult.reference.U = Data.test_U;
simResult.reference.Y = Data.test_Y;
if str2double(flg.normalize) == 1 %推定精度検証用データの正規化
    for i  = 1:size(simResult.reference.X,1)
        simResult.reference.X(i,1) = (simResult.reference.X(i,1)-Ndata.meanValue.x(i))/Ndata.stdValue.x(i); %状態の正規化
    end
    for i = 1:size(simResult.reference.U,1)
        simResult.reference.U(i,:) = (simResult.reference.U(i,:)-Ndata.meanValue.u(i))/Ndata.stdValue.u(i); %入力の正規化
    end
end

simResult.Z(:,1) = F(simResult.reference.X(:,1)); %検証用データの初期値を観測量に通して次元を合わせてる
simResult.Xhat(:,1) = simResult.reference.X(:,1);
simResult.U = simResult.reference.U(:,1:end);



%方程式を用いて計算を行う部分
if flg.bilinear == 1  %　flg.bilinear == 1:双線形
    for i = 1:length(simResult.reference.X)-2
        simResult.Z(:,i+1) = est{j}.ABE'*[simResult.Z(:,i);simResult.U(:,i);reshape(kron(simResult.Z(:,i),simResult.U(:,i)),[],1)];
    end
else
    for i = 1:length(simResult.reference.X)-1
        simResult.Z(:,i+1) = est{j}.A * simResult.Z(:,i) + est{j}.B * simResult.U(:,i); %状態方程式 z[k+1] = Az[k]+BU
    end
end
simResult.Xhat = est{j}.C * simResult.Z; %出力方程式 x[k] = Cz[k]，次元を元の12状態に戻してる

%正規化した場合には逆変換を行う必要がある
if str2double(flg.normalize) == 1 %逆変換
    for i = 1:size(simResult.Xhat,1)
        simResult.Xhat(i,:) = (simResult.Xhat(i,:) * Ndata.stdValue.x(i)) + Ndata.meanValue.x(i);
    end
end

rmse_each_state{j} = rmse(simResult.Xhat,simResult.reference.X);  % 状態ごとのRMSE（縦方向）
fprintf('\n＜%d回目の推定精度検証が完了しました(推定したA,B,C行列を用いた状態推定)＞\n',j)
end
%% Save Estimation Result(結果保存場所)
save(targetpath,'est','Data','simResult','F')
disp('Saved to')
disp(targetpath)