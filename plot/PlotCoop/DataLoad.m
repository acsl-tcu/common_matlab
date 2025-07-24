%複数機牽引実験データをGUIで保存した場合にDataPlotを使うために必要なプログラム
%DataPlotはセル配列にしないと回らないのでこのプログラムで構造体配列をセル配列に変換する

clc;clear;

% load("Data\multi_drone_12_Log.mat"); 例：ファイル名部分はData\~~.matとする
load("ファイル名"); %pc1のデータ（1機目、2機目）
log1=log;

% load("Data\multi_drne_34_Log.mat");　例：ファイル名部分はData\~~.matとする
load("ファイル名"); %pc2のデータ（3機目、4機目）
log2=log;

%pc1のデータをセル配列に変換
for i = 1:length(log1.Data.agent)
            logger1{i,1} = simplifyLogger(log1,i );
end

% pc1のデータをセル配列に変換
for i = 1:length(log2.Data.agent)
            logger2{i,1} = simplifyLogger(log2,i );
end

%この後にDataplotを実行することでグラフ出力できる