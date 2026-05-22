%% analysis_data.m
%  DATA_ANALYZERクラスを用いた時系列データ分析スクリプト
clear; clc; close all;

%%
data = DATA_ANALYZER(mode='all', step=2); % インスタンス生成
% data.runAll();
% data.plotScatterMatrix()
data.plotVarianceBar()
data.plotHeatmap(false)
% obj.plotScatterMatrixEACH();

%%
data2 = DATA_ANALYZER(mode='divide', step=2, loggers=data.Loggers, FileNames=data.FileNames); % インスタンス生成
% %%
% data2.plotVarianceBar()
data2.plotHeatmap(false)