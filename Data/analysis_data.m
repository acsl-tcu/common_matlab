%% analysis_data.m
%  DATA_ANALYZERクラスを用いた時系列データ分析スクリプト
clear; clc; close all;

%%
data_all = DATA_ANALYZER(mode='all', step=10); % インスタンス生成
data_all.runAll();
% data_all.plotScatterMatrix()
% data_all.plotVarianceBar()
% data_all.plotHeatmap(false)
% data_all.plotScatterMatrixEACH();
% data_all.plotLagCorr()

%%
data_divide = DATA_ANALYZER(mode='divide',...
                step=data_all.step, loggers=data_all.Loggers, FileNames=data_all.FileNames); % data_allのloggersを引継ぎ
data_divide.runAll();
% data_divide.plotVarianceBar()
% data_divide.plotHeatmap(false)
% data_divide.plotLagCorr()