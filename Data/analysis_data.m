%% analysis_data.m
%  DATA_ANALYZERクラスを用いた時系列データ分析スクリプト
clear; clc; close all;

data = DATA_ANALYZER(mode='all'); % インスタンス生成

% analyzer.runAll();

%%
% data.plotScatterMatrix()
% data.plotVarianceBar()
data.plotHeatmap(false)