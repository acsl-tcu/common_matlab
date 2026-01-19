%% plotGUI単体で使用する場合 -> 実行(F5)
%  mainGUIと併用する場合 -> 下部セクション実行
clear all
cf = pwd;

if contains(mfilename('fullpath'), "mainGUI")
    cd(fileparts(mfilename('fullpath')));
else
    tmp = matlab.desktop.editor.getActive;
    cd(fileparts(tmp.Filename));
end

[~, tmp] = regexp(genpath('.'), '\.\\\.git.*?;', 'match', 'split');
cellfun(@(xx) addpath(xx), tmp, 'UniformOutput', false);
close all hidden; clear; clc;
userpath('clear');

%%  mainGUIと併用する場合 -> 本セクション実行(Ctrl+Enter)
clearvars -except app % mainGUIのapp(SimExpハンドル) 削除を回避

plt = PLOTTER();