%%　説明
% Exp / Simデータをプロットすることができるファイル
% 最初は全てのセクションを実行する．
%settingを変更するだけでmatファイルはそのままで図のみを変更することができる
%% 初期化&パスの設定
% ※このセクションは最初に1回だけ実行してください（loggerが消えます）
clear all
cf = pwd;

if contains(mfilename('fullpath'), "mainGUI")
    cd(fileparts(mfilename('fullpath')));
else
    tmp = matlab.desktop.editor.getActive;
    cd(fileparts(erase(tmp.Filename, "plot\Frequency_analysis.m")));
end

[~, tmp] = regexp(genpath('.'), '\.\\\.git.*?;', 'match', 'split');
cellfun(@(xx) addpath(xx), tmp, 'UniformOutput', false);
close all hidden; clear; clc;
userpath('clear');

%% データの読み込み
% ※このセクションも最初に1回だけ実行してください（loggerを作成）
fprintf('MATファイルを選択してください:')
[filename, pathname] = uigetfile('*.mat', 'MATファイルを選択してください');
fprintf(filename);
fullpath = fullfile(pathname, filename);
logger = LOGGER(fullpath);

%% システム同定：設定・データ取得・プロット（settingsを書き換えてこのセクションだけ再実行可）
close all
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% settings %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

settings.phase       = "f";          % 抽出するフライトフェーズ
settings.fontsize    = 16;           % フォントサイズ
settings.linewidth   = 1.5;          % 線の太さ
settings.methods     = ["tfestimate", "spa", "etfe","ssest"];   % 比較したい手法
settings.position    = ["x","y"];        % 求めたい入出力の組（複数指定可）
% settings.position    = ["x","y"];
% settings.position    = ["x","Pitch"];
% settings.position    = ["Pitch","Roll"];
% settings.position    = ["z","x","y","yaw","Pitch","Roll"];
settings.ssest.order = 6;            % ssest使用時のモデル次数
settings.fig.methodsPerFig = 3;      % 1つの図に表示する手法の最大数

sampling.dt = 0.025;   % サンプリング周期 [s]
sampling.Fs = 1/sampling.dt;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% --- position 名 -> (入力ch, 出力データ種別, 出力ch) の対応表 ---
% "z"     : throttle -> pL(z)
% "x"     : pitch    -> pL(x)
% "y"     : roll     -> pL(y)
% "yaw"   : yaw      -> q(yaw)
% "Pitch" : pitch    -> q(pitch)   （トルク -> 角度）
% "Roll"  : roll     -> q(roll)    （トルク -> 角度）
posMap = containers.Map( ...
    {'z', 'x', 'y', 'yaw', 'Pitch', 'Roll'}, ...
    { {1, "pL", 3}, {3, "pL", 1}, {2, "pL", 2}, {4, "q", 3}, {3, "q", 2}, {2, "q", 1} } );

% --- 入出力データの取得（phase指定） ---
data.u    = logger.data(1, "controller.result.tmp", "", "phase", settings.phase);
data.pL   = logger.data(1, "estimator.result.state.pL", "e", "phase", settings.phase);
data.q    = logger.data(1, "estimator.result.state.q",  "e", "phase", settings.phase);

% --- position ごとに、複数手法を重ねたBode線図を作成 ---
fig.nMethods    = length(settings.methods);
fig.nGroups     = ceil(fig.nMethods / settings.fig.methodsPerFig);

for p = settings.position
    axis.name = char(p);

    if ~isKey(posMap, axis.name)
        warning('未対応の position: %s（スキップします）', axis.name);
        continue
    end
    entry = posMap(axis.name);
    axis.ci = entry{1};
    axis.outVar = entry{2};
    axis.co = entry{3};

    if axis.outVar == "pL"
        yAll = data.pL;
    else
        yAll = data.q;
    end

    shift.N = min(size(data.u,1), size(yAll,1));
    shift.u = data.u(1:shift.N-1, axis.ci);
    shift.y = yAll(2:shift.N, axis.co);

    % --- methodsをグループごとに分割して、グループごとに別figureにする ---
    for g = 1:fig.nGroups
        fig.idxStart = (g-1)*settings.fig.methodsPerFig + 1;
        fig.idxEnd   = min(g*settings.fig.methodsPerFig, fig.nMethods);
        fig.methodsGroup = settings.methods(fig.idxStart:fig.idxEnd);

        figure('Color','w', 'Position', [100 100 900 600]);
        t = tiledlayout(2,1, 'TileSpacing','compact', 'Padding','compact');
        title(t, sprintf('%s (in ch=%d \\rightarrow %s ch=%d), u(n) \\rightarrow y(n+1), phase=%s [%d/%d]', ...
            axis.name, axis.ci, axis.outVar, axis.co, settings.phase, g, fig.nGroups), 'FontSize', settings.fontsize+2);

        ax.gain  = nexttile; hold(ax.gain, 'on'); grid(ax.gain, 'on'); box(ax.gain, 'on');
        ax.phase = nexttile; hold(ax.phase, 'on'); grid(ax.phase, 'on'); box(ax.phase, 'on');

        for m = fig.methodsGroup
            result = estimate_frequency_response(m, shift.u, shift.y, sampling.dt, sampling.Fs, settings.ssest.order);
            if ~result.ok
                warning('未対応の手法: %s（スキップします）', m);
                continue
            end

            semilogx(ax.gain, result.omega, result.mag_db, 'LineWidth', settings.linewidth, 'DisplayName', m);
            semilogx(ax.phase, result.omega, result.ph_deg, 'LineWidth', settings.linewidth, 'DisplayName', m);
        end

        ylabel(ax.gain, 'Gain [dB]', 'FontSize', settings.fontsize);
        set(ax.gain, 'FontSize', settings.fontsize);
        legend(ax.gain, 'show', 'FontSize', settings.fontsize-4, 'Location', 'best');

        ylabel(ax.phase, 'Phase [deg]', 'FontSize', settings.fontsize);
        xlabel(ax.phase, '\omega [rad/s]', 'FontSize', settings.fontsize);
        set(ax.phase, 'FontSize', settings.fontsize);
    end
end

%% ===== Local function：手法ごとの周波数応答推定 =====
function result = estimate_frequency_response(method, uShift, yShift, dt, Fs, ssestOrder)
% method に応じて周波数応答（ゲイン・位相）を計算する
% result.ok = false のとき、未対応の手法として呼び出し側でスキップされる

result.ok = true;
result.omega = [];
result.mag_db = [];
result.ph_deg = [];

switch method
    case "tfestimate"
        [h, f] = tfestimate(uShift, yShift, [], [], [], Fs);
        result.omega = 2*pi*f;
        result.mag_db = 20*log10(abs(h));
        result.ph_deg = angle(h)*180/pi;

    case "etfe"
        dataId = iddata(yShift, uShift, dt);
        gSys = etfe(dataId);
        [mag, ph, w] = bode(gSys);
        result.omega = squeeze(w);
        result.mag_db = 20*log10(squeeze(mag));
        result.ph_deg = squeeze(ph);

    case "spa"
        dataId = iddata(yShift, uShift, dt);
        gSys = spa(dataId);
        [mag, ph, w] = bode(gSys);
        result.omega = squeeze(w);
        result.mag_db = 20*log10(squeeze(mag));
        result.ph_deg = squeeze(ph);

    case "ssest"
        dataId = iddata(yShift, uShift, dt);
        opt = ssestOptions('EnforceStability', true);
        sys = ssest(dataId, ssestOrder, opt);
        [mag, ph, w] = bode(sys, {0.1, 200});
        result.omega = squeeze(w);
        result.mag_db = 20*log10(squeeze(mag));
        result.ph_deg = squeeze(ph);

    case "tfest"
        dataId = iddata(yShift, uShift, dt);
        sys = tfest(dataId, 2, 2);
        [mag, ph, w] = bode(sys, {0.1, 200});
        result.omega = squeeze(w);
        result.mag_db = 20*log10(squeeze(mag));
        result.ph_deg = squeeze(ph);

    otherwise
        result.ok = false;
end

end