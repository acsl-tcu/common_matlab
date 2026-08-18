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
settings.methods     = ["tfestimate", "spa", "ssest"];   % 比較したい手法
settings.position    = ["x","y"];        % 求めたい入出力の組（複数指定可）
% settings.position    = ["x","y"];
% settings.position    = ["x","Pitch"];
% settings.position    = ["Pitch","Roll"];
% settings.position    = ["z","x","y","yaw","Pitch","Roll"];
settings.ssest_order = 6;            % ssest使用時のモデル次数

DT = 0.025;   % サンプリング周期 [s]
Fs = 1/DT;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% --- position 名 -> (入力ch, 出力データ種別, 出力ch) の対応表 ---
% "z"     : throttle -> pL(z)
% "x"     : pitch    -> pL(x)
% "y"     : roll     -> pL(y)
% "yaw"   : yaw      -> q(yaw)
% "Pitch" : pitch    -> q(pitch)   （トルク -> 角度）
% "Roll"  : roll     -> q(roll)    （トルク -> 角度）
% --- position 名 -> (入力ch, 出力データ種別, 出力ch) の対応表 ---
posMap = containers.Map( ...
    {'z', 'x', 'y', 'yaw', 'Pitch', 'Roll'}, ...
    { {1, "pL", 3}, {3, "pL", 1}, {2, "pL", 2}, {4, "q", 3}, {3, "q", 2}, {2, "q", 1} } );

% --- 入出力データの取得（phase指定） ---
u      = logger.data(1, "controller.result.tmp", "", "phase", settings.phase);
pL_all = logger.data(1, "estimator.result.state.pL", "e", "phase", settings.phase);
q_all  = logger.data(1, "estimator.result.state.q",  "e", "phase", settings.phase);

% --- position ごとに、複数手法を重ねたBode線図を作成 ---
for p = settings.position
    p_char = char(p);

    if ~isKey(posMap, p_char)
        warning('未対応の position: %s（スキップします）', p_char);
        continue
    end
    entry = posMap(p_char);
    ci = entry{1};
    out_var = entry{2};
    co = entry{3};

    if out_var == "pL"
        y_all = pL_all;
    else
        y_all = q_all;
    end

    N = min(size(u,1), size(y_all,1));
    u_shift = u(1:N-1, ci);
    y_shift = y_all(2:N, co);

    figure('Color','w', 'Position', [100 100 900 600]);
    t = tiledlayout(2,1, 'TileSpacing','compact', 'Padding','compact');
    title(t, sprintf('%s (in ch=%d \\rightarrow %s ch=%d), u(n) \\rightarrow y(n+1), phase=%s', ...
        p_char, ci, out_var, co, settings.phase), 'FontSize', settings.fontsize+2);

    ...

    ax_gain  = nexttile; hold(ax_gain, 'on'); grid(ax_gain, 'on'); box(ax_gain, 'on');
    ax_phase = nexttile; hold(ax_phase, 'on'); grid(ax_phase, 'on'); box(ax_phase, 'on');

    for m = settings.methods
        switch m
            case "tfestimate"
                [h, f] = tfestimate(u_shift, y_shift, [], [], [], Fs);
                omega = 2*pi*f;
                mag_db = 20*log10(abs(h));
                ph_deg = angle(h)*180/pi;

            case "etfe"
                data_id = iddata(y_shift, u_shift, DT);
                g = etfe(data_id);
                [mag, ph, w] = bode(g);
                omega = squeeze(w);
                mag_db = 20*log10(squeeze(mag));
                ph_deg = squeeze(ph);

            case "spa"
                data_id = iddata(y_shift, u_shift, DT);
                g = spa(data_id);
                [mag, ph, w] = bode(g);
                omega = squeeze(w);
                mag_db = 20*log10(squeeze(mag));
                ph_deg = squeeze(ph);

            case "ssest"
                data_id = iddata(y_shift, u_shift, DT);
                opt = ssestOptions('EnforceStability', true);
                sys = ssest(data_id, settings.ssest_order, opt);
                [mag, ph, w] = bode(sys, {0.1, 200});
                omega = squeeze(w);
                mag_db = 20*log10(squeeze(mag));
                ph_deg = squeeze(ph);

            case "tfest"
                data_id = iddata(y_shift, u_shift, DT);
                sys = tfest(data_id, 2, 2);
                [mag, ph, w] = bode(sys, {0.1, 200});
                omega = squeeze(w);
                mag_db = 20*log10(squeeze(mag));
                ph_deg = squeeze(ph);

            otherwise
                warning('未対応の手法: %s（スキップします）', m);
                continue
        end

        % 各線に DisplayName を直接付与し、legend('show') で自動収集する
        semilogx(ax_gain, omega, mag_db, 'LineWidth', settings.linewidth, 'DisplayName', m);
        semilogx(ax_phase, omega, ph_deg, 'LineWidth', settings.linewidth, 'DisplayName', m);
    end

    ylabel(ax_gain, 'Gain [dB]', 'FontSize', settings.fontsize);
    set(ax_gain, 'FontSize', settings.fontsize);
    legend(ax_gain, 'show', 'FontSize', settings.fontsize-4, 'Location', 'best');

    ylabel(ax_phase, 'Phase [deg]', 'FontSize', settings.fontsize);
    xlabel(ax_phase, '\omega [rad/s]', 'FontSize', settings.fontsize);
    set(ax_phase, 'FontSize', settings.fontsize);
end