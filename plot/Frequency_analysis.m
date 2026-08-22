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
settings.position    = ["x"];        % 求めたい入出力の組（複数指定可）
% settings.position    = ["x","y"];
% settings.position    = ["x","Pitch"];
% settings.position    = ["Pitch","Roll"];
% settings.position    = ["z","x","y","yaw","Pitch","Roll"];
settings.ssest.order = 6;            % ssest使用時のモデル次数
settings.fig.methodsPerFig = 3;      % 1つの図に表示する手法の最大数

%使用可能なsettig.method 一覧
% ノンパラメトリック（周波数領域）	tfestimate, etfe, spa, spafdr
% 相関・インパルス応答系	cra, impulseest
% プロセスモデル	procest
% 入出力多項式モデル	arx, armax, bj, iv4, ivx, oe, polyest, pem
% 状態空間モデル	ssest, ssregest, n4sid
% 伝達関数モデル	tfest

sampling.dt = 0.025;   % サンプリング周期 [s]
sampling.Fs = 1/sampling.dt;

% Phaseの個数を確認
% ログに含まれる、指定したフェーズ（settings.phaseで指定）が何点あるか確認する

% 事前に settings.phase を指定しておく（システム同定セクションのsettingsと合わせる）
checkPhase = settings.phase;   % 確認したいフェーズ
phaseAll = logger.Data.phase(1:logger.k);
% checkPhase の文字コードに変換して検索
phaseCode = double(char(checkPhase));
idx = find(phaseAll == phaseCode);
count = length(idx);
fprintf('===== Phase "%s" の内訳 =====\n', checkPhase);
fprintf(' 該当点数: %d 点\n', count);
if count > 1
    diffIdx = diff(idx);
    breakPoints = find(diffIdx > 1);
    nSegments = length(breakPoints) + 1;
    segStarts = [1; breakPoints + 1];
    segEnds   = [breakPoints; length(idx)];
    segLengths = segEnds - segStarts + 1;
    fprintf(' 連続区間数: %d\n', nSegments);
    for s = 1:nSegments
        fprintf('   区間%d: %d 点（インデックス %d ~ %d）\n', ...
            s, segLengths(s), idx(segStarts(s)), idx(segEnds(s)));
    end
    [~, longestSeg] = max(segLengths);
    fprintf(' -> 最長区間: 区間%d（%d 点）\n', longestSeg, segLengths(longestSeg));
else
    fprintf(' 連続区間数: %d\n', count);
end
fprintf('=========================\n\n');

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

    shift.N = min(size(data.u-1,1), size(yAll,1));
    shift.u = data.u(2:shift.N-1, axis.ci); %startのから回しのfを除外
    shift.y = yAll(3:shift.N, axis.co); %startのから回しのfを除外しf phse の入力の次時刻から使用

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
            result = estimate_frequency_response(m, shift.u, shift.y, sampling.dt, sampling.Fs, settings.ssest.order,count);
            if ~result.ok
                warning('未対応の手法: %s（スキップします）', m);
                continue
            end

            semilogx(ax.gain, result.omega, result.mag.db, 'LineWidth', settings.linewidth, 'DisplayName', m);
            semilogx(ax.phase, result.omega, result.ph.deg, 'LineWidth', settings.linewidth, 'DisplayName', m);
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
switch method
    case "tfestimate" %
        [h, f] = tfestimate(uShift, yShift, [], [], [], Fs); % uShiftとyShiftから伝達関数推定値hと周波数ベクトルfを計算
        result.omega = 2*pi*f; % 周波数を[rad/s]に変換してresultに格納
        result.mag.db = 20*log10(abs(h)); % 振幅hの絶対値を[dB]に変換して格納
        result.ph.deg = angle(h)*180/pi; % hの位相角を[deg]に変換して格納


        % 周波数応答のモデル https://jp.mathworks.com/help/ident/frequency-response-models.html
    case "etfe" %
        dataId = iddata(yShift, uShift, dt); % u,yとサンプリング周期dtからiddataオブジェクトを作成
        gSys = etfe(dataId);  % etfeで周波数応答モデルgSysを推定
        [mag, ph, w] = bode(gSys); % gSysからゲインmag・位相ph・周波数wを計算
        result.omega = squeeze(w);
        result.mag.db = 20*log10(squeeze(mag));
        result.ph.deg = squeeze(ph);

    case "spa" %
        dataId = iddata(yShift, uShift, dt); % u,yとdtからiddataオブジェクトを作成
        gSys = spa(dataId,1451);% spaで周波数応答モデルgSysを推定
        [mag, ph, w] = bode(gSys); % gSysからゲイン・位相・周波数を計算
        result.omega = squeeze(w);
        result.mag.db = 20*log10(squeeze(mag));
        result.ph.deg = squeeze(ph);
        % 手動
        % % FFT によるクロススペクトル推定
        % % 以下、上のspa()とは別に、u,yをFFTして
        % % クロススペクトル/自己スペクトルの比から周波数応答を計算する処理
        % % （countが未定義のためこのままでは実行時エラーになる／未使用のデッドコード）
        % U = fft(uShift, count);
        % Y = fft(yShift, count);
        % Gxy = Y .* conj(U) / length(uShift);  % クロススペクトル
        % Gxx = U .* conj(U) / length(uShift);  % 入力自己スペクトル
        % % 周波数応答（複素数）
        % H = Gxy ./ Gxx;
        % % 周波数軸（0〜Fs/2）
        % f = (0:(count/2)) * (Fs / count);
        % % 出力構造体に格納
        % % ※直前のspa()の結果を上書きしてしまう
        % result.omega   = 2*pi*f;                       % rad/s
        % result.mag.db  = 20*log10(abs(H(1:length(f)))); % dB
        % result.ph.deg  = rad2deg(angle(H(1:length(f))));% degrees

    case "spafdr" %
        %y(t)=G(q)u(t)+v(t)
        dataId = iddata(yShift, uShift, dt);% u,yとdtからiddataオブジェクトを作成
        gSys = spafdr(dataId);% spafdrで周波数応答モデルgSysを推定
        [mag, ph, w] = bode(gSys);% gSysからゲイン・位相・周波数を計算
        result.omega = squeeze(w);
        result.mag.db = 20*log10(squeeze(mag));
        result.ph.deg = squeeze(ph);

    case "idfrd" %
        % 伝達関数の構造を使用者が知っていて使える
        %y(t)=G(q)u(t)+H(q)e(t)

        % 相関モデル https://jp.mathworks.com/help/ident/correlation-models.html
    case "cra" %  https://jp.mathworks.com/help/ident/ref/cra.html
        dataId = iddata(yShift, uShift, dt);% u,yとdtからiddataオブジェクトを作成
        ir = cra(dataId, [], [], 'none'); %craで相互相関からインパルス応答係数irを推定 第4引数'none'でプロット表示を抑制（irのみ取得）
        % irは時間領域の波形データ（数式モデルではない）なので、 FFTで周波数領域に変換してゲイン・位相を求める     
        N = length(ir);
        H = fft(ir);
        f = (0:floor(N/2)) * (Fs / N);       
        result.omega = 2*pi*f;
        result.mag.db = 20*log10(abs(H(1:length(f))));      
        result.ph.deg = angle(H(1:length(f)))*180/pi;

    case"impulseest" % https://jp.mathworks.com/help/ident/ref/impulseest.html
        dataId = iddata(yShift, uShift, dt); % iddataオブジェクトを作成
        sys = impulseest(dataId);% impulseestでインパルス応答モデルsysを推定
        [mag, ph, w] = bode(sys, {0.1, 200}); % 周波数範囲{0.1, 200}[rad/s]を指定してゲイン・位相・周波数を計算
        result.omega = squeeze(w);
        result.mag.db = 20*log10(squeeze(mag));
        result.ph.deg = squeeze(ph);

    case"era" % https://jp.mathworks.com/help/ident/ref/era.html
        % 入力がインパルス応答データが必要になりiddataを活用して伝達関数を求めることができない
        % インパルス応答のデータが必要


        % プロセモデル https://jp.mathworks.com/help/ident/process-models.html
    case"procest" % https://jp.mathworks.com/help/ident/ref/procest.html
        dataId = iddata(yShift, uShift, dt);  % iddataオブジェクトを作成
        sys = procest(dataId, 'P1D');% procestでモデル構造'P1D'を指定してsysを推定
        [mag, ph, w] = bode(sys, {0.1, 200});% ゲイン・位相・周波数を計算
        result.omega = squeeze(w);
        result.mag.db = 20*log10(squeeze(mag));
        result.ph.deg = squeeze(ph);

        % 入出力多項式モデル https://jp.mathworks.com/help/ident/input-output-polynomial-models.html
    case "arx" % https://jp.mathworks.com/help/ident/ref/arx.html

        dataId = iddata(yShift, uShift, dt);% iddataオブジェクトを作成
        sys = arx(dataId, [2 2 1]);% arxに次数[na nb nk]=[2 2 1]を指定してsysを推定
        [mag, ph, w] = bode(sys, {0.1, 200});      % ゲイン・位相・周波数を計算
        result.omega = squeeze(w);
        result.mag.db = 20*log10(squeeze(mag));
        result.ph.deg = squeeze(ph);

    case "armax" % https://jp.mathworks.com/help/ident/ref/armax.html
        dataId = iddata(yShift, uShift, dt); % iddataオブジェクトを作成
        sys = armax(dataId, [2 2 2 1]); % armaxに次数[na nb nc nk]=[2 2 2 1]を指定してsysを推定
        [mag, ph, w] = bode(sys, {0.1, 200});
        result.omega = squeeze(w);
        result.mag.db = 20*log10(squeeze(mag));
        result.ph.deg = squeeze(ph);

    case "bj" % https://jp.mathworks.com/help/ident/ref/bj.html
        dataId = iddata(yShift, uShift, dt); % iddataオブジェクトを作成
        sys = bj(dataId, [2 2 2 2 1]);% bjに次数[nb nc nd nf nk]=[2 2 2 2 1]を指定してsysを推定
        [mag, ph, w] = bode(sys, {0.1, 200});
        result.omega = squeeze(w);
        result.mag.db = 20*log10(squeeze(mag));
        result.ph.deg = squeeze(ph);

    case "iv4" % https://jp.mathworks.com/help/ident/ref/iv4.html
        dataId = iddata(yShift, uShift, dt);% iddataオブジェクトを作成
        sys = iv4(dataId, [2 2 1]);% iv4に次数[na nb nk]=[2 2 1]を指定してsysを推定
        [mag, ph, w] = bode(sys, {0.1, 200});
        result.omega = squeeze(w);
        result.mag.db = 20*log10(squeeze(mag));
        result.ph.deg = squeeze(ph);

    case "ivx" % https://jp.mathworks.com/help/ident/ref/ivx.html
        dataId = iddata(yShift, uShift, dt);% iddataオブジェクトを作成
        gSys = ivx(dataId);% ivxで（次数指定なしで）モデルgSysを推定
        [mag, ph, w] = bode(gSys);
        result.omega = squeeze(w);
        result.mag.db = 20*log10(squeeze(mag));
        result.ph.deg = squeeze(ph);

    case "oe" % https://jp.mathworks.com/help/ident/ref/oe.html
        dataId = iddata(yShift, uShift, dt);% iddataオブジェクトを作成
        sys = oe(dataId, [2 2 1]);% oeに次数[nb nf nk]=[2 2 1]を指定してsysを推定
        [mag, ph, w] = bode(sys, {0.1, 200});
        result.omega = squeeze(w);
        result.mag.db = 20*log10(squeeze(mag));
        result.ph.deg = squeeze(ph);

    case "polyest" % https://jp.mathworks.com/help/ident/ref/polyest.html
        dataId = iddata(yShift, uShift, dt);% iddataオブジェクトを作成
        sys = polyest(dataId, [2 2 2 2 2 1]);% polyestに次数[na nb nc nd nf nk]=[2 2 2 2 2 1]を指定してsysを推定
        [mag, ph, w] = bode(sys, {0.1, 200});
        result.omega = squeeze(w);
        result.mag.db = 20*log10(squeeze(mag));
        result.ph.deg = squeeze(ph);

    case "pem" % https://jp.mathworks.com/help/ident/ref/pem.html
        dataId = iddata(yShift, uShift, dt);% iddataオブジェクトを作成
        sys = pem(dataId, ssestOrder);% pemに次数ssestOrder（関数引数で渡されたモデル次数）を指定してsysを推定
        [mag, ph, w] = bode(sys, {0.1, 200});
        result.omega = squeeze(w);
        result.mag.db = 20*log10(squeeze(mag));
        result.ph.deg = squeeze(ph);

        %状態空間モデル https://jp.mathworks.com/help/ident/state-space-models.html
    case "idss" % https://jp.mathworks.com/help/ident/ref/idss.html
        % A,B,C,D行列をユーザーが数値で指定してモデルオブジェクトを作るので使用不可

    case "ssest" % https://jp.mathworks.com/help/ident/ref/ssest.html
        dataId = iddata(yShift, uShift, dt);% iddataオブジェクトを作成
        opt = ssestOptions('EnforceStability', true);% ssestOptionsで安定性を強制するオプションoptを作成
        sys = ssest(dataId, ssestOrder, opt);% ssestOrderとoptを指定してssestでsysを推定
        [mag, ph, w] = bode(sys, {0.1, 200});
        result.omega = squeeze(w);
        result.mag.db = 20*log10(squeeze(mag));
        result.ph.deg = squeeze(ph);

    case "ssregest" % https://jp.mathworks.com/help/ident/ref/ssregest.html
        dataId = iddata(yShift, uShift, dt); % iddataオブジェクトを作成
        sys = ssregest(dataId);% ssregest（次数指定なし）でsysを推定
        [mag, ph, w] = bode(sys, {0.1, 200});
        result.omega = squeeze(w);
        result.mag.db = 20*log10(squeeze(mag));
        result.ph.deg = squeeze(ph);

    case "n4sid" % https://jp.mathworks.com/help/ident/ref/n4sid.html
        dataId = iddata(yShift, uShift, dt);% iddataオブジェクトを作成
        sys = n4sid(dataId, 'best');% n4sidに'best'（次数自動選択）を指定してsysを推定
        [mag, ph, w] = bode(sys, {0.1, 200});
        result.omega = squeeze(w);
        result.mag.db = 20*log10(squeeze(mag));
        result.ph.deg = squeeze(ph);

        % case "era" % https://jp.mathworks.com/help/ident/ref/era.html
        % case "pem" % https://jp.mathworks.com/help/ident/ref/pem.html

        %伝達関数モデル https://jp.mathworks.com/help/ident/transfer-function-models.html
    case "idtf" % https://jp.mathworks.com/help/ident/ref/idtf.html
        % 伝達関数の項の数が必要になってくるのでy/u で求めることができない

    case "tfest" % https://jp.mathworks.com/help/ident/ref/tfest.html
        dataId = iddata(yShift, uShift, dt);  % iddataオブジェクトを作成
        sys = tfest(dataId, 2, 2);  % tfestに分子2次・分母2次を指定してsysを推定
        [mag, ph, w] = bode(sys, {0.1, 200});
        result.omega = squeeze(w);
        result.mag.db = 20*log10(squeeze(mag));
        result.ph.deg = squeeze(ph);

        % case "pem" % https://jp.mathworks.com/help/ident/ref/pem.html

    case "spectrumest" %https://jp.mathworks.com/help/ident/ref/spectrumest.html
        % これは出力信号y単体のパワースペクトルを推定するもので、入力uとの関係の伝達関数を求めるものではない。

        %非線形モデルの同定 https://jp.mathworks.com/help/ident/nonlinear-model-identification.html


        %グレーボックスモデルの推定 https://jp.mathworks.com/help/ident/grey-box-model-estimation.html
    case "greyest" % https://jp.mathworks.com/help/ident/ref/greyest.html
        % 微分方程式やパラメータの物理的構造を自分で定義し、idgreyオブジェクトを作ってから渡す必要があり

    case "nlgreyest" % https://jp.mathworks.com/help/ident/ref/nlgreyest.html
        % 非線形状態方程式を記述した関数を事前に用意する必要があり

        % case "pem" % https://jp.mathworks.com/help/ident/ref/pem.html

    otherwise
        % どのcaseにも一致しない場合、result.okをfalseにして
        % 呼び出し元で「未対応の手法」として警告・スキップさせる
        result.ok = false;
end