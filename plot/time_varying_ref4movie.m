clc;clear;close all;

FS = 16; % FontSizeの定義

% --- 時間の設定（ここをいじるだけでOK） ---
phase = 0;    % 位相
T = 10;         % 周期
radius = 1;     % 半径
tail_time = 1.0; % 軌跡（しっぽ）の設定
circle = 4;     % 何周動かすか
initial_pause = 0;    % 最初の静止時間 [s]
fps = 30;             % 1秒あたりのフレーム数

angle = -120; % 視点の方位角
elevetor = 25; % 始点の仰角

% --- 保存先の設定 ---
fSave = 1;                                 % 動画を保存するか（1: 保存, 0: 表示のみ）
saveDir = "C:\Users\hiyou\Github\ATACS";                             % 保存フォルダ（例: fullfile(pwd, 'Data')）
fileName = 'leminscate_ref_orbit.mp4';  % 保存ファイル名

% --- 自動計算されるパラメータ ---
move_end_time = abs(T)*circle;   % 動きが終わる時間 [s]
t_all = linspace(0, move_end_time, move_end_time * fps);
dt = t_all(2) - t_all(1);
wait_frames = round(initial_pause * fps);
total_frames = wait_frames + length(t_all);
tail_len = round(tail_time / dt);
w = 2*pi/T;

% データの事前計算
% == saddle ================================
x = radius*cos(w*t_all + phase);
y = radius*sin(w*t_all + phase);
z = 0.25 * sin(2*w*t_all + phase/2) + 1;
% ==========================================

% == Lemniscate ============================
x = radius*sin(w*t_all+phase)./(1+cos(w*t_all+phase).^2);
y = radius*sin(w*t_all+phase).*cos(w*t_all+phase)./(1+cos(w*t_all+phase).^2);
z = ones(size(t_all));
% ==========================================



% --- ビデオライターの準備 ---
if fSave
    if ~isfolder(saveDir)
        mkdir(saveDir);
    end
    savePath = fullfile(saveDir, fileName);
    v = VideoWriter(savePath, 'MPEG-4');
    v.FrameRate = fps;
    open(v);
end


% --- プロットの準備 ---
fig = figure('Color', 'w');

% 1. リファレンス軌道 (Reference)
refEndIdx = find(t_all >= abs(T), 1); % 1周期分の終端（周期を跨ぐ最初の点まで含めて閉じる）
if isempty(refEndIdx)
    refEndIdx = length(t_all);
end
h_ref = plot3(x(1:refEndIdx), y(1:refEndIdx), z(1:refEndIdx), 'k--', 'LineWidth', 0.5);
hold on;

% 2. 動く軌跡 (Orbit)
h_trail = plot3(NaN, NaN, NaN, 'b', 'LineWidth', 1.5); 

% 3. 重心 (Center of mass)
h_head = plot3(NaN, NaN, NaN, 'ro', 'MarkerFaceColor', 'r', 'MarkerSize', 6); 

% 凡例の設定
legend([h_ref, h_trail, h_head], {'Reference', 'Orbit', 'Center of mass'}, ...
       'Location', 'northeast', 'Interpreter', 'none', 'Orientation','horizontal', 'FontSize',FS);
% legend([h_trail, h_head], {'Reference', 'Center of mass'}, ...
%        'Location', 'north', 'Interpreter', 'none', 'Orientation','horizontal', 'FontSize',FS);

grid on; grid minor;
view(angle, elevetor);
xlabel('$x$ [m]', 'Interpreter','latex', 'FontSize',FS);
ylabel('$y$ [m]', 'Interpreter','latex', 'FontSize',FS);
zlabel('$z$ [m]', 'Interpreter','latex', 'FontSize',FS);

% --- 軸の固定設定 ---
margin = 0.2; 
x_lims = [min(x)-margin, max(x)+margin];
y_lims = [min(y)-margin, max(y)+margin];
z_lims = [min(z)-margin, max(z)+margin];

axis equal; % 各軸の単位長さを等しくする（z方向が低く表示される）
axis([x_lims, y_lims, z_lims]); 
axis vis3d;

% --- アニメーション録画ループ ---
if fSave
    fprintf('録画を開始します（最初の%d秒は停止します）...\n', initial_pause);
end
for k = 1:total_frames
    % 最初の wait_frames まではインデックスを 1 (初期位置) に固定
    idx = max(1, k - wait_frames);
    
    % データの更新
    start_idx = max(1, idx - tail_len);
    set(h_trail, 'XData', x(start_idx:idx), 'YData', y(start_idx:idx), 'ZData', z(start_idx:idx));
    set(h_head,  'XData', x(idx), 'YData', y(idx), 'ZData', z(idx));
    
    drawnow;
    
    % フレームのキャプチャ
    if fSave
        frame = getframe(fig);
        writeVideo(v, frame);
    else
        pause(1/fps); % 保存しない場合は実時間に近い速度で表示
    end
end

% --- 終了処理 ---
if fSave
    close(v);
    fprintf('動画の保存が完了しました: %s\n', savePath);
end