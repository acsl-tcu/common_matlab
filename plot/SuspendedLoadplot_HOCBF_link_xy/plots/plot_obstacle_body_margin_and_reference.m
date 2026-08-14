function plot_obstacle_body_margin_and_reference(data)
% 障害物本体（マージンなし）・安全マージン領域・目標軌道（p_ref）の関係を描画する関数
% （3D視点・真上・真横の3つの独立したウィンドウを出力）

% --- 共通データの準備 ---
obs_center        = data.cbf.p_obs(:, 1);
obs_radius_raw    = data.cbf.r_obs(1);        % 障害物本体（マージンなし）半径
obs_radius_margin = data.cbf.r_obs_margin(1); % マージン込み判定半径

% 1. 障害物本体（マージンなし）の3D球体メッシュ
[X_raw, Y_raw, Z_raw] = sphere(50);
X_raw = X_raw * obs_radius_raw + obs_center(1);
Y_raw = Y_raw * obs_radius_raw + obs_center(2);
Z_raw = Z_raw * obs_radius_raw + obs_center(3);

% 2. 安全マージン領域（マージン込み）の3D球体メッシュ
[X_mar, Y_mar, Z_mar] = sphere(50);
X_mar = X_mar * obs_radius_margin + obs_center(1);
Y_mar = Y_mar * obs_radius_margin + obs_center(2);
Z_mar = Z_mar * obs_radius_margin + obs_center(3);

% 目標軌道データ (p_ref)
ref_x = data.ref.p(1, :);
ref_y = data.ref.p(2, :);
ref_z = data.ref.p(3, :);

%% =========================================================================
%% 1. 3D 視点 (Perspective View)
%% =========================================================================
figure('Name', '障害物（本体＆マージン）と目標軌道 (3D視点)');

% 障害物本体（赤色・透過度0.6）
surf(X_raw, Y_raw, Z_raw, 'FaceColor', [1, 0.0, 0.0], 'EdgeColor', 'none', 'FaceAlpha', 0.6);
hold on;
% マージン領域（オレンジ色・透過度0.25）
surf(X_mar, Y_mar, Z_mar, 'FaceColor', [1.0, 0.3, 0.0], 'EdgeColor', 'none', 'FaceAlpha', 0.25);

camlight;
lighting gouraud;

plot3(ref_x, ref_y, ref_z, 'r--', 'LineWidth', 2, 'DisplayName', 'Reference');

grid on;
axis equal;
set(gca, 'FontSize', 14);
xlabel('$x$ [m]', 'FontSize', 16, 'Interpreter', 'latex');
ylabel('$y$ [m]', 'FontSize', 16, 'Interpreter', 'latex');
zlabel('$z$ [m]', 'FontSize', 16, 'Interpreter', 'latex');
legend('Obstacle (Body)', 'Obstacle (Margin)', 'Reference', 'Location', 'best', 'FontSize', 16);
view(3); % 斜め3D視点
hold off;

%% =========================================================================
%% 2. 真上からの視点 (Top View: XY平面)
%% =========================================================================
figure('Name', '障害物（本体＆マージン）と目標軌道 (真上からの視点: XY Plane)');

% 障害物本体（赤色・透過度0.6）
surf(X_raw, Y_raw, Z_raw, 'FaceColor', [1, 0.0, 0.0], 'EdgeColor', 'none', 'FaceAlpha', 0.6);
hold on;
% マージン領域（オレンジ色・透過度0.25）
surf(X_mar, Y_mar, Z_mar, 'FaceColor', [1.0, 0.3, 0.0], 'EdgeColor', 'none', 'FaceAlpha', 0.25);

camlight;
lighting gouraud;

plot3(ref_x, ref_y, ref_z, 'r--', 'LineWidth', 2, 'DisplayName', 'Reference');

grid on;
axis equal;
set(gca, 'FontSize', 14);
xlabel('$x$ [m]', 'FontSize', 16, 'Interpreter', 'latex');
ylabel('$y$ [m]', 'FontSize', 16, 'Interpreter', 'latex');
zlabel('$z$ [m]', 'FontSize', 16, 'Interpreter', 'latex');
legend('Obstacle (Body)', 'Obstacle (Margin)', 'Reference', 'Location', 'best', 'FontSize', 16);
view(0, 90); % 真上（XY平面）カメラ視点
hold off;

%% =========================================================================
%% 3. 真横からの視点 (Side View: YZ平面)
%% =========================================================================
figure('Name', '障害物（本体＆マージン）と目標軌道 (真横からの視点: YZ Plane)');

% 障害物本体（赤色・透過度0.6）
surf(X_raw, Y_raw, Z_raw, 'FaceColor', [1, 0.0, 0.0], 'EdgeColor', 'none', 'FaceAlpha', 0.6);
hold on;
% マージン領域（オレンジ色・透過度0.25）
surf(X_mar, Y_mar, Z_mar, 'FaceColor', [1.0, 0.3, 0.0], 'EdgeColor', 'none', 'FaceAlpha', 0.25);

camlight;
lighting gouraud;

plot3(ref_x, ref_y, ref_z, 'r--', 'LineWidth', 2, 'DisplayName', 'Reference');

grid on;
axis equal;
set(gca, 'FontSize', 14);
xlabel('$x$ [m]', 'FontSize', 16, 'Interpreter', 'latex');
ylabel('$y$ [m]', 'FontSize', 16, 'Interpreter', 'latex');
zlabel('$z$ [m]', 'FontSize', 16, 'Interpreter', 'latex');
legend('Obstacle (Body)', 'Obstacle (Margin)', 'Reference', 'Location', 'best', 'FontSize', 16);
view(90, 0); % 真横（YZ平面）カメラ視点
hold off;

fprintf('障害物本体半径   : %f [m]\n', obs_radius_raw);
fprintf('障害物判定半径 (Margin込): %f [m]\n', obs_radius_margin);
end