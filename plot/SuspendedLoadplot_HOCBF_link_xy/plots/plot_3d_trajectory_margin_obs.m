function plot_3d_trajectory_margin_obs(data)
% 3D障害物（本体＆マージン）・荷物推定軌道・目標軌道の描画
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

% 軌道データ
pL_x  = data.estimator.pL(1, :);
pL_y  = data.estimator.pL(2, :);
pL_z  = data.estimator.pL(3, :);
ref_x = data.ref.p(1, :);
ref_y = data.ref.p(2, :);
ref_z = data.ref.p(3, :);

%% =========================================================================
%% 1. 3D 視点 (Perspective View)
%% =========================================================================
figure('Name', '障害物（本体＆マージン）・推定軌道・目標軌道 (3D視点)');

% 障害物本体（赤色・透過度0.6）
surf(X_raw, Y_raw, Z_raw, 'FaceColor', [1.0, 0.0, 0.0], 'EdgeColor', 'none', 'FaceAlpha', 0.6, 'DisplayName', 'Obstacle (Body)');
hold on;
% マージン領域（オレンジ色・透過度0.25）
surf(X_mar, Y_mar, Z_mar, 'FaceColor', [1.0, 0.3, 0.0], 'EdgeColor', 'none', 'FaceAlpha', 0.25, 'DisplayName', 'Obstacle (Margin)');

camlight;
lighting gouraud;

% 軌道プロット
plot3(pL_x, pL_y, pL_z, 'b-', 'LineWidth', 2, 'DisplayName', 'Estimator');
plot3(ref_x, ref_y, ref_z, 'r--', 'LineWidth', 1.5, 'DisplayName', 'Reference');

grid on;
axis equal;
set(gca, 'FontSize', 14);
xlabel('$x$ [m]', 'FontSize', 16, 'Interpreter', 'latex');
ylabel('$y$ [m]', 'FontSize', 16, 'Interpreter', 'latex');
zlabel('$z$ [m]', 'FontSize', 16, 'Interpreter', 'latex');
legend('Location', 'best', 'FontSize', 16);
view(3); % 斜め3D視点
hold off;

%% =========================================================================
%% 2. 真上からの視点 (Top View: XY平面)
%% =========================================================================
figure('Name', '障害物（本体＆マージン）・推定軌道・目標軌道 (真上からの視点: XY Plane)');

% 障害物本体（赤色・透過度0.6）
surf(X_raw, Y_raw, Z_raw, 'FaceColor', [1.0, 0.0, 0.0], 'EdgeColor', 'none', 'FaceAlpha', 0.6, 'DisplayName', 'Obstacle (Body)');
hold on;
% マージン領域（オレンジ色・透過度0.25）
surf(X_mar, Y_mar, Z_mar, 'FaceColor', [1.0, 0.3, 0.0], 'EdgeColor', 'none', 'FaceAlpha', 0.25, 'DisplayName', 'Obstacle (Margin)');

camlight;
lighting gouraud;

% 軌道プロット
plot3(pL_x, pL_y, pL_z, 'b-', 'LineWidth', 2, 'DisplayName', 'Estimator');
plot3(ref_x, ref_y, ref_z, 'r--', 'LineWidth', 1.5, 'DisplayName', 'Reference');

grid on;
axis equal;
set(gca, 'FontSize', 14);
xlabel('$x$ [m]', 'FontSize', 16, 'Interpreter', 'latex');
ylabel('$y$ [m]', 'FontSize', 16, 'Interpreter', 'latex');
zlabel('$z$ [m]', 'FontSize', 16, 'Interpreter', 'latex');
legend('Location', 'best', 'FontSize', 16);
view(0, 90); % 真上（XY平面）からのカメラ視点
hold off;

%% =========================================================================
%% 3. 真横からの視点 (Side View: YZ平面)
%% ※前進方向がX軸メインの場合は view(0, 0) （XZ平面）に変更してください
%% =========================================================================
figure('Name', '障害物（本体＆マージン）・推定軌道・目標軌道 (真横からの視点: YZ Plane)');

% 障害物本体（赤色・透過度0.6）
surf(X_raw, Y_raw, Z_raw, 'FaceColor', [1.0, 0.0, 0.0], 'EdgeColor', 'none', 'FaceAlpha', 0.6, 'DisplayName', 'Obstacle (Body)');
hold on;
% マージン領域（オレンジ色・透過度0.25）
surf(X_mar, Y_mar, Z_mar, 'FaceColor', [1.0, 0.3, 0.0], 'EdgeColor', 'none', 'FaceAlpha', 0.25, 'DisplayName', 'Obstacle (Margin)');

camlight;
lighting gouraud;

% 軌道プロット
plot3(pL_x, pL_y, pL_z, 'b-', 'LineWidth', 2, 'DisplayName', 'Estimator');
plot3(ref_x, ref_y, ref_z, 'r--', 'LineWidth', 1.5, 'DisplayName', 'Reference');

grid on;
axis equal;
set(gca, 'FontSize', 14);
xlabel('$x$ [m]', 'FontSize', 16, 'Interpreter', 'latex');
ylabel('$y$ [m]', 'FontSize', 16, 'Interpreter', 'latex');
zlabel('$z$ [m]', 'FontSize', 16, 'Interpreter', 'latex');
legend('Location', 'best', 'FontSize', 16);
view(90, 0); % 真横（YZ平面）からのカメラ視点
hold off;

% コンソールへの数値出力
fprintf('障害物本体半径   : %f [m]\n', obs_radius_raw);
fprintf('障害物判定半径 (Margin込): %f [m]\n', obs_radius_margin);

end