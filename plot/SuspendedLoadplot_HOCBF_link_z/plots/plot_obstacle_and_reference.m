function plot_obstacle_and_reference(data)
% 障害物と目標軌道（p_ref）のみの関係を描画する関数
% （3D視点・真上・真横の3つの独立したウィンドウを出力）

% --- 共通データの準備 ---
obs_center = data.cbf.p_obs(:, 1);
obs_radius = data.cbf.r_obs_margin(1);

% 3D球体メッシュデータ
[X_sp, Y_sp, Z_sp] = sphere(50);
X_sp = X_sp * obs_radius + obs_center(1);
Y_sp = Y_sp * obs_radius + obs_center(2);
Z_sp = Z_sp * obs_radius + obs_center(3);

% 目標軌道データ (p_ref)
ref_x = data.ref.p(1, :);
ref_y = data.ref.p(2, :);
ref_z = data.ref.p(3, :);

%% =========================================================================
%% 1. 3D 視点 (Perspective View)
%% =========================================================================
figure('Name', '障害物と目標軌道 (3D視点)');
surf(X_sp, Y_sp, Z_sp, 'FaceColor', [1, 0.5, 0], 'EdgeColor', 'none', 'FaceAlpha', 0.4);
hold on;
camlight;
lighting gouraud;

plot3(ref_x, ref_y, ref_z, 'r--', 'LineWidth', 2, 'DisplayName', 'Reference');

grid on;
axis equal;
set(gca, 'FontSize', 14);
xlabel('X [m]', 'FontSize', 16);
ylabel('Y [m]', 'FontSize', 16);
zlabel('Z [m]', 'FontSize', 16);
legend('Obstacle', 'Reference', 'Location', 'best', 'FontSize', 16);
view(3); % 斜め3D視点
hold off;

%% =========================================================================
%% 2. 真上からの視点 (Top View: XY平面)
%% =========================================================================
figure('Name', '障害物と目標軌道 (真上からの視点: XY Plane)');
surf(X_sp, Y_sp, Z_sp, 'FaceColor', [1, 0.5, 0], 'EdgeColor', 'none', 'FaceAlpha', 0.4);
hold on;
camlight;
lighting gouraud;

plot3(ref_x, ref_y, ref_z, 'r--', 'LineWidth', 2, 'DisplayName', 'Reference');

grid on;
axis equal;
set(gca, 'FontSize', 14);
xlabel('X [m]', 'FontSize', 16);
ylabel('Y [m]', 'FontSize', 16);
zlabel('Z [m]', 'FontSize', 16);
legend('Obstacle', 'Reference', 'Location', 'best', 'FontSize', 16);
view(0, 90); % 真上（XY平面）からのカメラ視点（3D回転可能）
hold off;

%% =========================================================================
%% 3. 真横からの視点 (Side View: YZ平面)
%% =========================================================================
figure('Name', '障害物と目標軌道 (真横からの視点: YZ Plane)');
surf(X_sp, Y_sp, Z_sp, 'FaceColor', [1, 0.5, 0], 'EdgeColor', 'none', 'FaceAlpha', 0.4);
hold on;
camlight;
lighting gouraud;

plot3(ref_x, ref_y, ref_z, 'r--', 'LineWidth', 2, 'DisplayName', 'Reference');

grid on;
axis equal;
set(gca, 'FontSize', 14);
xlabel('X [m]', 'FontSize', 16);
ylabel('Y [m]', 'FontSize', 16);
zlabel('Z [m]', 'FontSize', 16);
legend('Obstacle', 'Reference', 'Location', 'best', 'FontSize', 16);
view(90, 0); % 真横（YZ平面）からのカメラ視点（3D回転可能）
%% ※前進方向がX軸メインの場合は view(0, 0) に変更してください
hold off;

fprintf('障害物判定半径 (Margin込): %f [m]\n', obs_radius);
end