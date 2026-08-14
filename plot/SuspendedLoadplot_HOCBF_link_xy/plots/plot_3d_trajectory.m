function plot_3d_trajectory(data)
% 3D障害物・荷物推定軌道・目標軌道の描画（3D視点・真上・真横を個別のウィンドウで表示）

% --- 共通データの準備 ---
obs_center = data.cbf.p_obs(:, 1);
obs_radius = data.cbf.r_obs_margin(1);

% 3D球体メッシュデータ
[X_sp, Y_sp, Z_sp] = sphere(50);
X_sp = X_sp * obs_radius + obs_center(1);
Y_sp = Y_sp * obs_radius + obs_center(2);
Z_sp = Z_sp * obs_radius + obs_center(3);

% 2D断面（円）描画用データ
th = linspace(0, 2*pi, 100);
x_circ = obs_radius * cos(th) + obs_center(1);
y_circ = obs_radius * sin(th) + obs_center(2);
z_circ = obs_radius * sin(th) + obs_center(3);

% 軌道データ
pL_x = data.estimator.pL(1, :);
pL_y = data.estimator.pL(2, :);
pL_z = data.estimator.pL(3, :);

ref_x = data.ref.p(1, :);
ref_y = data.ref.p(2, :);
ref_z = data.ref.p(3, :);

%% =========================================================================
%% 1. 3D 視点 (Perspective View)
%% =========================================================================
figure('Name', '3Dの目標軌道と実際の軌道');
surf(X_sp, Y_sp, Z_sp, 'FaceColor', [1, 0.5, 0], 'EdgeColor', 'none', 'FaceAlpha', 0.4);
hold on;
camlight;
lighting gouraud;

plot3(pL_x, pL_y, pL_z, 'b-', 'LineWidth', 2, 'DisplayName', 'Estimator (pL)');
plot3(ref_x, ref_y, ref_z, 'r--', 'LineWidth', 1.5, 'DisplayName', 'Reference');

grid on;
axis equal;
set(gca, 'FontSize', 14);
xlabel('X [m]', 'FontSize', 16);
ylabel('Y [m]', 'FontSize', 16);
zlabel('Z [m]', 'FontSize', 16);
legend('Obstacle', 'Estimator', 'Reference', 'Location', 'best', 'FontSize', 16);
view(3);
hold off;

%% =========================================================================
%% 2. 真上からの視点 (Top View: XY平面)
%% =========================================================================
figure('Name', '真上からの視点 (XY Plane)');
fill(x_circ, y_circ, [1, 0.5, 0], 'FaceAlpha', 0.4, 'EdgeColor', 'none');
hold on;

plot(pL_x, pL_y, 'b-', 'LineWidth', 2);
plot(ref_x, ref_y, 'r--', 'LineWidth', 1.5);

grid on;
axis equal;
set(gca, 'FontSize', 14);
xlabel('X [m]', 'FontSize', 16);
ylabel('Y [m]', 'FontSize', 16);
legend('Obstacle', 'Estimator', 'Reference', 'Location', 'best', 'FontSize', 16);
hold off;

%% =========================================================================
%% 3. 真横からの視点 (Side View: YZ平面)
%% ※前進方向がX軸メインの場合は XZ平面（x_circ, z_circ）に変更してください
%% =========================================================================
figure('Name', '真横からの視点 (YZ Plane)');
fill(y_circ, z_circ, [1, 0.5, 0], 'FaceAlpha', 0.4, 'EdgeColor', 'none');
hold on;

plot(pL_y, pL_z, 'b-', 'LineWidth', 2);
plot(ref_y, ref_z, 'r--', 'LineWidth', 1.5);

grid on;
axis equal;
set(gca, 'FontSize', 14);
xlabel('Y [m]', 'FontSize', 16);
ylabel('Z [m]', 'FontSize', 16);
legend('Obstacle', 'Estimator', 'Reference', 'Location', 'best', 'FontSize', 16);
hold off;

%% =========================================================================
%% 6. 真上からの視点 (Top View: XY平面) ※3Dモデル＋カメラ角度調整
%% =========================================================================
figure('Name', '真上からの視点 (XY Plane)');
surf(X_sp, Y_sp, Z_sp, 'FaceColor', [1, 0.5, 0], 'EdgeColor', 'none', 'FaceAlpha', 0.4);
hold on;
camlight;
lighting gouraud;

plot3(pL_x, pL_y, pL_z, 'b-', 'LineWidth', 2, 'DisplayName', 'Estimator (pL)');
plot3(ref_x, ref_y, ref_z, 'r--', 'LineWidth', 1.5, 'DisplayName', 'Reference');

grid on;
axis equal;
set(gca, 'FontSize', 14);
xlabel('X [m]', 'FontSize', 16);
ylabel('Y [m]', 'FontSize', 16);
zlabel('Z [m]', 'FontSize', 16);
legend('Obstacle', 'Estimator', 'Reference', 'Location', 'best', 'FontSize', 16);
view(0, 90); % 真上（XY平面）からのカメラ視点（3D回転可能）
hold off;

%% =========================================================================
%% 5. 真横からの視点 (Side View: YZ平面) ※3Dモデル＋カメラ角度調整
%% =========================================================================
figure('Name', '真横からの視点 (YZ Plane)');
surf(X_sp, Y_sp, Z_sp, 'FaceColor', [1, 0.5, 0], 'EdgeColor', 'none', 'FaceAlpha', 0.4);
hold on;
camlight;
lighting gouraud;

plot3(pL_x, pL_y, pL_z, 'b-', 'LineWidth', 2, 'DisplayName', 'Estimator (pL)');
plot3(ref_x, ref_y, ref_z, 'r--', 'LineWidth', 1.5, 'DisplayName', 'Reference');

grid on;
axis equal;
set(gca, 'FontSize', 14);
xlabel('X [m]', 'FontSize', 16);
ylabel('Y [m]', 'FontSize', 16);
zlabel('Z [m]', 'FontSize', 16);
legend('Obstacle', 'Estimator', 'Reference', 'Location', 'best', 'FontSize', 16);
view(90, 0); % 真横（YZ平面）からのカメラ視点（3D回転可能）
%% ※前進方向がX軸メインの場合は view(0, 0) に変更してください
hold off;

fprintf('障害物判定半径 (Margin込): %f [m]\n', obs_radius);
end