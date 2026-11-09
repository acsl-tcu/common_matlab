classdef GROUND_INFORMATION
    % GROUND_INFORMATION  地面の幾何情報のみを定義する独立モジュール。
    %
    % 場所: reference/REPLANNING_SYSTEM/OBSTACLE_INFORMATION/
    % 地面は世界座標の z = 0 [m] にある無限水平面。
    % z <= 0 [m] が占有領域、z > 0 [m] が自由空間。
    %
    % 地面は模擬センサの検出対象ではない。Flight相('f')の計画では常時既知。
    % Arming/Takeoff/Landing相には地面非接触制約を課さない。
    % 相の判定は定義ファイルではなく TRAJECTORY_GROUND_SCOPE に集約する。
    % 障害物数や観測済み ID の増減とは独立した環境境界である。
    % OBSTACLE_INFORMATION.definitions() の有限障害物配列へ加えない。
    % 既存の障害物IDは付与しない。Obstacle Memoryに格納・解放しない。
    %
    % ここには安全マージン、機体・吊荷・索の半径、誤差チューブ、
    % 衝突判定・コリドー・軌道生成の処理を追加しない。
    % このファイルは定義専用。連続時間のUAV・索・吊荷それぞれの
    % 地面非接触証明はTRAJECTORY_GENERATIONの証明器が担当する。
    methods (Static)
        function ground = definition()
            % 地面の形状と既知状態のみを返す。ログ保存は行わない。
            ground = struct( ...
                'type',          'ground_halfspace', ...
                'name',          'Ground z=0', ...
                'normal_world',  [0;0;1], ...
                'height_m',      0, ...
                'occupied_side', 'negative', ...
                'always_known',  true);
        end
    end
end
