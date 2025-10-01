function ref = gen_ref_spline_kyo(param)
%スプライン軌道用
%% この関数でやりたい処理
% isManualSetting = 1のときにここでwaypointを設定する
% 保存したmatファイルからway_point_refを呼び出す

% filename: 読み込む目標軌道
% order: 何次までのスプラインか
% isManualSetting: 手動で設定するか、読み込むか
arguments
    param.point
    param.order
    param.filename
    param.ManualSetting
    param.point_dt
    param.check
end

ManualSetting = param.ManualSetting;
%% ここから処理開始
if ManualSetting ==1
    disp('Loading reference data from mat');
    % load(strcat('../Data/reference/', filename));
    load(strcat('Data/reference/', filename)); % for exp
else
    pointN = param.point; %waypointの数
    dt = param.point_dt;%waypoint間の時間
    time =  (0:dt:dt*(pointN-1))';

    %% ランダムな軌道の生成
    % xyz-directional xyz方向のランダムな軌道
    % wp_xy = max(-1.0, min(1.0, [round(1*randn(pointN-2,1),3), round(1*randn(pointN-2,1),3)]));
    % wp_z  = max(0.5, min(1.5, round(0.5*randn(pointN-2,1)+1,3)));
    %
    % wp = [0, 0, 0.6;wp_xy, wp_z; 0, 0, 0.6];
    wp = zeros(pointN, 3);
    wp(1, :) = [0, 0, 0.6];
    wp(pointN, :) = [0, 0, 0.6];
    for i = 1:(pointN - 2)
        previous_point = wp(i, :);
        next_point = previous_point;

        % 1、2、または3つの軸を変更するかをランダムに決定
        num_axes_to_change = randi(3);
        % 変更する軸をランダムに選択
        axes_to_change = randperm(3, num_axes_to_change);

        for axis_idx = axes_to_change
            if axis_idx == 1 % X軸を更新
                next_point(1) = round(2 * rand() - 1, 3);
            elseif axis_idx == 2 % Y軸を更新
                next_point(2) = round(2 * rand() - 1, 3);
            else % Z軸を更新
                next_point(3) = round(0.5 + rand(), 3);
            end
        end
        wp(i + 1, :) = next_point;
    end
end
    waypoints = [time, wp];
    order = param.order;%多項式の次数
    check = param.check;
    num_segments = size(waypoints, 1) - 1;
    segment_types = strings(num_segments, 1);
    for i = 1:num_segments
        % 座標が変化した軸の数を確認（微小な誤差を許容）
        changed_axes = abs(waypoints(i+1, 2:4) - waypoints(i, 2:4)) > 1e-6;
        if nnz(changed_axes) == 1
            segment_types(i) = "single_axis"; % 単軸
        else
            segment_types(i) = "multi_axis";  % 多軸
        end
    end

    % 多軸移動のために、グローバルなスプラインを事前計算
    spline_data  = way_point_ref(waypoints,order,0);%補間式の係数など計算
    ref = @(t) hybrid_trajectory(t, waypoints, spline_data, segment_types);
    if param.check == 1
    t_vec = 0:0.025:waypoints(end,1);
    xyz = zeros(3, length(t_vec));
    for k = 1:length(t_vec)
        state = ref(t_vec(k));
        xyz(:, k) = state(1:3);
    end
    
     figure(101); clf;
    plot3(xyz(1,:), xyz(2,:), xyz(3,:), 'b-', 'LineWidth', 2);
    hold on; grid on;
    plot3(waypoints(:,2), waypoints(:,3), waypoints(:,4), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
    h_start = plot3(waypoints(1,2), waypoints(1,3), waypoints(1,4), 'gp', 'MarkerSize', 12, 'MarkerFaceColor', 'g');
    
    xlabel('$x$ [m]', 'Interpreter', 'latex'); ylabel('$y$ [m]', 'Interpreter', 'latex'); zlabel('$z$ [m]', 'Interpreter', 'latex');
    title('Hybrid Trajectory (Spline + Linear)', 'FontSize', 14);
    set(gca, 'TickLabelInterpreter', 'latex', 'FontSize', 14);
    axis equal; view(30, 20);
    legend(h_start, {'Start Point'}, 'FontSize', 14);
    end
    function state = hybrid_trajectory(t, waypoints, spline_data, segment_types)
    % 現在時刻tが含まれる区間を特定
    idx = find(t >= waypoints(:,1), 1, 'last');
    
    % 境界条件の処理
    if isempty(idx) || idx >= size(waypoints, 1)
        idx = size(waypoints, 1) - 1;
        if t >= waypoints(end, 1)
            state = [waypoints(end, 2:4)'; zeros(12,1)];
            return;
        end
    end
    
    % 区間のタイプに応じて補間方法を選択
    current_segment_type = segment_types(idx);
    
    if current_segment_type == "single_axis"
        % --- 線形補間を使用 ---
        start_time = waypoints(idx, 1);
        end_time = waypoints(idx+1, 1);
        start_point = waypoints(idx, 2:4)';
        end_point = waypoints(idx+1, 2:4)';
        segment_duration = end_time - start_time;
        if segment_duration <= 0
            pos = start_point; vel = zeros(3,1);
        else
            tau = (t - start_time) / segment_duration;
            pos = start_point + tau * (end_point - start_point);
            vel = (end_point - start_point) / segment_duration;
        end
        acc = zeros(3,1); jerk = zeros(3,1); snap = zeros(3,1);
        state = [pos; 0; vel; 0; acc; 0; jerk; 0; snap; 0];
    else % "multi_axis"
        % --- スプライン補間を使用 ---
        t_mod = t;
        names = fieldnames(spline_data.coefficients);
        N=5;
        segment = idx;
        time_in_segment = t_mod - spline_data.time(segment);

        ref_initial = struct();
        for j = 1:N
            ref_initial.(names{j}) = spline_data.coefficients.(names{j})(:,:,segment) * spline_data.t_powers.(names{j})(time_in_segment);
        end
        
        state = [];
        for j=1:N
            state = [state; ref_initial.(names{j}); 0];
        end
    end
end
        
   

    if  ManualSetting ==1 %waypointを保存するか選べる
        isSaved = [];
        while ~any(ismember(isSaved, [0, 1]))
            isSaved = input("Save spline curve : '1'\nNo save : '0'\nFill in : ",'s');
            isSaved = str2double(isSaved);
         if isnan(isSaved) || ~ismember(isSaved,[0,1])
            disp('0または1を入力してください');
            isSaved = [];
         end
        end
        if isSaved==0
            disp("No save")
        else
            % save('../Data/reference/exp_ref.mat', 'waypoints');
            save('Data\reference\exp_ref.mat', 'waypoints');
        end
    end
end