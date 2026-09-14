function ref = gen_ref_spline_edmd(param)
% EDMD-oriented spline reference generator.
% Unlike gen_ref_spline, this version does not hard-code rng(0).
% It can use a per-run seed for reproducible but non-identical trajectories.

arguments
    param.point
    param.order
    param.filename = ''
    param.ManualSetting = 0
    param.point_dt = 5
    param.check = 0
    param.seed = []
    param.xy_limit = 1.0
    param.z_center = 0.70
    param.z_sigma = 0.18
    param.z_min = 0.35
    param.z_max = 1.05
    param.min_step_xy = 0.30
    param.min_step_z = 0.12
end

ManualSetting = param.ManualSetting;
order = param.order;
check = param.check;

if ManualSetting == 1
    disp('Loading reference data from mat');
    load(strcat('Data/reference/', param.filename));
else
    if ~isempty(param.seed)
        rng(param.seed);
    end

    pointN = param.point;
    dt = param.point_dt;
    time = (0:dt:dt*(pointN-1))';

    wp = zeros(pointN, 3);
    wp(1, :) = [0, 0, 0.6];
    wp(pointN, :) = [0, 0, 0.6];

    for i = 2:(pointN - 1)
        prev = wp(i - 1, :);
        candidate = prev;
        valid = false;
        attempt = 0;

        while ~valid && attempt < 40
            attempt = attempt + 1;

            candidate(1:2) = round((2 * rand(1, 2) - 1) * param.xy_limit, 3);

            if mod(i, 2) == 0
                z_raw = param.z_center + abs(param.z_sigma * randn());
            else
                z_raw = param.z_center - abs(param.z_sigma * randn());
            end

            candidate(3) = round(max(param.z_min, min(param.z_max, z_raw)), 3);

            xy_step = norm(candidate(1:2) - prev(1:2));
            z_step = abs(candidate(3) - prev(3));
            valid = (xy_step >= param.min_step_xy) || (z_step >= param.min_step_z);
        end

        wp(i, :) = candidate;
    end

    waypoints = [time, wp];
end

ref_data = way_point_ref(waypoints, order, check);

function state = spline_curve(ref_data, t)
    t_mod = mod(t, ref_data.period);
    names = fieldnames(ref_data.coefficients);
    N = 5;
    segment = find(t_mod >= ref_data.time(1:end-1) & t_mod < ref_data.time(2:end), 1);
    if isempty(segment)
        segment = size(ref_data.coefficients.(names{1}), 3);
    end

    ref_initial = struct();
    for j = 1:N
        ref_initial.(names{j}) = ref_data.coefficients.(names{j})(:, :, segment) * ...
            ref_data.t_powers.(names{j})(t_mod - ref_data.time(segment));
    end

    state = [];
    for j = 1:N
        state = [state; ref_initial.(names{j}); 0];
    end
end

ref = @(t) spline_curve(ref_data, t);

if ManualSetting == 1
    isSaved = [];
    while ~any(ismember(isSaved, [0, 1]))
        isSaved = input("Save spline curve : '1'\nNo save : '0'\nFill in : ", 's');
        isSaved = str2double(isSaved);
        if isnan(isSaved) || ~ismember(isSaved, [0, 1])
            disp('0 または 1 を入力してください');
            isSaved = [];
        end
    end
    if isSaved == 1
        save('Data\reference\exp_ref.mat', 'waypoints');
    else
        disp("No save")
    end
end
end
