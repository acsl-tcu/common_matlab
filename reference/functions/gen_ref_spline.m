function ref = gen_ref_spline(param)
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
end

ManualSetting = param.ManualSetting;
%% ここから処理開始
    if ManualSetting ==1
        disp('Loading reference data from mat');
        % load(strcat('../Data/reference/', filename));
        load(strcat('Data/reference/', filename)); % for exp
        fshowfig = 0; % 読み込んだ時はグラフは描画しない
    else
        pointN = param.point; %waypointの数 
        dt = param.point_dt;%waypoint間の時間
        time =  (0:dt:dt*(pointN-1))';

        %% ランダムな軌道の生成
        % only x-directional
        % wp_xy = max(-1.2, min(1.2, [zeros(pointN-2,1), round(1*randn(pointN-2,1),3)]));
        % wp_z  = ones(pointN-2,1);

        % only z-directional z方向の上下移動の軌道
        % wp_xy = max(-1.2, min(1.2, [zeros(pointN-2,1), zeros(pointN-2,1)]));
        % wp_z  = max(0.5, min(1.5, round(0.5*randn(pointN-2,1)+1,3)));

        % xyz-directional xyz方向のランダムな軌道
        wp_xy = max(-1.2, min(1.2, [round(1*randn(pointN-2,1),3), round(1*randn(pointN-2,1),3)]));
        wp_z  = max(0.5, min(1.5, round(0.5*randn(pointN-2,1)+1,3)));
        
        wp = [0, 0, 1;wp_xy, wp_z; 0, 0, 1];
        waypoints = [time, wp];
        order = param.order;%多項式の次数※3次までしかできない
        fshowfig = 1;
    end
    ref_data = way_point_ref(waypoints,order,fshowfig,t);%補間式の係数など計算
    % information = make_reference(ref_data);
    % function information = make_reference(ref_data)
    %     information = @(t) spline_curve(ref_data,t);
    % end
    
    function ref = spline_curve(ref_data,t)
            %区間ごとの補間式を作成
            t_mod = mod(t,ref_data.period);
            for i=1:ref_data.Sn
            interpolation_p(:,i) = ref_data.coefficients.d0(:,:,i)*ref_data.t_powers.d0(t_mod-ref_data.time(i));
            interpolation_v(:,i) = ref_data.coefficients.d1(:,:,i)*ref_data.t_powers.d1(t_mod-ref_data.time(i));
            interpolation_a(:,i) = ref_data.coefficients.d2(:,:,i)*ref_data.t_powers.d2(t_mod-ref_data.time(i));
            interpolation_j(:,i) = ref_data.coefficients.d3(:,:,i)*ref_data.t_powers.d3(t_mod-ref_data.time(i));
            interpolation_s(:,i) = ref_data.coefficients.d4(:,:,i)*ref_data.t_powers.d4(t_mod-ref_data.time(i));
            end

            p_ref = zeros(3,1); % 初期化
            v_ref = zeros(3,1);
            a_ref = zeros(3,1);
            j_ref = zeros(3,1);
            s_ref = zeros(3,1);

            %どの区間か判断
            for i = 1:ref_data.Sn
                h1 = heaviside(t_mod - ref_data.time(i));
                h2 = heaviside(ref_data.time(i+1) - t_mod);
                p_ref = p_ref + interpolation_p(:,i).*h1.*h2;
                v_ref = v_ref + interpolation_v(:,i).*h1.*h2;
                a_ref = a_ref + interpolation_a(:,i).*h1.*h2;
                j_ref = j_ref + interpolation_j(:,i).*h1.*h2;
                s_ref = s_ref + interpolation_s(:,i).*h1.*h2;
            end
            ref = [p_ref;0;v_ref;0;a_ref;0;j_ref;0;s_ref;0];
    end
    ref=@(t) spline_curve(ref_data,t);

    if  exist('ManualSetting','var') %waypointを保存するか選べる
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