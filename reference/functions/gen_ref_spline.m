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
        % xyz-directional xyz方向のランダムな軌道
        wp_xy = max(-1.0, min(1.0, [round(1*randn(pointN-2,1),3), round(1*randn(pointN-2,1),3)]));
        wp_z  = max(0.7, min(1.3, round(0.5*randn(pointN-2,1)+1,3)));
       
        wp = [0, 0, 1;wp_xy, wp_z; 0, 0, 1];
        waypoints = [time, wp];
        order = param.order;%多項式の次数
    end
    ref_data = way_point_ref(waypoints,order);%補間式の係数など計算
    
    function ref = spline_curve(ref_data,t)
        
            %区間ごとの補間式を作成
            t_mod = mod(t,ref_data.period);
            names = fieldnames(ref_data.coefficients);
            N=5;%HLが4階微分までだから5まで
            
            for i=1:ref_data.Sn
                for j=1:N   
                interpolation.(names{j})(:,i) = ref_data.coefficients.(names{j})(:,:,i)*ref_data.t_powers.(names{j})(t_mod-ref_data.time(i));
                end
            end
            
            for j =1:N
            ref_initial.(names{j}) = zeros(3,1); % 初期化
            end

            %どの区間か判断
            for i = 1:ref_data.Sn
                h1 = heaviside(t_mod - ref_data.time(i));
                h2 = heaviside(ref_data.time(i+1) - t_mod);
                for j=1:N
                ref_initial.(names{j}) = ref_initial.(names{j}) + interpolation.(names{j})(:,i).*h1.*h2;
                end
            end
            ref = [];
            for j=1:N
            ref = [ref;ref_initial.(names{j});0];%refに4階微分まで登録
            end
    end
    ref=@(t) spline_curve(ref_data,t);

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