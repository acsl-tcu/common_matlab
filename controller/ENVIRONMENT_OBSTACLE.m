% function obs = ENVIRONMENT_OBSTACLE()
% % ENVIRONMENT_OBSTACLE 障害物の配置情報を一元管理する関数（複数対応版）
% %   構造体配列として定義することで、コントローラ側を書き換えずに障害物を増減できます。
% 
%     % --- 障害物 1 の定義 (直線軌道の経路上付近) ---
%     obs(1).p_obs  = [1.0; 0.8; 3.0]; % 中心座標 [x; y; z]
%     obs(1).r_obs  = 0.25;            % 物理半径 [m]
%     obs(1).margin = 0.15;            % 安全マージン [m]
%     obs(1).R_safe = obs(1).r_obs + obs(1).margin; % 制御用安全半径
% 
%     % --- 障害物 2 の定義 (少し離れた位置、または2つ目の関門) ---
%     obs(2).p_obs  = [1.8; 1.5; 3.0]; % 中心座標 [x; y; z]
%     obs(2).r_obs  = 0.25;            % 物理半径 [m]
%     obs(2).margin = 0.15;            % 安全マージン [m]
%     obs(2).R_safe = obs(2).r_obs + obs(2).margin; % 制御用安全半径
% end

% function obs = ENVIRONMENT_OBSTACLE()
% % ENVIRONMENT_OBSTACLE 障害物の配置情報を一元管理する関数（複数対応版）
% 
%     % --- 障害物 1 の定義 (軌道のほぼ中心、かつ吊り荷が通りやすい位置) ---
%     obs(1).p_obs  = [0; 5.0; 3]; % z方向も少し下げて吊り荷にぶつけます
%     obs(1).r_obs  = 0.5;            
%     obs(1).margin = 0.1;              % マージンを少し広げて確実に捉えます
%     obs(1).R_safe = obs(1).r_obs + obs(1).margin; 
% 
%     % % --- 障害物 2 の定義 (後半の関門) ---
%     obs(2).p_obs  = [0; 5.0; 1.0]; 
%     obs(2).r_obs  = 0.5;            
%     obs(2).margin = 0.1;            
%     obs(2).R_safe = obs(2).r_obs + obs(2).margin; 
% end

function obs = ENVIRONMENT_OBSTACLE()
% ENVIRONMENT_OBSTACLE 障害物の配置情報を一元管理する関数（デバッグ強制発動版）

    % --- 障害物 1 の定義 (吊り荷の現在高度に完全一致) ---
    obs(1).p_obs  = [0.05; 7.0; 0.916]; 
    obs(1).r_obs  = 0.50;            
    obs(1).margin = 0.3;            
    obs(1).R_safe = obs(1).r_obs + obs(1).margin; 

    % --- 障害物 2 の定義 (十分に離した前進回廊へ配置) ---
    obs(2).p_obs  = [0.05; 15.0; 0.916]; 
    obs(2).r_obs  = 0.50;            
    obs(2).margin = 0.3;            
    obs(2).R_safe = obs(2).r_obs + obs(2).margin; 
end