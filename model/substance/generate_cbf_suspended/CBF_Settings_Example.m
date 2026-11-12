function settings = CBF_Settings_Example()
%CBF_SETTINGS_EXAMPLE  3次元CBFの設定例（数値は説明用、実機パラメータではありません）
%
%  自分の値を指定するとき:
%    settings = CBF_Settings_Example();
%    settings.physical.cableLength = 0.60;
%    settings.cable.maxExtraRadius = 0.01;
%    generateCBF_RealInput(settings);
%    generateCBF_VirtualInput(settings);
%
%  生成された関数も (x, settings, obstacle) の引数を受け取ります。
%  設定値を変えても、通常の幾何評価用関数を再生成する必要はありません。

%% 1. 物理パラメータ（ご自身のモデルの値に置き換える）
settings.physical.massDrone   = 1.00;           % 機体質量 [kg]（例）
settings.physical.massLoad    = 0.15;           % 吊り荷質量 [kg]（例）
settings.physical.cableLength = 0.46;           % ケーブル長 [m]（例）
settings.physical.gravity     = 9.81;           % 重力加速度 [m/s^2]
settings.physical.inertia     = [0.012;0.012;0.020]; % [Jx;Jy;Jz] [kg m^2]
settings.physical.disturbanceXY = [0;0];        % コントローラモデルの加速度外乱 [m/s^2]

%% 2. 機体・ケーブル・吊り荷の衝突判定用サイズ
settings.shape.droneRadius = 0.20;              % 機体外接球半径 [m]（例）
settings.shape.loadRadius  = 0.10;              % 吊り荷外接球半径 [m]（例）
settings.shape.cableRadius = 0.005;             % ケーブル実半径 [m]（例）

%% 3. 複数球方式 C のケーブル球数（長さに応じて自動算出）
settings.cable.numberOfSpheres = [];            % []: 自動、整数: 手動（両端を含む）
settings.cable.maxExtraRadius  = 0.010;         % 物理ケーブル半径からの最大膨張 [m]
settings.cable.maxNumberOfSpheres = 200;        % 異常な大量生成を防止
% 実際の球半径 = sqrt(ケーブル半径^2 + (球中心間隔/2)^2)
% これにより、半径のあるケーブル全区間を覆う。

%% 4. 障害物距離・地面
settings.cbf.safetyMargin = 0.04;               % 追加の安全距離 [m]
settings.cbf.smoothEpsilon = 1.0e-5;            % 支持関数の平滑化 [m相当]
settings.ground.height = 0.0;                   % 地面高さ [m]

%% 5. 実入力上下限（ロータ配分やモータ応答は扱わない）
settings.input.thrustMin = 0.0;                 % [N]
settings.input.thrustMax = 20.0;                % [N]
settings.input.torqueMin = [-1;-1;-1];          % [N m]
settings.input.torqueMax = [ 1; 1; 1];          % [N m]

%% 6. 生成に関する設定
settings.generate.exportSymbolicGeometry = false; % true: Symbolic Math Toolbox を使用
settings.generate.exportOneHOCBFRow = false;      % true: 1行だけ高次Lie微分を試行
settings.generate.HOCBFCase = 'C_sphere';         % 例: C_sphere / B_ellipsoid
settings.generate.HOCBFIndex = 1;                 % 安全関数ベクトルの何番目か
settings.generate.HOCBFOrder = 6;                 % 数学的に確認した次数を指定する
settings.generate.HOCBFGain = 1.5;                % (D+k)^r h の k
% 高次Lie微分は計算が非常に重いので初期設定では行いません。

%% 7. 実験例：外部障害物（この関数から別に受け取る方式を推奨）
%  sphereObs = struct('type','sphere','center',[1;0;1.2], ...
%                    'radius',0.25,'axes',[0.25;0.25;0.25],'R',eye(3));
%  ellipsoidObs = struct('type','ellipsoid','center',[1;0;1.2], ...
%                       'radius',NaN,'axes',[0.2;0.4;0.3],'R',Rxyz);
end
