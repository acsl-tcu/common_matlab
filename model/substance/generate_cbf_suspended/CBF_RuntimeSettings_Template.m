function settings = CBF_RuntimeSettings_Template()
%CBF_RUNTIMESETTINGS_TEMPLATE  実行時に渡す設定の空テンプレート。
% 数値例・固定ケーブル長・固定半径は設定しません。
%
% 使用例:
%   settings = CBF_RuntimeSettings_Template();
%   settings.physical.massDrone = ...;   % 既存モデルのm
%   settings.physical.massLoad = ...;    % mL
%   settings.physical.cableLength = ...; % cableL
%   [h,info] = CBF_real_C_sphere(xReal,settings,obstacle);
%
% 必須値が空のままならcbf_parameter_vectorがエラーを出します。

%% 1. 物理モデル：実行時にすべて入力
settings.physical.massDrone = [];
settings.physical.massLoad = [];
settings.physical.cableLength = [];
settings.physical.gravity = [];
settings.physical.inertia = [];          % [Jx;Jy;Jz]
settings.physical.disturbanceXY = [];    % [dstx;dsty]。使わない場合[0;0]

%% 2. 機体・ケーブル・吊り荷の外接サイズ
settings.shape.droneRadius = [];
settings.shape.loadRadius = [];
settings.shape.cableRadius = [];

%% 3. 複数球によるケーブル被覆（実行時に球数を決める）
settings.cable.numberOfSpheres = [];     % []: 自動選択。整数: 手動
settings.cable.maxExtraRadius = [];      % 実半径から許す追加膨張 [m]
settings.cable.maxNumberOfSpheres = [];  % 計算量上限（整数）

%% 4. CBFの安全距離、平滑化・地面
settings.cbf.safetyMargin = [];
settings.cbf.smoothEpsilon = [];
settings.cbf.fastGeometrySensitivity = false; % V6: 解析勾配高速法。比較テストに合格してからtrueにする
settings.cbf.hocbfGain = [];            % 高次CBFの(D+k)^r hに使うゲイン
settings.cbf.groundGain = [];           % 地面専用。[]ならhocbfGainを使用
settings.cbf.pruneRedundantGround = true; % 証明済みの冗長な地面CBFのみ省略
settings.cbf.inputGain = [];            % 拡張実入力の上下限に使うゲイン
settings.cbf.degeneracyTolerance = [];  % 入力係数の退化を診断する閾値
settings.ground.height = [];

%% 5. 実入力の制限（トルクは各軸で別指定）
settings.input.thrustMin = [];
settings.input.thrustMax = [];
settings.input.torqueMin = [];           % 3x1
settings.input.torqueMax = [];           % 3x1
end
