function P = cbf_parameter_vector(settings)
%CBF_PARAMETER_VECTOR 実行時settingsから内部20要素を作成します。
% 生成器に物理値を固定しないため、この関数は実行時のみ呼びます。
% 内部順序は既存のcbf_geometry_core / cbf_symbolic_modelのPと一致させます。

%% 1. 必要な設定フィールドを読み込む
try
    p = settings.physical;
    s = settings.shape;
    c = settings.cbf;
    ground = settings.ground;
    u = settings.input;
    J = p.inertia(:);
    d = p.disturbanceXY(:);
    tmin = u.torqueMin(:);
    tmax = u.torqueMax(:);
    % ケーブル球数の検証はcbf_choose_cable_spheresが行います。
    settings.cable.maxExtraRadius;
catch
    error('CBF:MissingRuntimeSettings', ...
        'settingsの項目が不足しています。CBF_RuntimeSettings_Templateを参考に実行時に指定してください。');
end

%% 2. 配列サイズと空欄を検証
if numel(J)~=3 || numel(d)~=2 || numel(tmin)~=3 || numel(tmax)~=3
    error('CBF:RuntimeShape','inertiaは3、disturbanceXYは2、torqueMin/Maxは各3要素です。');
end
P = [p.massDrone;p.massLoad;p.cableLength;p.gravity;J; ...
     s.droneRadius;s.cableRadius;s.loadRadius; ...
     c.safetyMargin;ground.height;c.smoothEpsilon; ...
     d;u.thrustMin;u.thrustMax;tmax];
if numel(P)~=20 || ~isnumeric(P) || any(~isfinite(P)) || ...
   numel(u.thrustMin)~=1 || numel(u.thrustMax)~=1
    error('CBF:RuntimeMissing','必要なパラメータが空欄または非数値です。実行時に設定してください。');
end

%% 3. 物理値の有効性
if any(P([1 2 3 4 5 6 7 8 9 10 13])<=0) || P(11)<0 || ...
   u.thrustMin>=u.thrustMax || any(tmin>=tmax) || ...
   ~isfinite(ground.height)
    error('CBF:InvalidParameters','質量・慣性・長さ・半径・入力限界等を確認してください。');
end
end
