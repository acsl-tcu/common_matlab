function plan = cbf_choose_cable_spheres(settings)
%CBF_CHOOSE_CABLE_SPHERES ケーブルを覆う球数と球半径を決める。
%
% 【入力】settings.physical.cableLength  ケーブル長 L [m]
%         settings.shape.cableRadius     実際の半径 rc [m]
%         settings.cable.maxExtraRadius  許容する余分な膨張 e [m]
%         settings.cable.numberOfSpheres []なら自動、整数なら手動
%
% 【原理】隣接球中心の間隔 d = L/(n-1)。
%         ケーブルの任意点が最寄りの球中心から離れる最大距離は
%         sqrt(rc^2 + (d/2)^2)。この距離を球半径とする。
%         本方式は端点を含む等間隔配置で、指定した膨張上限を
%         満たす最小の整数球数を選ぶ（QP全体の最適球数ではない）。

L = settings.physical.cableLength;
rc = settings.shape.cableRadius;
e  = settings.cable.maxExtraRadius;
cap = settings.cable.maxNumberOfSpheres;
requested = settings.cable.numberOfSpheres;
if ~isscalar(L) || ~isfinite(L) || L <= 0 || ...
   ~isscalar(rc) || ~isfinite(rc) || rc <= 0 || ...
   ~isscalar(e) || ~isfinite(e) || e <= 0 || ...
   ~isscalar(cap) || cap < 2 || cap ~= floor(cap)
    error('CBF:InvalidCable','ケーブル長・半径・膨張上限は正、最大球数は2以上の整数にしてください。');
end

if isempty(requested)
    % rSphere <= rc+e から中心間隔の最大値を導出する。
    dMax = 2*sqrt((rc+e)^2-rc^2);
    nSegments = max(1,ceil(L/dMax));
    n = nSegments+1;
    how = 'auto';
else
    n = requested;
    if ~isscalar(n) || ~isfinite(n) || n < 2 || n ~= floor(n)
        error('CBF:InvalidSphereCount','numberOfSpheres は [] または2以上の整数です。');
    end
    nSegments = n-1;
    how = 'manual';
end
if n > cap
    error('CBF:TooManySpheres', ...
        '必要な球数 %d が上限 %d を超えました。許容膨張量を増やすか球数上限を見直してください。',n,cap);
end

spacing = L/nSegments;
rSphere = sqrt(rc^2+(spacing/2)^2);
extra = rSphere-rc;
if extra > e+1e-12
    error('CBF:CoverageTolerance', ...
        '指定球数 %d では膨張が %.6g m で許容値 %.6g m を超えます。',n,extra,e);
end

plan = struct('numberOfSpheres',n,'numberOfSegments',nSegments, ...
    'sphereRadius',rSphere,'centerSpacing',spacing,'extraRadius',extra, ...
    'centerFractions',linspace(0,1,n),'selection',how);
end
