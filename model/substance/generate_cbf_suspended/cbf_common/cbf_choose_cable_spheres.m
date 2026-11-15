function plan = cbf_choose_cable_spheres(settings)
%CBF_CHOOSE_CABLE_SPHERES 機体球・吊り荷球を利用してケーブルを最少球数で覆う。
%
% 【形状】 機体中心 (s=0)、吊り荷中心 (s=1) とも球体のまま。
%           ケーブルの内部だけに追加球を配置。両端球は再生成しない。
% 【実行時の半径指定】
%   settings.cable.sphereRadius = R;   % R [m]、指定半径を変更しない
%   または
%   settings.cable.radiusRatioToSmallBody = q; % R=q*min(rDrone,rLoad)
%   または【旧形式との互換】
%   settings.cable.maxExtraRadius=e; % R=rCable+e（新指定が空のときのみ）
% 【球数】settings.cable.numberOfSpheres=[] で最少の追加球数を自動選択。
%         整数なら指定球数で均等配置。足りない場合はエラー。
%
% 【被覆の証明】ケーブルの有限半径 rc を持つ直円柱を考える。
% 半径 r>=rc の球がケーブル方向に覆える長さの半幅は
%                w(r)=sqrt(r^2-rc^2)
% である。機体球・吊り荷球についてもこの半幅 wD,wL を使う。
% 未被覆の中心区間は [wD,L-wL]（長さ G=max(0,L-wD-wL)）。
% 半径 R の追加球が覆える区間長は2*w(R)なので、
%                nMin=ceil(G/(2*w(R)))
% が必要十分な最少球数。n個の球を残り区間に等間隔配置する。
% 円柱両端は機体球と荷物球が含む（rDrone,rLoad >=rc が必須）。
% 半球キャップを含むケーブルカプセルについても端球が被覆する。
%
% 注意：幾何学の被覆保証であり、QP可行性の保証ではありません。

%% 1. 設定を外部から受け取る
L  = settings.physical.cableLength;
rc = settings.shape.cableRadius;
rD = settings.shape.droneRadius;
rL = settings.shape.loadRadius;
c  = settings.cable;
cap = c.maxNumberOfSpheres;
requested = c.numberOfSpheres;

vals=[L,rc,rD,rL,cap];
if any(~isfinite(vals)) || any(vals(1:4)<=0) || ...
        cap<0 || cap~=floor(cap)
    error('CBF:InvalidCable','ケーブル長、半径と球数上限を確認してください。');
end
if rD<rc || rL<rc
    error('CBF:EndpointCoverage', ...
        '両端の機体球・吊り荷球がケーブル断面を含める必要があります。');
end

%% 2. 追加球の半径（生成時ではなく実行時に決定）
if isfield(c,'sphereRadius') && ~isempty(c.sphereRadius)
    R=c.sphereRadius; source='explicitRadius';
elseif isfield(c,'radiusRatioToSmallBody') && ~isempty(c.radiusRatioToSmallBody)
    q=c.radiusRatioToSmallBody;
    if ~isscalar(q) || ~isfinite(q) || q<=0
        error('CBF:RadiusRatio','半径比は正の有限スカラーで指定してください。');
    end
    R=q*min(rD,rL); source='smallerBodyRatio';
elseif isfield(c,'maxExtraRadius') && ~isempty(c.maxExtraRadius)
    R=rc+c.maxExtraRadius; source='legacyExtraRadius';
else
    error('CBF:MissingCableRadius', ...
        'sphereRadius または radiusRatioToSmallBody を実行時に指定してください。');
end
if ~isscalar(R) || ~isfinite(R) || R<=rc
    error('CBF:InvalidCableSphereRadius', ...
        'ケーブル追加球の半径は実ケーブル半径より大きくしてください。');
end

%% 3. 両端の球が覆う範囲を先に差し引く
wD=sqrt(max(0,rD*rD-rc*rc));
wL=sqrt(max(0,rL*rL-rc*rc));
w =sqrt(R*R-rc*rc);
gap=max(0,L-wD-wL);
if gap<=0
    minNumber=0;
else
    minNumber=max(1,ceil(gap/(2*w)));
end

%% 4. 最少球数または指定球数を選択
if isempty(requested)
    n=minNumber; selection='autoMinimal';
else
    n=requested;selection='manual';
    if ~isscalar(n) || ~isfinite(n) || n<0 || n~=floor(n)
        error('CBF:InvalidSphereCount','追加球数は[]または0以上の整数です。');
    end
    if n<minNumber
        error('CBF:InsufficientSpheres', ...
            '指定半径 %.6g m では追加球%d個が必要です。指定は%d個です。',R,minNumber,n);
    end
end
if n>cap
    error('CBF:TooManySpheres', ...
        '必要な追加球%d個が上限%d個を超えます。半径または上限を見直してください。',n,cap);
end

%% 5. ケーブル内部への配置。最初と最後の球は必ず端球と連続被覆
if n==0
    fractions=zeros(1,0);centers=zeros(1,0);spacing=NaN;
else
    % 未被覆区間 [wD,L-wL] を n個の等長区間に分け、その中央へ球を置く。
    if gap<=0
        centers=(1:n)*L/(n+1); % 手動球が余剰のときもケーブル内部へ配置
        interval=L;
    else
        left=wD;right=L-wL;
        interval=right-left;
        centers=left + ((1:n)-0.5)*(interval/n);
    end
    fractions=centers/L;
    if n>=2,spacing=interval/n;else,spacing=NaN;end
end

plan=struct('numberOfSpheres',n, ... % 追加の内部球だけ（両端球は含まない）
    'minimumNumberOfSpheres',minNumber,'numberOfSegments',max(0,n-1), ...
    'sphereRadius',R,'centerSpacing',spacing,'extraRadius',R-rc, ...
    'centerFractions',fractions,'centerDistances',centers, ...
    'endpointCoverage',[wD;wL], 'interiorHalfWidth',w, ...
    'uncoveredLengthBeforeInterior',gap, 'selection',selection, ...
    'radiusSource',source, 'totalSystemSpheres',n+2);
end
