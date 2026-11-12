function [h, info] = cbf_geometry_core(mode, shape, obstacleType, X, P, O, cablePlan)
%CBF_GEOMETRY_CORE  機体・ケーブル・吊り荷の3次元衝突安全関数。
%
% 【機体形状】C: 全体を球（ケーブルは被覆球列）
%             B: 機体球 + ケーブルカプセル + 吊り荷球
%             D: 機体球 + ケーブル外接楕円体 + 吊り荷球
% 【障害物】球または任意姿勢楕円体
% 【注意】h>=0 は衝突しないための十分条件。最短距離そのものではない。
%          B/D は中央から障害物中心へ向かう方向の支持関数で分離する。
%          最近接点切替を避ける代わりに保守的となる。
%          h>=0 のみで高次CBFの前方不変性は保証されない。

if strcmp(mode,'real')
    pl = X(8:10);
    pt = X(14:16);    % 機体 -> 吊り荷、単位方向ベクトル
elseif strcmp(mode,'virtual')
    pl = X(1:3);
    a = X(7:9)+P(4)*[0;0;1]-[P(14);P(15);0];
    pt = -a/sqrt(a.'*a);  % 張り・正張力のときのみ有効
else
    error('CBF:UnknownMode','realまたはvirtualを指定してください。');
end

L = P(3); rD=P(8); rc=P(9); rL=P(10);
margin=P(11); ground=P(12); epss=P(13);
pd = pl-L*pt;
center = O(1:3);
axisLength = O(4:6);
R = reshape(O(7:15),3,3);

h = empty_barriers(X);  % 0x1、数値/シンボリック両対応
labels = cell(0,1);
orders = zeros(0,1);

%% 1. 機体中心の球
[h,labels,orders] = addSphere(h,labels,orders,pd,rD,'drone',4, ...
    center,axisLength,R,margin,epss,obstacleType);

%% 2. 吊り荷中心の球
[h,labels,orders] = addSphere(h,labels,orders,pl,rL,'load',6, ...
    center,axisLength,R,margin,epss,obstacleType);

%% 3. ケーブル全区間（3方式の違い）
switch upper(shape)
    case 'C'
        % 等間隔に配置した球によって有限半径ケーブルを完全に被覆する。
        for k=0:cablePlan.numberOfSegments
            s=k/cablePlan.numberOfSegments;
            pc=pd+s*(pl-pd);
            label=sprintf('cable_sphere_%02d',k+1);
            if k==cablePlan.numberOfSegments, degree=6; else, degree=4; end
            [h,labels,orders] = addSphere(h,labels,orders,pc, ...
                cablePlan.sphereRadius,label,degree,center,axisLength,R, ...
                margin,epss,obstacleType);
        end
    case 'B'
        % カプセルの支持関数による分離条件（滑らか、保守的）。
        c=(pd+pl)/2; n=center-c;
        nNorm=sqrt(n.'*n+epss^2);
        supportCable=(L/2)*sqrt((pt.'*n)^2+epss^2)+rc*nNorm;
        supportObs=obstacleSupport(n,axisLength,R,epss,obstacleType);
        h=[h;n.'*n-supportCable-supportObs-margin*nNorm];
        labels{end+1,1}='cable_capsule'; orders(end+1,1)=4;
    case 'D'
        % カプセルを必ず含む回転楕円体でケーブルを被覆する。
        % a=L/2+rc, b=sqrt(a*rc) とすれば支持関数で包含が成り立つ。
        c=(pd+pl)/2; n=center-c;
        a=L/2+rc; b=sqrt(a*rc);
        Q=b^2*eye(3)+(a^2-b^2)*(pt*pt.');
        nNorm=sqrt(n.'*n+epss^2);
        supportCable=sqrt(n.'*Q*n+epss^2);
        supportObs=obstacleSupport(n,axisLength,R,epss,obstacleType);
        h=[h;n.'*n-supportCable-supportObs-margin*nNorm];
        labels{end+1,1}='cable_ellipsoid'; orders(end+1,1)=4;
    otherwise
        error('CBF:UnknownShape','形状はC、B、Dのいずれかです。');
end

%% 4. 地面安全関数（機体・吊り荷は独立）
h=[h;pd(3)-rD-ground-margin;pl(3)-rL-ground-margin];
labels=[labels;{'ground_drone';'ground_load'}];
orders=[orders;4;6];
if upper(shape)=='C'
    for k=0:cablePlan.numberOfSegments
        s=k/cablePlan.numberOfSegments;
        pc=pd+s*(pl-pd);
        h=[h;pc(3)-cablePlan.sphereRadius-ground-margin];
        labels{end+1,1}=sprintf('ground_cable_sphere_%02d',k+1);
        if k==cablePlan.numberOfSegments, deg=6; else, deg=4;end
        orders(end+1,1)=deg;
    end
elseif upper(shape)=='B'
    h=[h;pd(3)-rc-ground-margin;pl(3)-rc-ground-margin];
    labels=[labels;{'ground_cable_drone_end';'ground_cable_load_end'}];
    orders=[orders;4;6];
else
    c=(pd+pl)/2;
    h=[h;c(3)-sqrt(Q(3,3))-ground-margin];
    labels{end+1,1}='ground_cable_ellipsoid';
    orders(end+1,1)=4;
end

%% 5. メタ情報
if strcmp(mode,'real')
    orders(:)=NaN; % 非線形の実入力系で一律の相対次数は成立しない。
end
info=struct('mode',mode,'shape',upper(shape),'obstacle',obstacleType, ...
            'labels',{labels},'orderHint',orders, ...
            'numberOfCableSpheres',cablePlan.numberOfSpheres, ...
            'cableSphereRadius',cablePlan.sphereRadius, ...
            'note','次数は仮想方式の参考次数。入力係数が退化する場合あり。h>=0のみでHOCBFは未保証。');
end

function [h,labels,orders] = addSphere(h,labels,orders,pc,r,label,degree,cen,axis,R,margin,epss,kind)
n=cen-pc;
nNorm=sqrt(n.'*n+epss^2);
% n方向で球+障害物を分離する安全側の条件。
h=[h;n.'*n-(r+margin)*nNorm-obstacleSupport(n,axis,R,epss,kind)];
labels{end+1,1}=['obstacle_' label];orders(end+1,1)=degree;
end

function s=obstacleSupport(n,axes,R,epss,kind)
if strcmp(kind,'sphere')
    s=axes(1)*sqrt(n.'*n+epss^2);
else
    z=R.'*n;
    s=sqrt(sum((axes.^2).*(z.^2))+epss^2);
end
end

function y=empty_barriers(x)
if isa(x,'sym'),y=sym(zeros(0,1));else,y=zeros(0,1);end
end
