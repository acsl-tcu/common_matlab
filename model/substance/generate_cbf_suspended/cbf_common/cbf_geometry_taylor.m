function H = cbf_geometry_taylor(shape,kind,pL,qCable,pDrone,P,O,plan,section)
%CBF_GEOMETRY_TAYLOR  C/B/Dの幾何安全関数を時間Taylor係数で評価する。
% pL,qCable,pDroneは3x(K+1)。各列はk階微分/k!。
% 既存cbf_geometry_coreと同じ関数・同じ行順を使用する。
% 障害物の中心・姿勢は評価区間中に一定（静的障害物）。
if nargin<9 || isempty(section), section='all'; end
if ~any(strcmp(section,{'all','obstacles'}))
    error('CBF:GeometrySection','section は all または obstacles です。');
end
K=size(pL,2)-1;
C=@(v)cbf_jet('const',v,K);
M=@(a,b)cbf_jet('mul',a,b);
D=@(a,b)cbf_jet('dot',a,b);
S=@(a)cbf_jet('sqrt',a);
L=P(3);rD=P(8);rc=P(9);rL=P(10);margin=P(11);
ground=P(12);epss=P(13);
center=O(1:3);axes=O(4:6);R=reshape(O(7:15),3,3);
H=zeros(0,K+1);

%% 1. 機体と吊り荷
H(end+1,:)=sphereRow(pDrone,rD);
H(end+1,:)=sphereRow(pL,rL);

%% 2. ケーブル（固定球列 / カプセル支持関数 / 楕円体支持関数）
switch upper(shape)
    case 'C'
        for k=0:plan.numberOfSegments
            s=k/plan.numberOfSegments;
            pc=(1-s)*pDrone+s*pL;
            H(end+1,:)=sphereRow(pc,plan.sphereRadius);
        end
    case 'B'
        c=(pDrone+pL)/2;n=C(center)-c;
        n2=D(n,n);len=S(n2+C(epss^2));axial=D(qCable,n);
        cableSupport=(L/2)*S(M(axial,axial)+C(epss^2))+rc*len;
        H(end+1,:)=n2-cableSupport-obstacleSupport(n)-margin*len;
    case 'D'
        c=(pDrone+pL)/2;n=C(center)-c;
        n2=D(n,n);len=S(n2+C(epss^2));axial=D(qCable,n);
        long=L/2+rc;short=sqrt(long*rc);
        cableSupport=S(short^2*n2+(long^2-short^2)*M(axial,axial)+C(epss^2));
        H(end+1,:)=n2-cableSupport-obstacleSupport(n)-margin*len;
    otherwise
        error('CBF:Shape','C、B、Dから指定してください。');
end

% 複数障害物で2個目以降を評価するときは、ここで終了する。
% 地面制約を計算してから捨てるのではなく、Taylor級数計算そのものを省く。
if strcmp(section,'obstacles'), return; end

%% 3. 地面（既存版と同じ行順）
H(end+1,:)=pDrone(3,:)-C(rD+ground+margin);
H(end+1,:)=pL(3,:)-C(rL+ground+margin);
if upper(shape)=='C'
    for k=0:plan.numberOfSegments
        s=k/plan.numberOfSegments;
        pc=(1-s)*pDrone+s*pL;
        H(end+1,:)=pc(3,:)-C(plan.sphereRadius+ground+margin);
    end
elseif upper(shape)=='B'
    H(end+1,:)=pDrone(3,:)-C(rc+ground+margin);
    H(end+1,:)=pL(3,:)-C(rc+ground+margin);
else
    c=(pDrone+pL)/2;
    long=L/2+rc;short=sqrt(long*rc);
    vertSupport=S(short^2*C(1)+(long^2-short^2)*M(qCable(3,:),qCable(3,:)));
    H(end+1,:)=c(3,:)-vertSupport-C(ground+margin);
end

    function h = sphereRow(pc,r)
        n=C(center)-pc;
        n2=D(n,n);
        nNorm=S(n2+C(epss^2));
        h=n2-(r+margin)*nNorm-obstacleSupport(n);
    end

    function support = obstacleSupport(n)
        if strcmp(kind,'sphere')
            support=axes(1)*S(D(n,n)+C(epss^2));
        elseif strcmp(kind,'ellipsoid')
            nr=R.'*n;
            support=S(D(axes.*nr,axes.*nr)+C(epss^2));
        else
            error('CBF:ObstacleType','球または楕円体のみ対応します。');
        end
    end
end
