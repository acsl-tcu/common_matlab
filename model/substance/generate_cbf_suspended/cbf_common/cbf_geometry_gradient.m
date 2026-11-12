function [Gd,Gl,Gq] = cbf_geometry_gradient(shape,kind,pL,qCable,pDrone,P,O,plan,section)
%CBF_GEOMETRY_GRADIENT 既存の幾何安全関数hの空間勾配（解析式）。
% 返り値は各行について [dh/dpDrone, dh/dpLoad, dh/dqCable]。
% 対象: 静止球・任意姿勢楕円体と形状C/B/D、地面。
% ※ pDrone,pLoad,qCableを形式上独立な幾何変数として偏微分する。
% ※ 高次CBFの係数そのものはcbf_hocbf_evaluateで作る。
% ※ 最短距離へ置き換えたり、保守性やゲインを変更する処理ではない。
if nargin<9 || isempty(section), section='all'; end
if ~any(strcmp(section,{'all','obstacles'}))
    error('CBF:GeometrySection','sectionはallまたはobstaclesです。');
end
pL=pL(:);qCable=qCable(:);pDrone=pDrone(:);
L=P(3);rD=P(8);rc=P(9);rL=P(10);margin=P(11);epss=P(13);
center=O(1:3);axes=O(4:6);R=reshape(O(7:15),3,3);
if strcmp(kind,'ellipsoid')
    Q=R*diag(axes.^2)*R.'; % 1障害物につき1回だけ回転楕円体行列を作る
else
    Q=[];
end
Gd=zeros(0,3);Gl=zeros(0,3);Gq=zeros(0,3);

% 1. 機体・吊り荷（既存cbf_geometry_taylorの行順に合わせる）
[gd,gl,gq]=sphereGrad(pDrone,rD,1,0); appendRow(gd,gl,gq);
[gd,gl,gq]=sphereGrad(pL,rL,0,1); appendRow(gd,gl,gq);

% 2. ケーブル
switch upper(shape)
    case 'C'
        for k=0:plan.numberOfSegments
            s=k/plan.numberOfSegments;
            pc=(1-s)*pDrone+s*pL;
            [gd,gl,gq]=sphereGrad(pc,plan.sphereRadius,1-s,s);
            appendRow(gd,gl,gq);
        end
    case {'B','D'}
        c=(pDrone+pL)/2;n=center-c;
        normN=sqrt(n.'*n+epss^2);
        axial=qCable.'*n;
        [~,gObs]=obstacleSupportGradient(n);
        if upper(shape)=='B'
            axDen=sqrt(axial^2+epss^2);
            gradN=2*n-(L/2)*(axial/axDen)*qCable- ...
                (rc+margin)*n/normN-gObs;
            gradQ=-(L/2)*(axial/axDen)*n;
        else
            long=L/2+rc; short=sqrt(long*rc);
            delta=long^2-short^2;
            den=sqrt(short^2*(n.'*n)+delta*axial^2+epss^2);
            gradN=2*n-(short^2*n+delta*axial*qCable)/den- ...
                margin*n/normN-gObs;
            gradQ=-delta*(axial/den)*n;
        end
        appendRow(-gradN/2,-gradN/2,gradQ);
    otherwise
        error('CBF:UnknownShape','形状はC/B/Dです。');
end
if strcmp(section,'obstacles'), return; end

% 3. 地面。低階と高階で同じhの勾配を使用。
e3=[0;0;1];z=zeros(3,1);
appendRow(e3,z,z);  % ground_drone
appendRow(z,e3,z);  % ground_load
switch upper(shape)
    case 'C'
        for k=0:plan.numberOfSegments
            s=k/plan.numberOfSegments;
            appendRow((1-s)*e3,s*e3,z);
        end
    case 'B'
        appendRow(e3,z,z);
        appendRow(z,e3,z);
    case 'D'
        long=L/2+rc;short=sqrt(long*rc);delta=long^2-short^2;
        den=sqrt(short^2+delta*qCable(3)^2);
        gradQ=-delta*qCable(3)/den*e3;
        appendRow(e3/2,e3/2,gradQ);
end

    function appendRow(gd,gl,gq)
        Gd(end+1,:)=gd(:).';
        Gl(end+1,:)=gl(:).';
        Gq(end+1,:)=gq(:).';
    end
    function [gd,gl,gq]=sphereGrad(pc,r,weightD,weightL)
        n=center-pc;
        normN=sqrt(n.'*n+epss^2);
        [~,gObs]=obstacleSupportGradient(n);
        gradN=2*n-(r+margin)*n/normN-gObs;
        gp=-gradN;
        gd=weightD*gp;
        gl=weightL*gp;
        gq=zeros(3,1);
    end
    function [support,gradN]=obstacleSupportGradient(n)
        if strcmp(kind,'sphere')
            normN=sqrt(n.'*n+epss^2);
            support=axes(1)*normN;
            gradN=axes(1)*n/normN;
        elseif strcmp(kind,'ellipsoid')
            % Qは障害物の任意姿勢回転を含む正定行列
            Qn=Q*n;
            support=sqrt(n.'*Qn+epss^2);
            gradN=Qn/support;
        else
            error('CBF:ObstacleType','球または楕円体のみ対応');
        end
    end
end
