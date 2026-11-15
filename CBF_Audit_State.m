function out=CBF_Audit_State(action,varargin)
% CBF_AUDIT_STATE: 今回の試験に限って使う状態・参照軌道・障害物の作成。
% 既存CBF本体や生成器からは呼ばない。質量は設定sのmassDrone / massLoadのみ。
% massLoadは吊り荷の推定質量として扱い、機体質量を変更しない。
% 確認用の「真の質量」との不一致や確率的ノイズは導入しない。
switch lower(action)
    case 'real'
        % s, 荷物高度, 荷物上下速度, 機体roll/pitch, ケーブル傾き,
        % 荷物水平速度, 機体/ケーブル角速度スケール
        s=varargin{1}; z=varargin{2}; vz=varargin{3};
        roll=varargin{4}; pitch=varargin{5}; tilt=varargin{6};
        vx=varargin{7}; ws=varargin{8};
        yaw=0; if numel(varargin)>=9, yaw=varargin{9}; end
        x=zeros(29,1);
        q=localQuat(roll,pitch,yaw);
        x(1:4)=q;
        x(5:7)=ws*[0.09;-0.07;0.04];
        x(8:10)=[0;0;z];
        x(11:13)=[vx;0;vz];
        cable=[sin(tilt);0;-cos(tilt)];
        x(14:16)=cable;
        wc=ws*[0.06;-0.04;0.02];
        x(17:19)=wc-cable*(cable.'*wc);
        b=localRot(q)*[0;0;1];
        qcDot=cross(x(17:19),cable);
        % 「瞬間的な吊り荷z加速度=0」を満たす推力を用いる。
        % 傾いた姿勢では全方向の平衡条件ではないことに注意。
        m=s.physical.massDrone; ml=s.physical.massLoad;
        L=s.physical.cableLength;g=s.physical.gravity;
        denominator=dot(cable,b);
        if abs(denominator)<0.1
            error('CBF:AuditPosture','姿勢で推力方向とケーブルがほぼ直交します。');
        end
        T=((m+ml)*g/cable(3) + m*L*dot(qcDot,qcDot))/denominator;
        if T<s.input.thrustMin || T>s.input.thrustMax
            error('CBF:AuditThrust','初期の推力が実入力限界外です。');
        end
        x(20)=T;
        % 推力・トルクの拡張微分状態は0、トルク自体も0。
        % 非平衡姿勢は運動を始めるが、架空の平衡入力は使わない。
        out=x;
    case 'virtualfromreal'
        xR=varargin{1}; s=varargin{2};
        P=cbf_parameter_vector(s);
        Z=cbf_real_state_taylor(xR,zeros(4,1),P,5);
        x=zeros(20,1);
        for j=0:5
            x(3*j+1:3*j+3)=factorial(j)*Z(8:10,j+1);
        end
        q=xR(1:4);qd=Z(1:4,2);
        a=2*(q(1)*q(4)+q(2)*q(3));
        b=q(1)^2+q(2)^2-q(3)^2-q(4)^2;
        ad=2*(qd(1)*q(4)+q(1)*qd(4)+qd(2)*q(3)+q(2)*qd(3));
        bd=2*(q(1)*qd(1)+q(2)*qd(2)-q(3)*qd(3)-q(4)*qd(4));
        x(19)=atan2(a,b);
        x(20)=(b*ad-a*bd)/(a*a+b*b);
        out=x;
    case 'obstacle'
        kind=char(varargin{1}); p=varargin{2}; dist=varargin{3};
        direction=char(varargin{4}); angle=varargin{5};
        switch direction
            case 'front', unit=[1;0;0];
            case 'side',  unit=[0;1;0];
            case 'above', unit=[0;0;1];
            case 'below', unit=[0;0;-1];
            otherwise,error('CBF:AuditDirection','未知の侵入方向');
        end
        center=p+dist*unit;
        if strcmp(kind,'sphere')
            out=struct('type','sphere','center',center,'radius',.14);
        elseif strcmp(kind,'ellipsoid')
            R=localRz(angle)*localRy(-0.7*angle)*localRx(0.5*angle);
            out=struct('type','ellipsoid','center',center, ...
                       'axes',[.13;.20;.29],'R',R);
        else
            error('CBF:AuditObstacle','sphereかellipsoidのみ');
        end
    case 'reference'
        name=char(varargin{1});t=varargin{2};k=varargin{3};
        if ~isscalar(k) || k~=floor(k) || k<0 || k>6
            error('CBF:AuditDerivative','微分次数は0～6');
        end
        switch name
            case 'hover'
                out=[0;0;1.5]*(k==0);
            case 'line'
                if k==0,out=[.30*t;0;1.5];
                elseif k==1,out=[.30;0;0];
                else,out=zeros(3,1);end
            case 'circle'
                w=.7;
                out=[.25*w^k*cos(w*t+k*pi/2); ...
                     .25*w^k*sin(w*t+k*pi/2);1.5*(k==0)];
            case 'ascent'
                w=.75;
                out=[0;0;1.5*(k==0)+.15*w^k*sin(w*t+k*pi/2)];
            case 'figure8'
                w=.6;
                out=[.25*w^k*sin(w*t+k*pi/2); ...
                     .16*(2*w)^k*sin(2*w*t+k*pi/2);1.5*(k==0)];
            otherwise,error('CBF:AuditReference','軌道が未知です');
        end
    case 'rk4real'
        x=varargin{1};nu=varargin{2};dt=varargin{3};s=varargin{4};
        k1=test_CBF_IndependentPlantRHS(x,nu,s);
        k2=test_CBF_IndependentPlantRHS(x+.5*dt*k1,nu,s);
        k3=test_CBF_IndependentPlantRHS(x+.5*dt*k2,nu,s);
        k4=test_CBF_IndependentPlantRHS(x+dt*k3,nu,s);
        out=x+dt*(k1+2*k2+2*k3+k4)/6;
    case 'stepvirtual'
        x=varargin{1};nu=varargin{2};dt=varargin{3};
        out=zeros(20,1);
        for j=0:5
            for ell=j:5
                out(3*j+1:3*j+3)=out(3*j+1:3*j+3)+ ...
                    x(3*ell+1:3*ell+3)*dt^(ell-j)/factorial(ell-j);
            end
            out(3*j+1:3*j+3)=out(3*j+1:3*j+3)+ ...
                nu(1:3)*dt^(6-j)/factorial(6-j);
        end
        out(19)=x(19)+dt*x(20)+dt^2*nu(4)/2;
        out(20)=x(20)+dt*nu(4);
    otherwise
        error('CBF:AuditStateAction','不明な操作');
end
end
function q=localQuat(r,p,y)
cr=cos(r/2);sr=sin(r/2);cp=cos(p/2);sp=sin(p/2);
cy=cos(y/2);sy=sin(y/2);
q=[cr*cp*cy+sr*sp*sy; sr*cp*cy-cr*sp*sy; ...
   cr*sp*cy+sr*cp*sy;cr*cp*sy-sr*sp*cy];
end
function R=localRot(q)
w=q(1);v=q(2:4);
R=(w*w-dot(v,v))*eye(3)+2*v*v.'+2*w*[0 -v(3) v(2);v(3) 0 -v(1);-v(2) v(1) 0];
end
function R=localRx(t),R=[1 0 0;0 cos(t) -sin(t);0 sin(t) cos(t)];end
function R=localRy(t),R=[cos(t) 0 sin(t);0 1 0;-sin(t) 0 cos(t)];end
function R=localRz(t),R=[cos(t) -sin(t) 0;sin(t) cos(t) 0;0 0 1];end
