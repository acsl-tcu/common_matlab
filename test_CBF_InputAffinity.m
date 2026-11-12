function test_CBF_InputAffinity()
%TEST_CBF_INPUTAFFINITY 4入力同時作用でもh^(r)がA*nuにアフィンか。
% 生成器のAは基底入力(0,ei)の差から計算されるため、この試験で
% 一般の混合入力に対する線形性（交差項なし）を追加チェックする。
test_CBF_SetupPaths();s=test_CBF_Fixture();
P=cbf_parameter_vector(s);plan=cbf_choose_cable_spheres(s);
xR=zeros(29,1);xR(1:4)=[cos(.15);sin(.15);0;0];
xR(5:7)=[.02;-.01;.04];xR(8:10)=[.10;-.08;1.5];
qc=[.15;-.12;-1];qc=qc/norm(qc);xR(14:16)=qc;
w=[.03;.01;0];xR(17:19)=w-qc*(qc.'*w);
xR(20)=(s.physical.massDrone+s.physical.massLoad)*s.physical.gravity;
xR(24:26)=[.01;-.02;.015];
xV=zeros(20,1);xV(1:3)=[.10;-.08;1.5];
xV(4:6)=[.05;-.03;.02];xV(7:9)=[.2;-.12;0];
xV(10:12)=[.03;-.01;.02];xV(13:15)=[.01;0;-.01];
xV(16:18)=[0;.02;-.01];
Rz=[cos(.27) -sin(.27) 0;sin(.27) cos(.27) 0;0 0 1];
Rx=[1 0 0;0 cos(.31) -sin(.31);0 sin(.31) cos(.31)];
Ry=[cos(-.24) 0 sin(-.24);0 1 0;-sin(-.24) 0 cos(-.24)];
obsS=struct('center',[1.2;-.4;1.6],'radius',.20);
obsE=struct('center',[1.2;-.4;1.6],'axes',[.17;.32;.23],'R',Rz*Rx*Ry);
inputs={[0.6;-0.8;0.1;0.2],[-0.1;0.3;-0.4;0.7]};
for im=1:2
    if im==1,mode='real';x=xR;else,mode='virtual';x=xV;end
    for sh={'C','B','D'}
        for typ={'sphere','ellipsoid'}
            if strcmp(typ{1},'sphere'),obs=obsS;else,obs=obsE;end
            [~,info,qp]=cbf_for_case(mode,sh{1},typ{1},x,s,obs);
            O=cbf_obstacle_vector(obs,typ{1});
            H0=evalJets(zeros(4,1));
            r=info.orderHint;
            if strcmp(mode,'real'),r=6*ones(numel(r),1);end
            for k=1:numel(inputs)
                nu=inputs{k}; H=evalJets(nu);
                for i=1:numel(r)
                    dr=factorial(r(i))*(H(i,r(i)+1)-H0(i,r(i)+1));
                    pred=qp.A(i,:)*nu;
                    assert(abs(dr-pred)<1e-7*(1+abs(dr)+abs(pred)), ...
                        '混合入力のアフィン性が不成立: %s %s %s row=%d', ...
                        mode,sh{1},typ{1},i);
                end
            end
        end
    end
end
fprintf(' [PASS] 12構成: 同時4入力の高階作用は基底差分係数と一致\n');

    function H=evalJets(nu)
        K=6;
        if strcmp(mode,'real')
            z=cbf_real_state_taylor(x,nu,P,K);
            lp=z(8:10,:);qc2=z(14:16,:);pd=lp-P(3)*qc2;
        else
            [lp,qc2,pd]=cbf_virtual_state_taylor(x,nu,P,K);
        end
        H=cbf_geometry_taylor(sh{1},typ{1},lp,qc2,pd,P,O,plan);
    end
end

function [h,info,qp]=cbf_for_case(mode,shape,kind,x,s,obs)
fn=str2func(sprintf('CBF_%s_%s_%s',mode,shape,kind));
[h,info,qp]=fn(x,s,obs);
end
