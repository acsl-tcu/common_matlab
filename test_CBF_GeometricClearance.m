function test_CBF_GeometricClearance()
%TEST_CBF_GEOMETRICCLEARANCE 楕円体までの厳密な点距離で幾何十分条件を独立検査。
% 生成器の支持関数実装は使わず、点-楕円体最近接距離を二分法で解く。
% 安全側の判定: 全障害物CBF h>=0なら機体球/荷物球/紐円筒の距離を満たす。
test_CBF_SetupPaths();s=test_CBF_Fixture();rng(73);
xR=zeros(29,1);xR(1)=1;xR(8:10)=[0;0;1.4];
qCable=[.15;-.12;-1];qCable=qCable/norm(qCable);xR(14:16)=qCable;
xR(20)=(s.physical.massDrone+s.physical.massLoad)*s.physical.gravity;
pL=xR(8:10);pD=pL-s.physical.cableLength*qCable;
margin=s.cbf.safetyMargin;
checks=0;
for k=1:45
    u=randn(3,3);[R,Q]=qr(u);R=R*diag(sign(diag(Q)));
    if det(R)<0,R(:,2)=-R(:,2);end
    o=struct('center',[0.5+rand();-.45+0.9*rand();1.2+0.7*rand()], ...
       'axes',[0.08+0.2*rand();0.10+0.2*rand();0.12+0.25*rand()], ...
       'R',R);
    for shape={'C','B','D'}
        [h,info]=cbf_evaluate_case('real',shape{1},'ellipsoid',xR,s,o);
        geo=~strncmp(info.labels,'ground_',7);
        if all(h(geo)>=0)
            dD=pointEllipsoidDistance(pD,o);dL=pointEllipsoidDistance(pL,o);
            assert(dD>=s.shape.droneRadius+margin-1e-7);
            assert(dL>=s.shape.loadRadius+margin-1e-7);
            % 中心線上のどの点も、外周半径と安全余裕を下回らない。
            for j=0:50
                p=pD+(j/50)*(pL-pD);
                dc=pointEllipsoidDistance(p,o);
                assert(dc>=s.shape.cableRadius+margin-1e-7, ...
                    'h>=0なのに、紐のサンプリング点が楕円体に近すぎます。');
            end
            checks=checks+1;
        end
    end
end
assert(checks>=10,'独立距離を照合できる安全配置が少なすぎます。');
fprintf(' [PASS] 任意姿勢楕円体: h>=0の%d件で実距離が形状+余裕を満たす\n',checks);
end

function d=pointEllipsoidDistance(p,obs)
% 点-3D楕円体までのEuclidean距離。内部点なら0。
z=obs.R.'*(p(:)-obs.center(:));a=obs.axes(:);
if sum((z./a).^2)<=1,d=0;return;end
f=@(u)sum((a.^2.*z.^2)./(a.^2+u).^2)-1;
lo=0;hi=max(a.^2);
for k=1:80
    if f(hi)<=0,break;end
    hi=hi*2;
end
for k=1:80
    mid=(lo+hi)/2;
    if f(mid)>0,lo=mid;else,hi=mid;end
end
y=a.^2.*z./(a.^2+(hi+lo)/2);
d=norm(z-y);
end
