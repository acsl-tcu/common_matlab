function test_CBF_CableSpheres()
%TEST_CBF_CABLESPHERES  可変ケーブル球数と被覆保証を検査。
test_CBF_SetupPaths();
s=test_CBF_Fixture();
a=cbf_choose_cable_spheres(s);
assert(a.numberOfSpheres>=2);
assert(a.extraRadius<=s.cable.maxExtraRadius+1e-12);

% ケーブルが長い場合は、同じ許容膨張に対して球数が増える。
sLong=s;sLong.physical.cableLength=2*s.physical.cableLength;
b=cbf_choose_cable_spheres(sLong);
assert(b.numberOfSpheres>a.numberOfSpheres);

% ケーブル半径方向の表面点について、球列がすべてを覆うか。
L=s.physical.cableLength;r=s.shape.cableRadius;
for t=linspace(0,L,401)
    for theta=linspace(0,2*pi,21)
        p=[r*cos(theta);r*sin(theta);t];
        centers=[zeros(2,a.numberOfSpheres);linspace(0,L,a.numberOfSpheres)];
        distances=sqrt(sum(bsxfun(@minus,centers,p).^2,1));
        assert(min(distances)<=a.sphereRadius+1e-11, ...
            'ケーブル表面が球列からはみ出しています。');
    end
end

% 手動で少なすぎる球数はエラー（黙って隙間を許さない）。
sBad=s;sBad.cable.numberOfSpheres=2;
failed=false;
try
    cbf_choose_cable_spheres(sBad);
catch
    failed=true;
end
assert(failed,'球数不足でエラーになりませんでした。');

fprintf('[PASS] ケーブル球数: %d、間隔 %.5f m、球半径 %.5f m、追加膨張 %.5f m\n', ...
    a.numberOfSpheres,a.centerSpacing,a.sphereRadius,a.extraRadius);
end
