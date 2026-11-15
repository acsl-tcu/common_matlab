function test_CBF_CableSpheres()
%TEST_CBF_CABLESPHERES 機体・吊り荷球を含む厳密被覆と最少球数を検証。
% 元のケーブルに置く「内部球数」は可変、端の球は重複配置しない。

test_CBF_SetupPaths();
s=test_CBF_Fixture();
plan=cbf_choose_cable_spheres(s);
verifyCoverage(s,plan);
assert(plan.totalSystemSpheres==plan.numberOfSpheres+2);

% 同じ球半径のままケーブルが長くなれば必要球数は非減少。
sl=s;sl.physical.cableLength=2*s.physical.cableLength;
longPlan=cbf_choose_cable_spheres(sl);
verifyCoverage(sl,longPlan);
assert(longPlan.numberOfSpheres>=plan.numberOfSpheres);

% 外部から球半径を指定したときは、指定半径を厳密に維持する。
se=s;se.cable.sphereRadius=0.035;
se.cable.radiusRatioToSmallBody=[];
ex=cbf_choose_cable_spheres(se);
verifyCoverage(se,ex);
assert(abs(ex.sphereRadius-0.035)<1e-14);

% 最少球数より1個少ないものを指定しても黙って隙間は開けない。
if ex.numberOfSpheres>0
    bad=se;bad.cable.numberOfSpheres=ex.numberOfSpheres-1;
    caught=false;
    try,cbf_choose_cable_spheres(bad);catch,caught=true;end
    assert(caught,'被覆できない球数を許可しました。');
end

% 機体・吊り荷の球だけで短いケーブルを覆える場合は追加球0個。
ss=s;ss.physical.cableLength=0.1;
zero=cbf_choose_cable_spheres(ss);
verifyCoverage(ss,zero);
assert(zero.numberOfSpheres==0);

% 異なる姿勢でも球の相対配置が回転するだけで被覆は不変。
for i=1:10
    a=[sin(i);cos(2*i);sin(3*i)];a=a/norm(a);
    verifyCoverage(s,plan,a);
end
fprintf('[PASS] C方式: 内部%d球＋機体/吊り荷2球=合計%d球、半径%.4f m\n', ...
    plan.numberOfSpheres,plan.totalSystemSpheres,plan.sphereRadius);
fprintf('[PASS] 外部半径0.035m：内部%d球。短いケーブルは内部球0個で被覆\n', ...
    ex.numberOfSpheres);
end

function verifyCoverage(s,plan,axisDirection)
if nargin<3,axisDirection=[0;0;1];end
axisDirection=axisDirection(:)/norm(axisDirection);
L=s.physical.cableLength;rc=s.shape.cableRadius;
rD=s.shape.droneRadius;rL=s.shape.loadRadius;
cs=plan.centerDistances;rs=plan.sphereRadius;
anchor=[1;0;0];
if abs(dot(anchor,axisDirection))>0.9,anchor=[0;1;0];end
e1=cross(axisDirection,anchor);e1=e1/norm(e1);
e2=cross(axisDirection,e1);
centers=axisDirection*cs;
% 真の3D位置で、ケーブル表面と円柱端キャップを含めて被覆確認。
for t=linspace(0,L,403)
    for angle=linspace(0,2*pi,19)
        p=t*axisDirection+rc*(cos(angle)*e1+sin(angle)*e2);
        nearest=min([norm(p),norm(p-L*axisDirection)]); %#ok<NASGU>
        ok=(norm(p)<=rD+1e-11) || ...
           (norm(p-L*axisDirection)<=rL+1e-11);
        if ~isempty(cs)
            ok=ok || any(sqrt(sum((centers-p).^2,1))<=rs+1e-11);
        end
        assert(ok,'CBF:CableCoverage','3Dでケーブルを被覆できていません。');
    end
end
assert(all(plan.centerFractions>=0) && all(plan.centerFractions<=1));
assert(numel(plan.centerFractions)==plan.numberOfSpheres);
end
