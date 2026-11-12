function test_CBF_Geometry()
%TEST_CBF_GEOMETRY  実入力・仮想入力と3形状×2障害物の幾何一致。
test_CBF_SetupPaths();
s=test_CBF_Fixture();
if exist('CBF_real_C_sphere','file')~=2
    generateCBF_RealInput(); generateCBF_VirtualInput();
end
obsS=struct('center',[2;0;1.2],'radius',0.25);
theta=pi/5;
R=[cos(theta),-sin(theta),0;sin(theta),cos(theta),0;0,0,1];
obsE=struct('center',[2;0;1.2],'axes',[0.25;0.38;0.50],'R',R);

% プラント平衡点: 吊り荷(0,0,1)、ケーブル鉛直、機体は吊り荷の上。
xr=zeros(29,1);xr(1)=1;xr(8:10)=[0;0;1];xr(14:16)=[0;0;-1];
xr(20)=(s.physical.massDrone+s.physical.massLoad)*s.physical.gravity;
% 理想平坦出力: pL^(2)=0、ケーブル方向は重力から復元。
xv=zeros(20,1);xv(1:3)=xr(8:10);
shapes={'C','B','D'};types={'sphere','ellipsoid'};

for i=1:numel(shapes)
    for j=1:numel(types)
        shape=shapes{i};kind=types{j};
        if strcmp(kind,'sphere'),obs=obsS;else,obs=obsE;end
        hRealFcn=str2func(sprintf('CBF_real_%s_%s',shape,kind));
        hVirtFcn=str2func(sprintf('CBF_virtual_%s_%s',shape,kind));
        [hr,infoR]=hRealFcn(xr,s,obs);
        [hv,infoV]=hVirtFcn(xv,s,obs);
        assert(all(isfinite(hr)) && all(isfinite(hv)));
        assert(isequal(size(hr),size(hv)));
        assert(norm(hr-hv,Inf)<1e-8,'実入力と仮想入力で幾何が一致しません。');
        assert(isequal(infoR.labels,infoV.labels),'安全関数ラベルが一致しません。');
        assert(all(hr>0),'この試験の安全な初期配置でhが正でありません。');
        fprintf('[PASS] %s + %s: %d個の幾何・地面条件\n',shape,kind,numel(hr));
    end
end

% 張力と平坦出力の復元が理想ホバリングで整合すること。
dReal=cbf_model_diagnostics('real',xr,s);
dVirt=cbf_model_diagnostics('virtual',xv,s);
assert(abs(dReal.tension-s.physical.massLoad*s.physical.gravity)<1e-9);
assert(abs(dVirt.tensionUnderTautAssumption-s.physical.massLoad*s.physical.gravity)<1e-9);
end
