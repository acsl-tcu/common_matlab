function test_CBF_ObstacleStress()
%TEST_CBF_OBSTACLESTRESS SO(3)姿勢変更・複数障害物の整合性。
test_CBF_SetupPaths();s=test_CBF_Fixture();rng(213);
xV=zeros(20,1);xV(1:3)=[0;0;1.5];
xR=zeros(29,1);xR(1)=1;xR(8:10)=xV(1:3);xR(14:16)=[0;0;-1];
xR(20)=(s.physical.massDrone+s.physical.massLoad)*s.physical.gravity;
shapes={'C','B','D'};
for i=1:24
    a=randn(3,3);[R,Q]=qr(a);R=R*diag(sign(diag(Q)));
    if det(R)<0,R(:,1)=-R(:,1);end
    obs=struct('center',[0.7;0;1.4]+0.1*randn(3,1), ...
      'axes',[0.14;0.27;0.38], 'R',R,'type','ellipsoid');
    for j=1:3
        hV=cbf_evaluate_case('virtual',shapes{j},'ellipsoid',xV,s,obs);
        hR=cbf_evaluate_case('real',shapes{j},'ellipsoid',xR,s,obs);
        assert(all(isfinite(hV)) && all(isfinite(hR)));
        assert(norm(hV-hR,Inf)<1e-8*(1+norm(hV,Inf)));
    end
end
% 球と楕円体の混合環境でも地面は一度しか出ないことを確認する。
obsS=struct('type','sphere','center',[0.9;-0.2;1.6],'radius',0.16);
obsE=struct('type','ellipsoid','center',[1.4;0.1;1.8], ...
            'axes',[0.18;0.26;0.35],'R',R);
for j=1:3
    for im=1:2
        if im==1,mode='real';x=xR;else,mode='virtual';x=xV;end
        [h,labels]=cbf_evaluate_scene(mode,shapes{j},x,s,{obsS,obsE});
        qp=cbf_hocbf_scene(mode,shapes{j},x,s,{obsS,obsE});
        assert(numel(h)==numel(qp.earlyInputResidual));
        assert(numel(qp.b)==numel(qp.labels));
        assert(all(isfinite(qp.A(:))) && all(isfinite(qp.b)));
        assert(sum(~cellfun(@isempty,strfind(labels,'ground_')))== ...
               sum(~cellfun(@isempty,strfind(qp.labels,'ground_'))));
        assert(sum(~cellfun(@isempty,strfind(labels,'ground_')))>0);
    end
end
fprintf(' [PASS] ランダム回転24姿勢×C/B/D・球+楕円体の複数障害物\n');
end
