function test_CBF_SceneEquivalence()
%TEST_CBF_SCENEEQUIVALENCE 重複計算除去後のQPと従来の全行計算を比較する。
% ①重複計算だけを省いた状態: 全行A,b,psiを一致確認。
% ②数学的に従う地面行だけを省いた状態: 残存行の係数一致と冗長関係確認。
% テスト専用の数値はtest_CBF_Fixtureから取得する。
test_CBF_SetupPaths(); s=test_CBF_Fixture();
base=buildScene(3);
xR=zeros(29,1);xR(1)=1;xR(8:10)=[0;0;1.4];xR(14:16)=[0;0;-1];
xR(20)=(s.physical.massDrone+s.physical.massLoad)*s.physical.gravity;
xV=zeros(20,1);xV(1:3)=[0;0;1.4];
for modeCell={'real','virtual'}
    mode=modeCell{1};
    if strcmp(mode,'real'), x=xR; else, x=xV; end
    for shapeCell={'C','B','D'}
        shape=shapeCell{1};
        s.cbf.pruneRedundantGround=false;
        slow=localSlowScene(mode,shape,x,s,base);
        fast=cbf_hocbf_scene(mode,shape,x,s,base);
        assert(isequal(slow.labels,fast.labels),'制約名/順序が不一致です。');
        checkValues(slow,fast,mode,shape);
        s.cbf.pruneRedundantGround=true;
        reduced=cbf_hocbf_scene(mode,shape,x,s,base);
        for i=1:numel(reduced.labels)
            j=find(strcmp(slow.labels,reduced.labels{i}),1);
            assert(~isempty(j),'残した制約が元の制約にありません。');
            assert(norm(slow.A(j,:)-reduced.A(i,:),Inf)<1e-8*(1+norm(slow.A(j,:),Inf)));
            assert(abs(slow.b(j)-reduced.b(i))<1e-8*(1+abs(slow.b(j))));
        end
        % Bの端点地面行は、より半径の大きい機体・荷物地面CBFから従う。
        if strcmp(shape,'B') && s.shape.droneRadius>=s.shape.cableRadius
            checkDominance(slow,'ground_drone','ground_cable_drone_end');
        end
        if strcmp(shape,'B') && s.shape.loadRadius>=s.shape.cableRadius
            checkDominance(slow,'ground_load','ground_cable_load_end');
        end
        % C(real)は4/6の混合がないため、固定点地面は両端凸結合から従う。
        plan=cbf_choose_cable_spheres(s);
        if strcmp(shape,'C') && strcmp(mode,'real') && ...
           s.shape.droneRadius>=plan.sphereRadius && ...
           s.shape.loadRadius>=plan.sphereRadius
            id=find(~cellfun('isempty',strfind(slow.labels,'ground_cable_sphere_'))); %#ok<STREMP>
            d=find(~cellfun('isempty',strfind(slow.labels,'ground_drone'))); %#ok<STREMP>
            l=find(~cellfun('isempty',strfind(slow.labels,'ground_load'))); %#ok<STREMP>
            for kk=1:numel(id)
                t=(kk-1)/(numel(id)-1);
                Aexpected=(1-t)*slow.A(d(1),:)+t*slow.A(l(1),:);
                bexpected=(1-t)*slow.b(d(1))+t*slow.b(l(1));
                assert(norm(slow.A(id(kk),:)-Aexpected,Inf)<1e-6*(1+norm(Aexpected,Inf)));
                assert(slow.b(id(kk))<=bexpected+1e-6*(1+abs(bexpected)));
            end
        end
        fprintf('[PASS] %7s %s : 旧%d行→新%d行（同一制約／証明済み冗長行のみ削減）\n', ...
            mode,shape,numel(slow.b),numel(reduced.b));
    end
end
end
function checkDominance(qp,base,secondary)
% ground_drone と ground_cable_drone_end の完全な接尾名を使う。
a=find(~cellfun('isempty',strfind(qp.labels,['_' base]))); %#ok<STREMP>
b=find(~cellfun('isempty',strfind(qp.labels,['_' secondary]))); %#ok<STREMP>
assert(~isempty(a)&&~isempty(b)); a=a(1);b=b(1);
assert(norm(qp.A(a,:)-qp.A(b,:),Inf)<1e-8*(1+norm(qp.A(a,:),Inf)));
assert(qp.b(a)>=qp.b(b)-1e-8*(1+abs(qp.b(a))));
end
function checkValues(one,two,mode,shape)
assert(isequal(size(one.A),size(two.A)), '行数が変わりました。%s %s',mode,shape);
assert(norm(one.A(:)-two.A(:),Inf)<1e-8*(1+norm(one.A(:),Inf)));
assert(norm(one.b(:)-two.b(:),Inf)<1e-8*(1+norm(one.b(:),Inf)));
end
function qp=localSlowScene(mode,shape,x,s,obstacles)
qp=[];
for j=1:numel(obstacles)
    obs=obstacles{j};
    [h,info]=cbf_evaluate_case(mode,shape,obs.type,x,s,obs);
    all=cbf_hocbf_evaluate(mode,shape,obs.type,x,s,obs,h,info);
    if j==1
        qp=all;
        qp.labels=cellfun(@(v)sprintf('obs%d_%s',j,v),all.labels,'UniformOutput',false);
    else
        keep=~strncmp(all.labels,'ground_',7) & ...
             ~strncmp(all.labels,'thrust_',7) & ~strncmp(all.labels,'tau_',4);
        qp.A=[qp.A;all.A(keep,:)];qp.b=[qp.b;all.b(keep)];
        qp.labels=[qp.labels;cellfun(@(v)sprintf('obs%d_%s',j,v), ...
            all.labels(keep),'UniformOutput',false)];
    end
end
end
function obs=buildScene(n)
obs=cell(n,1);
for j=1:n
    center=[2+0.8*j;(-1)^j*0.2;1.7];
    if mod(j,2)
        obs{j}=struct('type','sphere','center',center,'radius',0.14);
    else
        a=0.21*j;b=0.13*j;
        Rz=[cos(a) -sin(a) 0;sin(a) cos(a) 0;0 0 1];
        Rx=[1 0 0;0 cos(b) -sin(b);0 sin(b) cos(b)];
        obs{j}=struct('type','ellipsoid','center',center,'axes',[0.15;0.2;0.28],'R',Rz*Rx);
    end
end
end
