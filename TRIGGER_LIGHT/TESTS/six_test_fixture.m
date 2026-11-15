function [worldSphere,worldEllipsoid] = six_test_fixture(centers,axesLengths,useGround)
%SIX_TEST_FIXTURE Synthetic observed outer covers for one underlying ellipsoid
% per obstacle. NEVER used by the deployed trigger's obstacle acquisition.
if nargin<3,useGround=false;end
if isempty(centers),centers=zeros(3,0);end
num=size(centers,2);
tpl=struct('kind','sphere','center',[0 0 0],'R',eye(3), ...
    'params',struct('radius',0.2),'obstacleName','');
sp=repmat(tpl,1,num);
tpl.kind='ellipsoid';tpl.params=struct('axes',[0.2;0.2;0.2]);
el=repmat(tpl,1,num);
for i=1:num
    ax=axesLengths(:,i);
    R=rotationZ(0.31*i)*rotationY(0.17*i);
    name=sprintf('observed_%02d',i);
    sp(i).center=centers(:,i)';sp(i).params.radius=max(ax);
    sp(i).obstacleName=name;
    el(i).center=centers(:,i)';el(i).params.axes=ax;
    el(i).R=R;el(i).obstacleName=name;
end
ground=[];
if useGround
    ground=struct('kind','halfspace','forbiddenSide','negative', ...
        'normal',[0 0 1],'point',[0 0 0],'safetyMargin',0,'name','ground');
end
worldSphere=struct('parts',sp,'ground',ground,'dynamicIndices',[]);
worldEllipsoid=struct('parts',el,'ground',ground,'dynamicIndices',[]);
end
function R=rotationZ(a)
R=[cos(a),-sin(a),0;sin(a),cos(a),0;0,0,1];
end
function R=rotationY(a)
R=[cos(a),0,sin(a);0,1,0;-sin(a),0,cos(a)];
end
