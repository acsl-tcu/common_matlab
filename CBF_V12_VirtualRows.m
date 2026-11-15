function [H,labels,orders] = CBF_V12_VirtualRows(shape,xv,nu,s,obstacle)
%CBF_V12_VIRTUALROWS Geometry derivatives for virtual flat model (TEST ONLY).
% Drone and cable: 0..4 using corrected q derivatives, which depend on nu.
% Load: 0..6; high derivatives computed independently without q^(5/6).
% No artificial zeroing of q4 or arbitrary extrapolation of load pL.
P=cbf_parameter_vector(s);O=cbf_obstacle_vector(obstacle,obstacle.type);
plan=cbf_choose_cable_spheres(s);
[p,q,pD]=CBF_V12_VirtualJetFixed(xv,nu,P,4);
H4=cbf_geometry_taylor(shape,obstacle.type,p,q,pD,P,O,plan,'all');
[~,info]=cbf_evaluate_case('virtual',shape,obstacle.type,xv,s,obstacle);
labels=info.labels;orders=info.orderHint;
if size(H4,1)~=numel(labels)
    error('CBF:V12RowCount','Geometry row count differs from label count.');
end
H=zeros(size(H4,1),7);
H(:,1:5)=H4;
% Physical load position derivatives through sixth order are available
% directly in xv and the decision nu, even though q beyond fourth is not.
p6=zeros(3,7);
for j=0:5
    p6(:,j+1)=xv(3*j+1:3*j+3)/factorial(j);
end
p6(:,7)=nu(1:3)/factorial(6);
C=@(v)cbf_jet('const',v,6);
D=@(a,b)cbf_jet('dot',a,b);
S=@(a)cbf_jet('sqrt',a);
n=C(O(1:3))-p6;len=S(D(n,n)+C(P(13)^2));
if strcmp(obstacle.type,'sphere')
    obsSupport=O(4)*len;
else
    R=reshape(O(7:15),3,3);
    nr=R.'*n;
    obsSupport=S(D(O(4:6).*nr,O(4:6).*nr)+C(P(13)^2));
end
loadBarrier=D(n,n)-(P(10)+P(11))*len-obsSupport;
loadGround=p6(3,:)-C(P(10)+P(12)+P(11));
obIdx=find(strcmp(labels,'obstacle_load'));
grIdx=find(strcmp(labels,'ground_load'));
if numel(obIdx)~=1 || numel(grIdx)~=1
    error('CBF:V12LoadLabels','Missing or duplicated load obstacle/ground rows.');
end
H(obIdx,:)=loadBarrier;
H(grIdx,:)=loadGround;
% B capsule ground contains a load-end auxiliary height row with order 6.
% Its 5th/6th derivatives MUST use pL^(5/6), not fabricated zeros.
endIdx=find(strcmp(labels,'ground_cable_load_end'));
if ~isempty(endIdx)
    assert(numel(endIdx)==1,'CBF:V12EndCount','Duplicate capsule load-end ground row.');
    H(endIdx,:)=p6(3,:)-C(P(9)+P(12)+P(11));
end
% No attempt to fabricate 5th/6th derivatives for the other rows.
if any(~isfinite(H(:)))
    error('CBF:V12Nonfinite','Nonfinite virtual geometry Taylor series.');
end
end
