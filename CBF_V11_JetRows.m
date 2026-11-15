function [H,labels]=CBF_V11_JetRows(mode,shape,x,u,s,obs,K)
%CBF_V11_JETROWS  R1/R2の同じ幾何安全関数を元の実モデルで微分。
Z=CBF_V11_RealTaylor(mode,x,u,s,K);
pL=Z(8:10,:);qc=Z(14:16,:);pD=pL-s.physical.cableLength*qc;
P=cbf_parameter_vector(s);O=cbf_obstacle_vector(obs,obs.type);
plan=cbf_choose_cable_spheres(s);
H=cbf_geometry_taylor(shape,obs.type,pL,qc,pD,P,O,plan,'all');
x29=[x(1:19);u(1);zeros(3,1);u(2:4);zeros(3,1)];
if strcmpi(mode,'R2')
    x29=[x(:);u(2:4);zeros(3,1)];
end
[~,info]=cbf_evaluate_case('real',shape,obs.type,x29,s,obs);
labels=info.labels;
if size(H,1)~=numel(labels)
    error('CBF:V11Rows','幾何行とラベルの数が異なります。');
end
end
