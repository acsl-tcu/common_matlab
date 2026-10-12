function [h,psi,Araw,braw,info] = CBF_V_sphere(x,delta,refZ,F1,P,obsVec,geom,kappa)
% Generated numeric Taylor-jet wrapper; requires Generate_CBF_VirtualHierarchy_3D.m on path.
s = Generate_CBF_VirtualHierarchy_3D('evaluate','sphere',x,delta,refZ,F1,P,obsVec,geom,kappa);
h=s.h; psi=s.psi; Araw=s.Araw; braw=s.braw; info=s.info;
end
