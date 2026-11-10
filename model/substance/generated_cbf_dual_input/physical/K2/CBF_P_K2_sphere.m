function [h,psi,Araw,braw,info]=CBF_P_K2_sphere(x,aux,refZ,F1,P,obs,geom,kappa)
% Legacy generated wrapper; implementation in the master.
s=Generate_CBF_ActuatorAware_6Methods('evaluate','physical','K2','sphere',x,aux,refZ,F1,P,obs,geom,kappa);
h=s.h; psi=s.psi; Araw=s.Araw; braw=s.braw; info=s.info;
end
