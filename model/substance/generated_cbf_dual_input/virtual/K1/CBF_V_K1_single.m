function [h,psi,Araw,braw,info]=CBF_V_K1_single(x,aux,refZ,F1,P,obs,geom,kappa)
% Legacy generated wrapper; implementation in the master.
s=Generate_CBF_ActuatorAware_6Methods('evaluate','virtual','K1','single',x,aux,refZ,F1,P,obs,geom,kappa);
h=s.h; psi=s.psi; Araw=s.Araw; braw=s.braw; info=s.info;
end
