function R=test_CBF_V12_Smoke()
%TEST_CBF_V12_SMOKE Independent regression for missing pL^(6) -> q^(4).
% Only checks that corrected K4 jet uses sixth load derivative, without
% changing installed virtual CBF or requiring a MATLAB-generated HLC.
test_CBF_SetupPaths();s=test_CBF_Fixture();
P=cbf_parameter_vector(s);[x,u]=CBF_V12_PureHover(s);
Z=CBF_V11_RealTaylor('R1',x,u,s,6);
xv=zeros(20,1);
for j=0:5,xv(3*j+1:3*j+3)=factorial(j)*Z(8:10,j+1);end
nu=zeros(4,1);
[~,qcFixed,pDFixed]=CBF_V12_VirtualJetFixed(xv,nu,P,4);
assert(norm(qcFixed-Z(14:16,1:5),Inf)<1e-8,...
 'CBF:V12HoverCable','Fixed virtual cable jet differs at true hover.');
assert(norm(pDFixed-(Z(8:10,1:5)-P(3)*Z(14:16,1:5)),Inf)<1e-8,...
 'CBF:V12HoverDrone','Fixed virtual drone jet differs at true hover.');
nu(1)=1; % horizontal sixth derivative influences fourth cable derivative
[~,qcNew]=CBF_V12_VirtualJetFixed(xv,nu,P,4);
[~,qcOld]=cbf_virtual_state_taylor(xv,nu,P,4);
assert(norm(qcNew(:,5)-qcOld(:,5))>1e-5,...
 'CBF:V12NoDependence','Corrected cable jet did not recover nu_x dependence.');
R=struct('status','PASS','hoverJetParity',true,'nu6ToCable4Detected',true);
fprintf('[PASS] V12 smoke: exact hover parity + pL6 to cable q4 effect.\n');
end
