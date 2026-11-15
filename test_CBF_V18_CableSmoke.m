function out=test_CBF_V18_CableSmoke()
% Fast zero-input-coefficient and cross-model same-geometry smoke ONLY.
test_CBF_SetupPaths();s=CBF_V18_TestSettings();
[x,u,obs]=CBF_V11_Examples(1,s,'ellipsoid',.65);
x23=[x;u(1);zeros(3,1)];
Z=CBF_V11_RealTaylor('R1',x,u,s,6);
xv=zeros(20,1);for k=0:5,xv(3*k+1:3*k+3)=factorial(k)*Z(8:10,k+1);end
hv=CBF_V18_CableOnly('VIRTUAL',xv,zeros(4,1),s,obs,4);
h1=CBF_V18_CableOnly('R1',x,u,s,obs,2);
h2=CBF_V18_CableOnly('R2',x23,zeros(4,1),s,obs,6);
assert(max(abs([hv(1)-h1(1),h2(1)-h1(1)]))<1e-8,'V18 barrier h0 differs');
assert(h1(1)>=0,'Smoke case unexpectedly unsafe');
out=struct('status','GEOMETRY_H0_PARITY_PASS_NOT_CBF_PROOF',...
  'hR1',h1(1),'hR2',h2(1),'hVirtual',hv(1));
disp(out);
end
