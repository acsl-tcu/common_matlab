function [x,u] = CBF_V12_PureHover(s)
%CBF_V12_PUREHOVER Exact stationary equilibrium of the base 19-state model.
% Unlike CBF_V11_Examples(1), this has no horizontal velocity/cable rate.
x=zeros(19,1);x(1)=1;x(10)=1.5;x(16)=-1;
T=(s.physical.massDrone+s.physical.massLoad)*s.physical.gravity;
u=[T;0;0;0];
end
