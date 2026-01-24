function [ dX ] = point_mass_model(x,u,P)
% u : force to x,y,z axis and 0
% Input order:
% x (state) = [];
% u (input) = [];
% P (parameter) = [];

    if isfield(P,'A')
        A=P.A;
    else
        A=[zeros(3),eye(3);zeros(3,6)];
    end
    if isfield(P,'B')
        B = P.B;
    else
        B = [zeros(3);eye(3)];
    end
    u = u(1:3);
    dX = A*x+B*u;
end
