function vs_safe = CBF_QP_2D(vs_nom, x, obs)

% ============================================================
% 2D ECBF Safety Filter for Suspended Load
%
% state:
%   pL = x(8:10)
%   vL = x(11:13)
%
% virtual input:
%   vs = [vx, vy, vyaw]
%
% obstacle avoidance in XY plane
% ============================================================

%% payload position / velocity

pL = x(8:10);
vL = x(11:13);

px = pL(1);
py = pL(2);

vx = vL(1);
vy = vL(2);

%% nominal virtual input

vs_nom_xy = vs_nom(1:2);

%% ECBF parameters

k1 = 4.0;
k0 = 6.0;

%% QP cost
%
% minimize:
%
% ||vs - vs_nom||^2
%

H = eye(2);

f = -vs_nom_xy(:);

%% CBF constraints

A = [];
b = [];

for i = 1:length(obs)

    p_obs = obs(i).p_obs;

    ox = p_obs(1);
    oy = p_obs(2);

    R = obs(i).R_safe;

    %% barrier

    dx = px - ox;
    dy = py - oy;

    h = dx^2 + dy^2 - R^2;

    %% hdot

    hdot = 2*dx*vx + 2*dy*vy;

    %%
    % ECBF:
    %
    % hddot + k1 hdot + k0 h >= 0
    %
    % hddot =
    %   2*v^Tv + 2*(p-pobs)^T*vs
    %
    % =>
    %
    % 2*(p-pobs)^T vs
    % >=
    % -(2*v^Tv + k1*hdot + k0*h)
    %

    rhs = ...
        -( ...
        2*(vx^2 + vy^2) ...
        + k1*hdot ...
        + k0*h );

    Ai = -2*[dx dy];

    bi = -rhs;

    A = [A; Ai];
    b = [b; bi];

end

%% input bounds (important)

vmax = 5.0;

lb = [-vmax; -vmax];
ub = [ vmax;  vmax];

%% solve QP

fprintf('\n');

fprintf('========== CBF DEBUG ==========\n');

fprintf('h       = %.6f\n', h);
fprintf('hdot    = %.6f\n', hdot);

fprintf('vs_nom  = [%.3f %.3f]\n', ...
    vs_nom_xy(1), ...
    vs_nom_xy(2));

fprintf('A       = [%.3f %.3f]\n', ...
    Ai(1), Ai(2));

fprintf('b       = %.3f\n', bi);

options = optimoptions('quadprog', ...
    'Display', 'off');

[vs_xy,~,exitflag] = quadprog( ...
    H, ...
    f, ...
    A, ...
    b, ...
    [], ...
    [], ...
    lb, ...
    ub, ...
    [], ...
    options);

if isempty(vs_xy)

    disp('vs_xy is EMPTY');

else

    fprintf('vs_safe = [%.3f %.3f]\n', ...
        vs_xy(1), ...
        vs_xy(2));

end

%% fallback

if exitflag <= 0

    warning('CBF-QP infeasible');

    vs_xy = vs_nom_xy;

end

%% restore yaw channel

vs_safe = vs_nom;

vs_safe(1) = vs_xy(1);
vs_safe(2) = vs_xy(2);

end
