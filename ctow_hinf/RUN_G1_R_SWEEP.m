% ---- G1 のみ：実際の r を変えたときの指標と S2 の応答 ----
r_list = [0.5 1 2 3];
K1 = P.K.G1;  e3 = [0 0 1 0 0 0];
[Ar0, B, E] = ctow_chain(P.r0);
TD1 = norm(ss(Ar0 - B*K1, E, P.r0*P.Wr*e3, 0), Inf);   % 設計点 r0 での値（r_list によらない）
fprintf('\nG1: ||TD|| = %.3f（設計点 r0 = %.2f）\n', TD1, P.r0);
fprintf('%5s %9s %22s %7s %9s\n', 'r', '||Twth||', 'rho(10/30/50ms)', 'sigma', 'S2 maxθ');
figure; tiledlayout(2,1);
for i = 1:numel(r_list)
    r = r_list(i);
    [Ar, B, E] = ctow_chain(r);
    Twth  = norm(ss(Ar - B*K1, E, e3/P.g, 0), Inf);
    rho   = arrayfun(@(t) ctow_impl_rho(K1, P.dt, t, r), P.tau_list);
    sigma = -log(rho(2))/P.dt;
    sr = ctow_sim_impl(K1, P, struct('T',40, 'w',@(t) 0.5*double(t>=1), 'r', r));
    fprintf('%5g %9.4f   [%.4f %.4f %.4f] %7.3f %8.3f°\n', r, Twth, rho, sigma, rad2deg(max(abs(sr.theta))));
    nexttile(1); hold on; plot(sr.t, sr.p, 'DisplayName', sprintf('r=%g', r));
    nexttile(2); hold on; plot(sr.t, rad2deg(sr.theta), 'DisplayName', sprintf('r=%g', r));
end
nexttile(1); grid on; legend; ylabel('position p [m]'); title('G1, S2: position');
nexttile(2); grid on; legend; xlabel('t [s]'); ylabel('swing angle [deg]'); title('G1, S2: swing angle');
%%
% ---- G1 のみ：実際の w を変えたときの指標と S2 の応答 ----
w_list = [0.5 1 2 3];
K1 = P.K.G1;  e3 = [0 0 1 0 0 0];
r  = 1;                                                  % 実際の r（公称）
[Ar0, B, E] = ctow_chain(P.r0);
TD1 = norm(ss(Ar0 - B*K1, E, P.r0*P.Wr*e3, 0), Inf);   % 設計点 r0 での値（w によらない）
fprintf('\nG1: ||TD|| = %.3f（設計点 r0 = %.2f）\n', TD1, P.r0);
fprintf('%5s %9s %22s %7s %9s\n', 'w', '||Twth||', 'rho(10/30/50ms)', 'sigma', 'S2 maxθ');
figure; tiledlayout(2,1);
for i = 1:numel(w_list)
    w  = w_list(i);
    [Ar, B, E] = ctow_chain(r);
    Twth  = norm(ss(Ar - B*K1, E, e3/P.g, 0), Inf);
    rho   = arrayfun(@(t) ctow_impl_rho(K1, P.dt, t, r), P.tau_list);
    sigma = -log(rho(2))/P.dt;
    sr = ctow_sim_impl(K1, P, struct('T',40, 'w',@(t) w*double(t>=1), 'r', r));
    fprintf('%5g %9.4f   [%.4f %.4f %.4f] %7.3f %8.3f°\n', w, Twth, rho, sigma, rad2deg(max(abs(sr.theta))));
    nexttile(1); hold on; plot(sr.t, sr.p, 'DisplayName', sprintf('w=%g', w));
    nexttile(2); hold on; plot(sr.t, rad2deg(sr.theta), 'DisplayName', sprintf('w=%g', w));
end
nexttile(1); grid on; legend; ylabel('position p [m]'); title('G1, S2: position');
nexttile(2); grid on; legend; xlabel('t [s]'); ylabel('swing angle [deg]'); title('G1, S2: swing angle');