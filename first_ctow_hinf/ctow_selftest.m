function pass = ctow_selftest()
%CTOW_SELFTEST  参照値（Python 版）の再現確認。段階 1 の判定に使う。
P = ctow_params(); pass = true;
names = {'G0','G1','G2'};
for i = 1:3
    nm = names{i}; K = P.K.(nm);
    rho = arrayfun(@(t) ctow_impl_rho(K, P.dt, t), P.tau_list);
    ok = all(abs(rho - P.ref.rho.(nm)) < 1e-3);
    fprintf('%s rho = [%.4f %.4f %.4f]  ref = [%.4f %.4f %.4f]  %s\n', nm, rho, P.ref.rho.(nm), tf2str(ok));
    pass = pass && ok;
    [A1, B, E] = ctow_chain(1); Acl = A1 - B*K;
    twp = norm(ss(Acl, E, [1 0 0 0 0 0], 0), Inf);
    ok = abs(twp - P.ref.Twp_inf.(nm))/P.ref.Twp_inf.(nm) < 1e-2;
    fprintf('   ||Twp||inf = %.4f  ref = %.4f  %s\n', twp, P.ref.Twp_inf.(nm), tf2str(ok));
    pass = pass && ok;
end
for nm = {'G1','G2'}
    z = ctow_zeta_min(P.K.(nm{1}), P.dt, P.tau_r, 0);
    ok = abs(z - P.ref.zeta_min.(nm{1}))/P.ref.zeta_min.(nm{1}) < 0.05;
    fprintf('%s zeta_min = %.4f  ref = %.4f  %s\n', nm{1}, z, P.ref.zeta_min.(nm{1}), tf2str(ok));
    pass = pass && ok;
end
% 静的ゲインを動的制御器として評価したときの一致
K = P.K.G1;
r1 = ctow_impl_rho(K, P.dt, P.tau_r);
r2 = ctow_impl_rho_dyn(ss([],[],[],[-K 0], P.dt), P);
ok = abs(r1 - r2) < 1e-9;
fprintf('static vs dyn rho: %.6f / %.6f  %s\n', r1, r2, tf2str(ok));
pass = pass && ok;
fprintf('\nselftest: %s\n', tf2str(pass));
end
function s = tf2str(b)
if b, s = 'OK'; else, s = 'NG'; end
end
