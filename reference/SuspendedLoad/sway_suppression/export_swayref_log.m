function L = export_swayref_log(sway)
% sway: SWAY_REF_MOD object
% L: purely-numeric struct for save/compare

k = find(isfinite(sway.t_log));   % 有効サンプル
L.t      = sway.t_log(k);
L.on     = sway.sway_on_log(k);
L.c_xy   = sway.c_xy_log(k,:);        % Nx2
L.S_use  = sway.S_use_log(k);
L.theta  = sway.theta_log(k);
L.danger = sway.danger_log(k);

cmag = vecnorm(L.c_xy,2,2);           % |c|
L.on_ratio   = mean(L.on > 0.5, 'omitnan');
L.on_time    = sum(L.on > 0.5) * median(diff(L.t));
L.c_mean     = mean(cmag,'omitnan');
L.c_p95      = prctile(cmag,95);
L.c_max      = max(cmag);
L.theta_p95  = prctile(L.theta,95);
end
