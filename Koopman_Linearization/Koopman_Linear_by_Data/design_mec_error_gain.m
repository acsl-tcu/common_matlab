function gain = design_mec_error_gain(est,setting)
% 誤差と入力の単位を考慮して離散LQRを解き、26観測量の座標へゲインを戻す。
assert(isfield(est,'encode') && isfield(est,'error_definition'), ...
    'MEC:LegacyModel','fit_mec_error_modelで作ったモデルを使用してください。');
Qphysical = diag([1./setting.error_scale.^2;zeros(14,1)]);
Rphysical = setting.input_cost_multiplier*diag(1./setting.input_scale.^2);
% z=encode*e、e≈decode*z、v=delta_u./scale_u。変換はQ,Rにも適用する。
A = est.A;
B = est.B.*est.scale_u';
Q = est.decode'*Qphysical*est.decode;
R = diag(est.scale_u)*Rphysical*diag(est.scale_u);
% 全費用を同じ定数で割っても最適ゲインは変わらない。
cost_scale = max(norm(Q,2),norm(R,2));
Q = (Q+Q')/(2*cost_scale);
R = R/cost_scale;
P = Q;
converged = false;
for iteration = 1:20000
    K = (R+B'*P*B)\(B'*P*A);
    next = Q+A'*P*A-A'*P*B*K;
    next = (next+next')/2;
    if norm(next-P,'fro')<1e-11*max(1,norm(next,'fro'))
        P = next;
        converged = true;
        break;
    end
    P = next;
end
assert(converged,'MEC:RiccatiConvergence','Riccati反復が収束しませんでした。');
K = (R+B'*P*B)\(B'*P*A);
K_full = (est.scale_u.*K)*est.encode;
eigClosed = eig(A-B*K);
assert(all(isfinite(K_full(:))) && max(abs(eigClosed))<1, ...
    'MEC:UnstableGain','有限で安定なゲインが得られませんでした。');
gain = struct('K_full',K_full,'K_reduced',K,'A',est.A_original,'B',est.B_original, ...
    'Qfull',Qphysical,'R',Rphysical,'eigClosed',eigClosed,'dt',est.dt, ...
    'error_definition',est.error_definition,'design_method','scaled_svd_ridge_lqr_v1', ...
    'setting',setting,'encode',est.encode,'decode',est.decode, ...
    'riccati_residual',norm(Q+A'*P*A-A'*P*B*K-P,'fro'));
fprintf('設計モデルの閉ループ最大固有値絶対値：%.6g\n',max(abs(eigClosed)));
fprintf('ゲインの行ごとの最大絶対値：'); fprintf(' %.4g',max(abs(K_full),[],2)); fprintf('\n');
end
