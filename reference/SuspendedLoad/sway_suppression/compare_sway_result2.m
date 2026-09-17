clear; clc;

file_no = "aaa_Log(09-Feb-2026_04_43_27).mat";
file_on = "ccc_Log(09-Feb-2026_04_48_09).mat";

S_no = extract_series_from_logmat_cellresult_with_phase(file_no);
S_on = extract_series_from_logmat_cellresult_with_phase(file_on);

opt.t_range_no = [10 30];
opt.t_range_on = [10 30];
opt.align_zero = true;

OUT = compare_sway_robust(S_no, S_on, opt);

disp("=== summary (robust) ===");
disp(OUT.summary);

plot_compare_timeseries_ok(OUT);

%% ===== functions =====

function OUT = compare_sway_robust(S_no, S_on, opt)

A = crop_and_align_fix(S_no, opt.t_range_no, opt.align_zero);
B = crop_and_align_fix(S_on, opt.t_range_on, opt.align_zero);

% ---- 速度がゼロ/無効なら位置から作る（推奨：こっちを使う） ----
fc = 3; % まずは 3 または 5 Hz を推奨
[A.v_est, A.vL_est] = vel_lp_diff(A.t, A.p, A.pL, fc);
[B.v_est, B.vL_est] = vel_lp_diff(B.t, B.p, B.pL, fc);


% 相対量（位置）
rA  = A.pL - A.p;
rB  = B.pL - B.p;

% 相対量（速度）…ログのvではなく推定v_estを使用
vrA = A.vL_est - A.v_est;
vrB = B.vL_est - B.v_est;

rxyA  = vecnorm(rA(1:2,:), 2, 1);
rxyB  = vecnorm(rB(1:2,:), 2, 1);
vrxyA = vecnorm(vrA(1:2,:),2, 1);
vrxyB = vecnorm(vrB(1:2,:),2, 1);

% 揺れ角：座標系の符号が違っても壊れにくいよう abs(rz) を使う
rzA = rA(3,:);  rzB = rB(3,:);
thetaA = atan2(rxyA, max(1e-6, abs(rzA)));   % rad
thetaB = atan2(rxyB, max(1e-6, abs(rzB)));

dtA = median(diff(A.t)); dtB = median(diff(B.t));
E_vrA = sum(vrxyA.^2) * dtA;
E_vrB = sum(vrxyB.^2) * dtB;

s = struct();
s.rxy_rms_no   = rms(rxyA);     s.rxy_rms_on   = rms(rxyB);
s.vrxy_rms_no  = rms(vrxyA);    s.vrxy_rms_on  = rms(vrxyB);
s.theta_rms_no = rms(thetaA);   s.theta_rms_on = rms(thetaB);

s.rxy_p95_no   = prctile(rxyA,95);   s.rxy_p95_on   = prctile(rxyB,95);
s.vrxy_p95_no  = prctile(vrxyA,95);  s.vrxy_p95_on  = prctile(vrxyB,95);
s.theta_p95_no = prctile(thetaA,95); s.theta_p95_on = prctile(thetaB,95);

s.E_vr_no = E_vrA;  s.E_vr_on = E_vrB;

s.rxy_rms_ratio   = s.rxy_rms_on   / max(1e-12, s.rxy_rms_no);
s.vrxy_rms_ratio  = s.vrxy_rms_on  / max(1e-12, s.vrxy_rms_no);
s.theta_rms_ratio = s.theta_rms_on / max(1e-12, s.theta_rms_no);
s.E_vr_ratio      = s.E_vr_on      / max(1e-12, s.E_vr_no);

% ドローン速度（推定）も比較
vA = vecnorm(A.v_est(1:2,:),2,1);  vB = vecnorm(B.v_est(1:2,:),2,1);
s.vxy_rms_no = rms(vA);  s.vxy_rms_on = rms(vB);
s.vxy_rms_ratio = s.vxy_rms_on / max(1e-12, s.vxy_rms_no);

OUT = struct();
OUT.A = A; OUT.B = B;
OUT.rxyA = rxyA; OUT.rxyB = rxyB;
OUT.vrxyA = vrxyA; OUT.vrxyB = vrxyB;
OUT.thetaA = thetaA; OUT.thetaB = thetaB;
OUT.summary = s;

end

function S = crop_and_align_fix(S, t_range, align_zero)
% --- 必須フィールドの向きを 3×N, 1×N に正規化 ---
S = normalize_shapes(S);

t = S.t;
m = (t >= t_range(1)) & (t <= t_range(2));
S = mask_struct_time(S, m);

if align_zero && ~isempty(S.t)
    S.t = S.t - S.t(1);
end
end

function S = normalize_shapes(S)
% t を 1×N に
S.t = S.t(:)'; 
N = numel(S.t);

% 3×N にそろえる（p,v,pL,vL）
S.p  = ensure3xN(S.p,  N, "p");
S.v  = ensure3xN(S.v,  N, "v");
S.pL = ensure3xN(S.pL, N, "pL");
S.vL = ensure3xN(S.vL, N, "vL");
end

function X = ensure3xN(X, N, name)
if isempty(X), X = zeros(3,N); return; end
sz = size(X);
if isequal(sz, [3 N])
    return;
elseif isequal(sz, [N 3])
    X = X.';  % transpose
    return;
elseif numel(X)==3*N
    X = reshape(X, 3, N);
    return;
else
    error("'%s' のサイズが想定外です: %s (tはN=%d)", name, mat2str(sz), N);
end
end

function S = mask_struct_time(S, m)
fn = fieldnames(S);
for i=1:numel(fn)
    k = fn{i};
    x = S.(k);
    if isempty(x), continue; end

    if isnumeric(x) && isvector(x) && numel(x)==numel(m)
        S.(k) = x(m);
    elseif isnumeric(x) && size(x,2)==numel(m)
        S.(k) = x(:,m);
    elseif isnumeric(x) && size(x,1)==numel(m)
        S.(k) = x(m,:);   % N×d 形式も対応
    elseif (isstring(x) || iscell(x)) && numel(x)==numel(m)
        S.(k) = x(m);
    elseif ischar(x) && size(x,1)==numel(m)
        S.(k) = x(m,:);
    end
end
end

function [v, vL] = vel_lp_diff(t, p, pL, fc)
% vel_lp_diff: ローパス後に差分で速度推定
% t: 1xN
% p, pL: 3xN
% fc: cutoff [Hz] (例: 3〜5)

dt = median(diff(t));
N  = size(p,2);

% 1次ローパス（指数移動平均）
alpha = exp(-2*pi*fc*dt);

pf  = zeros(size(p));
pLf = zeros(size(pL));
pf(:,1)  = p(:,1);
pLf(:,1) = pL(:,1);

for k=2:N
    pf(:,k)  = alpha*pf(:,k-1)  + (1-alpha)*p(:,k);
    pLf(:,k) = alpha*pLf(:,k-1) + (1-alpha)*pL(:,k);
end

% 中央差分（端は前後差分）
v  = zeros(size(p));
vL = zeros(size(pL));

v(:,2:N-1)  = (pf(:,3:N)  - pf(:,1:N-2))  / (2*dt);
vL(:,2:N-1) = (pLf(:,3:N) - pLf(:,1:N-2)) / (2*dt);

v(:,1)  = (pf(:,2)  - pf(:,1))  / dt;
vL(:,1) = (pLf(:,2) - pLf(:,1)) / dt;

v(:,N)  = (pf(:,N)  - pf(:,N-1))  / dt;
vL(:,N) = (pLf(:,N) - pLf(:,N-1)) / dt;
end



function plot_compare_timeseries_ok(OUT)
A = OUT.A; B = OUT.B;

figure;
plot(A.t(:), OUT.rxyA(:)); hold on;
plot(B.t(:), OUT.rxyB(:));
grid on; xlabel("t [s]"); ylabel("||r_{xy}|| [m]");
legend("no","on");

figure;
plot(A.t(:), OUT.vrxyA(:)); hold on;
plot(B.t(:), OUT.vrxyB(:));
grid on; xlabel("t [s]"); ylabel("||v_{r,xy}|| [m/s]");
legend("no","on");

figure;
plot(A.t(:), rad2deg(OUT.thetaA(:))); hold on;
plot(B.t(:), rad2deg(OUT.thetaB(:)));
grid on; xlabel("t [s]"); ylabel("\theta [deg]");
legend("no","on");
end
