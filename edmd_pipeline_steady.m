%% edmd_pipeline_steady : 定常状態ベースの残差モデル生成 [v5]
%  前提となる新事実: 円運動のyawは(payload有無によらず) −20°付近の安定平衡に収束
%  → 旧yaw門限(発散対策)を「非定常検出」に置換し, 定常セグメントのみで学習する
%  使い方: FILESを書き換え → データフォルダ(または親)をカレントに → 一键実行
%  流れ: 抽出(定常選別) → 分析表示 → 学習 → 体検 → 複放関門 → mat保存
clearvars -except agent; clc; close all;

%% ===== ここだけ手動編集 =====
REPO = 'C:\Users\student\Documents\GitHub\common_matlab';
NOM = { ...   % 無payload (hover / circle / fig8 推奨。p2pは任意)
    'lpvmpc_hover_84s_Log(16-Jul-2026_16_16_18)', ...
    'lpvmpc_circle_90s_Log(16-Jul-2026_16_10_14)', ...
    'lpvmpc_figure_87s_Log(16-Jul-2026_16_13_32)',...
    'lpvmpc_p2p_92s_Log(16-Jul-2026_16_18_36)'};
PAY = { ...   % 有payload (同構成)
   'payload_lpvmpc_circle_89s_Log(16-Jul-2026_15_35_34)', ...
    'payload_lpvmpc_figure_85s_Log(16-Jul-2026_16_00_07)', ...
    'payload_lpvmpc_hover_75s_Log(16-Jul-2026_16_02_34)',...
    'payload_lpvmpc_p2p_100s_Log(16-Jul-2026_16_05_35)'};
OUTMAT = "edmd_residual_model_steady_v5.mat";
MASS = 0.75;                  % 名義質量 (配重は含めない)
%% ===========================

G_ACC = 9.81; DT = 0.025;
LAMBDA_NOM = 1e-3; LAMBDA_ERR = 1e-2;        % 正規化ridge基準
WARMUP_SEC = 5.0;                             % 飛行開始後に一律捨てる秒数
PSI_BAND  = 0.18;    % 0.10 → 0.18 (±10°: payload円の揺れ幅を収容)
PSI_REDIV = 0.30;    % 0.15 → 0.30                            % 再発散保護: 定常後にこれを超えたら以降切捨て

if isempty(which('LOGGER'))
    assert(isfolder(REPO), 'REPOパスを修正せよ: %s', REPO);
    addpath(regexprep(genpath(REPO), '[^;]*\.git[^;]*;', ''));
end
assert(~isempty(which('LOGGER')), 'LOGGERが見つからない');
u_hover = [MASS * G_ACC; 0; 0; 0];

%% ===== 1. 抽出 (定常セグメントのみ) =====
fprintf('========== 抽出 (定常選別) ==========\n');
[Zn, Znp, Un, Kn, info_n] = collect_group(NOM, u_hover, DT, WARMUP_SEC, PSI_BAND, PSI_REDIV);
[Zp, Zpp, Up, Kp_, info_p] = collect_group(PAY, u_hover, DT, WARMUP_SEC, PSI_BAND, PSI_REDIV);
fprintf('[data] nominal %d対 / payload %d対\n', size(Zn,2), size(Zp,2));
fprintf('[data] ホバ推力中央値: nom=%.3f pay=%.3f (m*g=%.3f)\n', info_n.th, info_p.th, MASS*G_ACC);
assert(size(Zn,2) > 800 && size(Zp,2) > 800, '定常対が不足(>800必要) → PSI_BAND/WARMUPを確認');
assert(info_p.th > info_n.th + 0.1, 'payload推力が有意に大きくない → ファイル取り違えの疑い');
% yaw平衡点の群間一致チェック (共模除去の前提)
fprintf('[yaw] 円のψ平衡点: nom=%.1f° pay=%.1f° (差%.1f°)\n', ...
    rad2deg(info_n.psi_circle), rad2deg(info_p.psi_circle), ...
    rad2deg(abs(info_n.psi_circle - info_p.psi_circle)));
fprintf('nom: %.1f°  pay: %.1f°  差: %.1f°\n', rad2deg(info_n.psi_circle), rad2deg(info_p.psi_circle), rad2deg(abs(info_n.psi_circle-info_p.psi_circle)))
dpsi = abs(info_n.psi_circle - info_p.psi_circle);
fprintf('[yaw] 円のψ平衡点: nom=%.1f° pay=%.1f° (差%.1f°)\n', ...
    rad2deg(info_n.psi_circle), rad2deg(info_p.psi_circle), rad2deg(dpsi));
fprintf('      → 平衡点差は配重(重心オフセット)の物理効果として残差モデルに学習させる\n');
assert(dpsi < deg2rad(25), '差が25°超 → 配置バージョン混在の疑い, 両群の設定を確認');

%% ===== 2. 学習 =====
fprintf('========== 学習 ==========\n');
[AB, vaf_nom] = ridge_norm(Zn, Un, Znp, LAMBDA_NOM);
A_nom = AB(:,1:26); B_nom = AB(:,27:30);
fprintf('[nom] VAF median=%.1f%% / max|eig|=%.4f\n', median(vaf_nom), max(abs(eig(A_nom))));
assert(median(vaf_nom) > 80, '名義VAF低すぎ → 抽出異常');

R = Zpp - (A_nom * Zp + B_nom * Up);
r_dc = mean(R, 2);
fprintf('[res] 残差DC |mean|=%.4g (v系=%.4g 定数項=%.4g)\n', norm(r_dc), norm(r_dc(7:9)), abs(r_dc(16)));
% 軌道種別ごとの残差DC (定常円の配重トルク特性を可視化 — 論文素材)
for k = ["hover", "circle", "fig8"]
    m = strcmp(Kp_, k);
    if any(m)
        rk = mean(R(:, m), 2);
        fprintf('  [res/%-6s] |DC|=%.4g  w系(10:12)=[%+.3g %+.3g %+.3g]\n', k, norm(rk), rk(10), rk(11), rk(12));
    end
end
assert(norm(r_dc) > 5e-4, '残差DCほぼ零 → 配重誤差が見えていない');

[AB, vaf_err] = ridge_norm(Zp, Up, R, LAMBDA_ERR);
A_err = AB(:,1:26); B_err = AB(:,27:30);
fprintf('[err] 残差VAF median=%.1f%% (定常データなので旧版より改善するはず)\n', median(vaf_err));

%% ===== 3. 係数体検 =====
mA = max(abs(A_err(:))); mB = max(abs(B_err(:)));
fprintf('[体検] max|A_err|=%.3g max|B_err|=%.3g rank(B_err)=%d\n', mA, mB, rank(B_err));
assert(mA < 0.5 && mB < 0.5, '病態回帰 → LAMBDA_ERRを10倍に');
assert(rank(B_err) == 4, 'rank不足 → 入力励起不足');
try, dlqr(A_err, B_err, eye(26)*200, eye(4)*0.02); fprintf('dlqr: OK\n');
catch, fprintf('dlqr: 失敗 (mode1不可, mode2/5影響なし)\n'); end

%% ===== 4. 複放関門 (payloadの定常セグメント上でδu再現) =====
fprintf('========== 複放関門 ==========\n');
figure('Name','replay'); tiledlayout(numel(PAY),1,'TileSpacing','compact');
for f = 1:numel(PAY)
    [X, U, pairs] = extract_one(PAY{f}, DT, WARMUP_SEC, PSI_BAND, PSI_REDIV);
    du = zeros(4, numel(pairs));
    for j = 1:numel(pairs)
        k = pairs(j);
        z = lift_z(X(:,k));
        rhs = lift_z(X(:,k+1)) - (A_nom*z + B_nom*(U(:,k)-u_hover)) - A_err*z;
        du(:,j) = (B_err'*B_err + 1e-2*eye(4)) \ (B_err'*rhs);
    end
    p95 = prctile(abs(du(2:4,:)), 95, 2);
    [~, base] = fileparts(char(PAY{f}));
    nexttile; plot(du(2:4,:)'); yline([0.05 -0.05],'r--');
    title(base,'Interpreter','none'); legend('roll','pitch','yaw');
    fprintf('[replay] %s: p95|δu|=[%.3f %.3f %.3f]\n', base, p95);
    assert(all(isfinite(du(:))) && all(p95 < 0.05), '複放FAIL(%s) → LAMBDA_ERR 10倍で再実行', base);
end

%% ===== 5. 保存 =====
results_edmd = struct('A_nom',A_nom,'B_nom',B_nom,'A_err',A_err,'B_err',B_err); %#ok<NASGU>
meta = struct('created',datestr(now),'files',{[NOM, PAY]},'m',MASS, ...
    'u_convention','u_tilde = u - [m*g;0;0;0]', 'regime','steady-state (psi_ss band)', ...
    'psi_ss_nom',info_n.psi_circle,'psi_ss_pay',info_p.psi_circle, ...
    'vaf_nom',vaf_nom,'vaf_err',vaf_err,'residual_dc',r_dc); %#ok<NASGU>
save(OUTMAT,'results_edmd','meta');
fprintf(['========== 全関門PASS → %s ==========\n' ...
  '部署前チェック:\n' ...
  ' 1. get_residual_reference_state に ref_state(6)=現在ψ の一行を追加済みか (yaw中立化, 必須)\n' ...
  ' 2. mode=5, du_max=[0;0.1;0.1;0], full_beta=0.1, torque_only=1\n' ...
  ' 3. matをmodel_fileフォルダへ + git commit → agent再構築 → loaded=1確認\n' ...
  ' 4. A/B/A: mode 0→5→0, T=12円(定常後を評価区間に)\n'], OUTMAT);

%% ======================= 局所関数 =======================
function [Z,Zp,Ut,kinds,info] = collect_group(files, u_hover, dt, wu, band, rediv)
Z=[]; Zp=[]; Ut=[]; kinds=strings(1,0); th=[]; psi_c=NaN;
for i = 1:numel(files)
    [X,U,pairs,psi_ss] = extract_one(files{i}, dt, wu, band, rediv);
    kd = classify(files{i});
    if kd == "circle", psi_c = psi_ss; end
    for k = pairs
        Z=[Z, lift_z(X(:,k))]; Zp=[Zp, lift_z(X(:,k+1))]; %#ok<AGROW>
        Ut=[Ut, U(:,k)-u_hover]; kinds(end+1)=kd; th(end+1)=U(1,k); %#ok<AGROW>
    end
end
info = struct('th', median(th), 'psi_circle', psi_c);
end

function [X,U,pairs,psi_ss] = extract_one(matfile, dt, warmup_sec, band, rediv)
hit = dir(fullfile(pwd,'**',[char(matfile),'.mat']));
assert(~isempty(hit), 'ファイル未発見: %s.mat', matfile);
lg = LOGGER(fullfile(hit(1).folder, hit(1).name));
ph = lg.Data.phase(:)';
i102 = find(ph==102); assert(~isempty(i102),'%s: 102なし',matfile);
blk = i102([true, diff(i102)>1]);
if numel(blk)>=2, idx = blk(2):i102(end); else, idx = i102(1):i102(end); end
idx = idx(ph(idx)==102);

U = lg.Data.agent.input;
if iscell(U), U = cellfun(@(c) u4(c), U(:)','UniformOutput',false); U = [U{:}]; end
if size(U,1)~=4, U = U'; end
res = lg.Data.agent.estimator.result;
n = min([numel(ph), size(U,2), size(res,2)]);
idx = idx(idx <= n-1);
X = zeros(12,n); need = false(1,n); need(idx)=true; need(min(idx+1,n))=true;
for k = find(need), st = res{1,k}.state; X(:,k) = [st.p(:); st.q(:); st.v(:); st.w(:)]; end

% --- 定常選別 [v5核心] ---
idx = idx(round(warmup_sec/dt):end);                    % warmup一律切捨て
psi = X(6, idx);
psi_ss = median(psi(round(0.6*end):end));               % 後半40%の中央値 = 平衡点
steady = abs(psi - psi_ss) < band;                      % 定常バンド内のみ採用
first_s = find(steady, 1);
assert(~isempty(first_s), '%s: 定常セグメントなし (ψが収束していない)', matfile);
re = find(abs(psi - psi_ss) > rediv & (1:numel(psi)) > first_s + 40, 1);  % 再発散保護
if ~isempty(re)
    fprintf('  [再発散] %s: %d/%d 以降切捨て\n', matfile, re, numel(idx));
    steady(re:end) = false;
end
idx = idx(steady);

% --- 飽和除外 + 連続対 ---
ok = false(1,n); ok(idx) = true;
ok = ok & all(abs(U(2:4,1:n)) < 1.45,1) & U(1,1:n) > 0.2;
pairs = idx(ok(idx) & ok(min(idx+1,n)));
fprintf('  %s: ψss=%+.1f°, 定常採用%d対\n', matfile, rad2deg(psi_ss), numel(pairs));
end

function [AB,vaf] = ridge_norm(Z,U,Y,lambda)
% [v6] 固定スケール正規化: データstdではなく物理レンジで正規化
% 理由: 定常データでは弱励起特徴(W積項等)のstdが極小 → std正規化の
%       逆変換(÷std)が係数を爆発させる。固定レンジなら増幅は有界。
S = [ones(3,1);          % p [m]           ~1
     0.3*ones(3,1);      % q [rad]         ~0.3
     0.5*ones(3,1);      % v [m/s]         ~0.5
     0.5*ones(3,1);      % w [rad/s]       ~0.5
     0.3; 0.3; 1;        % R13, R23, R33
     1;                  % 定数項
     0.1*ones(10,1);     % W積・Euler率項 (弱励起でも増幅上限10倍)
     1; 0.3; 0.3; 0.3];  % u~: 推力偏差, 力矩x3
Gm = [Z; U];
Gn = Gm ./ S;
AB = (Y*Gn') / (Gn*Gn' + lambda*size(Gn,2)*eye(size(Gn,1)));
AB = AB ./ S';
E = Y - AB*Gm;
vaf = max(0, (1 - var(E,0,2)./max(var(Y,0,2),1e-12))*100);
end

function z = lift_z(x)
Q1=x(4);Q2=x(5);Q3=x(6);W1=x(10);W2=x(11);W3=x(12);
c1=cos(Q1);s1=sin(Q1);c2=cos(Q2);s2=sin(Q2);c3=cos(Q3);s3=sin(Q3);
c1s=sign(c1)*max(abs(c1),1e-3); if c1==0,c1s=1e-3; end
c2s=sign(c2)*max(abs(c2),1e-3); if c2==0,c2s=1e-3; end
z = [x(1:12); c3*s2*c1+s3*s1; s3*s2*c1-c3*s1; c2*c1; 1; ...
     W1*W2; W2*W3; W3*W1; W2*c1; W3*s1; ...
     W1*c2/c1s; W2*s1/c2s; W3*c1/c2s; W2*s1*s2/c2s; W3*c1*s2/c2s];
end

function k = classify(name)
n = char(name);
if contains(n,'hover'), k="hover"; elseif contains(n,'circle'), k="circle";
elseif contains(n,'fig')||contains(n,'figure'), k="fig8";
elseif contains(n,'p2p'), k="p2p"; else, k="other"; end
end

function u = u4(c)
if isempty(c), u = nan(4,1); else, u=c(:); if numel(u)~=4, u=nan(4,1); end, end
end