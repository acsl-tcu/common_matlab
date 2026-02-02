function S = extract_series_from_logmat_cellresult_with_phase(matfile)
% extract_series_from_logmat_cellresult の出力に cha を付与（最近傍で確実マップ）

S = extract_series_from_logmat_cellresult(matfile);

L = load(matfile, "log");
log = L.log;

Tf = log.Data.t(:);
ph = log.Data.phase(:);

% 有効サンプルだけ
mask = (Tf ~= 0) & isfinite(Tf) & isfinite(ph);
Tf = Tf(mask);
ph = ph(mask);

t = S.t(:);

% --- 近傍インデックスを作る（必ず何かに対応） ---
idx = interp1(Tf, 1:numel(Tf), t, 'nearest', 'extrap');
idx = max(1, min(numel(Tf), round(idx)));

% 一応誤差チェック（大きくズレたら '?' にする）
tol = 5e-3;  % 5ms くらい（必要なら増やしてOK）
ok = abs(Tf(idx) - t) <= tol;

cha = repmat('?', size(t));
cha(ok) = char(ph(idx(ok)));    % ASCII -> char ('f','t','l',...)

S.cha = cha;
end
