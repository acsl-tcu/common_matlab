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

% --- PATCH: Tf must be unique for interp1 ---
Tf = Tf(:);  % column
[Tu, ia] = unique(Tf, 'stable');     % unique time stamps
% 以降、Tfに対応する配列は「ia」で同じように間引く必要がある
% ここで index mapping を作る
idx = interp1(Tu, 1:numel(Tu), t, 'nearest', 'extrap');
idx = ia(idx);   % 元のTf上のindexに戻す
idx = max(1, min(numel(Tf), idx));

% --- 近傍インデックスを作る（必ず何かに対応） ---
% ===== robust index mapping: handles duplicate / unsorted Tf =====
Tf = Tf(:);
tq = t(:);

% 1) drop NaN/Inf just in case
goodTf = isfinite(Tf);
Tf = Tf(goodTf);

% 2) make unique, keep first occurrence (stable)
[Tu, ia] = unique(Tf, 'stable');   % Tu: unique times, ia: indices into original Tf (after goodTf)

% 3) interp1 requires X to be strictly monotonic increasing -> sort
[Tu, ord] = sort(Tu);
ia = ia(ord);

% 4) map query time -> nearest unique index, then back to original index
idxu = interp1(Tu, 1:numel(Tu), tq, 'nearest', 'extrap');
idx  = ia(idxu);

% 5) reshape back to t's shape
idx = reshape(idx, size(t));
% ===== end =====

idx = max(1, min(numel(Tf), round(idx)));

% 一応誤差チェック（大きくズレたら '?' にする）
tol = 5e-3;  % 5ms くらい（必要なら増やしてOK）
ok = abs(Tf(idx) - t) <= tol;

cha = repmat('?', size(t));
cha(ok) = char(ph(idx(ok)));    % ASCII -> char ('f','t','l',...)

S.cha = cha;
end
