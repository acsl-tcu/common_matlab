function S = extract_series_from_logmat_cellresult(matPath)
% log.Data.agent.estimator.result が cell のログ向け
% 出力:
%  S.t, S.p, S.v, S.pL, S.vL  (N×1 or N×3)


d = load(matPath);
log = d.log;

t = log.Data.t(:);
R = log.Data.agent.estimator.result;  % cell, each R{k}.state is STATE_CLASS

N = min(numel(t), numel(R));

p  = nan(N,3); v  = nan(N,3);
pL = nan(N,3); vL = nan(N,3);

for k = 1:N
    st = R{k}.state; % STATE_CLASS

    % 必要変数（この4つが今回の評価の核）
    pk  = st.get('p');
    vk  = st.get('v');
    pLk = st.get('pL');
    vLk = st.get('vL');

    p(k,:)  = pk(:).';
    v(k,:)  = vk(:).';
    pL(k,:) = pLk(:).';
    vL(k,:) = vLk(:).';
end

S.t  = t(1:N);
S.p  = p;
S.v  = v;
S.pL = pL;
S.vL = vL;
end