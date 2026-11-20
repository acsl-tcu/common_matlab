function gopt = hinf_gamma_opt(A, B1, B2, C1, rho_v)
% 元: ctow_hinf_gamma_opt   状態フィードバック H∞ の最適 gamma を二分探索
lo = 1e-3; hi = 1e4; %gammaが存在していると想定している範囲
[~, ~, ok] = hinf_hinf_sf(A, B1, B2, C1, rho_v, hi); %上限でゲインを求められるか
if ~ok, error('gamma = %g でも解けない。重みを見直すこと。', hi); end
for it = 1:80 %精度保証のために入れる　γ は小さいほど外乱に強い設計
    mid = sqrt(lo*hi); %次に試すγを生成　収束させていく　
    [~, ~, ok] = hinf_hinf_sf(A, B1, B2, C1, rho_v, mid); %ここで繰り替えし
    if ok %それぞれの解を保管
        hi = mid; %解あり
    else 
        lo = mid; %解なし
    end
end
gopt = hi; %解が求まったものを保管する
end