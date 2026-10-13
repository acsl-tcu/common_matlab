function CLic = build_closed_loop_ic(P, K)
% build_closed_loop_ic  状態を保った負帰還閉ループを作る（初期値応答の評価用）.
%   K は e = r - y を入力, u を出力とする制御器（負帰還 u=K(r-y)）.
%   出力 CLic は入力を持たない自律系 ss で, 初期状態 [xp; xk] から initial() で回す.
Pss = ss(P);  Kss = ss(K);
Ap=Pss.A; Bp=Pss.B; Cp=Pss.C;             % Dp=0 前提（プラントに直達項なし）
Ak=Kss.A; Bk=Kss.B; Ck=Kss.C; Dk=Kss.D;   % u = Ck xk + Dk e,  e = -y = -Cp xp
% xp' = (Ap - Bp Dk Cp) xp + Bp Ck xk
% xk' =  -Bk Cp xp        + Ak xk
Acl = [Ap - Bp*Dk*Cp,  Bp*Ck;
       -Bk*Cp,         Ak    ];
Ccl = [Cp, zeros(1, size(Ak,1))];
CLic = ss(Acl, zeros(size(Acl,1),0), Ccl, zeros(1,0));
end
