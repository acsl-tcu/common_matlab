function Controller = Controller_HL_Hinf_Suspended_Load(dt, agent)  %example
% H∞ 状態フィードバック制御器 設計（単機牽引ドローン用）
% 構成：外乱Wdstはプラント出力（位置）に加算、測定ノイズWnoiseはその後に加算
%       Wpos は外乱込みの位置、Wact は Act の出力(act*)を評価
%       hinfsyn（公称モデル）→ 閉ループ極 → place で静的ゲインF1〜F4を作成

A2 = diag(1, 1);
B2 = [0; 1];
C2 = eye(2);
A6 = diag([1,1,1,1,1], 1);
B6 = [0;0;0;0;0;1];
C6 = eye(6);

Controller.P = agent.parameter.get();

s = tf('s');

%% プラント定義（公称モデル）
% 位置の出力名は 0 付き（外乱を足す前の生の位置）。足した後を px 等と呼ぶ。
Z = ss(A2, B2, C2, 0);
Z.InputName  = {'uz'};
Z.OutputName = {'pz0'; 'vz'};

X = ss(A6, B6, C6, 0);
X.InputName  = {'ux'};
X.OutputName = {'pLx';'vLx';'px0';'vx';'axd';'jx'};

Y = ss(A6, B6, C6, 0);
Y.InputName  = {'uy'};
Y.OutputName = {'pLy';'vLy';'py0';'vy';'ay';'jy'};

Yaw = ss(A2, B2, C2, 0);
Yaw.InputName  = {'uyaw'};
Yaw.OutputName = {'pyaw0'; 'vyaw'};

if class(agent.plant) ~= "DRONE_EXP_MODEL" % sim用重み
    % 外乱の重み：低周波の外乱（ゲイン1、帯域 wd_dst [rad/s]）。
    % ss(1.0)（全周波数一定）だと、外乱が Wpos=(s/wb+1) に直接入って非プロパーになり hinfsyn が失敗する
    wd_dst = 10;
    Wd = tf(1, [1/wd_dst 1]);
    Wdst.z   = Wd; Wdst.z.u   = 'dz';   Wdst.z.y   = 'rz';
    Wdst.x   = Wd; Wdst.x.u   = 'dx';   Wdst.x.y   = 'rx';
    Wdst.y   = Wd; Wdst.y.u   = 'dy';   Wdst.y.y   = 'ry';
    Wdst.yaw = Wd; Wdst.yaw.u = 'dyaw'; Wdst.yaw.y = 'ryaw';

    Wact.z   = 0.8*tf([1 2],  [1 20]);  Wact.z.u   = 'actz';   Wact.z.y   = 'e1z';
    Wact.x   = 0.8*tf([1 0.5],[1 5]);   Wact.x.u   = 'actx';   Wact.x.y   = 'e1x';
    Wact.y   = 0.8*tf([1 0.5],[1 5]);   Wact.y.u   = 'acty';   Wact.y.y   = 'e1y';
    Wact.yaw = 0.8*tf([1 1],  [1 10]);  Wact.yaw.u = 'actyaw'; Wact.yaw.y = 'e1yaw';

    PosTarget.z   = tf(1, [1/2.0 1]);
    PosTarget.x   = tf(1, [1/0.5 1]);
    PosTarget.yaw = tf(1, [1/1.0 1]);
    Wpos.z   = 1/PosTarget.z;   Wpos.z.u   = 'pz';   Wpos.z.y   = 'e2z';
    Wpos.x   = 1/PosTarget.x;   Wpos.x.u   = 'px';   Wpos.x.y   = 'e2x';
    Wpos.y   = 1/PosTarget.x;   Wpos.y.u   = 'py';   Wpos.y.y   = 'e2y';
    Wpos.yaw = 1/PosTarget.yaw; Wpos.yaw.u = 'pyaw'; Wpos.yaw.y = 'e2yaw';

    % 測定ノイズ重み
    Wnoise.z   = ss(0.01); Wnoise.z.u   = 'nz';   Wnoise.z.y   = 'Wnz';
    Wnoise.x   = ss(0.01); Wnoise.x.u   = 'nx';   Wnoise.x.y   = 'Wnx';
    Wnoise.y   = ss(0.01); Wnoise.y.u   = 'ny';   Wnoise.y.y   = 'Wny';
    Wnoise.yaw = ss(0.01); Wnoise.yaw.u = 'nyaw'; Wnoise.yaw.y = 'Wnyaw';

else    % exp用重み（保守的）

    % 外乱の重み：低周波の外乱（ゲイン1、帯域 wd_dst [rad/s]）。
    % ss(1.0)（全周波数一定）だと、外乱が Wpos=(s/wb+1) に直接入って非プロパーになり hinfsyn が失敗する
    wd_dst = 10;
    Wd = tf(1, [1/wd_dst 1]);
    Wdst.z   = Wd; Wdst.z.u   = 'dz';   Wdst.z.y   = 'rz';
    Wdst.x   = Wd; Wdst.x.u   = 'dx';   Wdst.x.y   = 'rx';
    Wdst.y   = Wd; Wdst.y.u   = 'dy';   Wdst.y.y   = 'ry';
    Wdst.yaw = Wd; Wdst.yaw.u = 'dyaw'; Wdst.yaw.y = 'ryaw';

    Wact.z   = 0.8*tf([1 1],  [1 10]);  Wact.z.u   = 'actz';   Wact.z.y   = 'e1z';
    Wact.x   = 0.8*tf([1 0.3],[1 3]);   Wact.x.u   = 'actx';   Wact.x.y   = 'e1x';
    Wact.y   = 0.8*tf([1 0.3],[1 3]);   Wact.y.u   = 'acty';   Wact.y.y   = 'e1y';
    Wact.yaw = 0.8*tf([1 0.5],[1 5]);   Wact.yaw.u = 'actyaw'; Wact.yaw.y = 'e1yaw';

    PosTarget.z   = tf(1, [1/1.0 1]);
    PosTarget.x   = tf(1, [1/0.3 1]);
    PosTarget.yaw = tf(1, [1/0.5 1]);
    Wpos.z   = 1/PosTarget.z;   Wpos.z.u   = 'pz';   Wpos.z.y   = 'e2z';
    Wpos.x   = 1/PosTarget.x;   Wpos.x.u   = 'px';   Wpos.x.y   = 'e2x';
    Wpos.y   = 1/PosTarget.x;   Wpos.y.u   = 'py';   Wpos.y.y   = 'e2y';
    Wpos.yaw = 1/PosTarget.yaw; Wpos.yaw.u = 'pyaw'; Wpos.yaw.y = 'e2yaw';

    % 測定ノイズ重み
    Wnoise.z   = ss(0.01); Wnoise.z.u   = 'nz';   Wnoise.z.y   = 'Wnz';
    Wnoise.x   = ss(0.01); Wnoise.x.u   = 'nx';   Wnoise.x.y   = 'Wnx';
    Wnoise.y   = ss(0.01); Wnoise.y.u   = 'ny';   Wnoise.y.y   = 'Wny';
    Wnoise.yaw = ss(0.01); Wnoise.yaw.u = 'nyaw'; Wnoise.yaw.y = 'Wnyaw';
end

%% アクチュエータ（評価用の枝。出力名は Wact の入力名 act* に合わせる）
% hinfsyn では不確かさ unc.* は使わず、Act は実質ゲイン1として扱われる
rng("default")

ActNom.z = tf(1,1);
ra.z    = makeweight(0.05, 8, 1.5);
unc.z    = ultidyn('uncz',[1 1],'SampleStateDim',5);
Act.z    = ActNom.z + ra.z*unc.z;
Act.z.InputName  = 'uz';
Act.z.OutputName = 'actz';

ActNom.x = tf(1,1);
ra.x    = makeweight(0.05, 3, 1.5);
unc.x    = ultidyn('uncx',[1 1],'SampleStateDim',5);
Act.x    = ActNom.x + ra.x*unc.x;
Act.x.InputName  = 'ux';
Act.x.OutputName = 'actx';

ActNom.y = tf(1,1);
ra.y    = makeweight(0.05, 3, 1.5);
unc.y    = ultidyn('uncy',[1 1],'SampleStateDim',5);
Act.y    = ActNom.y + ra.y*unc.y;
Act.y.InputName  = 'uy';
Act.y.OutputName = 'acty';

ActNom.yaw = tf(1,1);
ra.yaw    = makeweight(0.05, 5, 1.5);
unc.yaw    = ultidyn('uncyaw',[1 1],'SampleStateDim',5);
Act.yaw    = ActNom.yaw + ra.yaw*unc.yaw;
Act.yaw.InputName  = 'uyaw';
Act.yaw.OutputName = 'actyaw';

%% 外乱rをプラント出力（位置）に加算する合流点
% 足す前: pz0 / 足した後: pz （Wpos と meas はこの pz を読む）
dst.z   = sumblk('pz   = pz0   + rz');
dst.x   = sumblk('px   = px0   + rx');
dst.y   = sumblk('py   = py0   + ry');
dst.yaw = sumblk('pyaw = pyaw0 + ryaw');

%% 測定ブロック（外乱を足した後の信号に、測定ノイズを加算）
meas.pz   = sumblk('y1z = pz + Wnz');
meas.vz   = sumblk('y2z = vz + Wnz');

meas.pLx  = sumblk('y1x = pLx + Wnx');
meas.vLx  = sumblk('y2x = vLx + Wnx');
meas.px   = sumblk('y3x = px + Wnx');
meas.vx   = sumblk('y4x = vx + Wnx');
meas.axd  = sumblk('y5x = axd + Wnx');
meas.jx   = sumblk('y6x = jx + Wnx');

meas.pLy  = sumblk('y1y = pLy + Wny');
meas.vLy  = sumblk('y2y = vLy+Wny ');
meas.py   = sumblk('y3y = py + Wny');
meas.vy   = sumblk('y4y = vy + Wny');
meas.ay   = sumblk('y5y = ay + Wny');
meas.jy   = sumblk('y6y = jy + Wny');

meas.pyaw = sumblk('y1yaw = pyaw + Wnyaw');
meas.vyaw = sumblk('y2yaw = vyaw + Wnyaw');
%% ブロック線図接続
% z軸
IC.z = connect( ...
    Z, Act.z, ...              % uz → プラント → pz0, vz / Act.z は actz を出す評価用の枝
    Wdst.z, dst.z, ...         % dz → rz、pz = pz0 + rz（外乱を先に足す）
    Wpos.z, Wact.z, ...        % 評価：pz → e2z、actz → e1z
    Wnoise.z, ...              % nz → Wnz
    meas.pz, meas.vz, ...      % 後でノイズを足す：y1z = pz + Wnz、y2z = vz + Wnz
    {'dz';'nz';'uz'}, ...
    {'e1z';'e2z';'y1z';'y2z'});

% x軸（y1x=pLx, y2x=vLx, y3x=px, y4x=vx, y5x=axd, y6x=jx）
IC.x = connect( ...
    X, Act.x, ...
    Wdst.x, dst.x, ...
    Wpos.x, Wact.x, ...
    Wnoise.x, ...
    meas.pLx, meas.vLx, meas.px, meas.vx, meas.axd, meas.jx, ...
    {'dx';'nx';'ux'}, ...
    {'e1x';'e2x'; ...
    'y1x';'y2x';'y3x';'y4x';'y5x';'y6x'});

% y軸（y1y=pLy, y2y=vLy, y3y=py, y4y=vy, y5y=ay, y6y=jy）
IC.y = connect( ...
    Y, Act.y, ...
    Wdst.y, dst.y, ...
    Wpos.y, Wact.y, ...
    Wnoise.y, ...
    meas.pLy, meas.vLy, meas.py, meas.vy, meas.ay, meas.jy, ...
    {'dy';'ny';'uy'}, ...
    {'e1y';'e2y'; ...
    'y1y';'y2y';'y3y';'y4y';'y5y';'y6y'});

% yaw軸
IC.yaw = connect( ...
    Yaw, Act.yaw, ...
    Wdst.yaw, dst.yaw, ...
    Wpos.yaw, Wact.yaw, ...
    Wnoise.yaw, ...
    meas.pyaw, meas.vyaw, ...
    {'dyaw';'nyaw';'uyaw'}, ...
    {'e1yaw';'e2yaw';'y1yaw';'y2yaw'});

% サイズ確認
[ny.z,   nu.z]   = size(IC.z);
[ny.x,   nu.x]   = size(IC.x);
[ny.y,   nu.y]   = size(IC.y);
[ny.yaw, nu.yaw] = size(IC.yaw);
fprintf('IC.z:   出力=%d, 入力=%d\n', ny.z,   nu.z);
fprintf('IC.x:   出力=%d, 入力=%d\n', ny.x,   nu.x);
fprintf('IC.y:   出力=%d, 入力=%d\n', ny.y,   nu.y);
fprintf('IC.yaw: 出力=%d, 入力=%d\n', ny.yaw, nu.yaw);

%% H∞設計（4軸とも公称モデルでhinfsyn。不確かさunc.*はここでは使わない）
nmeas.n2 = 2;
nmeas.n6 = 6;
ncon     = 1;

[K1, ~, gamma1] = hinfsyn(IC.z,   nmeas.n2, ncon);
fprintf('z   gamma = %.4f\n', gamma1);

[K2, ~, gamma2] = hinfsyn(IC.x,   nmeas.n6, ncon);
fprintf('x   gamma = %.4f\n', gamma2);

[K3, ~, gamma3] = hinfsyn(IC.y,   nmeas.n6, ncon);
fprintf('y   gamma = %.4f\n', gamma3);

[K4, ~, gamma4] = hinfsyn(IC.yaw, nmeas.n2, ncon);
fprintf('yaw gamma = %.4f\n', gamma4);

%% （次数低減は行わない：初期版に合わせて無効化）
% K1 = reduce(K1, min(order(K1), 4));
% K2 = reduce(K2, min(order(K2), 8));
% K3 = reduce(K3, min(order(K3), 8));
% K4 = reduce(K4, min(order(K4), 4));

%% 閉ループ極から静的ゲイン抽出
[AK1,BK1,CK1,DK1] = ssdata(ss(K1));
[AK2,BK2,CK2,DK2] = ssdata(ss(K2));
[AK3,BK3,CK3,DK3] = ssdata(ss(K3));
[AK4,BK4,CK4,DK4] = ssdata(ss(K4));

A.cl.z   = [A2+B2*DK1*C2, B2*CK1; BK1*C2, AK1];
A.cl.x   = [A6+B6*DK2*C6, B6*CK2; BK2*C6, AK2];
A.cl.y   = [A6+B6*DK3*C6, B6*CK3; BK3*C6, AK3];
A.cl.yaw = [A2+B2*DK4*C2, B2*CK4; BK4*C2, AK4];

poles.z   = sort(eig(A.cl.z),   'ComparisonMethod', 'real');
poles.x   = sort(eig(A.cl.x),   'ComparisonMethod', 'real');
poles.y   = sort(eig(A.cl.y),   'ComparisonMethod', 'real');
poles.yaw = sort(eig(A.cl.yaw), 'ComparisonMethod', 'real');

fprintf('z  閉ループ極: '); disp(poles.z.');
fprintf('x  閉ループ極: '); disp(poles.x.');
fprintf('y  閉ループ極: '); disp(poles.y.');
fprintf('yaw閉ループ極: '); disp(poles.yaw.');

%% placeで静的ゲイン設計（共役ペアを崩さずに極を選ぶ）
try
    Controller.F1 = place(A2, B2, pickpoles(poles.z, 2));
catch
    error('z: place失敗 閉ループ極を確認してください');
end

try
    Controller.F2 = place(A6, B6, pickpoles(poles.x, 6));
    % Controller.F3 = Controller.F2;
catch
    error('x/y: place失敗 閉ループ極を確認してください');
end

try
    Controller.F3 = place(A6 ,B6, pickpoles(poles.y, 6));
catch
    error('y: place失敗 閉ループ極を確認してください');
end


try
    Controller.F4 = place(A2, B2, pickpoles(poles.yaw, 2));
catch
    error('yaw: place失敗 閉ループ極を確認してください');
end

%% 確認
fprintf('z  固有値: ');   disp(eig(A2-B2*Controller.F1).');
fprintf('x  固有値: ');   disp(eig(A6-B6*Controller.F2).');
fprintf('y  固有値: ');   disp(eig(A6-B6*Controller.F3).');
fprintf('yaw固有値: ');   disp(eig(A2-B2*Controller.F4).');

disp('F1 ='); disp(Controller.F1);
disp('F2 ='); disp(Controller.F2);
disp('F3 ='); disp(Controller.F3);
disp('F4 ='); disp(Controller.F4);

Controller.dt = dt;
eig(A6 - B6*Controller.F2)
end

%% ローカル関数：実部が小さい（左側）順に、共役ペアを崩さず n 個の極を選ぶ
function p = pickpoles(P, n)
P = P(:);
[~, idx] = sort(real(P));
P = P(idx);
p = [];
used = false(numel(P), 1);
for k = 1:numel(P)
    if used(k), continue; end
    used(k) = true;
    if abs(imag(P(k))) < 1e-9*max(1, abs(P(k)))
        % 実極
        if numel(p) + 1 <= n
            p(end+1, 1) = real(P(k)); %#ok<AGROW>
        end
    else
        % 複素極：共役の相手を探して2個セットで扱う
        d = abs(P - conj(P(k)));
        d(used) = inf;
        [dm, j] = min(d);
        if ~isempty(j) && dm < 1e-6*max(1, abs(P(k)))
            used(j) = true;
            if numel(p) + 2 <= n
                p = [p; P(k); conj(P(k))]; %#ok<AGROW>
            end
        end
    end
    if numel(p) == n, break; end
end
if numel(p) < n
    error('pickpoles: 共役ペアを崩さずに %d 個選べませんでした', n);
end
end