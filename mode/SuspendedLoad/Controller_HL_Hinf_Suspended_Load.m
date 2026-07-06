function Controller = Controller_HL_Hinf_Suspended_Load(dt, agent)
% H∞ 状態フィードバック制御器 設計（単機牽引ドローン用）
%z=1,x=2,y=3,yaw=4 に対応させた変数を作成(yawの重みw41)

% 変数の設定
% サブシステム行列
A2 = diag(1, 1);
B2 = [0; 1];
C2=[1 0];
% C2=eye(2);
A6 = diag([1,1,1,1,1], 1); % 6次積分チェーン（x, y用）
B6 = [0;0;0;0;0;1];
C6=[1 0 0 0 0 0];
% C6=eye(6);
Controller.P=agent.parameter.get();
% 状態空間モデルに変更
P2=ss(A2,B2,C2,0);%z,yawシステム
P6=ss(A6,B6,C6,0);%x,yシステム

% 重みの設定　出力と観測
s = tf('s');
if class(agent.plant) ~= "DRONE_EXP_MODEL"
    % sim用
    w12 = []; w13 = [];  % z
    w22 = []; w23 = [];  % x
    w32 = []; w33 = [];  % y
    w42 = []; w43 = [];  % yaw
else
    % exp用
    w12 = []; w13 = [];  % z
    w22 = []; w23 = [];  % x
    w32 = []; w33 = [];  % y
    w42 = []; w43 = [];  % yaw
end

% 出力の数nmeas 入力の数ncon
nmeas2=2;
nmeas6=6;
ncon=1;

% 重みを含めた伝達関数を作成
% p1 = augw(P2,[],w12,w13);
% p2 = augw(P6,[], w22, w23);
% p3 = augw(P6,[],w32,w33);
% p4 = augw(P2,[],w42,w43);

F1_lqr = lqrd(A2, B2, diag([150, 10]), 0.01, dt);
F2_lqr = lqrd(A6, B6, diag([150,150,100,10,1,0.1]), 0.001, dt);

% hinfsynでのコントローラー作成
if class(agent.plant) ~= "DRONE_EXP_MODEL"
[K1,CL1,gamma1]=hinfsyn(P2,nmeas2,ncon);
fprintf('z   gamma = %.4f\n', gamma1);
% Controller.F1=dcgain(K1);
[K2,CL2,gamma2]=hinfsyn(P6,nmeas6,ncon);
fprintf('x   gamma = %.4f\n', gamma2);
% Controller.F2=dcgain(K2);
[K3,CL3,gamma3]=hinfsyn(P6,nmeas6,ncon);
fprintf('y   gamma = %.4f\n', gamma3);
% Controller.F3=dcgain(K3);
[K4,CL4,gamma4]=hinfsyn(P2,nmeas2,ncon);
fprintf('yaw gamma = %.4f\n', gamma4);
% Controller.F4=dcgain(K4);
disp(class(K1));
disp(size(K1));
% tf型の場合
% 適切な作動周波数（例：1 rad/s）でゲインを取得
omega = 1.0;
Controller.F1 = real(evalfr(K1, 1j*omega));
Controller.F2 = real(evalfr(K2, 1j*omega));
Controller.F3 = real(evalfr(K3, 1j*omega));
Controller.F4 = real(evalfr(K4, 1j*omega));

disp('F1 ='); disp(Controller.F1);
disp('F2 ='); disp(Controller.F2);
disp('F1_lqr ='); disp(lqrd(A2,B2,diag([150,10]),0.01,dt));
disp('F2_lqr ='); disp(lqrd(A6,B6,diag([150,150,100,10,1,0.1]),0.001,dt));
fprintf('F1: %dx%d, class=%s\n', size(Controller.F1,1), size(Controller.F1,2), class(Controller.F1));
fprintf('F2: %dx%d, class=%s\n', size(Controller.F2,1), size(Controller.F2,2), class(Controller.F2));
fprintf('F3: %dx%d, class=%s\n', size(Controller.F3,1), size(Controller.F3,2), class(Controller.F3));
fprintf('F4: %dx%d, class=%s\n', size(Controller.F4,1), size(Controller.F4,2), class(Controller.F4));
disp('F1 ='); disp(Controller.F1);
disp('F2 ='); disp(Controller.F2);
disp('F3 ='); disp(Controller.F3);
disp('F4 ='); disp(Controller.F4);
% lqrdで同じ系に対して設計した場合
F1_lqr = lqrd(A2, B2, diag([150, 10]), 0.01, dt);
F2_lqr = lqrd(A6, B6, diag([150,150,100,10,1,0.1]), 0.001, dt);
disp('F1_lqr ='); disp(F1_lqr);
disp('F2_lqr ='); disp(F2_lqr);
else
 [K1,CL1,gamma1]=hinfsyn(p1,nmeas,ncon);
    fprintf('z   gamma = %.4f\n', gamma1);
    Controller.F1=K1;
    [K2,CL2,gamma2]=hinfsyn(p2,nmeas,ncon);
    fprintf('x   gamma = %.4f\n', gamma2);
    Controller.F2=K2;
    [K3,CL3,gamma3]=hinfsyn(p3,nmeas,ncon);
    fprintf('y   gamma = %.4f\n', gamma3);
    Controller.F3=K3;
    [K4,CL4,gamma4]=hinfsyn(p4,nmeas,ncon);
    fprintf('yaw gamma = %.4f\n', gamma4);
    Controller.F4=K4;
end
Controller.dt = dt;
eig(diag([1,1,1,1,1],1)-[0;0;0;0;0;1]*Controller.F2)

end

% %%  コントローラー作成の可能化の判断
% %ここを変更してそれぞれのサブシステムで判断
% size(p1);
% [A,B,C,D] = ssdata(p1);
%
% B1 = B(:,1:nw);
% B2 = B(:,nw+1:nw+nu);
% C1 = C(1:nz,:);
% C2 = C(nz+1:nz+ny,:);
% D11 = D(1:nz,1:nw);
% D12 = D(1:nz,nw+1:nw+nu);
% D21 = D(nz+1:nz+ny,1:nw);
% D22 = D(nz+1:nz+ny,nw+1:nw+nu);
%
% %(A,B2)に依る可安定の判断
% rank_ctrb = rank(ctrb(A,B2));
% if rank_ctrb == size(A,1)
%     disp('(A,B2) は可制御（したがって可安定）')
% else
%     disp('(A,B2) は可制御でない')
% end
%
% %(C2,A)に依る可検出の判断
% rank_obsv = rank(obsv(A,C2));
% if rank_obsv == size(A,1)
%     disp('(C2,A) は可観測（したがって可検出）')
% else
%     disp('(C2,A) は可観測でない')
% end
%
% %D12の列のフルランクの確認
% if rank(D12) == size(D12,2)
%     disp('D12 は列フルランク')
% else
%     disp('D12 は列フルランクでない')
% end
%
% %D21の行のフルランクの確認
% if rank(D21) == size(D21,1)
%     disp('D21 は行フルランク')
% else
%     disp('D21 は行フルランクでない')
% end
%
%
% w = logspace(-3,3,1000);% 調べる周波数範囲
%
% % 条件を満たしているかどうかのフラグ
% flag_M1 = true;
% flag_M2 = true;
%
% n1 = size([A B2],2);% M1の列数
% m2 = size([A B1],1);% M2の行数
%
% % 全ての周波数で確認
% for k = 1:length(w)
%     jw = 1j*w(k);
%     M1 = [A-jw*eye(size(A)) B2;C1 D12];
%     % [A-jωI B1; C2 D21]
%     M2 = [A-jw*eye(size(A))  B1; C2 D21];
%
%     if rank(M1) < n1   % M1が列フルランクか判定
%         flag_M1 = false;
%         fprintf('M1 is not column full rank at w = %g rad/s\n',w(k));
%         break
%     end
%
%     if rank(M2) < m2 % M2が行フルランクか判定
%         flag_M2 = false;
%         fprintf('M2 is not row full rank at w = %g rad/s\n',w(k));
%         break
%     end
% end
% % 結果表示
% if flag_M1
%     disp('[A-jwI B2; C1 D12] は全周波数で列フルランク')
% else
%     disp('[A-jwI B2; C1 D12] は列フルランクではない')
% end
% if flag_M2
%     disp('[A-jwI B1; C2 D21] は全周波数で行フルランク')
% else
%     disp('[A-jwI B1; C2 D21] は行フルランクではない')
% end