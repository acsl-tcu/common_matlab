function Controller= Controller_FHLMECK(dt)
% 階層型線形化コントローラの設定
%% dt = 0.025 くらいの時に有効（これより粗いdtの時はZOH誤差を無視しているためもっと穏やかなゲインの方が良い）
Ac2 = [0,1;0,0];
Bc2 = [0;1];
Ac4 = diag([1,1,1],1);
Bc4 = [0;0;0;1];
Controller.F1=lqrd(Ac2,Bc2,diag([100,1]),[0.1],dt);                                % 
Controller.F2=lqrd(Ac4,Bc4,diag([100,10,10,1]),[0.01],dt); % xdiag([100,10,10,1])
Controller.F3=lqrd(Ac4,Bc4,diag([100,10,10,1]),[0.01],dt); % ydiag([100,10,10,1])
Controller.F4=lqrd(Ac2,Bc2,diag([100,10]),[0.1],dt);                       % ヨー角
Controller.F = blkdiag(Controller.F1,Controller.F2,Controller.F3,Controller.F4);    % フィードバックゲインをまとめる

%% Koopman
% modeファイルとファイル名をそろえる
load("koopman.mat",'est'); 
% load("EstimationResult_12state_2_7_Exp_sprine+zsprine+P2Pz_torque_incon_150data_vzからz算出.mat",'est');
Controller.est = est;

syms sz1 [2 1] real
syms sF1 [1 2] real
[Ad1,Bd1,~,~] = ssdata(c2d(ss(Ac2,Bc2,[1,0],0),dt));  % 状態空間表現から係数行列A, Bを取得
Controller.Vf = matlabFunction([-sF1*sz1, -sF1*(Ad1-Bd1*sF1)*sz1, -sF1*(Ad1-Bd1*sF1)^2*sz1, -sF1*(Ad1-Bd1*sF1)^3*sz1],"Vars",{sz1,sF1});

syms sz2 [4 1] real
syms sF2 [1 4] real
syms sz3 [4 1] real
syms sF3 [1 4] real
syms sz4 [2 1] real
syms sF4 [1 2] real
Controller.Vs = matlabFunction([-sF2*sz2;-sF3*sz3;-sF4*sz4],"Vars",{sz2,sz3,sz4,sF2,sF3,sF4});
Controller.dt = dt;
Controller.type="FUNCTIONAL_HLMECKC";
Controller.name="hlkc";
Controller.param=Controller;
end

