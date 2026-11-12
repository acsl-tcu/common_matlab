function model = cbf_symbolic_model(mode)
%CBF_SYMBOLIC_MODEL 既存の張りケーブルモデルをCBF用拡張状態へ書く。
%
% 実入力（29状態）:
% [quaternion(4);omegaBody(3);pLoad(3);vLoad(3);pT(3);omegaCable(3);
%  T;dT;ddT;dddT;tau(3);dTau(3)]
% [T^(4); tau_x^(2);tau_y^(2);tau_z^(2)] が拡張系の決定入力。
%
% 仮想入力（20状態）:
% [pLoad(3);dpLoad(3);...;pLoad^(5)(3);yaw;dyaw]
% [pLoad_x^(6);pLoad_y^(6);pLoad_z^(6);ddYaw] が決定入力。
%
% 【重要】前述の「入力チャネルの次数」は一律の相対次数の証明ではない。
% 早期のLie微分に入力が現れる行では、通常の固定次数HOCBFは生成不可。

P = sym('P',[20 1]); O = sym('O',[15 1]);
assume(P,'real');assume(O,'real');
m=P(1);mL=P(2);L=P(3);g=P(4);J=diag(P(5:7)); e3=sym([0;0;1]);
if strcmp(mode,'real')
    X=sym('xReal',[29 1]);assume(X,'real');
    q=X(1:4);omega=X(5:7);vL=X(11:13);pT=X(14:16);omL=X(17:19);
    T=X(20); tau=X(24:26);
    E=[-q(2),q(1),-q(4),q(3);-q(3),q(4),q(1),-q(2);-q(4),-q(3),q(2),q(1)];
    Lq=[-q(2),q(1),q(4),-q(3);-q(3),-q(4),q(1),q(2);-q(4),q(3),-q(2),q(1)];
    R=E*Lq.';
    dpT=cross(omL,pT);
    domL=cross(-pT,T*R*e3)/(m*L);
    ddPL=-g*e3+(dot(pT,T*R*e3)-m*L*dot(dpT,dpT))*pT/(m+mL) ...
        +[P(14);P(15);0];
    dOmega=J\(cross(-omega,J*omega)+tau);
    F=[Lq.'*omega/2;dOmega;vL;ddPL;dpT;domL; ...
       X(21:23);0;X(27:29);zeros(3,1)];
    G=sym(zeros(29,4));G(23,1)=1;G(27:29,2:4)=sym(eye(3));
elseif strcmp(mode,'virtual')
    X=sym('xVirtual',[20 1]);assume(X,'real');
    % 6次まで一貫した吊り荷の理想平坦出力ジェット。
    F=[X(4:18);zeros(3,1);X(20);0];
    G=sym(zeros(20,4));G(16:18,1:3)=sym(eye(3));G(20,4)=1;
else
    error('CBF:UnknownMode','real/virtualを指定してください。');
end
model=struct('mode',mode,'x',X,'p',P,'obstacle',O,'f',F,'g',G);
end
