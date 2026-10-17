%% generateAltitude4_ThrustExtension_ZOH.m
% 21状態 [x19;u1;du1] の吊り荷高度 z_L の2～4階 Lie 微分を生成。
% 障害物HOCBFの生成ファイルは変更しない。
% 生成先: このスクリプトと同じフォルダの Altitude4_ThrustExtension_ZOH.m
% 必要: Symbolic Math Toolbox, FLxyDst.m, GLxyDst.m
%
% [z,zdot,zddot,zjerk,z4drift,z4gain] = ...
%      Altitude4_ThrustExtension_ZOH(xAug,P9)
% P9=[mass,jx,jy,jz,gravity,loadmass,cableL,dstx,dsty]
% z''''=z4drift+z4gain*[nu;u2;u3;u4],
% xAug=[x19;u1;du1]. ただしこれは連続時間拡張モデルの厳密式。
% 実際のプラントへの入力は各制御周期ごとにZOHされる。

outName = 'Altitude4_ThrustExtension_ZOH';
scriptDir = fileparts(mfilename('fullpath'));
if isempty(scriptDir), scriptDir = pwd; end
assert(exist('FLxyDst','file')==2 && exist('GLxyDst','file')==2, ...
    'FLxyDst.m / GLxyDst.m をMATLABパスに置いてください。');
fprintf('\n=== Altitude 4th-order Lie model for thrust extension ===\n');

syms q0 q1 q2 q3 o1 o2 o3 real
syms pl1 pl2 pl3 dpl1 dpl2 dpl3 pT1 pT2 pT3 ol1 ol2 ol3 real
syms m jx jy jz gravity mL cableL dstx dsty real
syms u1 du1 real
x19 = [q0;q1;q2;q3;o1;o2;o3;pl1;pl2;pl3; ...
       dpl1;dpl2;dpl3;pT1;pT2;pT3;ol1;ol2;ol3];
P9 = [m,jx,jy,jz,gravity,mL,cableL,dstx,dsty];
P22 = sym(zeros(1,22));
P22([1,6,7,8,9,19,20,21,22])=P9;
f19=FLxyDst(x19,P22);
g19=GLxyDst(x19,P22);
assert(isequal(size(f19),[19,1])&&isequal(size(g19),[19,4]), ...
    '19状態モデルの次元が不一致です。');

xAug=[x19;u1;du1];
fAug=[f19+g19(:,1)*u1;du1;sym(0)];
gAug=[[sym(zeros(19,1));sym(0);sym(1)], ...
      [g19(:,2:4);sym(zeros(2,3))]];

fprintf('[STEP] z, z-dot, z-ddot, z-jerk, z4を導出中...\n');
z0=xAug(10);
z1=xAug(13);
z2=jacobian(z1,xAug)*fAug;
z3=jacobian(z2,xAug)*fAug;
z4drift=jacobian(z3,xAug)*fAug;
z4gain=jacobian(z3,xAug)*gAug;

% 4階未満に [nu,u2,u3,u4] が現れないことを記号的に検証
for zlow={z0,z1,z2}
    assert(isequal(jacobian(zlow{1},xAug)*gAug,sym(zeros(1,4))), ...
        '高度出力の相対次数が想定と一致しません。');
end
assert(isequal(jacobian(z2,xAug)*gAug,sym(zeros(1,4))), ...
    'zddotに操作入力が直接現れています。');
assert(~isequal(z4gain(1),sym(0)),'nuの4階係数が恒等的にゼロです。');
fprintf('[PASS] zの相対次数4、nu係数は非零式\n');

outFile=fullfile(scriptDir,[outName '.m']);
matlabFunction(z0,z1,z2,z3,z4drift,z4gain, ...
    'File',outFile,'Vars',{xAug,P9}, ...
    'Outputs',{'z','zdot','zddot','zjerk','z4drift','z4gain'}, ...
    'Optimize',true);
rehash;
assert(exist(outFile,'file')==2,'生成ファイルが見つかりません。');
fprintf('[PASS] Generated %s\n',outFile);

% 生成関数を複数の物理的に有効な状態で数値微分と照合。
rng(1017,'twister');
Pn=[0.75,0.02,0.025,0.035,9.81,0.14,2.0,0,0];
P22n=zeros(1,22);P22n([1,6,7,8,9,19,20,21,22])=Pn;
maxZ2=0;maxZ4=0;maxNu=0;
for k=1:6
    qn=[1;0.06*randn(3,1)]; qn=qn/norm(qn);
    ptn=[0.05*randn(2,1);-1];ptn=ptn/norm(ptn);
    oln=0.05*randn(3,1);oln=oln-ptn*(ptn'*oln);
    x=[qn;0.05*randn(3,1);randn(3,1);0.1*randn(3,1);ptn;oln];
    xe=[x;7+2*rand;0.05*randn];
    ff=FLxyDst(x,P22n);gg=GLxyDst(x,P22n);
    fa=[ff+gg(:,1)*xe(20);xe(21);0];
    ga=[[zeros(19,1);0;1], [gg(:,2:4);zeros(2,3)]];
    u=[15*(rand-0.5);0.05*randn(3,1)];
    [~,~,a,j,d4,g4]=feval(outName,xe,Pn);
    eps0=1e-5;
    [~,~,~,jPlus]=feval(outName,xe+eps0*(fa+ga*u),Pn);
    [~,~,~,jMinus]=feval(outName,xe-eps0*(fa+ga*u),Pn);
    err4=abs((jPlus-jMinus)/(2*eps0)-(d4+g4(:).'*u));
    maxZ4=max(maxZ4,err4);
    maxZ2=max(maxZ2,abs(a-(ff(13)+gg(13,1)*xe(20))));
    maxNu=max(maxNu,abs(g4(1)-gg(13,1)));
end
fprintf('[CHECK] max |zddot-model|  = %.3e\n',maxZ2);
fprintf('[CHECK] max |z4-FD|       = %.3e\n',maxZ4);
fprintf('[CHECK] max |Gnu-Gz,u1|  = %.3e\n',maxNu);
assert(maxZ2<1e-8 && maxZ4<1e-3 && maxNu<1e-8, ...
    '高度4階制御式の数値照合が失敗しました。');
fprintf('[PASS] 高度4階モデルの数値照合完了。\n');
fprintf('[NOTE] ZOHの推力プラントとの閉ループ安定性はSimで別途検証。\n');
