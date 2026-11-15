function [x19,u,obstacle]=CBF_V11_Examples(caseId,s,kind,distance)
%CBF_V11_EXAMPLES  非平衡姿勢と吊り荷の張力を明示した共通試験状態。
% 実入力の与え方は物理モデルに対して同一。推定質量は吊り荷のみ。
switch caseId
    case 1, roll=0; pitch=0; ctilt=0; vz=0; omega=[0;0;0];
    case 2, roll=0; pitch=0.20; ctilt=0; vz=-0.25; omega=[0;0;0];
    case 3, roll=0; pitch=0; ctilt=0.12; vz=-0.65; omega=[0;0;0];
    case 4, roll=0.15; pitch=0.20; ctilt=-0.12; vz=0.15; omega=[0.2;-0.1;0];
    otherwise, error('CBF:V11Case','ケースは1～4です。');
end
cr=cos(roll/2);sr=sin(roll/2);cp=cos(pitch/2);sp=sin(pitch/2);
quat=[cr*cp;sr*cp;cr*sp;-sr*sp];
qc=[sin(ctilt);0;-cos(ctilt)];
x19=zeros(19,1);
x19(1:4)=quat; x19(5:7)=omega;
x19(8:10)=[0;0;1.5];x19(11:13)=[0.2;0;vz];
x19(14:16)=qc; x19(17:19)=[0;0.03;0];
x19(17:19)=x19(17:19)-qc*(qc.'*x19(17:19));
% 初期推力は姿勢に応じた鉛直補正（非平衡状態を無理に平衡とは呼ばない）。
T=(s.physical.massDrone+s.physical.massLoad)*s.physical.gravity/ ...
  max(0.5,cos(roll)*cos(pitch));
u=[T;0;0;0];
if strcmp(kind,'sphere')
    obstacle=struct('type','sphere','center',[distance;0;1.5], 'radius',0.25);
else
    th=pi/5;R=[cos(th),-sin(th),0;sin(th),cos(th),0;0,0,1];
    obstacle=struct('type','ellipsoid','center',[distance;0;1.5], ...
                   'axes',[0.25;0.38;0.50],'R',R);
end
end
