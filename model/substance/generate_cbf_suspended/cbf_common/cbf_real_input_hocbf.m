function [A,b,labels,psi]=cbf_real_input_hocbf(x,settings)
%CBF_REAL_INPUT_HOCBF  実入力拡張器の上下限を高次CBF不等式に変換。
% 決定入力: [T^(4);tau_x'';tau_y'';tau_z'']。
% thrust: 4次、tau: 2次。各変数の範囲自体も初期に満たす必要あり。
if ~isfield(settings.cbf,'inputGain') || isempty(settings.cbf.inputGain)
    error('CBF:MissingInputGain','settings.cbf.inputGainを実行時に設定してください。');
end
k=settings.cbf.inputGain;
if ~isscalar(k) || ~isfinite(k) || k<=0,error('CBF:InvalidGain','inputGainは正です。');end
u=settings.input;
T=x(20);tDer=x(21:23);tau=x(24:26);dtau=x(27:29);
if numel(u.torqueMin)~=3 || numel(u.torqueMax)~=3
    error('CBF:InputLimitSize','torqueMin/Maxは各3次元です。');
end
A=zeros(8,4);b=zeros(8,1);psi=cell(8,1);
labels={'thrust_min';'thrust_max';'tau_x_min';'tau_y_min';'tau_z_min'; ...
        'tau_x_max';'tau_y_max';'tau_z_max'};
% h=T-Tmin, Tmax-T の4次条件
for j=1:2
    if j==1,sgn=1;h0=T-u.thrustMin;else,sgn=-1;h0=u.thrustMax-T;end
    d=[h0;sgn*tDer(:)];
    A(j,1)=sgn;
    b(j)=-sum(arrayfun(@(r)nchoosek(4,r)*k^(4-r)*d(r+1),0:3));
    p=zeros(4,1);
    for r=0:3
        for i=0:r,p(r+1)=p(r+1)+nchoosek(r,i)*k^(r-i)*d(i+1);end
    end
    psi{j}=p;
end
for j=1:6
    if j<=3,i=j;sgn=1;d0=tau(i)-u.torqueMin(i);
    else,i=j-3;sgn=-1;d0=u.torqueMax(i)-tau(i);end
    idx=j+2;A(idx,i+1)=sgn;
    b(idx)=-(k^2*d0+2*k*sgn*dtau(i));
    psi{idx}=[d0;sgn*dtau(i)+k*d0];
end
end
