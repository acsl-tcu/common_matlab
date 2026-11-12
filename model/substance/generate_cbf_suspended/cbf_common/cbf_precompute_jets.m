function cache=cbf_precompute_jets(mode,x,settings)
%CBF_PRECOMPUTE_JETS 状態に依存する6次Taylor係数を1周期あたり1回計算する。
% 障害物の数・半径・姿勢・位置に依存しないため、同一QPの全障害物で共有。
% nu=0と4個の単位基底、計5回。毎周期作り直す（状態変化で再利用不可）。
% シーン以外の単体CBFでは従来どおり内部で計算できる。
P=cbf_parameter_vector(settings);
K=6; jets=cell(5,1);
for i=0:4
    u=zeros(4,1);
    if i>0,u(i)=1;end
    if strcmp(mode,'real')
        Z=cbf_real_state_taylor(x,u,P,K);
        lp=Z(8:10,:);q=Z(14:16,:);pd=lp-P(3)*q;
    elseif strcmp(mode,'virtual')
        [lp,q,pd]=cbf_virtual_state_taylor(x,u,P,K);
    else
        error('CBF:Mode','real / virtual のどちらかを指定してください。');
    end
    jets{i+1}=struct('pLoad',lp,'qCable',q,'pDrone',pd);
end
cache=struct('mode',mode,'P',P,'jets',{jets});
end
