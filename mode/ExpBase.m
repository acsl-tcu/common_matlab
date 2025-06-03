%本来のExpBaseは一つ目のセクションのみ
% for i = 1:length(agent)
%   agent(i).reference.takeoff = TAKEOFF_REFERENCE(agent(i),[]);
%   agent(i).reference.landing = LANDING_REFERENCE(agent(i),dt,0.1);
%   if isfield(agent,"cha_allocation")
%      if  isfield(agent(i).cha_allocation,"t")
%         agent(i).cha_allocation.t.reference = "takeoff";
%      end   
%      if  isfield(agent(i).cha_allocation,"t")
%       agent(i).cha_allocation.l.reference = "landing";
%      end
%   else
%       agent(i).cha_allocation = struct("t",struct("reference","takeoff"),"l",struct("reference","landing"));
%   end
% end
%% SimMEC_Kを動かすために無理やり付けた(mianGUI.mの形に合わせるとき修正必須：工藤6/3(火))
takeoff_ref = TAKEOFF_REFERENCE(agent,[]);
landing_ref = LANDING_REFERENCE(agent,dt,0.1);
