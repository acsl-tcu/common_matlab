% function Xd = gen_ref_for_hovering(X)
% %% Setting
% 
% %% Variable set
%     Xd = X(1:3);
% end
function ref = gen_ref_for_hovering(param)

arguments
    param.position = [0;0;1];
end

ref = @(t) param.position;