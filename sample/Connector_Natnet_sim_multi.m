function motive = Connector_Natnet_sim_multi(dt, drones, opts)
% Connector_Natnet_sim(N,dt,num)
% assign : {{1,["p","q"]},{2,["pL","qL"]}}
% dt : sampling time% num : on_marker_num = 4*N+num
% noise : 1 means active
arguments
    dt
    drones
    opts.sigmaw = [6.716E-5; 7.058E-5; 7.058E-5];
    opts.Flag = struct('Noise', 0); % 1 : active
end

Nactors = numel(drones);  % agentの数（牽引物+ドローン）
natnet_param.dt = dt;
natnet_param.rigid_num = Nactors;
natnet_param.Flag = opts.Flag;
natnet_param.sigmaw = opts.sigmaw;

% assignを生成
assign = cell(1, Nactors);

for i=1:Nactors
    if i==1 %牽引物
        assign{i} = {1, "p", "Q"};   % Qを使う
    else %ドローン
        assign{i} = {i, "p", "q"}; % qを使う
    end

end 
natnet_param.assignment = assign;
% NATNET_CONNECTOR_SIM を作成
motive = NATNET_CONNECTOR_SIM(natnet_param);
end


