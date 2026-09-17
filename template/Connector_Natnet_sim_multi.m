function motive = Connector_Natnet_sim_multi(dt, N, opts)
% Connector_Natnet_sim(N,dt,num)
% assign : {{1,["p","q"]},{2,["pL","qL"]}}
% dt : sampling time% num : on_marker_num = 4*N+num
% noise : 1 means active
arguments
    dt
    N
    opts.sigmaw = [6.716E-5; 7.058E-5; 7.058E-5];
    opts.Flag = struct('Noise', 0); % 1 : active
end

Nactors = N + 1;  % agentの数（牽引物+ドローン）

natnet_param.dt = dt;
natnet_param.rigid_num = Nactors;
natnet_param.Flag = opts.Flag;
natnet_param.sigmaw = opts.sigmaw;

% --- assignを生成 ---
assign = {};
for i = 1:Nactors
    if i == 1
        % 牽引物
        assign{end+1} = {i, "p", "Q"};
    else
        % ドローン
        assign{end+1} = {i, "p", "q"};
        assign{end+1} = {i, "pL", "pT"};
    end
end

natnet_param.assignment = assign;

% --- NATNET_CONNECTOR_SIM を作成 ---
motive = NATNET_CONNECTOR_SIM(natnet_param);
end