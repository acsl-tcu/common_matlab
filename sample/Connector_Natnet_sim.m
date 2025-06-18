function motive = Connector_Natnet_sim(N, dt, opts)
% Connector_Natnet_sim(N,dt,num)
% N : number of rigid body
% dt : sampling time
% num : on_marker_num = 4*N+num
% noise : 1 means active
arguments
  N
  dt
  opts.state_name = ["p","q"];
  opts.sigmaw = [6.716E-5; 7.058E-5; 7.058E-5];
  opts.Flag = struct('Noise', 0) % 1 : active
end
natnet_param.dt = dt;
natnet_param.rigid_num = N;
natnet_param.Flag = opts.Flag; 
natnet_param.sigmaw = opts.sigmaw;

natnet_param.state_name = opts.state_name;

% N =3 でon makerを任意の数，配置に設定する例
% natnet_param.local_marker = {[ 0.075, -0.075,  0.015;-0.075, -0.075, -0.015; -0.075,  0.075,  0.015; 0.075,  0.075, -0.015],
%     [ 0.075, -0.075,  0.015;-0.075, -0.075, -0.015; -0.075,  0.075,  0.015; 0.075,  0.075, -0.015; 0.075,  0.075, -0.01; 0.07,  0.075, -0.015],
%     [ 0.075, -0.075,  0.015;-0.075, -0.075, -0.015; -0.075,  0.075,  0.015; 0.075,  0.075, -0.015; 0.075,  0.07, -0.015]};
motive = NATNET_CONNECTOR_SIM(natnet_param);
end
