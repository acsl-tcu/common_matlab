clear
agent = DRONE(Model_Drone_Exp(0.025,[0;0;0], "serial", "COM19"),DRONE_PARAM("DIATONE"));
%%
%agent.plant.connector.sendData([1 0 0 0 0 0 0 0 0 2 3 4 0 0 0 0])
org_CH_AUX = [500 500 0 500 0 0 0 0]
% CH1,  2,      3,      4   AUX1,2,3,4
% roll, pitch,  thrust, yaw


CH_AUX = org_CH_AUX;
for N = 4
    for i = 0:1000
        CH_AUX(N) = i;
        agent.plant.connector.sendData(gen_msg(CH_AUX))
    end
    CH_AUX = org_CH_AUX;
end
%write(agent.plant.connector.serial,[1 0 0 0 0 0 0 0 0 2 3 4 0 0 0 0],"uint8");%[1 2 3 4 5 6 7 8 1 2 3 4 5 6 7 8],"uint8")

%%
clear