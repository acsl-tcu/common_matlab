clear
agent = DRONE;
agent.plant = DRONE_EXP_MODEL(agent, Model_Drone_Exp(0.025,[0;0;0], "serial", "COM3"));
%%
%agent.plant.connector.sendData([1 0 0 0 0 0 0 0 0 2 3 4 0 0 0 0])
org_CH_AUX = [500 500 0 500 0 0 0 0]
% CH1,  2,      3,      4   AUX1,2,3,4
% roll, pitch,  thrust, yaw
key_msg = 'コマンドウィンドウ上にカーソルを合わせてキーボードを押すと再開します';


CH_AUX = org_CH_AUX;
for N = 1:4
    for i = 0:1000
        CH_AUX(N) = i;
        agent.plant.connector.sendData(gen_msg(CH_AUX))
        disp(CH_AUX)
        pause(0.01) % 表示時間調整用
        if i == 500
            disp(key_msg)
            pause;
        end
    end
    if N~=4
        disp(key_msg)
        pause
    end
    CH_AUX = org_CH_AUX;
end
%write(agent.plant.connector.serial,[1 0 0 0 0 0 0 0 0 2 3 4 0 0 0 0],"uint8");%[1 2 3 4 5 6 7 8 1 2 3 4 5 6 7 8],"uint8")

%%
clear