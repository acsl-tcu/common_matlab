function dX = zup_eul_cable_suspended_rigid_body_with_6_drones(x,u,P)
% Input order:
% x (state) = [];
% u (input) = [];
% P (parameter) = ["g", "m0", "j01", "j02", "j03", "rho1_1", "rho2_1", "rho3_1", "rho1_2", "rho2_2", "rho3_2", "rho1_3", "rho2_3", "rho3_3", "rho1_4", "rho2_4", "rho3_4", "rho1_5", "rho2_5", "rho3_5", "rho1_6", "rho2_6", "rho3_6", "li1", "li2", "li3", "li4", "li5", "li6", "mi1", "mi2", "mi3", "mi4", "mi5", "mi6", "ji1_1", "ji2_1", "ji3_1", "ji1_2", "ji2_2", "ji3_2", "ji1_3", "ji2_3", "ji3_3", "ji1_4", "ji2_4", "ji3_4", "ji1_5", "ji2_5", "ji3_5", "ji1_6", "ji2_6", "ji3_6"];
ddX = zup_eul_ddx0do0_6(x,u,P,inv(zup_eul_Addx0do0_6(x,u,P)));
dX = zup_eul_tmp_cable_suspended_rigid_body_with_6_drones(x,u,P,ddX);
end
%% 状態：
% 牽引物: 位置，姿勢角，速度，角速度，
% リンク: 角度，角速度
% ドローン:姿勢角，角速度
% x = [p0 Q0 v0 O0 qi wi Qi Oi]
