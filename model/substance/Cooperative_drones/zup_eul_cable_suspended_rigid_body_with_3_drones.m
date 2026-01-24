function dX = zup_eul_cable_suspended_rigid_body_with_3_drones(x,u,P)
% Input order:
% x (state) = [];
% u (input) = [];
% P (parameter) = ["g", "m0", "j01", "j02", "j03", "rho1_1", "rho2_1", "rho3_1", "rho1_2", "rho2_2", "rho3_2", "rho1_3", "rho2_3", "rho3_3", "li1", "li2", "li3", "mi1", "mi2", "mi3", "ji1_1", "ji2_1", "ji3_1", "ji1_2", "ji2_2", "ji3_2", "ji1_3", "ji2_3", "ji3_3"];
ddX = zup_eul_ddx0do0_3(x,u,P,inv(zup_eul_Addx0do0_3(x,u,P)));
dX = zup_eul_tmp_cable_suspended_rigid_body_with_3_drones(x,u,P,ddX);
end
%% 状態：
% 牽引物: 位置，姿勢角，速度，角速度，
% リンク: 角度，角速度
% ドローン:姿勢角，角速度
% x = [p0 Q0 v0 O0 qi wi Qi Oi]
