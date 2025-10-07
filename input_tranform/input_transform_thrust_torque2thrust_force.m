function [torque2thrusts_matrix, thrusts2torque_matrix] = input_transform_thrust_torque2thrust_force(parameter)
%INPUT_TRANSFORM_THRUST_TORQUE2TRUST_FORCE
%   入力:u = [Thrust, roll, pitch, yaw] と 各ロータの推力:u = [T1, T2, T3, T4]; それぞれ変換するための行列を生成する関数
%
%   変換行列はbeta flight機体準拠 (参考：model\substance\generateModel.m)
%   ロータ配置   右後:T1 右前:T2 左後:T3 左前:T4
%   回転向き    T1,T4=CCW   T2,T3=CW
%
%   作成者：2212044 小関      作成日：2025/10/07
%   [Inputs]
%    parameter: クアッドコプタの物理パラメータクラス
%
%   [Outpus]
%    torque2thrusts_matrix: [Thrust;roll;pitch;yaw]->[T1;T2;T3;T4] への変換行列
%    thrusts2torque_matrix: 各ロータ推力[T1;T2;T3;T4]->[Thrust;roll;pitch;yaw] への変換行列
arguments
    parameter % paremeter class
end

Lx = parameter.Lx;
Ly = parameter.Ly;
lx = parameter.lx;
ly = parameter.ly;
km1 = parameter.km1;
km2 = parameter.km2;
km3 = parameter.km3;
km4 = parameter.km4;

thrusts2torque_matrix =   [[              1,                      1,                     1,                      1]
                            [-(2^(1/2)*ly)/2,        -(2^(1/2)*ly)/2, (2^(1/2)*(Ly - ly))/2,  (2^(1/2)*(Ly - ly))/2]
                            [ (2^(1/2)*lx)/2, -(2^(1/2)*(Lx - lx))/2,        (2^(1/2)*lx)/2, -(2^(1/2)*(Lx - lx))/2]
                            [            km1,                   -km2,                  -km3,                    km4]];
torque2thrusts_matrix = inv(thrusts2torque_matrix);
end
