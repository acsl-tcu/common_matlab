function label = subfig_label_mapping(clearn_targets)
%subfig_label_mapping プロット関連のx,yラベル設定をする関数
%   デフォルトとして
%   ["p", "v", "q", "w", "input", "input24",  "inner_input", "inner_input24", "p1_p2", "p1_p2_p3"] のみ記述されている

len = length(clearn_targets);
label.x = cell(1,len);
label.y = cell(1,len);
label.z = cell(1,len);
for i = 1:len
    target = clearn_targets(i);
    switch target
        case "p"
            label.x{i} = ["Time [s]"];
            label.y{i} = ["$x$ [m]", "$y$ [m]", "$z$ [m]"];
        case "v"
            label.x{i} = ["Time [s]"];
            label.y{i} = ["$v_x$ [m/s]", "$v_y$ [m/s]", "$v_z$ [m/s]"];
        case "q"
            label.x{i} = ["Time [s]"];
            label.y{i} = ["$\phi$ [rad]", "$\theta$ [rad]", "$\psi$ [rad]"];
        case "w"
            label.x{i} = ["Time [s]"];
            label.y{i} = ["$\Omega_{roll}$ [rad/s]", "$\Omega_{pitch}$ [rad/s]", "$\Omega_{yaw}$ [rad/s]"];
        case "input"
            label.x{i} = ["Time [s]"];
            label.y{i} = ["$T$ [N]", "$\tau_{roll}$ [Nm]", "$\tau_{pitch}$ [Nm]", "$\tau_{yaw}$ [Nm]"];
        case "input24"
            label.x{i} = ["Time [s]"];
            label.y{i} = ["$\tau_{roll}$ [Nm]", "$\tau_{pitch}$ [Nm]", "$\tau_{yaw}$ [Nm]"];
        case "inner_input"
            label.x{i} = ["Time [s]"];
            label.y{i} = ["$T$ [N]", "$\tau_{roll}$ [Nm]", "$\tau_{pitch}$ [Nm]", "$\tau_{yaw}$ [Nm]"];
        case "inner_input24"
            label.x{i} = ["Time [s]"];
            label.y{i} = ["$\tau_{roll}$ [Nm]", "$\tau_{pitch}$ [Nm]", "$\tau_{yaw}$ [Nm]"];
        case "p1_p2"
            label.x{i} = "$x$ [m]";
            label.y{i} = "$y$ [m]";
        case "p1_p2_p3"
            label.x{i} = "$x$ [m]";
            label.y{i} = "$y$ [m]";
            label.z{i} = "$z$ [m]";
        otherwise
            label.x{i} = "";
            label.y{i} = "";
            label.z{i} = "";
    end
end

end
