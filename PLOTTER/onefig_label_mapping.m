function label = onefig_label_mapping(clearn_targets)
%onefig_label_mapping プロット関連のx,yラベル設定をする関数
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
            label.y{i} = ["Position [m]"];
        case "v"
            label.x{i} = ["Time [s]"];
            label.y{i} = ["Velocity [m/s]"];
        case "q"
            label.x{i} = ["Time [s]"];
            label.y{i} = ["Angle [rad]"];
        case "w"
            label.x{i} = ["Time [s]"];
            label.y{i} = ["Angular velocity [rad/s]"];
        case "input"
            label.x{i} = ["Time [s]"];
            label.y{i} = ["Controller input [N, Nm]"];
        case "input24"
            label.x{i} = ["Time [s]"];
            label.y{i} = ["Controller input [Nm]"];
        case "inner_input"
            label.x{i} = ["Time [s]"];
            label.y{i} = ["Transmitter input [N, Nm]"];
        case "inner_input24"
            label.x{i} = ["Time [s]"];
            label.y{i} = ["Transmitter input [Nm]"];
        case "p1_p2"
            label.x{i} = ["$x$ [m]"];
            label.y{i} = ["$y$ [m]"];
        case "p1_p2_p3"
            label.x{i} = ["$x$ [m]"];
            label.y{i} = ["$y$ [m]"];
            label.z{i} = ["$z$ [m]"];
    end
end
end
