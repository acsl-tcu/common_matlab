function legend = onefig_legend_mapping(clearn_targets, atts)
%onefig_legend_mapping プロット関連の凡例を設定をする関数
%   デフォルトとして
%   ["p", "v", "q", "w", "input", "input24",  "inner_input", "inner_input24", "p1_p2", "p1_p2_p3"]
%   ["er", "er", "e", "e", "", "", "", "", "er", "er"] の場合のみ記述されている

attmap = struct("p","plant", "e","est.", "r","ref.", "s","sensor");
len = length(clearn_targets);
legend = cell(1,len);
for i = 1:len
    target = char(clearn_targets{i}); % char型に変換
    att = atts{i};
    lgd_tmp = {};
    for splited_att = string(char(att)')'
        if ~isequal(splited_att,""), att_tmp = attmap.(splited_att); end
        switch target
            case "p"
                lgd_tmp = [lgd_tmp, "$x$ "+att_tmp, "$y$ "+att_tmp, "$z$ "+att_tmp];
            case "v"
                lgd_tmp = [lgd_tmp, "$v_x$ "+att_tmp, "$v_y$ "+att_tmp, "$v_z$ "+att_tmp];
            case "q"
                lgd_tmp = [lgd_tmp, "$\phi$ "+att_tmp, "$\theta$ "+att_tmp, "$\psi$ "+att_tmp];
            case "w"
                lgd_tmp = [lgd_tmp, "$\Omega_{\phi}$ "+att_tmp, "$\Omega_{\theta}$ "+att_tmp, "$\Omega_{\psi}$ "+att_tmp];
            case "input"
                lgd_tmp = [lgd_tmp, "$T$", "$\tau_{roll}$", "$\tau_{pitch}$", "$\tau_{yaw}$"];
            case "input24"
                lgd_tmp = [lgd_tmp, "$\tau_{roll}$", "$\tau_{pitch}$", "$\tau_{yaw}$"];
            case "inner_input"
                lgd_tmp = [lgd_tmp, "$T$", "$\tau_{roll}$", "$\tau_{pitch}$", "$\tau_{yaw}$"];
            case "inner_input24"
                lgd_tmp = [lgd_tmp, "$\tau_{roll}$", "$\tau_{pitch}$", "$\tau_{yaw}$"];
            case "p1_p2"
                lgd_tmp = [lgd_tmp, att_tmp];
            case "p1_p2_p3"
                lgd_tmp = [lgd_tmp, att_tmp];
            otherwise
                lgd_tmp = [lgd_tmp, ""];
        end
    end
    legend{i} = lgd_tmp;
end
end
