function default_settings = default_plot_settings4table(mode, legend, agentID)
%DEFAULT_PLOT_SETTINGS この関数の概要をここに記述
%   詳細説明をここに記述
arguments
    mode string
    legend (:,1)cell = cell(1,1);
    agentID double = 1;
end
uses    = [true, true, true, true, true, true, true];
targets = ["p", "v", "q", "w", "input", "p1-p2", "p1-p2-p3"];
atts    = ["er", "er", "e", "e", "", "er", "er"];
[clearn_targets, fault_flag] = create_clearn_target(targets);
len = length(targets);

if mode == "subfig"
    labels = subfig_label_mapping(clearn_targets);
    tmp = cellfun(@string, legend, UniformOutput=false);
    for i=1:length(tmp)
        if i==1, lgd_tmp = tmp{i};
        else,    lgd_tmp = lgd_tmp+","+tmp{i}; end
    end
    legends = repmat({lgd_tmp}, 1,len);
elseif mode =="onefig"
    labels = onefig_label_mapping(clearn_targets);
    legends = onefig_legend_mapping(clearn_targets, atts);
end

default_settings = cell(len,8);

for i = 1:len % app.UItableに入れるためにはcharじゃないとダメっぽい
    default_settings{i,1} = uses(i);
    default_settings{i,2} = char(targets(i));
    default_settings{i,3} = char(atts(i));
    default_settings{i,4} = char(strjoin(labels.x{i}, ","));
    default_settings{i,5} = char(strjoin(labels.y{i}, ","));
    if ~isempty(labels.z{i})
        default_settings{i,6} = char(strjoin(labels.z{i}, ","));
    end
    default_settings{i,7} = char(strjoin(legends{i}, ","));
    default_settings{i,8} = num2str(agentID);
end

end

