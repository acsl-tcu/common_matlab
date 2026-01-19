function tableData = set_plot_settings4table(currentTableData, mode, legend, agentID)
%set_plot_settings4table この関数の概要をここに記述
%   詳細説明をここに記述
arguments
    currentTableData cell
    mode string
    legend (:,1)cell
    agentID double = 1;
end
uses    = currentTableData(:,1);
targets = currentTableData(:,2);
atts    = currentTableData(:,3);
xlabels = currentTableData(:,4);
ylabels = currentTableData(:,5);
zlabels = currentTableData(:,6);

[clearn_targets, ~] = create_clearn_target(targets);
columnNum = length(targets);

if mode == "subfig" % 引数のlegendを基に生成
    tmp = cellfun(@string, legend, UniformOutput=false);
    for col=1:length(tmp)
        if col==1, lgd_tmp = tmp{col};
        else,    lgd_tmp = lgd_tmp+","+tmp{col}; end
    end
    legends = repmat({lgd_tmp}, 1,columnNum);
elseif mode =="onefig" % 既存TableDataのtargetsから生成
    legends = onefig_legend_mapping(clearn_targets, atts);
end

tableData = cell(columnNum,8);

for col = 1:columnNum % app.UItableに入れるためにはcharじゃないとダメっぽい
    tableData{col,1} = uses{col};
    tableData{col,2} = char(targets{col});
    tableData{col,3} = char(atts{col});
    tableData{col,4} = char(xlabels{col});
    tableData{col,5} = char(ylabels{col});
    tableData{col,6} = char(zlabels{col});
    tableData{col,7} = char(strjoin(legends{col}, ","));
    tableData{col,8} = num2str(agentID);
end

end

