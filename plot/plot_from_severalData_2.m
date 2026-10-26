function figures = plot_from_severalData_2(files, settings)
% 保存ログを、この1ファイルだけで選択・描画・保存する。
% 引数なしで実行するとMATファイルを複数選択できる。
% 使用例: plot_from_severalData_2("C:\data\Log.mat")
% 設定を外から渡す例:
% s = struct('target',["p","input2:4"],'phase',"f",'fsave',4);
% plot_from_severalData_2("C:\data\Log.mat",s)
% LOGGERやC_Logger_Plotの追加、作業フォルダの変更は不要。
% 関数の引数を保持するため、clearは実行しない。
clc;

%% 設定（普段はこのブロックを変更する）
% defaults.target = ["p1-p2-p3"];

defaults.target = ["p1-p2-p3", "input1", "input2:4", "controller.result.delta_u"];
% input1: スラストのみ、input2:4: 回転方向の入力のみ（別々のFigure）。
% このログのinput2:4はトルク[Nm]。実際の角速度[rad/s]は "w" を指定する。
% 選択例: ["p","v","q","w","input","input2:4","p1-p2"]
% 任意の保存項目も指定可能: "controller.result.nominal_input"
% 末尾の成分指定: "p1", "input2:4", "controller.result.delta_u2:4"
defaults.attribute = "er"; % e:推定、s:センサ、r:目標、p:真値
defaults.phase = "fl";      % "tfl":離陸・飛行・着陸、"":全フェーズ
defaults.time = [-Inf Inf]; % 絶対時刻[s]の範囲（phaseと両方適用）
defaults.agent_id = 1;     % 保存されたData.agentの番号
defaults.fsave = 1;       % 0:保存なし 1:fig 2:png 3:jpg 4:pdf 5:eps
defaults.savefolder = "";  % 保存時に空ならフォルダ選択
defaults.ftitle = false;
defaults.fcolor = true;    % 時系列のフェーズ背景
defaults.fontsize = 18;
defaults.linewidth = 1.5;
defaults.lgd_loc = 'eastoutside';
defaults.visible = 'on';
defaults.rmse = true;      % p,vの推定値と目標値で計算・表示
if nargin < 2, settings = struct(); end
for name = string(fieldnames(defaults))'
    if ~isfield(settings,name), settings.(name) = defaults.(name); end
end
validateattributes(settings.fsave,{'numeric'},{'scalar','integer','>=',0,'<=',5});
validateattributes(settings.time,{'numeric'},{'numel',2,'nonnan'});
assert(settings.time(1)<=settings.time(2),'時刻範囲の順序が逆です。');
validateattributes(settings.agent_id,{'numeric'},{'scalar','integer','positive'});
if nargin < 1 || isempty(files)
    [names,folder] = uigetfile('*.mat','ログを選択（複数選択可）','MultiSelect','on');
    if isequal(names,0), figures = gobjects(0); return; end
    files = fullfile(folder,string(names));
end
files = string(files);
settings.target = string(settings.target);
if settings.fsave > 0
    if strlength(settings.savefolder)==0
        folder = uigetdir(pwd,'グラフとRMSEの保存先');
        if isequal(folder,0), figures = gobjects(0); return; end
        settings.savefolder = string(folder);
    end
    if ~isfolder(settings.savefolder), mkdir(settings.savefolder); end
end
figures = gobjects(0);
for file = reshape(files,1,[])
    S = load(file,'log');
    assert(isfield(S,'log') && isfield(S.log,'Data'),'log.Dataが見つかりません: %s',file);
    D = S.log.Data;
    n = numel(D.t);
    % 事前確保された未記録領域を除く。先頭4点を無条件には削らない。
    if isfield(S.log,'k'), n = min(n,S.log.k); end
    t = D.t(1:n); t = t(:);
    phase = D.phase(1:n); phase = phase(:);
    mask = isfinite(t) & t>=settings.time(1) & t<=settings.time(2);
    if strlength(string(settings.phase))>0
        mask = mask & ismember(phase,double(char(settings.phase)));
    end
    ids = find(mask);
    if isempty(ids), warning('PlotLog:Empty','対象フェーズ・時刻にデータがありません: %s',file); continue; end
    assert(settings.agent_id<=numel(D.agent),'指定したagent_idは保存されていません。');
    agent = D.agent(settings.agent_id);
    [~,base] = fileparts(file);
    fprintf('\n%s (%d samples)\n',base,numel(ids));
    for target = reshape(settings.target,1,[])
        % 項目が未保存なら、その項目だけ警告して次へ進む。
        try
            [series,atts,components,isPath] = collect(agent,target,settings.attribute,ids);
        catch err
            warning('PlotLog:Target','%s: %s',target,err.message); continue;
        end
        fig = figure('Color','w','Name',base+" / "+target,'NumberTitle','off', ...
            'Visible',settings.visible);
        ax = axes(fig); hold(ax,'on'); grid(ax,'on'); box(ax,'on');
        ax.FontSize = settings.fontsize;
        colors = lines(max(4,max(components)));
        styles = {'-','--',':','-.'};
        for a = 1:numel(series)
            Y = series{a};
            expanded = nan(ids(end)-ids(1)+1,size(Y,2));
            expanded(ids-ids(1)+1,:) = Y; Y = expanded;
            if isPath
                label = attribute_name(atts(a));
                if size(Y,2)==2
                    plot(ax,Y(:,1),Y(:,2),'LineWidth',settings.linewidth,'DisplayName',label);
                else
                    plot3(ax,Y(:,1),Y(:,2),Y(:,3),'LineWidth',settings.linewidth,'DisplayName',label);
                end
            else
                % 非選択フェーズをNaNで区切り、離れた区間を線で結ばない。
                tx = t(ids(1):ids(end));
                for c = 1:size(Y,2)
                    plot(ax,tx,Y(:,c),'Color',colors(components(c),:), ...
                        'LineStyle',styles{1+mod(a-1,4)},'LineWidth',settings.linewidth, ...
                        'DisplayName',component_name(target,components(c),atts(a)));
                end
            end
        end
        if isPath
            xlabel(ax,'x [m]'); ylabel(ax,'y [m]'); axis(ax,'equal');
            if size(series{1},2)==3, zlabel(ax,'z [m]'); view(ax,3); end
        else
            xlabel(ax,'Time [s]'); ylabel(ax,axis_label(target),'Interpreter','none');
            if numel(unique(t(ids)))>1, xlim(ax,[min(t(ids)) max(t(ids))]); end
            if settings.fcolor, shade_phase(ax,t,phase,ids); end
        end
        if settings.ftitle, title(ax,base+" / "+target,'Interpreter','none'); end
        legend(ax,'show','Location',settings.lgd_loc,'Interpreter','none', ...
            'FontSize',max(8,settings.fontsize-4));
        figures(end+1,1) = fig; %#ok<AGROW>
        if settings.fsave>0
            extensions = ["fig","png","jpg","pdf","eps"];
            % ファイル・項目・agentを含め、既存ファイルは連番で保護する。
            name = regexprep(base+"_a"+settings.agent_id+"_"+target,'[<>:"/\\|?*]','_');
            out = unused_name(fullfile(settings.savefolder,name+"."+extensions(settings.fsave)));
            if settings.fsave==1, savefig(fig,out);
            elseif ismember(settings.fsave,[4 5]), exportgraphics(fig,out,'ContentType','vector');
            else, exportgraphics(fig,out,'Resolution',300); end
        end
    end
    if settings.rmse
        report = "File: "+file+newline+"Phase: "+string(settings.phase)+newline;
        for variable = ["p","v"]
            try
                E = read_series(agent,"estimator.result.state."+variable,ids);
                R = read_series(agent,"reference.result.state."+variable,ids);
                assert(isequal(size(E),size(R)),'推定値と目標値の寸法が不一致です。');
                delta = E-R; valid = all(isfinite(delta),2);
                assert(any(valid),'有効な比較データがありません。');
                value = sqrt(mean(delta(valid,:).^2,1));
                report = report + sprintf('%s RMSE: %s (valid %d/%d)\nsum = %.6g\n', ...
                    variable,strtrim(sprintf('%.6g ',value)),sum(valid),numel(valid),sum(value));
            catch err
                report = report+variable+" RMSE: unavailable ("+string(err.message)+")"+newline;
            end
        end
        fprintf('%s',report);
        if settings.fsave>0
            out = unused_name(fullfile(settings.savefolder,base+"_a"+settings.agent_id+"_RMSE.txt"));
            fid = fopen(out,'w','n','UTF-8');
            assert(fid>=0,'RMSEファイルを開けません: %s',out);
            cleanup = onCleanup(@() fclose(fid)); fprintf(fid,'%s',report); clear cleanup;
        end
    end
end
end

function [series,atts,components,isPath] = collect(agent,target,attribute,ids)
isPath = any(target==["p1-p2","p1-p2-p3"]);
if isPath
    variable = "p"; components = 1:(2+(target=="p1-p2-p3"));
else
    % 成分番号は末尾だけ解析。フィールド名の途中にある数字は保持する。
    token = regexp(char(target),'^(.*?)(\d+(?::\d+(?::\d+)?)?)$','tokens','once');
    variable = target; components = [];
    if ~isempty(token)
        variable = string(token{1}); parts = str2double(split(string(token{2}),':'));
        if numel(parts)==1, components = parts;
        elseif numel(parts)==2, components = parts(1):parts(2);
        else, components = parts(1):parts(2):parts(3); end
        assert(~isempty(components) && all(components>=1),'成分指定が不正です。');
    end
end
if any(variable==["p","v","q","w","z"])
    atts = string(cellstr(char(attribute)'))';
    roots = ["estimator","sensor","reference","plant"];
    codes = ["e","s","r","p"];
    assert(all(ismember(atts,codes)),'attributeはe,s,r,pで指定してください。');
    paths = strings(size(atts));
    for k=1:numel(atts), paths(k) = roots(codes==atts(k))+".result.state."+variable; end
else
    atts = ""; paths = variable;
end
series = {}; kept = strings(1,0);
for k=1:numel(paths)
    try
        Y = read_series(agent,paths(k),ids);
        if isempty(components), cols=1:size(Y,2); else, cols=components; end
        assert(all(cols<=size(Y,2)),'指定した成分がありません。');
        series{end+1} = Y(:,cols); kept(end+1)=atts(k); %#ok<AGROW>
    catch err
        warning('PlotLog:Series','%s (%s): %s',target,atts(k),err.message);
    end
end
assert(~isempty(series),'描画できるデータがありません。');
atts = kept;
if isempty(components), components = 1:size(series{1},2); end
end

function Y = read_series(agent,path,ids)
% cell配列で保存された各時刻の値を読み、行=時刻、列=成分に統一。
fields = split(path,'.'); rows = cell(numel(ids),1);
for k=1:numel(ids)
    value = agent;
    for j=1:numel(fields)
        if isempty(value), break; end
        value = value.(fields(j));
        if iscell(value)
            assert(numel(value)>=ids(k),'記録長が足りません。');
            value = value{ids(k)};
        end
    end
    if isempty(value), continue; end
    assert(isnumeric(value) && isreal(value) && isvector(value) && ~isempty(value), ...
        '空または実数ベクトル以外のデータです: %s',path);
    rows{k} = double(value(:)');
end
% 起動直後などstateが空のサンプルは、時刻を詰めず欠損として扱う。
valid = find(~cellfun(@isempty,rows),1);
assert(~isempty(valid),'有効なデータがありません: %s',path);
width = numel(rows{valid});
for k=1:numel(rows)
    if isempty(rows{k}), rows{k}=nan(1,width); end
end
Y = vertcat(rows{:});
end

function name = attribute_name(att)
codes = ["e","s","r","p",""]; names = ["est.","sen.","ref.","true",""];
name = names(codes==att);
end

function name = component_name(target,c,att)
if startsWith(target,"p"), labels=["x","y","z"];
elseif startsWith(target,"v"), labels=["vx","vy","vz"];
elseif startsWith(target,"q"), labels=["roll","pitch","yaw"];
elseif startsWith(target,"w"), labels=["omega_x","omega_y","omega_z"];
elseif startsWith(target,"inner_input"), labels=["roll","pitch","thrust","yaw"];
elseif contains(target,"input") || contains(target,"delta_u"), labels=["T","tau_roll","tau_pitch","tau_yaw"];
else, labels=strings(0); end
if c<=numel(labels), name=labels(c); else, name="component "+c; end
name = strtrim(name+" "+attribute_name(att));
end

function label = axis_label(target)
if ~isempty(regexp(char(target),'^p\d*(:\d+)*$','once')), label="Position [m]";
elseif ~isempty(regexp(char(target),'^v\d*(:\d+)*$','once')), label="Velocity [m/s]";
elseif startsWith(target,"q"), label="Angle [rad]";
elseif startsWith(target,"w"), label="Angular velocity [rad/s]";
elseif contains(target,"input") || contains(target,"delta_u")
    if ~isempty(regexp(char(target),'(?:input|delta_u)1$','once'))
        label="Thrust [N]";
    elseif endsWith(target,"2:4"), label="Torque [Nm]";
    else, label="Compensation Input [N], [Nm]"; end
else, label=target; end
end

function shade_phase(ax,t,phase,ids)
% フェーズが変わる度に区切るため、中断・再開にも対応する。
edges = [1;find(diff(ids)>1 | diff(phase(ids))~=0)+1;numel(ids)+1];
yl = ylim(ax); codes = double('tfl'); colors = [1 1 .9;.9 1 1;1 .9 1];
for k=1:numel(edges)-1
    ix = ids(edges(k):edges(k+1)-1); c = find(codes==phase(ix(1)),1);
    if isempty(c) || numel(ix)<2, continue; end
    h = patch(ax,t(ix([1 end end 1])),yl([1 1 2 2]),colors(c,:), ...
        'EdgeColor','none','FaceAlpha',.4,'HandleVisibility','off');
    uistack(h,'bottom');
end
ylim(ax,yl);
end

function path = unused_name(path)
% 既に存在する保存結果を上書きしない。
[folder,base,ext] = fileparts(path); original = base; index = 1;
while isfile(path)
    path = fullfile(folder,original+"_"+index+ext); index=index+1;
end
end

