% ── figuresの保存 ──────────────────────────────────────
function saveFigs(obj, style)
    % saveFigs  現在開いている全 figure を指定形式で保存する
    %
    % 保存先フォルダをGUIで選択後、以下のディレクトリ構造で保存する:
    %   <選択フォルダ>/
    %     ├ png/  *.png
    %     ├ pdf/  *.pdf
    %     └ fig/  *.fig
    %
    % 引数
    %   style : 保存形式を指定する文字列セル配列（省略時は全形式）
    %           例) {'png'}  {'png','pdf'}  {'fig'}
    %
    % 使用例
    %   obj.saveFigs();                 % png, pdf, fig の3形式で保存
    %   obj.saveFigs({'png'});          % png のみ
    %   obj.saveFigs({'png','pdf'});    % png と pdf

    arguments
        obj
        style cell = {'png', 'pdf', 'fig'}
    end

    % --- 有効な形式のみ残す ---
    valid_styles = {'png', 'pdf', 'fig'};
    style = intersect(style, valid_styles);
    if isempty(style)
        warning('saveFigs: 有効な形式が指定されていません。png/pdf/fig から選択してください。');
        return;
    end

    % --- 開いている figure を取得 ---
    all_figs = findall(0, 'Type', 'figure');
    if isempty(all_figs)
        warning('saveFigs: 保存対象の figure が見つかりません。');
        return;
    end
    % figure 番号の昇順に並べる
    [~, sort_idx] = sort([all_figs.Number]);
    all_figs = all_figs(sort_idx);
    n_figs = numel(all_figs);
    fprintf('saveFigs: %d 個の figure を検出しました。\n', n_figs);

    % --- 保存先フォルダをGUIで選択 ---
    save_dir = uigetdir(pwd, '保存先フォルダを選択してください');
    if isequal(save_dir, 0)
        fprintf('saveFigs: キャンセルされました。\n');
        return;
    end
    time_str = datetime('now');
    time_str.Format = 'yyyy-MM-dd_HH-mm-ss';
    save_dir = [save_dir, '\Data_analysis__', char(time_str)];

    % --- figure Name → ファイル名の変換テーブル ---
    % figure('Name', ...) の先頭キーワードに基づいてファイル名を決定する。
    % キーワードが一致しない場合は 'figure_<番号>' をフォールバックとして使用。
    name_map = {
        '相関ヒートマップ',     'heatmap';
        'Lag Correlation',      'lagCorr';
        '散布図行列',           'scatterMatrix';
        '分散・標準偏差',       'varianceBar';
    };

    % --- 各形式のサブフォルダを作成 ---
    for s = style
        sub_dir = fullfile(save_dir, s{1});
        if ~exist(sub_dir, 'dir')
            mkdir(sub_dir);
        end
    end

    % --- ファイル名の重複を管理するカウンタ ---
    name_counter = containers.Map('KeyType', 'char', 'ValueType', 'double');

    % --- 各 figure を保存 ---
    saved_count = 0;
    for fi = 1:n_figs
        fig = all_figs(fi);

        % figure Name からベースファイル名を決定
        fig_name = fig.Name;
        base_name = obj.resolveFileName(fig_name, name_map);

        % 同名が複数ある場合は連番を付与
        if isKey(name_counter, base_name)
            name_counter(base_name) = name_counter(base_name) + 1;
            file_name = sprintf('%s_%d', base_name, name_counter(base_name));
        else
            name_counter(base_name) = 1;
            file_name = base_name;
        end

        fprintf('  [%d/%d] "%s" → %s\n', fi, n_figs, fig_name, file_name);

        % --- 各形式で保存 ---
        for s = style
            fmt     = s{1};
            sub_dir = fullfile(save_dir, fmt);
            out_path = fullfile(sub_dir, file_name);

            try
                switch fmt
                    case 'png'
                        exportgraphics(fig, [out_path, '.png'], ...
                            'Resolution', 150);
                    case 'pdf'
                        exportgraphics(fig, [out_path, '.pdf'], ...
                            'ContentType', 'vector');
                    case 'fig'
                        savefig(fig, [out_path, '.fig']);
                end
                fprintf('    -> %s.%s を保存しました。\n', out_path, fmt);
            catch ME
                warning('saveFigs: %s.%s の保存に失敗しました。\n  理由: %s', ...
                    out_path, fmt, ME.message);
            end
        end

        saved_count = saved_count + 1;
    end

    fprintf('\nsaveFigs: %d 個の figure を保存しました。\n保存先: %s\n', ...
        saved_count, save_dir);
end