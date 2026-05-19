function data = extract_data(plt, agentID)
%extract_data プロット関連のラベル管理をする関数
%   詳細説明をここに記述
arguments
    plt
    agentID = 1;
end

% エラーの原因になる"p1-p2"を削除するようなアルゴリズム
% Geminiで作成 URL: https://gemini.google.com/app/277ef4b62711ef50?utm_source=app_launcher&utm_medium=owned&utm_campaign=base_all
hasDigit = contains(plt.settings.target, regexpPattern('\d'));
isAllowedPattern = contains(plt.settings.target, regexpPattern('\d:\d'));
toRemove = hasDigit & ~isAllowedPattern;
plt.settings.target(toRemove) = [];

phase = plt.settings.phase;
for i = 1:length(plt.settings.target)
    clearn_target = regexprep(plt.settings.target(i), '[^a-zA-Z0-9_]', '');
    c_array = char(plt.settings.att(i)); % (char型) に変換することで分割
    splited_att = string(c_array');      % sttringに戻す
    for att  = splited_att'
        if att == ""
            data.(clearn_target) = cell(plt.Num,1);
            for N = 1:plt.Num
                tmp = plt.logger{N}.data(agentID, plt.settings.target(i), att, "phase", phase);
                data.(clearn_target){N} = tmp(plt.data.idx{N}(1):plt.data.idx{N}(2),:);
            end
        else
            data.(clearn_target).(att) = cell(plt.Num,1);
            for N = 1:plt.Num
                tmp = plt.logger{N}.data(agentID, plt.settings.target(i), att, "phase", phase);
                data.(clearn_target).(att){N} = tmp(plt.data.idx{N}(1):plt.data.idx{N}(2),:);
            end
        end
    end
end
end
