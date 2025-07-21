% Data/Exp_data の全てのmatファイルを学習用フォルダData/learning_dataに格納するようにしたい！！
% 2025.06.02

phase = 'tfl';

% folder_path = 'Data/Exp_data/'; % 参照したいデータが保存されているフォルダのパス
folder_path = 'Data/Exp_data/2025.07.09_Exp/'; % 参照したいデータが保存されているフォルダのパス
learning_folder = 'Data/learning_data/'; % 学習データ保存用フォルダのパス

matfiles = dir(fullfile(folder_path, '*.mat'));
file_names = {matfiles.name};
for i = 1:length(file_names)
    file_name = load([folder_path, file_names{i}]);
    [input, p, q, v, w] = trim_Exp_data(file_name, phase);
    save(fullfile(learning_folder, file_names{i}), 'input', 'p', 'q', 'v', 'w')
end


function [input, p, q, v, w] = trim_Exp_data(Data, phase)
% 任意のフェーズをdouble型に変換する関数
    % Data: cell配列になっていてDNN用に整形したいExpの元データ
    % phase: 'f', 'tf', 'atf', 'fl', 'tfl', 'atfl'から選択可能
    % 'f' -> flightフェーズのみ(default)
    % 'tfl' -> takeoff + flight + landingフェーズ
    arguments
        Data;
        phase = 'f';
    end

    if Data.log.fExp ~= 1
        error('Cannot use Sim data')
    end

    data = simplifyLogger(Data.log, Data.log.target);
    switch phase
        % arming  -> 97
        % takeoff -> 116
        % flight  -> 102
        % landing -> 108
        case 'tf'
            phase = 'takeoff + flight';
            firstindex = find(data.phase == 116, 1, 'first');
            lastindex = find(data.phase == 102, 1, 'last');
        case 'atf'
            phase = 'arming + takeoff + flight';
            firstindex = find(data.phase == 97, 1, 'first');
            lastindex = find(data.phase == 102, 1, 'last');
        case 'fl'
            phase = 'flight + landing';
            firstindex = find(data.phase == 102, 1, 'first');
            lastindex = find(data.phase == 108, 1, 'last');
        case 'tfl'
            phase = 'takeoff + flight + landing';
            firstindex = find(data.phase == 116, 1, 'first');
            lastindex = find(data.phase == 108, 1, 'last');
        case 'atfl'
            phase = 'arming + takeoff + flight + landing';
            firstindex = find(data.phase == 97, 1, 'first');
            lastindex = find(data.phase == 108, 1, 'last');
        otherwise
            phase = 'flight only';
            firstindex = find(data.phase == 102, 1, 'first');
            lastindex = find(data.phase == 102, 1, 'last');
    end

    fprintf('selected phase: %s', phase)
    % i = Data.log.target
    input = data.controller.input(:,firstindex:lastindex);
    q = data.estimator.q(:,firstindex:lastindex);
    p = data.estimator.p(:,firstindex:lastindex);
    v = data.estimator.v(:,firstindex:lastindex);
    w = data.estimator.w(:,firstindex:lastindex);
    

% Square_coloring(Xrange, color, Yrange, Bace, ax) 使えそう
    % 確認用
    time = data.t(firstindex:lastindex);
    plot(time, p(1,:), time, p(2,:), time, p(3,:))
    hold on
    plot(time, data.reference.p(1,firstindex:lastindex), '--', ...
        time, data.reference.p(2,firstindex:lastindex), '--', ...
        time, data.reference.p(3,firstindex:lastindex), '--')
    xlabel('t [s]')
    ylabel('position [m]')
    legend('xp', 'yp', 'xp', 'xr', 'yr', 'zr')
    title(['Selected phase: ',phase])
    grid on
    ax = gca;
    ax.FontSize = 12;
    hold off
end