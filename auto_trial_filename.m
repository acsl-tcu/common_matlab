function filename = auto_trial_filename(metadata, completed, trial)
% 自動試行の保存名を、実際のモデル設定とゲインから作成する。
if completed
    status = "done";
else
    status = "false";
end
model_parts = strings(1, numel(metadata.model_indices));
for index = 1:numel(model_parts)
    value = strrep(sprintf('%.15g', metadata.model_values(index)), '.', 'p');
    model_parts(index) = sprintf('ME%d_%s', metadata.model_indices(index), value);
end
if isempty(model_parts)
    model_description = "ME_none";
else
    model_description = strjoin(model_parts, '_');
end
filename = status + "_" + metadata.method + "_" + metadata.trajectory + ...
    "_" + sprintf('%03d', trial) + "_" + model_description;
if metadata.method ~= "HL"
    [~, gain_name] = fileparts(metadata.gain_file);
    filename = filename + "_" + gain_name;
end
end
