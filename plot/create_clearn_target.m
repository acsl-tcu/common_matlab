function [clearn_target, fault_flag] = create_clearn_target(targets)
%CREATE_CLEARN_TARGET 構造体構成時&logger.dataメソッド使用時に使えない文字等を削除・変換する関数
    clearn_target = regexprep(targets, ':', ''); % コロンを削除
    clearn_target = matlab.lang.makeValidName(clearn_target)'; % 不適切な文字を自動で変換

    cond1 = contains(targets, regexpPattern("[a-zA-Z]+\d+"));
    cond2 = ~contains(targets, ":");
    fault_flag = (cond1 & cond2)';
    if any(fault_flag), fprintf("logger.dataメソッドで使えないtarget: %s\n", char(targets(fault_flag))); end
end

