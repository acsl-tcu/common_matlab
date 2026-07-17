function calc_rmse(logger,target,phase)
%CALC_RMSE targetで指定したstateのRMSE: Root Mean Squared Errorを計算
%   logger: LOGGERクラス
%   target: referenceデータが存在していることが望ましい。存在しない場合は0をtarget dataとする。
%   phase: RMSE値を計算したいphase
arguments
    logger
    target = ["p","v"]
    phase = "f"
end

RMSE = [];
for i=1:length(target)
    data = logger.data(1,target(i),"e", "phase",phase);
    if isprop(logger.Data.agent.reference.result{1}.state, target(i))
        ref = logger.data(1,target(i),"r", "phase",phase);
    else
        ref = zeros(size(data));
    end
    RMSE = [RMSE; rmse(ref, data, 1)];
    fprintf('%s RMSE:\n', target(i))
    disp(RMSE(i,:))
end
end
