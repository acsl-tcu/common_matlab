function OUT = AF7R33C_PREINSTALL_GATE(replanner)
clc;
fprintf('\n====================================================================================================\n');
fprintf(' AF7R33C | R13C PREINSTALL GATE\n');
fprintf('====================================================================================================\n');

A=AF7R33A_REPEAT_REPLAN_BATCH_MODEL_AUDIT(replanner);
B=AF7R33B_TRUE_TRIGGER_ALL_OBS_EPISODE_AUDIT(replanner);
Q=AF7R32A_TRUE_TRIGGER_QP_SHADOW_AUDIT(replanner);
W=AF7R32B_TRUE_TRIGGER_WHOLEBODY_SHADOW_AUDIT(replanner);

closed=A.closed&&B.closed&&Q.closed&&W.closed;
fprintf(' repeat-replan model                      = %d\n',A.closed);
fprintf(' all-obstacle episode                     = %d\n',B.closed);
fprintf(' exact small QP                           = %d\n',Q.closed);
fprintf(' whole-body batch                         = %d\n',W.closed);
fprintf(' R13C preinstall closed                   = %d\n',closed);
fprintf('====================================================================================================\n');

if closed
    src=which('REPLANNING_SYSTEM'); root=fileparts(src);
    token=struct('schema','AF7R33_PREINSTALL_TOKEN_V1','closed',true, ...
        'repeat_model',A,'episode',B,'qp',Q,'wholebody',W);
    save(fullfile(root,'AF7R33_PREINSTALL_TOKEN.mat'),'token');
end
OUT=struct('schema','AF7R33C_PREINSTALL_GATE_V1','closed',closed, ...
    'repeat_model',A,'episode',B,'qp',Q,'wholebody',W);
assignin('base','AF7R33C_PREINSTALL_GATE_OUT',OUT);
end
