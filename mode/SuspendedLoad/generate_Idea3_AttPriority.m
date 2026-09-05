% 案3：姿勢優先 Compatibility-Aware CBF ジェネレータ
% 論文③の枠組みに「姿勢崩壊限界（推力限界）」のハード制約を導入
clear; clc;
disp('Generating Attitude Priority CBF equations...');
disp('This method relies heavily on the QP formulation rather than symbolic gradients.');
disp('See Sim_Idea3_AttPriority.m and HLC_CBF_ATT_PRIORITY.m for implementation details.');
