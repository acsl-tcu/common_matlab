function T = AF7R52D2_DEEP_GATE_AUDIT
% Diagnostic only. Does not modify or accept any candidate.

M = af7r36_single_core_mex(20);
if isempty(M)
    error('[AF7R52D2] diagnostic matrix is empty. Run the simulation once with the R52D2 MEX.');
end

names = { ...
'seq','family','exact_c7','max_ctrl_m','corr_vel_ub','corr_acc_ub','corr_jerk_ub', ...
'phase_alpha','phase_delay_s','actual_rejoin_s', ...
'VQ_limit','base_VL','base_A','base_J','base_sigma', ...
'stageA_Uv','stageA_Ua','stageA_Uj','stageA_sigma','stageA_VQ', ...
'stageA_cert','stageA_impossible','stageA_repr', ...
'stageB_attempt','stageB_result','stageB_leaves','stageB_splits', ...
'stageB_worst_VQ','stageB_worst_t','stageB_worst_seg','stageB_worst_VL', ...
'stageB_worst_sigma','stageB_worst_qdot','stageB_worst_witness', ...
'speed_pass','heading_attempt','heading_pass','heading_reason', ...
'heading_min_g0','heading_min_lb','heading_leaves','heading_splits','heading_gamma', ...
'rejoin_attempt','rejoin_pass','geometry_attempt','geometry_pass','geometry_min_margin','reserved'};

assert(size(M,2)==numel(names),'[AF7R52D2] unexpected diagnostic matrix width.');
T = array2table(M,'VariableNames',names);

fprintf('\n====================================================================================================\n');
fprintf(' AF7R52D2 | PER-CANDIDATE HARD-GATE AUDIT (DIAGNOSTIC ONLY)\n');
fprintf('====================================================================================================\n');
disp(T);

fprintf('\n[COUNTS]\n');
fprintf(' candidates             = %d\n',height(T));
fprintf(' exact C7               = %d\n',sum(T.exact_c7==1));
fprintf(' Stage-A certified      = %d\n',sum(T.stageA_cert==1));
fprintf(' Stage-B attempted      = %d\n',sum(T.stageB_attempt==1));
fprintf(' speed pass             = %d\n',sum(T.speed_pass==1));
fprintf(' heading attempted      = %d\n',sum(T.heading_attempt==1));
fprintf(' heading pass           = %d\n',sum(T.heading_pass==1));
fprintf(' rejoin attempted       = %d\n',sum(T.rejoin_attempt==1));
fprintf(' rejoin pass            = %d\n',sum(T.rejoin_pass==1));
fprintf(' geometry attempted     = %d\n',sum(T.geometry_attempt==1));
fprintf(' geometry pass          = %d\n',sum(T.geometry_pass==1));

fprintf('\n[FAILURE CODE LEGEND]\n');
fprintf(' Stage-B result: 1=certified, 2=finite violating witness, 3=representation stop, 4=terminal failure\n');
fprintf(' heading reason: 0=pass, 1=input certificate, 2=Lipschitz bound, 3=midpoint/representation, 4=no positive continuous gamma, 5=zero/nonfinite horizontal anchor\n');

fprintf('\n[WORST / BEST NUMERICS]\n');
fprintf(' finite-sensing VQ limit = %.12g m/s\n',T.VQ_limit(1));
if any(isfinite(T.stageA_VQ))
    fprintf(' min Stage-A VQ UB       = %.12g m/s\n',min(T.stageA_VQ(isfinite(T.stageA_VQ))));
end
if any(isfinite(T.stageB_worst_VQ))
    fprintf(' min Stage-B worst VQ    = %.12g m/s\n',min(T.stageB_worst_VQ(isfinite(T.stageB_worst_VQ))));
end
if any(isfinite(T.heading_min_g0))
    fprintf(' best min heading g0     = %.12g\n',max(T.heading_min_g0(isfinite(T.heading_min_g0))));
end
if any(isfinite(T.heading_min_lb))
    fprintf(' best min heading LB     = %.12g\n',max(T.heading_min_lb(isfinite(T.heading_min_lb))));
end

fprintf('====================================================================================================\n');
assignin('base','R52D2',T);
end
