function A = AF7R49_PHASE_TIMING_AUDIT
assert(evalin('base','exist(''D38'',''var'')==1'), ...
    'Run: D38 = AF7R38_SPEED_ROOTCAUSE_AUDIT; first.');
D38=evalin('base','D38');
P=D38.pipeline;

fprintf('\n====================================================================================================\n');
fprintf(' AF7R49 | CONSTRUCTIVE PHASE / NO-CATCH-UP AUDIT\n');
fprintf('====================================================================================================\n');

F={ ...
 'predeadline_candidate_success','predeadline_candidate_family', ...
 'predeadline_t0','predeadline_t1','predeadline_payload_speed', ...
 'predeadline_uav_speed','predeadline_qdot', ...
 'r34_delay_candidate_count','r34_terminal_hold_exact_count', ...
 'r34_monotone_progress_count','r34_selected_delay_s', ...
 'r34_selected_hold_start_s','r34_selected_alpha', ...
 'r32_best_entry_T','r32_best_flow_T','r32_best_exit_T', ...
 'r49_phase_timing_trial_count','r49_phase_gate_pass_count', ...
 'r49_hard_certificate_trial_count','r49_selected_Ke', ...
 'r49_selected_Kflow','r49_selected_Kx', ...
 'native_episode_ms','native_corridor_ms','native_qp_ms', ...
 'native_candidate_build_ms','native_total_ms', ...
 'last_stage1_ms','last_stage2_ms','last_stage3_ms'};

for k=1:numel(F)
    f=F{k};
    if isfield(P,f)
        v=P.(f);
        if isnumeric(v)||islogical(v)
            if isscalar(v)
                fprintf('%-52s = %.17g\n',f,double(v));
            else
                fprintf('%-52s = %s\n',f,mat2str(double(v),17));
            end
        else
            fprintf('%-52s = %s\n',f,char(string(v)));
        end
    else
        fprintf('%-52s = <NOT EXPOSED>\n',f);
    end
end

fprintf('\nInterpretation:\n');
fprintf(' phase_gate_pass > 0: 0 <= ds/dt <= 1 is certified continuously by Bernstein controls.\n');
fprintf(' selected_alpha <= 1: no schedule catch-up is used.\n');
fprintf(' selected_delay_s > 0: the old spatial path is followed with retained phase delay.\n');
fprintf('====================================================================================================\n');

A=P;
assignin('base','R49A',A);
end
