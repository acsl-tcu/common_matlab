function A = AF7R51D1_QP_SPLIT_AUDIT
assert(evalin('base','exist(''D38'',''var'')==1'), ...
    'Run: D38 = AF7R38_SPEED_ROOTCAUSE_AUDIT; first.');
D38=evalin('base','D38');
P=D38.pipeline;

fprintf('\n====================================================================================================\n');
fprintf(' AF7R51D1 | TANGENT-QP / RETURN-MONOTONE SPLIT AUDIT\n');
fprintf('====================================================================================================\n');

F={ ...
 'predeadline_candidate_success','predeadline_candidate_family', ...
 'r37_side_assignments','r37_spatial_qp_solutions', ...
 'r51_tangent_candidate_count','r51_tangent_certified_count', ...
 'r48_exit_monotone_row_count', ...
 'r51d_side_count','r51d_tangent_only_qp_pass_count', ...
 'r51d_full_qp_pass_count','r51d_tangent_row_total', ...
 'r51d_monotone_row_total','r51d_return_dir_total', ...
 'r51d_return_dir_min_dot','r51d_tangent_best_slack', ...
 'r51d_full_best_slack', ...
 'r45_best_fixed_peak_req_m','r45_best_tangent_peak_req_m', ...
 'r51_phase_alpha','r51_phase_delay_s','r51_phase_exit_time_s', ...
 'native_corridor_ms','native_qp_ms','native_total_ms','last_stage1_ms'};

for k=1:numel(F)
    f=F{k};
    if isfield(P,f)
        v=P.(f);
        if isnumeric(v)||islogical(v)
            if isscalar(v)
                fprintf('%-58s = %.17g\n',f,double(v));
            else
                fprintf('%-58s = %s\n',f,mat2str(double(v),17));
            end
        else
            fprintf('%-58s = %s\n',f,char(string(v)));
        end
    else
        fprintf('%-58s = <NOT EXPOSED>\n',f);
    end
end

fprintf('\nInterpretation:\n');
fprintf(' tangent_only_qp_pass > 0 and full_qp_pass = 0 => R48 monotone-return rows caused the regression.\n');
fprintf(' tangent_only_qp_pass = 0 => inspect active-set/fixed-corridor numerical fallback or phased corridor itself.\n');
fprintf(' return_dir_min_dot < 0 => selected local return directions conflict geometrically.\n');
fprintf(' This audit is diagnostic only; acceptance still uses the unchanged full R51 QP.\n');
fprintf('====================================================================================================\n');

A=P;
assignin('base','R51D1',A);
end
