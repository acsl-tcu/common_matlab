function A = AF7R48R1_PHASE_REJOIN_AUDIT
% AF7R48 production diagnostic summary. No control/safety authority.
% Run after:
%   D38 = AF7R38_SPEED_ROOTCAUSE_AUDIT;

assert(evalin('base','exist(''D38'',''var'')==1'), ...
    'Run: D38 = AF7R38_SPEED_ROOTCAUSE_AUDIT; first.');
D38=evalin('base','D38');
assert(isstruct(D38)&&isfield(D38,'pipeline'));
P=D38.pipeline;

fprintf('\n====================================================================================================\n');
fprintf(' AF7R48R1 | DELAY-PRESERVING PHASE REJOIN / RUNTIME AUDIT\n');
fprintf('====================================================================================================\n');

fprintf('\n[A. PIPELINE COUNTS]\n');
keys={'stage1_count','stage2_count','commit_count','fail_count','last_reason', ...
      'last_stage1_ms','last_stage2_ms','last_stage3_ms'};
for k=1:numel(keys)
    f=keys{k};
    if ~isfield(P,f), fprintf('%-46s = <NOT EXPOSED>\n',f); continue; end
    v=P.(f);
    if isnumeric(v)||islogical(v)
        fprintf('%-46s = %s\n',f,mat2str(double(v),17));
    else
        fprintf('%-46s = %s\n',f,char(string(v)));
    end
end

fprintf('\n[B. LAST NATIVE / REJOIN STATE]\n');
keys={ ...
    'predeadline_candidate_success','predeadline_candidate_family', ...
    'predeadline_t0','predeadline_t1','predeadline_payload_speed', ...
    'predeadline_uav_speed','predeadline_qdot', ...
    'r37_support_start_s','r37_actual_rejoin_s', ...
    'r34_delay_candidate_count','r34_terminal_hold_exact_count', ...
    'r34_monotone_progress_count','r34_selected_delay_s', ...
    'r34_selected_hold_start_s','r34_selected_alpha', ...
    'r34_selected_velocity_ub','r34_selected_accel_ub','r34_selected_jerk_ub', ...
    'r48_natural_rejoin_trial_count','r48_natural_rejoin_selected_count', ...
    'r48_natural_rejoin_seed_t1','r48_exit_monotone_row_count', ...
    'rejoin_search_full_certificate_tests','rejoin_search_endpoint_safe_count', ...
    'rejoin_search_baseline_t1','rejoin_search_selected_t1','rejoin_search_selected', ...
    'r47_active_base_replan_count','r47_nonaffine_base_count', ...
    'r47_heading_leaf_count','r47_heading_split_count', ...
    'native_episode_ms','native_hessian_ms','native_corridor_ms','native_qp_ms', ...
    'native_candidate_build_ms','native_synthesis_ms','native_candidate_ms','native_total_ms'};
for k=1:numel(keys)
    f=keys{k};
    if ~isfield(P,f), fprintf('%-50s = <NOT EXPOSED>\n',f); continue; end
    v=P.(f);
    if isnumeric(v)||islogical(v)
        if isscalar(v), fprintf('%-50s = %.17g\n',f,double(v));
        else, fprintf('%-50s = %s\n',f,mat2str(double(v),17)); end
    else
        fprintf('%-50s = %s\n',f,char(string(v)));
    end
end

fprintf('\n[C. ALL STAGED EVENTS]\n');
EV=[];
if isfield(P,'r47_event_ring') && isfield(P,'r47_event_ring_count') && ...
        isfield(P,'r47_event_ring_head')
    R=double(P.r47_event_ring);
    n=min(double(P.r47_event_ring_count),size(R,1));
    h=double(P.r47_event_ring_head);
    if n>0
        cap=size(R,1);
        first=mod(h-n,cap)+1;
        idx=mod((first-1)+(0:n-1),cap)+1;
        EV=R(idx,:);
        T=array2table(EV,'VariableNames',{ ...
            'seq','time_s','stage','stage_ms','success','status', ...
            't0','t1','episode_ms','hessian_ms','corridor_ms','qp_ms', ...
            'candidate_build_ms','synthesis_ms','candidate_ms','native_total_ms', ...
            'tangent_intervals','geometry_pass'});
        disp(T);
    else
        fprintf('No staged events recorded.\n');
    end
else
    fprintf('R47/R48 event ring not exposed.\n');
end

fprintf('\n[D. QUICK CHECK]\n');
if isfield(P,'r48_natural_rejoin_selected_count') && ...
        double(P.r48_natural_rejoin_selected_count)>0
    fprintf(' natural-time search seed selected                = YES\n');
else
    fprintf(' natural-time search seed selected                = NO / last event not successful\n');
end
if isfield(P,'r34_selected_delay_s') && isfinite(double(P.r34_selected_delay_s))
    fprintf(' schedule delay Delta [s]                         = %.9g\n',double(P.r34_selected_delay_s));
    fprintf(' same-clock catch-up required                     = 0\n');
end
if isfield(P,'r48_exit_monotone_row_count') && ...
        isfinite(double(P.r48_exit_monotone_row_count))
    fprintf(' exit monotone/no-overshoot rows                  = %.0f\n', ...
        double(P.r48_exit_monotone_row_count));
end
if isfield(P,'last_stage2_ms') && isfinite(double(P.last_stage2_ms))
    fprintf(' last Stage2 < 10 ms target                        = %d\n',double(P.last_stage2_ms)<10);
    fprintf(' last Stage2 < 25 ms control-period condition      = %d\n',double(P.last_stage2_ms)<25);
end
if isfield(P,'last_stage1_ms') && isfinite(double(P.last_stage1_ms))
    fprintf(' last Stage1 < 10 ms target                        = %d\n',double(P.last_stage1_ms)<10);
    fprintf(' last Stage1 < 25 ms control-period condition      = %d\n',double(P.last_stage1_ms)<25);
end
fprintf('====================================================================================================\n');

A=struct('pipeline',P,'events',EV);
assignin('base','R48A',A);
end
