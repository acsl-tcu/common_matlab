function A = AF7R56_SOURCE_PHASE_SHADOW_AUDIT
assert(evalin('base','exist(''D38'',''var'')==1'), ...
    'Run: D38 = AF7R38_SPEED_ROOTCAUSE_AUDIT; first.');
D38=evalin('base','D38');
P=D38.pipeline;

fprintf('\n====================================================================================================\n');
fprintf(' AF7R56 | SOURCE-PHASE LOAD-PATH + INDEPENDENT TIME-LAW SHADOW\n');
fprintf(' diagnostic only: R55 production acceptance is unchanged\n');
fprintf('====================================================================================================\n');

F={ ...
 'r56_shadow_ran', ...
 'r56_source_qp_ok','r56_source_exact_c7','r56_source_knot_count', ...
 'r56_source_episode_count','r56_source_corridor_row_count','r56_source_normal_row_count', ...
 'r56_source_side_plus_count','r56_source_side_minus_count','r56_source_side_fail_count', ...
 'r56_source_support_start_s','r56_source_rejoin_s', ...
 'r56_source_corr_v_ub','r56_source_corr_a_ub','r56_source_corr_j_ub', ...
 'r56_source_min_hxy','r56_yaw_source_regular', ...
 'r56_const_sample_count','r56_alpha_max_pointwise', ...
 'r56_const_dynamic_pass','r56_const_geometry_point_pass', ...
 'r56_const_max_vl','r56_const_max_vq','r56_const_min_sigma','r56_const_max_qdot', ...
 'r56_const_min_geom_margin', ...
 'r56_entry_Tmax_s','r56_entry_bestcase_pass', ...
 'r56_entry_max_vq','r56_entry_min_sigma','r56_entry_max_qdot', ...
 'r56_exit_Tmax_s','r56_exit_bestcase_pass', ...
 'r56_exit_max_vq','r56_exit_min_sigma','r56_exit_max_qdot', ...
 'r56_chain_rule_max_error','r56_shadow_ms', ...
 'r56_old_clock_binding_count','r56_horizon_duration_coupling_count', ...
 'r55_obstacle_count','r55_min_ellipsoid_axis_m','r55_max_ellipsoid_axis_m', ...
 'r55_max_rotation_orth_error', ...
 'native_total_ms','last_stage1_ms'};

for k=1:numel(F)
    f=F{k};
    if isfield(P,f)
        v=P.(f);
        if isnumeric(v)||islogical(v)
            if isscalar(v)
                fprintf('%-66s = %.17g\n',f,double(v));
            else
                fprintf('%-66s = %s\n',f,mat2str(double(v),17));
            end
        else
            fprintf('%-66s = %s\n',f,char(string(v)));
        end
    else
        fprintf('%-66s = <NOT EXPOSED>\n',f);
    end
end

fprintf('\n[SOURCE CONTRACT]\n');
src=fileread(which('af7r36_core.hpp'));
p0=strfind(src,'AF7R56 THEORY/SHADOW.');
p1=strfind(src,'AF7R55 PRIMARY:');
assert(~isempty(p0)&&~isempty(p1)&&p1(find(p1>p0(1),1))>p0(1), ...
    'R56 shadow markers unavailable.');
blk=src(p0(1):p1(find(p1>p0(1),1)));

hasH=contains(blk,'L.horizon');
has4=~isempty(regexp(blk,'(?<![A-Za-z0-9_])4\.0(?![A-Za-z0-9_])','once'));
hasOldClock=contains(blk,'ref.eval(t1)')||contains(blk,'ref.eval(tR)');
hasSource=contains(src,'r56_eval_const_alpha_samples') && ...
          contains(src,'candidate_eval(ref,Cs,SJ[0])');
hasChain=contains(src,'r51_compose_vec_jet(X,SJ)') && ...
         contains(src,'3.0*SJ[1]*SJ[2]');
hasExactSupport=contains(src,'r52_best_support_plane') && ...
                contains(src,'o.axes.x') && contains(src,'tmul(o.R');

fprintf(' shadow block reads L.horizon as maneuver duration       = %d\n',hasH);
fprintf(' shadow block contains literal 4.0                       = %d\n',has4);
fprintf(' shadow block directly binds old-clock reference at join = %d\n',hasOldClock);
fprintf(' source-path / independent-time evaluator present        = %d\n',hasSource);
fprintf(' exact chain-rule composition present                    = %d\n',hasChain);
fprintf(' oriented-ellipsoid exact-support machinery present      = %d\n',hasExactSupport);

closed=~hasH && ~has4 && ~hasOldClock && hasSource && hasChain && hasExactSupport && ...
       isfield(P,'r56_old_clock_binding_count') && P.r56_old_clock_binding_count==0 && ...
       isfield(P,'r56_horizon_duration_coupling_count') && P.r56_horizon_duration_coupling_count==0;
fprintf(' R56 SOURCE/TIME SEPARATION CONTRACT CLOSED               = %d\n',closed);

fprintf('\nInterpretation rules:\n');
fprintf(' 1) source_qp_ok=0          -> spatial/source-phase corridor issue.\n');
fprintf(' 2) source_qp_ok=1 but const_dynamic_pass=0 -> load-path timing issue.\n');
fprintf(' 3) const_dynamic_pass=1 but geometry_point_pass=0 -> q-dependent UAV/cable geometry coupling remains.\n');
fprintf(' 4) entry_bestcase_pass=0   -> even using all detected pre-unsafe source lead, the C7 phase-rate transition is too aggressive.\n');
fprintf(' 5) chain_rule_max_error should be near binary64 roundoff; otherwise composition implementation is wrong.\n');
fprintf('\nNOTE: pointwise dynamic/geometry values are SHADOW DIAGNOSTICS, not continuous acceptance certificates.\n');
fprintf('      Production hard safety remains unchanged and fail-closed.\n');
fprintf('====================================================================================================\n');

A=P;
assignin('base','R56A',A);
end
