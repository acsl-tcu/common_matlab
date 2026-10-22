function OUT = AF7R54_PHASE_HORIZON_AUDIT
assert(evalin('base','exist(''D38'',''var'')==1'), ...
    'Run: D38 = AF7R38_SPEED_ROOTCAUSE_AUDIT; first.');
D38=evalin('base','D38');
P=D38.pipeline;

fprintf('\n====================================================================================================\n');
fprintf(' AF7R54 | HORIZON-DECOUPLED ADAPTIVE-PHASE AUDIT\n');
fprintf('====================================================================================================\n');

F={ ...
 'predeadline_candidate_success','predeadline_candidate_family', ...
 'predeadline_payload_speed','predeadline_uav_speed','predeadline_qdot', ...
 'r54_phase_candidate_count','r54_phase_trial_count', ...
 'r54_phase_base_certified_count','r54_phase_episode_pass_count', ...
 'r54_phase_episode_empty_count','r54_selected_tau_multiple', ...
 'r54_selected_alpha','r54_selected_delay_s', ...
 'r54_selected_first_unsafe_lead_s','r54_max_first_unsafe_lead_s', ...
 'r54_maneuver_horizon_coupling_count', ...
 'r51_phase_alpha','r51_phase_delay_s','r51_phase_exit_time_s', ...
 'r37_side_assignments','r37_spatial_qp_solutions','r37_exact_c7_candidates', ...
 'r24e_family_speed_pass','r24e_family_speed_fail', ...
 'r35_heading_candidate_count','r35_heading_pass_count','r35_heading_reject_count', ...
 'r52_rejoin_time_trial_count','r52_selected_spatial_rejoin_s', ...
 'native_episode_ms','native_hessian_ms','native_corridor_ms','native_qp_ms', ...
 'native_candidate_build_ms','native_total_ms','last_stage1_ms'};

for k=1:numel(F)
    f=F{k};
    if isfield(P,f)
        v=P.(f);
        if isnumeric(v)||islogical(v)
            if isscalar(v), fprintf('%-62s = %.17g\n',f,double(v));
            else, fprintf('%-62s = %s\n',f,mat2str(double(v),17)); end
        else
            fprintf('%-62s = %s\n',f,char(string(v)));
        end
    else
        fprintf('%-62s = <NOT EXPOSED>\n',f);
    end
end

core=fileread(fullfile(fileparts(mfilename('fullpath')),'af7r36_core.hpp'));
a=strfind(core,'AF7R54 PRIMARY: HORIZON-DECOUPLED');
b=strfind(core,'Single production theory: no alternative trajectory-family fallback.');
assert(numel(a)==1 && ~isempty(b),'[AF7R54] primary source markers not unique.');
b=b(find(b>a,1,'first'));
primary=core(a:b-1);

S=struct();
S.primary_reads_L_horizon=contains(primary,'L.horizon');
S.primary_contains_literal_4_0=contains(primary,'4.0');
S.primary_contains_entryDur=contains(primary,'entryDur');
S.primary_uses_natural_time=contains(primary,'sqrt(L.L/L.g)');
S.primary_rebuilds_phase_episode=contains(primary,'r52_build_phase_episode');
S.primary_uses_tau_multiple_continuation=contains(primary,'r54_phase_candidates');
S.finite_sensing_still_uses_horizon=contains(core,'/(L.horizon+L.dt)');

fprintf('\n[SOURCE CONTRACT]\n');
fprintf(' R54 primary reads L.horizon                     = %d\n',S.primary_reads_L_horizon);
fprintf(' R54 primary contains literal 4.0               = %d\n',S.primary_contains_literal_4_0);
fprintf(' R54 primary contains old entryDur coupling      = %d\n',S.primary_contains_entryDur);
fprintf(' R54 primary uses sqrt(L/g) search coordinate    = %d\n',S.primary_uses_natural_time);
fprintf(' R54 primary rebuilds phase-specific episode     = %d\n',S.primary_rebuilds_phase_episode);
fprintf(' R54 primary uses adaptive tau continuation      = %d\n',S.primary_uses_tau_multiple_continuation);
fprintf(' finite-sensing theorem still uses horizon       = %d\n',S.finite_sensing_still_uses_horizon);

closed = ~S.primary_reads_L_horizon && ...
         ~S.primary_contains_literal_4_0 && ...
         ~S.primary_contains_entryDur && ...
          S.primary_uses_natural_time && ...
          S.primary_rebuilds_phase_episode && ...
          S.primary_uses_tau_multiple_continuation && ...
          S.finite_sensing_still_uses_horizon;
fprintf(' HORIZON / MANEUVER SEPARATION CLOSED            = %d\n',closed);

fprintf('\nMeaning: H is retained for trigger/finite-sensing authority, but is not an entry/maneuver/rejoin duration.\n');
fprintf('====================================================================================================\n');

OUT=struct('pipeline',P,'source',S,'closed',closed);
assignin('base','R54A',OUT);
end
