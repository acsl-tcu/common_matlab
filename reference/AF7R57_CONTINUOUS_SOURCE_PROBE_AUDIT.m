function A = AF7R57_CONTINUOUS_SOURCE_PROBE_AUDIT
assert(evalin('base','exist(''D38'',''var'')==1'), ...
    'Run: D38 = AF7R38_SPEED_ROOTCAUSE_AUDIT; first.');
D38=evalin('base','D38');
P=D38.pipeline;

fprintf('\n====================================================================================================\n');
fprintf(' AF7R57 | CONTINUOUS SOURCE-PATH HARD-CERTIFICATE PROBE\n');
fprintf(' diagnostic only: NO COMMIT / NO PRODUCTION REJOIN SELECTION\n');
fprintf('====================================================================================================\n');

F={ ...
 'r57_probe_ran','r57_source_qp_ok','r57_source_exact_c7', ...
 'r57_source_support_start_s','r57_source_support_end_s', ...
 'r57_source_corr_v_ub','r57_source_corr_a_ub','r57_source_corr_j_ub', ...
 'r57_speed_attempt','r57_speed_pass','r57_speed_stageA_pass', ...
 'r57_speed_stageB_attempt','r57_speed_stageB_finite_witness', ...
 'r57_speed_stageB_repr_fail','r57_speed_VL','r57_speed_A','r57_speed_J', ...
 'r57_speed_sigma','r57_speed_qdot','r57_speed_VQ', ...
 'r57_heading_attempt','r57_heading_pass','r57_heading_gamma', ...
 'r57_heading_leaf_count','r57_heading_split_count', ...
 'r57_geometry_attempt','r57_geometry_pass','r57_geometry_min_margin', ...
 'r57_commit_attempted','r57_rejoin_selected', ...
 'r57_old_clock_commit_binding_count','r57_horizon_duration_coupling_count', ...
 'r57_source_build_ms','r57_speed_ms','r57_heading_ms','r57_geometry_ms','r57_probe_ms', ...
 'native_total_ms','last_stage1_ms'};

for k=1:numel(F)
    f=F{k};
    if isfield(P,f)
        v=P.(f);
        if isnumeric(v)||islogical(v)
            if isscalar(v)
                fprintf('%-64s = %.17g\n',f,double(v));
            else
                fprintf('%-64s = %s\n',f,mat2str(double(v),17));
            end
        else
            fprintf('%-64s = %s\n',f,char(string(v)));
        end
    else
        fprintf('%-64s = <NOT EXPOSED>\n',f);
    end
end

fprintf('\n[INTERPRETATION]\n');
fprintf(' source_qp=0     : source-phase spatial corridor/QP failed.\n');
fprintf(' speed_pass=0    : exact-C7 source path fails continuous towing speed certificate.\n');
fprintf(' heading_pass=0  : no certified positive continuous horizontal-speed lower bound.\n');
fprintf(' geometry_pass=0 : continuous UAV/cable/load whole-body certificate failed.\n');
fprintf(' all three pass  : source-path core is continuously feasible; remaining design problem is automatic rejoin/time-law.\n');

fprintf('\n[SOURCE CONTRACT]\n');
src=fileread(which('af7r36_core.hpp'));
p0=strfind(src,'AF7R57 CONTINUOUS SOURCE-PATH PROBE');
p1=strfind(src,'R.finite_sensing_limit=lim;');
assert(~isempty(p0)&&~isempty(p1)&&p1(find(p1>p0(1),1))>p0(1),'R57 probe markers unavailable.');
blk=src(p0(1):p1(find(p1>p0(1),1)));

hasCommit=contains(blk,'R.success=true') || contains(blk,'R.cand=');
hasH=contains(blk,'L.horizon');
hasLiteral4=~isempty(regexp(blk,'(?<![A-Za-z0-9_])4\.0(?![A-Za-z0-9_])','once'));
hasOldClockEval=contains(blk,'ref.eval(t1)') || contains(blk,'ref.eval(tR)');
hasGeneralYaw=contains(src,'r57_apply_heading_certificate');

fprintf(' probe can commit a candidate                         = %d\n',hasCommit);
fprintf(' probe reads L.horizon as maneuver duration           = %d\n',hasH);
fprintf(' probe contains literal 4.0                           = %d\n',hasLiteral4);
fprintf(' probe directly evaluates old-clock rejoin target     = %d\n',hasOldClockEval);
fprintf(' general nonvanishing-horizontal-speed yaw theorem    = %d\n',hasGeneralYaw);

closed=~hasCommit && ~hasH && ~hasLiteral4 && ~hasOldClockEval && hasGeneralYaw && ...
       P.r57_commit_attempted==0 && P.r57_rejoin_selected==0 && ...
       P.r57_old_clock_commit_binding_count==0 && ...
       P.r57_horizon_duration_coupling_count==0;
fprintf(' R57 NON-COMMIT / NO-OLD-CLOCK CONTRACT CLOSED        = %d\n',closed);
fprintf('====================================================================================================\n');

A=P;
assignin('base','R57A',A);
end
