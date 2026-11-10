function R=CHECK_TRAJECTORY_C9_GEOMETRY_TIGHT_WCET_AB()
% 15 scenarios, SAME integrated nominal pipeline, baseline vs two
% mathematical geometry/SEARCH-only changes. No claim of 25ms WCET,
% physical execution safety or a certified ideal-closed-loop tube.
% Every output lives in common_matlab, independent of pwd.
root=fileparts(mfilename('fullpath'));
name='TRAJECTORY_C9_GEOMETRY_TIGHT_WCET_AB_RESULT';
R=struct('status','NOT_STARTED','stage','SETUP','error_message','', ...
    'safe_to_commit',false,'baseline_status','', 'tight_status','', ...
    'property_test_count',0,'property_test_pass',false, ...
    'property_tight_max_improvement_m',NaN, ...
    'property_outer_ball_screen_pass',false, ...
    'baseline_nominal_success',NaN,'tight_nominal_success',NaN, ...
    'baseline_unknown_count',NaN,'tight_unknown_count',NaN, ...
    'baseline_median_ms',NaN,'tight_median_ms',NaN, ...
    'baseline_p95_ms',NaN,'tight_p95_ms',NaN, ...
    'baseline_max_ms',NaN,'tight_max_ms',NaN, ...
    'baseline_repeat_max_ms',NaN(2,1), ...
    'tight_repeat_max_ms',NaN(2,1), ...
    'baseline_repeat_p95_ms',NaN(2,1), ...
    'tight_repeat_p95_ms',NaN(2,1), ...
    'baseline_repeat_jitter_ms',NaN(2,1), ...
    'tight_repeat_jitter_ms',NaN(2,1), ...
    'geometry_median_ms',NaN(2,1), ...
    'plane_median_ms',NaN(2,1), ...
    'qp_outer_ball_groups_tested',0, ...
    'qp_outer_ball_groups_skipped',0, ...
    'lambda_envelope_axes_tested',0, ...
    'lambda_envelope_axes_improved',0, ...
    'nominal_regressions',0,'new_nominal_successes',0, ...
    'case_status',{cell(15,2)}, ...
    'case_full_ms',NaN(15,2), ...
    'case_geometry_ms',NaN(15,2), ...
    'case_planes_ms',NaN(15,2), ...
    'case_qp_skips',zeros(15,1), ...
    'case_cable_axes',zeros(15,1), ...
    'case_failure',{cell(15,2)}, ...
    'hlc_remained_unproven',false, ...
    'result_zip',fullfile(root,[name '_FOR_CHATGPT.zip']));
try
    R.stage='MATH_ENCLOSURE_NECESSARY_CHECK';
    % Algebraic all-time cable proof: q is in a chord-radius ball about
    % d and lambda in [0,1]. The signed support must include lambda.
    rngState=rng;
    cleanup=onCleanup(@()rng(rngState)); %#ok<NASGU>
    rng(967,'twister');
    N=20000; maxgain=0;
    for k=1:N
        mn=20*randn; sh=20*randn; pad=5*rand;
        old=mn-max(0,sh)-pad;
        new=mn-max(0,sh+pad);
        if new+1e-12<old
            error('C9_GEOM:LambdaBound','New projected lower bound is weaker');
        end
        maxgain=max(maxgain,new-old);
        % For any lambda in [0,1], a direction error component bounded
        % by pad/L is covered by the new expression.
        for t=1:3
            lambda=rand;
            err=pad*(2*rand-1);
            actual=mn-lambda*(sh+err);
            if actual+1e-12<new
                error('C9_GEOM:CableContainment', ...
                    'Some cable parameter violates the lower bound');
            end
        end
    end
    R.property_test_count=N;
    R.property_test_pass=true;
    R.property_tight_max_improvement_m=maxgain;
    % Broadphase necessary skip: cable/UAV/payload centres are within L
    % of payload. A shape within its canonical outer sphere cannot touch
    % any body if the strict centre-distance inequality holds.
    for k=1:10000
        L=0.1+5*rand;rad=rand(1,3);outer=2*rand;delta=0.1*rand;
        threshold=L+max(rad)+outer+delta;
        distance=threshold+1e-5+5*rand;
        pointNorm=distance-(L*rand+outer*rand+max(rad)*rand);
        if ~(pointNorm>delta)
            error('C9_GEOM:OuterBall','Outer sphere screen violated');
        end
    end
    R.property_outer_ball_screen_pass=true;
    R.stage='BASELINE_15_AND_REPEAT';
    A=CHECK_TRAJECTORY_C9_WHOLE_FULL_PIPELINE_AUDIT('baseline');
    R.baseline_status=A.status;
    if ~isempty(A.error_message)
        error('C9_GEOM:Baseline','%s',A.error_message);
    end
    R.stage='TIGHT_15_AND_REPEAT';
    B=CHECK_TRAJECTORY_C9_WHOLE_FULL_PIPELINE_AUDIT('tight');
    R.tight_status=B.status;
    if ~isempty(B.error_message)
        error('C9_GEOM:Tight','%s',B.error_message);
    end
    if A.matrix_count~=15||B.matrix_count~=15
        error('C9_GEOM:Matrix','Both 15-case matrices must finish');
    end
    R.baseline_nominal_success=A.matrix_nominal_complete_count;
    R.tight_nominal_success=B.matrix_nominal_complete_count;
    R.baseline_unknown_count=A.matrix_unknown_count;
    R.tight_unknown_count=B.matrix_unknown_count;
    R.baseline_median_ms=A.matrix_median_ms;
    R.tight_median_ms=B.matrix_median_ms;
    R.baseline_p95_ms=A.matrix_p95_ms;
    R.tight_p95_ms=B.matrix_p95_ms;
    R.baseline_max_ms=A.matrix_max_ms;
    R.tight_max_ms=B.matrix_max_ms;
    R.baseline_repeat_max_ms=A.native_max_ms;
    R.tight_repeat_max_ms=B.native_max_ms;
    R.baseline_repeat_p95_ms=A.native_p95_ms;
    R.tight_repeat_p95_ms=B.native_p95_ms;
    R.baseline_repeat_jitter_ms=A.native_jitter_range_ms;
    R.tight_repeat_jitter_ms=B.native_jitter_range_ms;
    R.geometry_median_ms=[median([A.matrix_cases.geometry_force_ms]); ...
        median([B.matrix_cases.geometry_force_ms])];
    R.plane_median_ms=[median([A.matrix_cases.plane_ms]); ...
        median([B.matrix_cases.plane_ms])];
    R.hlc_remained_unproven=A.hlc_unbound_stays_unproven&& ...
        B.hlc_unbound_stays_unproven&&A.hlc_partial_binding_rejected&& ...
        B.hlc_partial_binding_rejected;
    for j=1:15
        X=A.matrix_cases(j);Y=B.matrix_cases(j);
        R.case_status(j,:)={X.status,Y.status};
        R.case_failure(j,:)={X.search_failure_detail,Y.search_failure_detail};
        R.case_full_ms(j,:)=[X.total_ms,Y.total_ms];
        R.case_geometry_ms(j,:)=[X.geometry_force_ms,Y.geometry_force_ms];
        R.case_planes_ms(j,:)=[X.plane_ms,Y.plane_ms];
        R.case_qp_skips(j)=Y.qp_outer_ball_groups_skipped;
        R.case_cable_axes(j)=Y.cable_envelope_axis_evaluations;
        if X.nominal_complete && ~Y.nominal_complete
            R.nominal_regressions=R.nominal_regressions+1;
        elseif ~X.nominal_complete && Y.nominal_complete
            R.new_nominal_successes=R.new_nominal_successes+1;
        end
    end
    R.qp_outer_ball_groups_tested=sum( ...
        [B.matrix_cases.qp_outer_ball_groups_tested]);
    R.qp_outer_ball_groups_skipped=sum( ...
        [B.matrix_cases.qp_outer_ball_groups_skipped]);
    R.lambda_envelope_axes_tested=sum( ...
        [B.matrix_cases.cable_envelope_axis_evaluations]);
    R.lambda_envelope_axes_improved=sum( ...
        [B.matrix_cases.cable_envelope_strictly_improved]);
    R.stage='FINAL_REVIEW';
    if R.nominal_regressions==0 && R.hlc_remained_unproven && ...
            R.property_test_pass && R.property_outer_ball_screen_pass
        R.status='PASS_AB_NO_NOMINAL_REGRESSION__WCET_AND_HLC_PENDING';
    else
        R.status='REVIEW_AB_NOMINAL_REGRESSION_OR_HLC_SAFETY';
    end
catch ME
    R.status='FAIL_GEOMETRY_TIGHT_WCET_AB';
    R.error_message=[ME.identifier ' | ' ME.message];
end
try
    save(fullfile(root,[name '.mat']),'R');
    fid=fopen(fullfile(root,[name '.txt']),'w');
    if fid>=0
        fprintf(fid,'status=%s\nstage=%s\nerror=%s\n',R.status,R.stage,R.error_message);
        fprintf(fid,'baseline=%s tight=%s\n',R.baseline_status,R.tight_status);
        fprintf(fid,'nominal_success=%g %g unknown=%g %g regressions=%d new=%d\n', ...
            R.baseline_nominal_success,R.tight_nominal_success, ...
            R.baseline_unknown_count,R.tight_unknown_count, ...
            R.nominal_regressions,R.new_nominal_successes);
        fprintf(fid,'full_median_ms=%.6f %.6f\nfull_p95_ms=%.6f %.6f\nfull_max_ms=%.6f %.6f\n', ...
            R.baseline_median_ms,R.tight_median_ms, ...
            R.baseline_p95_ms,R.tight_p95_ms,R.baseline_max_ms,R.tight_max_ms);
        fprintf(fid,'repeat_max_ms=%s %s\n',mat2str(R.baseline_repeat_max_ms), ...
            mat2str(R.tight_repeat_max_ms));
        fprintf(fid,'repeat_jitter_ms=%s %s\n', ...
            mat2str(R.baseline_repeat_jitter_ms),mat2str(R.tight_repeat_jitter_ms));
        fprintf(fid,'geometry_median_ms=%s plane_median_ms=%s\n', ...
            mat2str(R.geometry_median_ms),mat2str(R.plane_median_ms));
        fprintf(fid,'qp_group_tests=%d qp_skips=%d cable_axes=%d strengthened=%d\n', ...
            R.qp_outer_ball_groups_tested,R.qp_outer_ball_groups_skipped, ...
            R.lambda_envelope_axes_tested,R.lambda_envelope_axes_improved);
        fprintf(fid,'hlc_not_falsely_proven=%d\n',R.hlc_remained_unproven);
        for j=1:15
            fprintf(fid,'%02d old=%.5f new=%.5f old_status=%s new_status=%s ', ...
                j,R.case_full_ms(j,1),R.case_full_ms(j,2), ...
                R.case_status{j,1},R.case_status{j,2});
            fprintf(fid,'skip=%d cable=%d old_fail=%s new_fail=%s\n', ...
                R.case_qp_skips(j),R.case_cable_axes(j), ...
                R.case_failure{j,1},R.case_failure{j,2});
        end
        fclose(fid);
    end
    zip(R.result_zip,{fullfile(root,[name '.mat']),fullfile(root,[name '.txt'])});
catch ME
    if isempty(R.error_message),R.error_message=['ZIP: ' ME.message];end
end
R.safe_to_commit=false;
end
