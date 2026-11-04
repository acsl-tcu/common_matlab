function R=CHECK_TRIGGER_THREE_RUN_FULL_STACK()
% COMMON_ROOT disposable analysis. Reads 3 previously captured actual
% Flight logs. DO NOT call this during the Flight loop. In particular,
% run_flight_headless has a `clear all`, so use SAVE-per-run captures.
%
% Disjoint replanning_detail_ms(1:6) are outer sensor, memory,
% prepared-binding, trigger, obstacle log, and result attachment.
% The sensor.elapsed and memory.elapsed fields are INNER/nested timers.
% Neither these nor predictor/reference nested clocks may be added as
% independent full-cycle work. A residual is still only wall-clock.
root=fileparts(mfilename('fullpath'));
fields={'sensor_outer','memory_outer','prepared_attach','trigger', ...
    'obstacle_log','result_attach','unattributed_outer'};
R=struct('status','FAIL_THREE_RUN_FULL_STACK', ...
    'reason','','runs',struct([]),'detail_labels',{fields}, ...
    'cases',0,'failures',0,'run_count',0, ...
    'top_update',zeros(0,10), ...
    'max_common_over_4_ms_k',zeros(1,0), ...
    'max_common_all_phase_over_4_ms_k',zeros(1,0), ...
    'status_is_wall_clock_only',true,'physical_safety_proved',false);
try
    durations=nan(3,0);active_prediction=false(3,0);
    out=struct([]);
    for run=1:3
        file=fullfile(root,sprintf('TRIGGER_DIAG_RUN_%02d.mat',run));
        assert(exist(file,'file')==2, ...
            'CHECK_TRIGGER_FULL_STACK:MissingRun', ...
            'Missing %s. Capture all 3 real Flights first.',file);
        load_data=load(file,'C');
        assert(isfield(load_data,'C')&&isstruct(load_data.C), ...
            'CHECK_TRIGGER_FULL_STACK:Capture', ...
            'Expected saved C capture, not a nominal recreated trace.');
        C=load_data.C;
        needed={'replanner_ms','sensor_ms','memory_ms','detail', ...
            'predicted','rhs_evals','rejected_steps', ...
            'new_count','detected_changed','t','predictor_ms'};
        % Original C provides 't' in some historical captures as 'time_s'.
        needed_without_time=needed(~strcmp(needed,'t'));
        assert(all(isfield(C,needed_without_time)), ...
            'CHECK_TRIGGER_FULL_STACK:RequiredFields', ...
            'Detailed real Flight capture is missing a required field.');
        u=reshape(double(C.replanner_ms),1,[]);
        N=numel(u);
        % Real captures store N-by-6: one row per tick and 6 disjoint
        % outer clocks.  Earlier versions of this auditor wrongly required
        % 6-by-N and rejected perfectly valid flight data.  Accept either
        % layout explicitly; never silently reshape or reorder data.
        raw_detail=double(C.detail);
        if isequal(size(raw_detail),[N 6])
            d=raw_detail.';
            detail_layout='Nx6_FLIGHT_ROWS';
        elseif isequal(size(raw_detail),[6 N])
            d=raw_detail;
            detail_layout='6xN_STAGE_ROWS';
        else
            error('CHECK_TRIGGER_FULL_STACK:DetailShape', ...
                'Expected Nx6 or 6xN detail array for N=%d; received %s.', ...
                N,mat2str(size(raw_detail)));
        end
        assert(N>1&&all(isfinite(u)) && ...
            all(isfinite(d(:))) && all(d(:)>=0), ...
            'CHECK_TRIGGER_FULL_STACK:Detail', ...
            'Six finite, nonnegative disjoint per-tick stage clocks required.');
        if isfield(C,'t')
            t=reshape(double(C.t),1,[]);
        elseif isfield(C,'time_s')
            t=reshape(double(C.time_s),1,[]);
        else
            t=(0:N-1); % INDEX-ONLY substitute, never claim physical time.
        end
        if numel(t)~=N || any(~isfinite(t)),t=(0:N-1);end
        pred=reshape(logical(C.predicted),1,[]);
        sensor=reshape(double(C.sensor_ms),1,[]);
        memory=reshape(double(C.memory_ms),1,[]);
        rhs=reshape(double(C.rhs_evals),1,[]);
        reject=reshape(double(C.rejected_steps),1,[]);
        acquired=reshape(double(C.new_count),1,[]);
        detect=reshape(double(C.detected_changed),1,[]);
        native=reshape(double(C.predictor_ms),1,[]);
        assert(all([numel(pred),numel(sensor),numel(memory), ...
            numel(rhs),numel(reject),numel(acquired),numel(detect), ...
            numel(native)]==N) && all(isfinite(sensor)) && ...
            all(isfinite(memory)), ...
            'CHECK_TRIGGER_FULL_STACK:Alignment', ...
            'Every timing vector must match saved frame count.');
        % d was normalized to 6xN and verified above.
        % Confirm the fourth stage is actually the full trigger wall time,
        % not some unrelated clock. This was exact in the three real
        % saved Flight captures supplied on 2026-11-05.
        if isfield(C,'trigger_ms') && ~isempty(C.trigger_ms)
            direct=reshape(double(C.trigger_ms),1,[]);
            assert(numel(direct)==N && all(isfinite(direct)) && ...
                all(abs(direct-d(4,:))<=1e-9*max(1,abs(direct))), ...
                'CHECK_TRIGGER_FULL_STACK:Stage4', ...
                'Captured stage 4 disagrees with the trigger log clock.');
        end
        % Six disjoint OUTER stages plus all uninstrumented gaps. Do not
        % treat the nested sensor/memory timer as an additive stage.
        residual=u-sum(d,1);
        outside_trigger=u-d(4,:); % includes sensor, memory, outer code
        pred_u=u(pred);pred_trigger=d(4,pred);
        assert(~isempty(pred_u),'CHECK_TRIGGER_FULL_STACK:Prediction', ...
            'This run has no genuine Flight predictions.');
        [~,rank]=sort(u,'descend');
        top=rank(1:min(20,N));
        U=struct();
        U.run=run;U.detail_layout=detail_layout;U.frames=N;U.prediction_count=nnz(pred);
        U.update_median_ms=median(u);U.update_p95_ms=local_prct(u,95);
        U.update_p99_ms=local_prct(u,99);U.update_max_ms=max(u);
        U.predicted_update_median_ms=median(pred_u);
        U.predicted_update_p99_ms=local_prct(pred_u,99);
        U.predicted_update_max_ms=max(pred_u);
        U.trigger_median_ms=median(d(4,:));
        U.trigger_p95_ms=local_prct(d(4,:),95);
        U.trigger_p99_ms=local_prct(d(4,:),99);
        U.trigger_max_ms=max(d(4,:));
        U.sensor_inner_median_ms=median(sensor);
        U.sensor_inner_max_ms=max(sensor);
        U.memory_inner_median_ms=median(memory);
        U.memory_inner_max_ms=max(memory);
        U.outer_nontrigger_median_ms=median(outside_trigger);
        U.outer_nontrigger_p99_ms=local_prct(outside_trigger,99);
        U.outer_nontrigger_max_ms=max(outside_trigger);
        U.stage_medians_ms=median(d,2).';
        U.stage_maxima_ms=max(d,[],2).';
        U.unattributed_median_ms=median(residual);
        U.unattributed_p99_ms=local_prct(residual,99);
        U.unattributed_max_ms=max(residual);
        U.over_4_ms=nnz(u(pred)>4);
        U.over_5_ms=nnz(u(pred)>5);
        U.top_columns={'k','t_or_index','update','trigger', ...
            'sensor_inner','memory_inner','nontrigger_outer', ...
            'unattributed_outer','new_ids','rhs_evals', ...
            'rejected_steps','detected_change'};
        U.top=[top(:),t(top).',u(top).',d(4,top).', ...
            sensor(top).',memory(top).', ...
            outside_trigger(top).',residual(top).', ...
            acquired(top).',rhs(top).',reject(top).',detect(top).'];
        if isempty(out),out=U;else,out(end+1)=U;end %#ok<AGROW>
        if run==1
            durations=nan(3,N);
            active_prediction=false(3,N);
        else
            assert(N==size(durations,2), ...
                'CHECK_TRIGGER_FULL_STACK:FlightLength', ...
                'All captures must use the same frame grid for cross-run comparison.');
        end
        durations(run,:)=u;
        active_prediction(run,:)=pred;
    end
    R.runs=out;R.run_count=3;R.cases=3;
    % Exclude Arming/Takeoff/non-Flight, including k=1, from the
    % recurrent predicted-Flight spike index. Keep the all-phase result
    % separately to avoid silently discarding diagnostic information.
    R.max_common_all_phase_over_4_ms_k=find(sum(durations>4,1)>=2);
    R.max_common_over_4_ms_k=find( ...
        sum((durations>4)&active_prediction,1)>=2);
    A=zeros(0,10);
    for run=1:3
        x=out(run).top;
        % Main top rows: keep the top-10 per run, with no double-counted
        % nested timings. `k` and `run` identify the originating log.
        for j=1:min(10,size(x,1))
            A(end+1,:)=[run,x(j,1),x(j,3:9),x(j,12)]; %#ok<AGROW>
        end
    end
    [~,p]=sort(A(:,3),'descend');
    R.top_update=A(p,:);
    R.status='PASS_THREE_RUN_FULL_STACK__WALL_CLOCK_NOT_WCET';
catch ME
    R.failures=R.failures+1;
    R.reason=[ME.identifier ' | ' ME.message];
end
save(fullfile(root,'TRIGGER_THREE_RUN_FULL_STACK_RESULT.mat'),'R');
f=fopen(fullfile(root,'TRIGGER_THREE_RUN_FULL_STACK_RESULT.txt'),'w');
if f>=0
    fprintf(f,'status=%s\nruns=%d\nfailures=%d\n', ...
        R.status,R.run_count,R.failures);
    fprintf(f,'reason=%s\n',R.reason);
    fprintf(f,['NOTE: Six disjoint OUTER stages are sensor,memory,prepared,' ...
        'trigger,obstacle-log,result-attachment.\n']);
    fprintf(f,['NOTE: INNER sensor/memory/predictor/reference timings are' ...
        ' NESTED and must not be added.\n']);
    fprintf(f,'NOTE: MATLAB/Windows wall-clock maxima are not WCET.\n');
    for run=1:numel(R.runs)
        Q=R.runs(run);
        fprintf(f,['RUN %d: frames=%d predictions=%d ' ...
            'update_median=%.5f p95=%.5f p99=%.5f max=%.5f ms\n'], ...
            Q.run,Q.frames,Q.prediction_count, ...
            Q.update_median_ms,Q.update_p95_ms, ...
            Q.update_p99_ms,Q.update_max_ms);
        fprintf(f,['  trigger_median=%.5f p95=%.5f p99=%.5f max=%.5f; ' ...
            'nontrigger_median=%.5f p99=%.5f max=%.5f\n'], ...
            Q.trigger_median_ms,Q.trigger_p95_ms,Q.trigger_p99_ms, ...
            Q.trigger_max_ms,Q.outer_nontrigger_median_ms, ...
            Q.outer_nontrigger_p99_ms,Q.outer_nontrigger_max_ms);
        fprintf(f,['  sensor_inner_median=%.5f max=%.5f; ' ...
            'memory_inner_median=%.5f max=%.5f\n'], ...
            Q.sensor_inner_median_ms,Q.sensor_inner_max_ms, ...
            Q.memory_inner_median_ms,Q.memory_inner_max_ms);
        fprintf(f,'  disjoint_outer_stage_medians_ms=%s\n', ...
            mat2str(Q.stage_medians_ms,5));
        fprintf(f,'  disjoint_outer_stage_maxima_ms=%s\n', ...
            mat2str(Q.stage_maxima_ms,5));
        fprintf(f,['  unattributed_outer_median=%.5f p99=%.5f max=%.5f; ' ...
            'prediction_update_over_4ms=%d over_5ms=%d\n'], ...
            Q.unattributed_median_ms,Q.unattributed_p99_ms, ...
            Q.unattributed_max_ms,Q.over_4_ms,Q.over_5_ms);
        fprintf(f,'  top columns=%s\n',strjoin(Q.top_columns,','));
        for k=1:min(15,size(Q.top,1))
            fprintf(f,'  %s\n',sprintf('%.5f,',Q.top(k,:)));
        end
    end
    fprintf(f,'k where >=2/3 predicted Flight runs update>4ms=%s\n', ...
        mat2str(R.max_common_over_4_ms_k));
    fprintf(f,'k where >=2/3 any-phase runs update>4ms=%s\n', ...
        mat2str(R.max_common_all_phase_over_4_ms_k));
    fclose(f);
end
end
function out=local_prct(x,p)
% P% empirical order statistic, deterministic for all sample counts.
y=sort(reshape(double(x),1,[]));
n=numel(y);
assert(n>=1&&all(isfinite(y)));
out=y(max(1,ceil((p/100)*n)));
end
