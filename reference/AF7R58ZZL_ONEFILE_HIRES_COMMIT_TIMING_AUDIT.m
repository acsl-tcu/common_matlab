function OUT = AF7R58ZZL_ONEFILE_HIRES_COMMIT_TIMING_AUDIT()
% AF7R58ZZL
% Read-only audit for AF7R58ZZK high-resolution rotating timing.
%
% No tic/toc is used for substage attribution.
% Raw QPC durations are NOT optimistically overhead-subtracted.
% Too-short windows are labeled UNRESOLVED_SHORT_STAGE.

OUT=struct('schema','AF7R58ZZL_HIRES_COMMIT_TIMING_V1', ...
    'production_modified',false,'status','OPEN','error',struct());

fprintf('\n====================================================================================================\n');
fprintf(' AF7R58ZZL | HIGH-RES COMMIT TIMING AUDIT\n');
fprintf('====================================================================================================\n');
fprintf('[AUTHORITY] readOnly=1 | QPC=1 | threadCycles=1 | processCycles=1 | ticTocSubstage=0\n');

try
    assert(exist('af7r58zzk_trace_mex','file')==3, ...
        '[AF7R58ZZL] af7r58zzk_trace_mex is not resolved.');

    D=af7r58zzk_trace_mex('dump');
    E=uint64(D.events);
    fq=double(D.freq);
    assert(fq>0,'[AF7R58ZZL] invalid QPC frequency.');
    OUT.qpc_frequency_hz=fq;
    OUT.raw_events=E;

    % Conservative external boundary overhead: two separate MEX dispatches.
    Ncal=300;
    qms=nan(Ncal,1);
    tcy=nan(Ncal,1);
    pcy=nan(Ncal,1);
    for k=1:Ncal
        a=af7r58zzk_trace_mex('stamp');
        b=af7r58zzk_trace_mex('stamp');
        qms(k)=1e3*double(b(1)-a(1))/double(b(4));
        tcy(k)=double(b(2)-a(2));
        pcy(k)=double(b(3)-a(3));
    end
    qms=sort(qms(isfinite(qms)&qms>=0));
    tcy=tcy(isfinite(tcy)&tcy>=0);
    pcy=pcy(isfinite(pcy)&pcy>=0);

    OUT.overhead=struct( ...
        'pair_min_ms',min(qms), ...
        'pair_median_ms',median(qms), ...
        'pair_p95_ms',pct(qms,95), ...
        'pair_max_ms',max(qms), ...
        'thread_cycle_median',median(tcy), ...
        'process_cycle_median',median(pcy));

    OUT.reliable_threshold_ms=max(5*OUT.overhead.pair_median_ms, ...
                                  3*OUT.overhead.pair_p95_ms);

    fprintf(' QPC freq [Hz]                       = %.0f\n',fq);
    fprintf(' MEX boundary pair min/med/p95/max   = %.6f / %.6f / %.6f / %.6f ms\n', ...
        OUT.overhead.pair_min_ms,OUT.overhead.pair_median_ms, ...
        OUT.overhead.pair_p95_ms,OUT.overhead.pair_max_ms);
    fprintf(' reliable-window threshold            = %.6f ms\n',OUT.reliable_threshold_ms);
    fprintf(' NOTE: raw stage times are NOT overhead-subtracted.\n');

    names={ ...
        'RESOURCE_MODEL', ...
        'REUSE_CACHE_GATE', ...
        'PACKET_REUSE_RETARGET_REBUILD', ...
        'CANONICAL_EVAL_HANDLES', ...
        'CERT_METADATA', ...
        'ROLLBACK_SNAPSHOT', ...
        'ATOMIC_TRANSACTION'};

    rows=struct('seq',{},'window',{},'name',{},'ms',{}, ...
        'thread_cycles',{},'process_cycles',{},'aux',{},'quality',{});

    for w=1:7
        c1=uint64(10*w+1);
        c2=uint64(10*w+2);
        seqs=unique(E(E(:,2)==c1,1));
        for j=1:numel(seqs)
            s=seqs(j);
            i1=find(E(:,1)==s & E(:,2)==c1,1,'first');
            candidates=find(E(:,1)==s & E(:,2)==c2);
            candidates=candidates(candidates>i1);
            if isempty(i1)||isempty(candidates), continue; end
            i2=candidates(1);

            ms=1e3*double(E(i2,3)-E(i1,3))/fq;
            tc=double(E(i2,4)-E(i1,4));
            pc=double(E(i2,5)-E(i1,5));

            if ms>=OUT.reliable_threshold_ms
                q='RELIABLE';
            elseif ms>=2*OUT.overhead.pair_median_ms
                q='MARGINAL';
            else
                q='UNRESOLVED_SHORT_STAGE';
            end

            rows(end+1)=struct( ...
                'seq',double(s),'window',w,'name',names{w}, ...
                'ms',ms,'thread_cycles',tc,'process_cycles',pc, ...
                'aux',double(E(i2,6)),'quality',q); %#ok<AGROW>
        end
    end
    OUT.rows=rows;

    fprintf('\n seq  window                             qpc_ms    threadCycles  processCycles  quality\n');
    for k=1:numel(rows)
        fprintf('%4d  %-32s %9.6f %13.0f %14.0f  %s\n', ...
            rows(k).seq,rows(k).name,rows(k).ms, ...
            rows(k).thread_cycles,rows(k).process_cycles,rows(k).quality);
    end

    OUT.summary=struct();
    fprintf('\n[WINDOW SUMMARY]\n');
    for w=1:7
        ix=find([rows.window]==w);
        vals=[rows(ix).ms];

        S=struct('sample_count',numel(ix),'reliable_count',0, ...
            'min_ms',NaN,'median_ms',NaN,'max_ms',NaN, ...
            'median_thread_cycles',NaN,'quality','NO_COMPLETE_SAMPLE');

        if ~isempty(ix)
            qq={rows(ix).quality};
            S.reliable_count=nnz(strcmp(qq,'RELIABLE'));
            S.min_ms=min(vals);
            S.median_ms=median(vals);
            S.max_ms=max(vals);
            S.median_thread_cycles=median([rows(ix).thread_cycles]);
            if S.reliable_count>0
                S.quality='HAS_RELIABLE_SAMPLE';
            else
                S.quality='SHORT_OR_MARGINAL_ONLY';
            end
        end

        OUT.summary.(names{w})=S;
        fprintf(' %-32s n=%d reliable=%d  min/med/max=%.6f / %.6f / %.6f ms  %s\n', ...
            names{w},S.sample_count,S.reliable_count, ...
            S.min_ms,S.median_ms,S.max_ms,S.quality);
    end

    OUT.runtime_profile_found=false;
    if evalin('base','exist(''AF7R58ZZB_RUNTIME_PROFILE'',''var'')')==1
        P=evalin('base','AF7R58ZZB_RUNTIME_PROFILE');
        if isstruct(P) && isscalar(P) && isfield(P,'phase') && isfield(P,'gen_request_delta')
            mask=(P.phase=='f') & P.gen_request_delta>0;
            OUT.runtime_profile_found=true;
            OUT.runtime=struct( ...
                'request_t_s',double(P.t(mask)), ...
                'hard_ms',double(P.gen_last_cycle_critical_ms(mask)), ...
                'callback_ms',double(P.gen_last_callback_ms(mask)), ...
                'deadline_fail',double(P.gen_deadline_delta(mask)), ...
                'commit',double(P.gen_commit_delta(mask)));
        elseif istable(P)
            mask=(P.phase=='f') & P.gen_request_delta>0;
            OUT.runtime_profile_found=true;
            OUT.runtime=struct( ...
                'request_t_s',double(P.t(mask)), ...
                'hard_ms',double(P.gen_last_cycle_critical_ms(mask)), ...
                'callback_ms',double(P.gen_last_callback_ms(mask)), ...
                'deadline_fail',double(P.gen_deadline_delta(mask)), ...
                'commit',double(P.gen_commit_delta(mask)));
        end
    end

    med=nan(1,7);
    for w=1:7
        ix=find([rows.window]==w & strcmp({rows.quality},'RELIABLE'));
        if ~isempty(ix)
            med(w)=median([rows(ix).ms]);
        end
    end
    OUT.reliable_window_medians_ms=med;

    finite=find(isfinite(med));
    if isempty(finite)
        OUT.dominant_window='';
        OUT.status='NO_RELIABLE_WINDOW__INSTRUMENT_RESOLUTION_TOO_COARSE_OR_PATH_NOT_REACHED';
    else
        [mx,j]=max(med(finite));
        iw=finite(j);
        OUT.dominant_window=names{iw};
        OUT.dominant_median_ms=mx;
        OUT.status='HIRES_WINDOW_ATTRIBUTION_AVAILABLE__REMOVE_INSTRUMENTATION_BEFORE_FINAL_TIMING';
    end

    fprintf('\n dominant reliable window = %s\n',OUT.dominant_window);
    if isfield(OUT,'dominant_median_ms')
        fprintf(' dominant reliable median = %.6f ms\n',OUT.dominant_median_ms);
    end
    fprintf(' status = %s\n',OUT.status);
    fprintf(' IMPORTANT: instrumentation must be removed before final <=4 ms timing claims.\n');

catch ME
    OUT.status='AUDIT_ERROR__NO_PRODUCTION_CHANGE';
    OUT.error=struct('identifier',ME.identifier,'message',ME.message,'stack',{ME.stack});
    fprintf('[AUDIT ERROR] %s | %s\n',ME.identifier,ME.message);
end

root=fileparts(which('REPLANNING_SYSTEM'));
if isempty(root), root=pwd; end
outdir=tempname(root); [ok,msg]=mkdir(outdir); assert(ok,'%s',msg);
report=fullfile(outdir,'AF7R58ZZL_REPORT.txt');
matp=fullfile(outdir,'AF7R58ZZL_OUT.mat');

fid=fopen(report,'wt','n','UTF-8'); assert(fid>=0);
fprintf(fid,'status=%s\n',OUT.status);
if isfield(OUT,'qpc_frequency_hz')
    fprintf(fid,'qpcFreqHz=%.0f\n',OUT.qpc_frequency_hz);
end
if isfield(OUT,'overhead')
    fprintf(fid,'pairMedianMs=%.9g\n',OUT.overhead.pair_median_ms);
    fprintf(fid,'pairP95Ms=%.9g\n',OUT.overhead.pair_p95_ms);
    fprintf(fid,'reliableThresholdMs=%.9g\n',OUT.reliable_threshold_ms);
end
if isfield(OUT,'dominant_window')
    fprintf(fid,'dominantWindow=%s\n',OUT.dominant_window);
end
if isfield(OUT,'dominant_median_ms')
    fprintf(fid,'dominantMedianMs=%.9g\n',OUT.dominant_median_ms);
end
if isfield(OUT,'summary')
    fn=fieldnames(OUT.summary);
    for k=1:numel(fn)
        S=OUT.summary.(fn{k});
        fprintf(fid,'%s n=%d reliable=%d min=%.9g median=%.9g max=%.9g quality=%s\n', ...
            fn{k},S.sample_count,S.reliable_count, ...
            S.min_ms,S.median_ms,S.max_ms,S.quality);
    end
end
if isfield(OUT,'error') && isstruct(OUT.error) && isfield(OUT.error,'message')
    fprintf(fid,'errorIdentifier=%s\n',OUT.error.identifier);
    fprintf(fid,'errorMessage=%s\n',OUT.error.message);
end
fclose(fid);

OUT.report_path=report;
OUT.mat_path=matp;
save(matp,'OUT','-v7');
assignin('base','AF7R58ZZL_OUT',OUT);
fprintf('RETURN: %s\nRETURN: %s\n',report,matp);
end

function y=pct(x,p)
x=sort(x(:));
n=numel(x);
if n==0, y=NaN; return; end
r=1+(n-1)*p/100;
i=floor(r);
j=ceil(r);
if i==j
    y=x(i);
else
    y=x(i)+(r-i)*(x(j)-x(i));
end
end
