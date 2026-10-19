function OUT = AF7R58ZW_ONEFILE_NATIVE_STAGE_POSTRUN_AUDIT()
% AF7R58ZW
% Read-only post-run audit of the ACTUAL live generator:
%   @(CTX)obj.af7r46_single_core_proposal_compact(CTX)
%     -> af7r34_make_core_packet
%     -> af7r36_single_core_mex
%
% Requires a completed:
%   run_flight_headless_AF7R58ZV_native_stage_audit
%
% No controller timing is used for classification.
% No production source is modified.
% Current native .hpp/.cpp/.m sources are copied into a return ZIP so the
% next optimization can be exact-source-bound rather than guessed.

fprintf('\n====================================================================================================\n');
fprintf(' AF7R58ZW | ACTUAL AF7R36 NATIVE-STAGE POST-RUN AUDIT\n');
fprintf('====================================================================================================\n');
fprintf('[AUTHORITY] readOnly=1 | productionModified=0 | controllerExcluded=1 | liveMexPath=1\n');

OUT=struct();
OUT.schema='AF7R58ZW_NATIVE_STAGE_POSTRUN_V1';
OUT.production_modified=false;
OUT.status='OPEN';
OUT.error=struct();

try
    assert(evalin('base','exist(''AF7R58ZV_RUNTIME_PROFILE'',''var'')')==1, ...
        '[AF7R58ZW] AF7R58ZV_RUNTIME_PROFILE missing.');
    assert(evalin('base','exist(''AF7R58ZV_REPLANNER'',''var'')')==1, ...
        '[AF7R58ZW] AF7R58ZV_REPLANNER missing.');

    P=evalin('base','AF7R58ZV_RUNTIME_PROFILE');
    R=evalin('base','AF7R58ZV_REPLANNER');

    reqFields={'t','phase','reference_s','gen_request_delta', ...
        'gen_deadline_delta','gen_commit_delta', ...
        'gen_last_callback_ms','gen_last_cycle_critical_ms', ...
        'r9_episode_ms','r9_synthesis_ms','r9_candidate_ms','r9_total_ms'};
    for k=1:numel(reqFields)
        assert(isfield(P,reqFields{k}),'[AF7R58ZW] profile missing %s.',reqFields{k});
    end

    % Exact live callback identity.
    fh=R.generator_backend_locked_callback;
    assert(isa(fh,'function_handle'),'[AF7R58ZW] live callback missing.');
    info=functions(fh);
    OUT.live_callback_name=func2str(fh);
    if isfield(info,'file'), OUT.live_callback_file=info.file; else, OUT.live_callback_file=''; end

    fprintf('\n[A. LIVE GENERATOR IDENTITY]\n');
    fprintf(' callback = %s\n',OUT.live_callback_name);
    fprintf(' file     = %s\n',OUT.live_callback_file);
    fprintf(' expected architecture = internal af7r46 compact -> AF7R36 MEX\n');

    flight=(P.phase=='f') & isfinite(P.reference_s);
    request=flight & (P.gen_request_delta>0);
    nonrequest=flight & ~request;
    ridx=find(request);

    n=numel(ridx);
    OUT.request_count=n;
    OUT.request=struct();
    names={'t_s','reference_ms','hard_ms','callback_ms','native_total_ms', ...
           'episode_ms','synthesis_ms','candidate_ms','callback_minus_native_ms', ...
           'hard_minus_callback_ms','deadline_fail','commit'};
    for k=1:numel(names), OUT.request.(names{k})=nan(1,n); end

    fprintf('\n[B. REQUEST-BY-REQUEST NATIVE BREAKDOWN]\n');
    fprintf(' t[s]   ref    hard     cb   native  episode synthesis candidate  cb-native hard-cb fail commit\n');

    for k=1:n
        j=ridx(k);
        t=double(P.t(j));
        ref=1e3*double(P.reference_s(j));
        hard=double(P.gen_last_cycle_critical_ms(j));
        cb=double(P.gen_last_callback_ms(j));
        ep=double(P.r9_episode_ms(j));
        sy=double(P.r9_synthesis_ms(j));
        ca=double(P.r9_candidate_ms(j));
        nt=double(P.r9_total_ms(j));
        cbn=cb-nt;
        hcb=hard-cb;

        OUT.request.t_s(k)=t;
        OUT.request.reference_ms(k)=ref;
        OUT.request.hard_ms(k)=hard;
        OUT.request.callback_ms(k)=cb;
        OUT.request.native_total_ms(k)=nt;
        OUT.request.episode_ms(k)=ep;
        OUT.request.synthesis_ms(k)=sy;
        OUT.request.candidate_ms(k)=ca;
        OUT.request.callback_minus_native_ms(k)=cbn;
        OUT.request.hard_minus_callback_ms(k)=hcb;
        OUT.request.deadline_fail(k)=double(P.gen_deadline_delta(j));
        OUT.request.commit(k)=double(P.gen_commit_delta(j));

        fprintf('%6.3f %6.2f %7.2f %6.2f %8.2f %8.2f %9.2f %9.2f %10.2f %7.2f %4.0f %6.0f\n', ...
            t,ref,hard,cb,nt,ep,sy,ca,cbn,hcb, ...
            OUT.request.deadline_fail(k),OUT.request.commit(k));
    end

    OUT.reference_max_ms=1e3*max(P.reference_s(flight));
    OUT.nonrequest_reference_max_ms=1e3*max(P.reference_s(nonrequest));
    OUT.reference_over_25ms=nnz(P.reference_s(flight)>0.025);

    finiteHard=OUT.request.hard_ms(isfinite(OUT.request.hard_ms));
    finiteNative=OUT.request.native_total_ms(isfinite(OUT.request.native_total_ms));
    OUT.hard_max_ms=max_or_nan(finiteHard);
    OUT.native_max_ms=max_or_nan(finiteNative);
    OUT.hard_over_4ms=nnz(OUT.request.hard_ms>4);
    OUT.deadline_failures=sum(OUT.request.deadline_fail);

    % Dominance based only on requests with all three native coarse stages.
    dom={'EPISODE','SYNTHESIS','CANDIDATE'};
    counts=zeros(1,3);
    for k=1:n
        v=[OUT.request.episode_ms(k),OUT.request.synthesis_ms(k),OUT.request.candidate_ms(k)];
        if all(isfinite(v))
            [~,ii]=max(v);
            counts(ii)=counts(ii)+1;
        end
    end
    OUT.native_dominant_labels=dom;
    OUT.native_dominant_counts=counts;

    fprintf('\n[C. SUMMARY]\n');
    fprintf(' request count                    = %d\n',n);
    fprintf(' deadline failures                = %.0f\n',OUT.deadline_failures);
    fprintf(' hard-cycle >4 ms                 = %d / %d\n',OUT.hard_over_4ms,n);
    fprintf(' max hard-cycle [ms]              = %.6f\n',OUT.hard_max_ms);
    fprintf(' max native AF7R36 total [ms]     = %.6f\n',OUT.native_max_ms);
    fprintf(' max reference stage [ms]         = %.6f\n',OUT.reference_max_ms);
    fprintf(' max nonrequest reference [ms]    = %.6f\n',OUT.nonrequest_reference_max_ms);
    fprintf(' reference >25 ms                 = %d\n',OUT.reference_over_25ms);
    fprintf(' dominant native stage counts [episode synthesis candidate] = %s\n',mat2str(counts));

    % ---------------------------------------------------------------------
    % Exact current native-source capture for the next optimization.
    % ---------------------------------------------------------------------
    pR=which('REPLANNING_SYSTEM');
    assert(~isempty(pR)&&isfile(pR),'[AF7R58ZW] REPLANNING_SYSTEM source unresolved.');
    root=fileparts(pR);

    srcNames={ ...
        'REPLANNING_SYSTEM.m', ...
        'af7r36_core.hpp', ...
        'af7r36_single_core_mex.cpp', ...
        'af7r34_make_core_packet.m', ...
        'af7r36_runtime_prime.m', ...
        'af7r58i_resource_contract.m', ...
        'af7r58i_prepare_batch_model.m'};

    stamp=datestr(now,'yyyymmdd_HHMMSS');
    outdir=fullfile(root,['AF7R58ZW_' stamp]);
    [ok,msg]=mkdir(outdir); assert(ok,'%s',msg);

    sourceRows=struct('name',{},'path',{},'sha',{},'present',{});
    fprintf('\n[D. EXACT CURRENT SOURCE CAPTURE]\n');
    for k=1:numel(srcNames)
        p=fullfile(root,srcNames{k});
        if ~isfile(p)
            q=which(erase(srcNames{k},'.m'));
            if ~isempty(q)&&isfile(q), p=q; end
        end
        present=isfile(p);
        sha='';
        if present
            sha=file_hash(p);
            copyfile(p,fullfile(outdir,srcNames{k}));
        end
        sourceRows(end+1)=struct('name',srcNames{k},'path',p,'sha',sha,'present',present); %#ok<AGROW>
        fprintf(' %-36s present=%d | %s\n',srcNames{k},present,sha);
    end
    OUT.sources=sourceRows;

    report=fullfile(outdir,'AF7R58ZW_REPORT.txt');
    matp=fullfile(outdir,'AF7R58ZW_OUT.mat');

    fid=fopen(report,'wt','n','UTF-8'); assert(fid>=0);
    fprintf(fid,'AF7R58ZW | ACTUAL AF7R36 NATIVE-STAGE POST-RUN AUDIT\n');
    fprintf(fid,'productionModified=0\n');
    fprintf(fid,'liveCallback=%s\n',OUT.live_callback_name);
    fprintf(fid,'requestCount=%d\n',n);
    fprintf(fid,'deadlineFailures=%.0f\n',OUT.deadline_failures);
    fprintf(fid,'hardOver4ms=%d\n',OUT.hard_over_4ms);
    fprintf(fid,'hardMaxMs=%.17g\n',OUT.hard_max_ms);
    fprintf(fid,'nativeMaxMs=%.17g\n',OUT.native_max_ms);
    fprintf(fid,'referenceMaxMs=%.17g\n',OUT.reference_max_ms);
    fprintf(fid,'nonrequestReferenceMaxMs=%.17g\n',OUT.nonrequest_reference_max_ms);
    fprintf(fid,'referenceOver25ms=%d\n',OUT.reference_over_25ms);
    fprintf(fid,'dominantCountsEpisodeSynthesisCandidate=%s\n',mat2str(counts));
    fprintf(fid,'\nREQUEST TABLE\n');
    fprintf(fid,'t_s,reference_ms,hard_ms,callback_ms,native_total_ms,episode_ms,synthesis_ms,candidate_ms,callback_minus_native_ms,hard_minus_callback_ms,deadline_fail,commit\n');
    for k=1:n
        fprintf(fid,'%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.0f,%.0f\n', ...
            OUT.request.t_s(k),OUT.request.reference_ms(k),OUT.request.hard_ms(k), ...
            OUT.request.callback_ms(k),OUT.request.native_total_ms(k), ...
            OUT.request.episode_ms(k),OUT.request.synthesis_ms(k),OUT.request.candidate_ms(k), ...
            OUT.request.callback_minus_native_ms(k),OUT.request.hard_minus_callback_ms(k), ...
            OUT.request.deadline_fail(k),OUT.request.commit(k));
    end
    fprintf(fid,'\nSOURCE HASHES\n');
    for k=1:numel(sourceRows)
        fprintf(fid,'%s | present=%d | SHA=%s | %s\n', ...
            sourceRows(k).name,sourceRows(k).present,sourceRows(k).sha,sourceRows(k).path);
    end
    fclose(fid);

    OUT.report_path=report;
    OUT.mat_path=matp;
    save(matp,'OUT','-v7');

    zipPath=fullfile(root,'AF7R58ZW_NATIVE_STAGE_AND_SOURCE_BUNDLE.zip');
    if isfile(zipPath), delete(zipPath); end
    zip(zipPath,outdir);
    OUT.zip_path=zipPath;
    OUT.status='ACTUAL_NATIVE_STAGE_CAPTURED__SEND_ZIP__NO_PRODUCTION_CHANGE';
    save(matp,'OUT','-v7');
    assignin('base','AF7R58ZW_OUT',OUT);

    fprintf('\n[E. RESULT]\n');
    fprintf(' production modified = 0\n');
    fprintf(' REPORT = %s\n',report);
    fprintf(' ZIP    = %s\n',zipPath);
    fprintf(' status = %s\n',OUT.status);

catch ME
    OUT.status='AUDIT_ERROR__NO_PRODUCTION_CHANGE';
    OUT.error=struct('identifier',ME.identifier,'message',ME.message,'stack',{ME.stack});
    assignin('base','AF7R58ZW_OUT',OUT);
    fprintf('\n[AUDIT ERROR] %s | %s\n',ME.identifier,ME.message);
    fprintf(' production modified = 0\n');
end
end


function y=max_or_nan(x)
if isempty(x), y=NaN; else, y=max(x); end
end


function h=file_hash(p)
fid=fopen(p,'rb'); assert(fid>=0);
c=onCleanup(@()fclose(fid)); %#ok<NASGU>
b=fread(fid,Inf,'*uint8');
m=java.security.MessageDigest.getInstance('SHA-256');
m.update(b);
d=typecast(m.digest(),'uint8');
h=lower(reshape(dec2hex(d,2).',1,[]));
end
