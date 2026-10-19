function OUT = AF7R58ZZA_ONEFILE_POST_ZZ_TIMING_AUDIT()
% Read-only timing audit after AF7R58ZZ. Controller excluded.
EXPECTED_SHA='59e24920394b53faf2d832d6b84dead3e94e9728ea767cf7dca1ef3a1a5f19fb';
fprintf('\n====================================================================================================\n');
fprintf(' AF7R58ZZA | POST-ZZ COMMIT HOT-PATH TIMING AUDIT\n');
fprintf('====================================================================================================\n');
OUT=struct('schema','AF7R58ZZA_V1','production_modified',false,'status','OPEN','error',struct());
try
 pR=which('REPLANNING_SYSTEM'); assert(~isempty(pR)&&isfile(pR));
 OUT.source_sha=file_hash(pR); OUT.source_exact=strcmp(OUT.source_sha,EXPECTED_SHA);
 assert(evalin('base','exist(''replanner_instance'',''var'')')==1,'replanner_instance missing');
 R=evalin('base','replanner_instance');
 OUT.requests=double(R.generator_request_count); OUT.deadline_failures=double(R.generator_deadline_fail_count);
 OUT.commits=double(R.replan_commit_count); OUT.cache_hits=double(R.reference_only_prepared_packet_cache_hit_count);
 OUT.cache_retargets=double(R.reference_only_prepared_packet_cache_retarget_count);
 OUT.cache_invalidates=double(R.reference_only_prepared_packet_cache_invalidate_count);
 profile='';
 if evalin('base','exist(''AF7R58ZH_R4_RUNTIME_PROFILE'',''var'')')==1, profile='AF7R58ZH_R4_RUNTIME_PROFILE';
 elseif evalin('base','exist(''AF7R58ZX_RUNTIME_PROFILE'',''var'')')==1, profile='AF7R58ZX_RUNTIME_PROFILE'; end
 assert(~isempty(profile),'no supported runtime profile found'); P=evalin('base',profile);
 flight=(P.phase=='f')&isfinite(P.reference_s); req=flight&P.gen_request_delta>0; idx=find(req); n=numel(idx);
 OUT.t=zeros(1,n);OUT.ref=zeros(1,n);OUT.hard=zeros(1,n);OUT.cb=zeros(1,n);OUT.post=nan(1,n);OUT.fail=zeros(1,n);OUT.commit=zeros(1,n);
 for k=1:n
  j=idx(k);OUT.t(k)=double(P.t(j));OUT.ref(k)=1e3*double(P.reference_s(j));OUT.hard(k)=double(P.gen_last_cycle_critical_ms(j));
  OUT.cb(k)=double(P.gen_last_callback_ms(j));OUT.fail(k)=double(P.gen_deadline_delta(j));OUT.commit(k)=double(P.gen_commit_delta(j));
  if isfield(P,'gen_elapsed_before_ms')&&isfield(P,'gen_context_ms')
   OUT.post(k)=OUT.hard(k)-double(P.gen_elapsed_before_ms(j))-double(P.gen_context_ms(j))-OUT.cb(k);
  end
 end
 OUT.hard_max=maxf(OUT.hard);OUT.hard_over4=nnz(OUT.hard>4);OUT.post_max=maxf(OUT.post);
 OUT.ref_max=1e3*max(P.reference_s(flight));OUT.nonreq_ref_max=1e3*max(P.reference_s(flight&~req));OUT.ref_over25=nnz(P.reference_s(flight)>0.025);
 fprintf('\n[A. LIVE]\n source exact=%d | %s\n requests/fails/commits=%.0f/%.0f/%.0f\n cache hit/retarget/invalidate=%.0f/%.0f/%.0f\n',OUT.source_exact,OUT.source_sha,OUT.requests,OUT.deadline_failures,OUT.commits,OUT.cache_hits,OUT.cache_retargets,OUT.cache_invalidates);
 fprintf('\n[B. REQUESTS]\n t[s] ref[ms] hard[ms] cb[ms] postcb[ms] fail commit\n');
 for k=1:n,fprintf('%6.3f %7.3f %8.3f %7.3f %10.3f %4.0f %6.0f\n',OUT.t(k),OUT.ref(k),OUT.hard(k),OUT.cb(k),OUT.post(k),OUT.fail(k),OUT.commit(k));end
 fprintf('\n[C. SUMMARY]\n hard max=%.6f ms | >4ms=%d/%d\n postcallback max=%.6f ms\n ref max=%.6f ms | nonrequest max=%.6f ms | ref>25=%d\n',OUT.hard_max,OUT.hard_over4,n,OUT.post_max,OUT.ref_max,OUT.nonreq_ref_max,OUT.ref_over25);
 if ~OUT.source_exact,OUT.status='SOURCE_NOT_EXACT_ZZ';
 elseif OUT.deadline_failures==0&&OUT.hard_over4==0&&OUT.ref_over25==0,OUT.status='ZZ_4MS_AND_25MS_EMPIRICALLY_CLOSED_THIS_RUN';
 elseif OUT.deadline_failures==0&&OUT.hard_over4==0,OUT.status='ZZ_4MS_CLOSED__REFERENCE_25MS_REMAINS';
 else,OUT.status='ZZ_4MS_STILL_OPEN';end
 fprintf(' status=%s\n empirical timing is not formal OS/WCET theorem.\n',OUT.status);
catch ME
 OUT.status='AUDIT_ERROR__NO_PRODUCTION_CHANGE';OUT.error=struct('identifier',ME.identifier,'message',ME.message,'stack',{ME.stack});fprintf('[AUDIT ERROR] %s | %s\n',ME.identifier,ME.message);
end
root=fileparts(which('REPLANNING_SYSTEM'));outdir=tempname(root);[ok,msg]=mkdir(outdir);assert(ok,'%s',msg);
report=fullfile(outdir,'AF7R58ZZA_REPORT.txt');matp=fullfile(outdir,'AF7R58ZZA_OUT.mat');fid=fopen(report,'wt','n','UTF-8');assert(fid>=0);
fprintf(fid,'status=%s\n',OUT.status);if isfield(OUT,'source_sha'),fprintf(fid,'source SHA=%s\n',OUT.source_sha);end
if isfield(OUT,'hard_max'),fprintf(fid,'requests/fails/commits=%.0f/%.0f/%.0f\nhardMaxMs=%.17g\nhardOver4=%d\npostCallbackMaxMs=%.17g\nrefMaxMs=%.17g\nrefOver25=%d\n',OUT.requests,OUT.deadline_failures,OUT.commits,OUT.hard_max,OUT.hard_over4,OUT.post_max,OUT.ref_max,OUT.ref_over25);end
fclose(fid);OUT.report_path=report;OUT.mat_path=matp;save(matp,'OUT','-v7');assignin('base','AF7R58ZZA_OUT',OUT);fprintf('RETURN: %s\nRETURN: %s\n',report,matp);
end
function y=maxf(x),x=x(isfinite(x));if isempty(x),y=NaN;else,y=max(x);end,end
function h=file_hash(p),fid=fopen(p,'rb');assert(fid>=0);c=onCleanup(@()fclose(fid));b=fread(fid,Inf,'*uint8');m=java.security.MessageDigest.getInstance('SHA-256');m.update(b);d=typecast(m.digest(),'uint8');h=lower(reshape(dec2hex(d,2).',1,[]));end
