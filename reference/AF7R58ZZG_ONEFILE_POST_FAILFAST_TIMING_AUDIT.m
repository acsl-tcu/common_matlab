function OUT = AF7R58ZZG_ONEFILE_POST_FAILFAST_TIMING_AUDIT()
% AF7R58ZZG - read-only post-run timing audit after ZZF.
EXPECTED_SHA='c2e5f087a4ce9294057243624effab0d9eec9b2d9996b7cab2f18770c09a7f6c';
fprintf('\n====================================================================================================\n');
fprintf(' AF7R58ZZG | POST-FAILFAST TIMING AUDIT\n');
fprintf('====================================================================================================\n');
fprintf('[AUTHORITY] readOnly=1 | productionModified=0 | controllerExcluded=1\n');
OUT=struct('schema','AF7R58ZZG_V1','production_modified',false,'status','OPEN','error',struct());
try
 pR=which('REPLANNING_SYSTEM'); assert(~isempty(pR)&&isfile(pR));
 OUT.source_sha=file_hash(pR); OUT.source_exact=strcmp(OUT.source_sha,EXPECTED_SHA);
 assert(evalin('base','exist(''AF7R58ZZB_RUNTIME_PROFILE'',''var'')')==1, ...
   '[AF7R58ZZG] run run_flight_headless_AF7R58ZZB_reference_cycle_audit first.');
 P=evalin('base','AF7R58ZZB_RUNTIME_PROFILE');
 flight=(P.phase=='f') & isfinite(P.reference_s);
 req=flight & P.gen_request_delta>0; idx=find(req); n=numel(idx);
 OUT.t=double(P.t(idx));
 OUT.hard_ms=double(P.gen_last_cycle_critical_ms(idx));
 OUT.callback_ms=double(P.gen_last_callback_ms(idx));
 OUT.fail=double(P.gen_deadline_delta(idx));
 OUT.commit=double(P.gen_commit_delta(idx));
 OUT.reference_ms=1e3*double(P.reference_s(idx));
 OUT.hard_max_ms=max(OUT.hard_ms);
 OUT.hard_over_4=nnz(OUT.hard_ms>4);
 OUT.fail_count=sum(OUT.fail);
 OUT.commit_count=sum(OUT.commit);
 OUT.reference_max_ms=max(OUT.reference_ms);
 fprintf(' source exact = %d | %s\n',OUT.source_exact,OUT.source_sha);
 fprintf(' requests / fails / commits = %d / %.0f / %.0f\n',n,OUT.fail_count,OUT.commit_count);
 fprintf(' max hard-cycle [ms] = %.6f\n',OUT.hard_max_ms);
 fprintf(' hard >4 ms = %d / %d\n',OUT.hard_over_4,n);
 fprintf(' max reference [ms] = %.6f\n',OUT.reference_max_ms);
 fprintf('\n t[s] hard callback ref fail commit\n');
 for k=1:n
   fprintf('%6.3f %7.3f %8.3f %7.3f %4.0f %6.0f\n',OUT.t(k),OUT.hard_ms(k),OUT.callback_ms(k),OUT.reference_ms(k),OUT.fail(k),OUT.commit(k));
 end
 if ~OUT.source_exact
   OUT.status='SOURCE_NOT_EXACT_ZZF';
 elseif OUT.fail_count==0 && OUT.hard_over_4==0
   OUT.status='ZZF_4MS_EMPIRICALLY_CLOSED_THIS_RUN';
 else
   OUT.status='ZZF_FAILFAST_ACTIVE__4MS_STILL_OPEN';
 end
 fprintf('status = %s\n',OUT.status);
catch ME
 OUT.status='AUDIT_ERROR__NO_PRODUCTION_CHANGE';
 OUT.error=struct('identifier',ME.identifier,'message',ME.message,'stack',{ME.stack});
 fprintf('[AUDIT ERROR] %s | %s\n',ME.identifier,ME.message);
end
root=fileparts(which('REPLANNING_SYSTEM')); outdir=tempname(root); [ok,msg]=mkdir(outdir); assert(ok,'%s',msg);
report=fullfile(outdir,'AF7R58ZZG_REPORT.txt'); matp=fullfile(outdir,'AF7R58ZZG_OUT.mat');
fid=fopen(report,'wt','n','UTF-8'); assert(fid>=0); fprintf(fid,'status=%s\n',OUT.status);
if isfield(OUT,'hard_max_ms'), fprintf(fid,'hardMaxMs=%.17g\nhardOver4=%d\nfails=%.0f\ncommits=%.0f\nreferenceMaxMs=%.17g\n',OUT.hard_max_ms,OUT.hard_over_4,OUT.fail_count,OUT.commit_count,OUT.reference_max_ms); end
fclose(fid); OUT.report_path=report; OUT.mat_path=matp; save(matp,'OUT','-v7'); assignin('base','AF7R58ZZG_OUT',OUT);
fprintf('RETURN: %s\nRETURN: %s\n',report,matp);
end
function h=file_hash(p)
fid=fopen(p,'rb');assert(fid>=0);c=onCleanup(@()fclose(fid)); %#ok<NASGU>
b=fread(fid,Inf,'*uint8');m=java.security.MessageDigest.getInstance('SHA-256');m.update(b);d=typecast(m.digest(),'uint8');h=lower(reshape(dec2hex(d,2).',1,[]));
end
