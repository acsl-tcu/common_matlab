function OUT = AF7R58ZZJ_ONEFILE_REQUEST_REASON_AUDIT()
% AF7R58ZZJ
% Read-only request-by-request reason audit after AF7R58ZZH/ZZI.
%
% Purpose:
%   Determine whether a mathematically/validator-accepted proposal existed
%   BEFORE the eventual late commit, or whether the first validation pass
%   itself occurred only on the late retry.
%
% Safety / sensing:
%   - NO production modification.
%   - NO synthetic obstacle.
%   - NO obstacle-map injection.
%   - NO commit.
%   - Uses only the already-recorded AF7R58ZZB runtime profile and live
%     replanner counters.
%
% This audit intentionally does NOT interpret a late retry as satisfying the
% first-detection -> target-modification deadline.

EXPECTED_SHA='2a0013a583cbcff3afd62e4495fa5e784427132390616ec5886000f134f2c05f';

OUT=struct('schema','AF7R58ZZJ_REQUEST_REASON_V1', ...
    'production_modified',false,'status','OPEN','error',struct());

fprintf('\n====================================================================================================\n');
fprintf(' AF7R58ZZJ | REQUEST-BY-REQUEST VALIDITY-vs-DEADLINE AUDIT\n');
fprintf('====================================================================================================\n');
fprintf('[AUTHORITY] readOnly=1 | productionModified=0 | noSyntheticObstacle=1 | noCommit=1\n');

try
    pR=which('REPLANNING_SYSTEM');
    assert(~isempty(pR)&&isfile(pR),'[AF7R58ZZJ] REPLANNING_SYSTEM unresolved.');
    OUT.source_sha=file_hash(pR);
    OUT.source_exact=strcmp(OUT.source_sha,EXPECTED_SHA);

    R=read_utf8(pR);
    OUT.sensor_contract=struct( ...
        'generator_detected_only',contains(R,'The generator receives ONLY sensor-known obstacles:'), ...
        'initially_unknown',contains(R,'Initial known set is empty.'), ...
        'raw_sensor_only_admission',contains(R,'raw_sensor_gate alone may admit a previously unknown obstacle.'));
    OUT.sensor_contract.closed=OUT.sensor_contract.generator_detected_only && ...
        OUT.sensor_contract.initially_unknown && OUT.sensor_contract.raw_sensor_only_admission;

    assert(evalin('base','exist(''AF7R58ZZB_RUNTIME_PROFILE'',''var'')')==1, ...
        '[AF7R58ZZJ] AF7R58ZZB_RUNTIME_PROFILE is absent. Re-run the SAME finite-sensing scenario first.');
    P=evalin('base','AF7R58ZZB_RUNTIME_PROFILE');
    assert(istable(P),'[AF7R58ZZJ] AF7R58ZZB_RUNTIME_PROFILE must be a table.');

    vn=string(P.Properties.VariableNames);
    OUT.profile_variables=cellstr(vn);

    reqMask=(P.phase=='f') & isfinite(P.reference_s) & P.gen_request_delta>0;
    idx=find(reqMask);
    OUT.request_index=idx(:);
    OUT.request_count=numel(idx);
    OUT.t_s=double(P.t(idx));
    OUT.hard_ms=double(P.gen_last_cycle_critical_ms(idx));
    OUT.callback_ms=double(P.gen_last_callback_ms(idx));
    OUT.reference_ms=1e3*double(P.reference_s(idx));
    OUT.deadline_fail=double(P.gen_deadline_delta(idx));
    OUT.commit=double(P.gen_commit_delta(idx));

    [OUT.status_code,OUT.bindings.status_code]= ...
        get_first_numeric(P,idx,vn,["gen_last_status_code","generator_last_status_code","gen_status_code"]);
    [OUT.candidate_delta,OUT.bindings.candidate_delta]= ...
        get_first_numeric(P,idx,vn,["gen_candidate_delta","generator_candidate_delta","candidate_delta"]);
    [OUT.validation_pass_delta,OUT.bindings.validation_pass_delta]= ...
        get_first_numeric(P,idx,vn,["gen_validation_pass_delta","generator_validation_pass_delta","validation_pass_delta"]);
    [OUT.validation_fail_delta,OUT.bindings.validation_fail_delta]= ...
        get_first_numeric(P,idx,vn,["gen_validation_fail_delta","generator_validation_fail_delta","validation_fail_delta"]);
    [OUT.proposal_delta,OUT.bindings.proposal_delta]= ...
        get_first_numeric(P,idx,vn,["compact_native_proposal_delta","gen_proposal_delta","generator_proposal_delta","proposal_delta"]);
    [OUT.reference_eval_delta,OUT.bindings.reference_eval_delta]= ...
        get_first_numeric(P,idx,vn,["reference_only_evaluation_delta","reference_only_eval_delta","refonly_eval_delta"]);
    [OUT.cache_hit_delta,OUT.bindings.cache_hit_delta]= ...
        get_first_numeric(P,idx,vn,["reference_only_prepared_packet_cache_hit_delta","prepared_cache_hit_delta","cache_hit_delta"]);
    [OUT.cache_store_delta,OUT.bindings.cache_store_delta]= ...
        get_first_numeric(P,idx,vn,["reference_only_prepared_packet_cache_store_delta","prepared_cache_store_delta","cache_store_delta"]);
    [OUT.cache_retarget_delta,OUT.bindings.cache_retarget_delta]= ...
        get_first_numeric(P,idx,vn,["reference_only_prepared_packet_cache_retarget_delta","prepared_cache_retarget_delta","cache_retarget_delta"]);
    [OUT.packet_rebuild_delta,OUT.bindings.packet_rebuild_delta]= ...
        get_first_numeric(P,idx,vn,["reference_only_runtime_packet_rebuild_delta","runtime_packet_rebuild_delta","packet_rebuild_delta"]);

    OUT.first_commit_request=first_positive(OUT.commit);
    OUT.first_deadline_fail_request=first_positive(OUT.deadline_fail);
    OUT.first_validation_pass_request=first_positive(OUT.validation_pass_delta);
    OUT.first_candidate_request=first_positive(OUT.candidate_delta);
    OUT.first_proposal_request=first_positive(OUT.proposal_delta);
    OUT.first_reference_eval_request=first_positive(OUT.reference_eval_delta);

    % Live final counters, if the object still exists.
    OUT.object_found=false;
    OUT.object=struct();
    if evalin('base','exist(''replanner_instance'',''var'')')==1
        obj=evalin('base','replanner_instance');
        if isa(obj,'REPLANNING_SYSTEM')
            OUT.object_found=true;
            names={ ...
                'generator_request_count', ...
                'generator_candidate_count', ...
                'generator_validation_pass_count', ...
                'generator_validation_fail_count', ...
                'generator_deadline_fail_count', ...
                'generator_last_status_code', ...
                'compact_native_proposal_count', ...
                'reference_only_evaluation_count', ...
                'reference_only_commit_count', ...
                'replan_commit_count', ...
                'reference_only_prepared_packet_cache_store_count', ...
                'reference_only_prepared_packet_cache_hit_count', ...
                'reference_only_prepared_packet_cache_retarget_count', ...
                'reference_only_prepared_packet_cache_commit_count'};
            for k=1:numel(names)
                nm=names{k};
                OUT.object.(nm)=get_prop_num(obj,nm);
            end
            if isprop(obj,'active_reference_mode')
                OUT.object.active_reference_mode=char(string(obj.active_reference_mode));
            end
            if isprop(obj,'active_reference_version')
                OUT.object.active_reference_version=double(obj.active_reference_version);
            end
        end
    end

    passLogged=~all(isnan(OUT.validation_pass_delta));
    failLogged=~all(isnan(OUT.validation_fail_delta));

    if ~OUT.source_exact
        OUT.status='SOURCE_NOT_EXACT_ZZH';
    elseif ~OUT.sensor_contract.closed
        OUT.status='FINITE_SENSING_CONTRACT_NOT_CLOSED';
    elseif OUT.request_count==0
        OUT.status='NO_REAL_REPLAN_REQUEST';
    elseif passLogged && isfinite(OUT.first_validation_pass_request) && ...
            isfinite(OUT.first_commit_request) && ...
            OUT.first_validation_pass_request < OUT.first_commit_request
        OUT.status='VALID_PROPOSAL_PRECEDES_COMMIT__COMMIT_PATH_BLOCKING_CONFIRMED';
    elseif passLogged && isfinite(OUT.first_validation_pass_request) && ...
            isfinite(OUT.first_commit_request) && ...
            OUT.first_validation_pass_request == OUT.first_commit_request
        OUT.status='FIRST_VALIDATION_PASS_IS_LATE_COMMIT__GENERATOR_VALIDITY_AND_DEADLINE_BOTH_MATTER';
    elseif passLogged && ~isfinite(OUT.first_validation_pass_request)
        OUT.status='NO_VALIDATION_PASS_IN_REQUEST_ROWS__CHECK_COUNTER_LOGGING';
    else
        OUT.status='PER_REQUEST_VALIDATION_PASS_NOT_LOGGED__USE_STATUS_AND_DELTAS';
    end

    fprintf(' source exact                         = %d | %s\n',OUT.source_exact,OUT.source_sha);
    fprintf(' finite-sensing contract closed       = %d\n',OUT.sensor_contract.closed);
    fprintf(' request count                        = %d\n',OUT.request_count);
    fprintf(' first validation pass request        = %s\n',numtxt(OUT.first_validation_pass_request));
    fprintf(' first commit request                 = %s\n',numtxt(OUT.first_commit_request));
    fprintf(' validation-pass field                = %s\n',OUT.bindings.validation_pass_delta);
    fprintf(' validation-fail field                = %s\n',OUT.bindings.validation_fail_delta);
    fprintf(' status-code field                    = %s\n',OUT.bindings.status_code);

    fprintf('\n request table:\n');
    fprintf(' i  t[s]   hard  callback  fail commit  status  cand  vpass vfail prop eval cacheH cacheS cacheR rebuild\n');
    for k=1:OUT.request_count
        fprintf('%2d %7.3f %6.3f %8.3f %4.0f %6.0f %7s %5s %5s %5s %4s %4s %6s %6s %6s %7s\n', ...
            k,OUT.t_s(k),OUT.hard_ms(k),OUT.callback_ms(k), ...
            OUT.deadline_fail(k),OUT.commit(k), ...
            valtxt(OUT.status_code(k)),valtxt(OUT.candidate_delta(k)), ...
            valtxt(OUT.validation_pass_delta(k)),valtxt(OUT.validation_fail_delta(k)), ...
            valtxt(OUT.proposal_delta(k)),valtxt(OUT.reference_eval_delta(k)), ...
            valtxt(OUT.cache_hit_delta(k)),valtxt(OUT.cache_store_delta(k)), ...
            valtxt(OUT.cache_retarget_delta(k)),valtxt(OUT.packet_rebuild_delta(k)));
    end

    if OUT.object_found
        fprintf('\n final live counters:\n');
        fn=fieldnames(OUT.object);
        for k=1:numel(fn)
            v=OUT.object.(fn{k});
            if isnumeric(v)&&isscalar(v)
                fprintf(' %-52s = %.17g\n',fn{k},v);
            elseif ischar(v)
                fprintf(' %-52s = %s\n',fn{k},v);
            end
        end
    end

    fprintf('\nstatus = %s\n',OUT.status);
    fprintf('NOTE: This audit does not expose any obstacle before sensor detection.\n');
    fprintf('NOTE: This audit does not change the 4-ms gate or any acceptance/certificate logic.\n');

catch ME
    OUT.status='AUDIT_ERROR__NO_PRODUCTION_CHANGE';
    OUT.error=struct('identifier',ME.identifier,'message',ME.message,'stack',{ME.stack});
    fprintf('[AUDIT ERROR] %s | %s\n',ME.identifier,ME.message);
end

root=fileparts(which('REPLANNING_SYSTEM'));
if isempty(root), root=pwd; end
outdir=tempname(root); [ok,msg]=mkdir(outdir); assert(ok,'%s',msg);
report=fullfile(outdir,'AF7R58ZZJ_REPORT.txt');
matp=fullfile(outdir,'AF7R58ZZJ_OUT.mat');

fid=fopen(report,'wt','n','UTF-8'); assert(fid>=0);
fprintf(fid,'status=%s\n',OUT.status);
if isfield(OUT,'source_sha'), fprintf(fid,'sourceSHA=%s\n',OUT.source_sha); end
if isfield(OUT,'request_count')
    fprintf(fid,'requests=%d\n',OUT.request_count);
    fprintf(fid,'firstValidationPassRequest=%s\n',numtxt(OUT.first_validation_pass_request));
    fprintf(fid,'firstCommitRequest=%s\n',numtxt(OUT.first_commit_request));
    fprintf(fid,'validationPassField=%s\n',OUT.bindings.validation_pass_delta);
    fprintf(fid,'validationFailField=%s\n',OUT.bindings.validation_fail_delta);
    fprintf(fid,'statusCodeField=%s\n',OUT.bindings.status_code);
end
fclose(fid);

OUT.report_path=report;
OUT.mat_path=matp;
save(matp,'OUT','-v7');
assignin('base','AF7R58ZZJ_OUT',OUT);
fprintf('RETURN: %s\nRETURN: %s\n',report,matp);
end

function [x,bound]=get_first_numeric(P,idx,vn,candidates)
x=nan(numel(idx),1);
bound='';
for j=1:numel(candidates)
    nm=string(candidates(j));
    if any(vn==nm)
        v=P.(char(nm));
        if isnumeric(v)||islogical(v)
            vv=double(v(idx));
            x=vv(:);
            bound=char(nm);
            return;
        end
    end
end
end

function k=first_positive(x)
k=NaN;
if isempty(x), return; end
j=find(isfinite(x)&x>0,1,'first');
if ~isempty(j), k=double(j); end
end

function s=valtxt(x)
if isempty(x)||~isfinite(x)
    s='-';
elseif abs(x-round(x))<1e-12
    s=sprintf('%.0f',x);
else
    s=sprintf('%.3g',x);
end
end

function s=numtxt(x)
if isempty(x)||~isfinite(x)
    s='NaN';
else
    s=sprintf('%.0f',x);
end
end

function x=get_prop_num(obj,name)
x=NaN;
try
    if isprop(obj,name)
        v=obj.(name);
        if isnumeric(v)||islogical(v)
            if isscalar(v)
                x=double(v);
            end
        end
    end
catch
end
end

function h=file_hash(path)
md=java.security.MessageDigest.getInstance('SHA-256');
fid=fopen(path,'rb'); assert(fid>=0); c=onCleanup(@()fclose(fid));
while true
    b=fread(fid,1024*1024,'*uint8');
    if isempty(b), break; end
    md.update(b);
end
u=typecast(md.digest(),'uint8');
h=lower(reshape(dec2hex(u,2).',1,[]));
end

function s=read_utf8(path)
fid=fopen(path,'rt','n','UTF-8'); assert(fid>=0); c=onCleanup(@()fclose(fid));
s=fread(fid,'*char')';
end
