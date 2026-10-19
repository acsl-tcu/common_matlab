function OUT = AF7R58ZZI_ONEFILE_SOLUTION_VS_DEADLINE_AUDIT()
% AF7R58ZZI
% Read-only audit after AF7R58ZZH.
%
% Purpose:
%   Distinguish
%     (A) no replan request,
%     (B) generator/proposal no-solution,
%     (C) mathematically usable proposal exists but commit is rejected by the
%         4-ms online modification deadline,
%     (D) only a later retry commits.
%
% Finite-sensing authority:
%   This audit does not inject any obstacle. It verifies that production still
%   gives the generator only sensor-known obstacles and that sensor memory is
%   initially empty. The private simulation truth map may be used by the sensor
%   emulator, but must not be exposed to the generator before detection.
%
% IMPORTANT:
%   A later retry commit is NOT evidence that the first-detection -> target
%   modification 4-ms requirement is closed. It is reported separately.

EXPECTED_SHA='2a0013a583cbcff3afd62e4495fa5e784427132390616ec5886000f134f2c05f';
OUT=struct('schema','AF7R58ZZI_SOLUTION_VS_DEADLINE_V1', ...
    'production_modified',false,'status','OPEN','error',struct());

fprintf('\n====================================================================================================\n');
fprintf(' AF7R58ZZI | SOLUTION-vs-DEADLINE + FINITE-SENSING AUDIT\n');
fprintf('====================================================================================================\n');
fprintf('[AUTHORITY] readOnly=1 | productionModified=0 | noSyntheticObstacle=1 | noCommit=1\n');
fprintf('[SENSING] detectedOnlyGeneratorRequired=1 | initiallyUnknownRequired=1\n');

try
    pR=which('REPLANNING_SYSTEM');
    assert(~isempty(pR)&&isfile(pR),'[AF7R58ZZI] REPLANNING_SYSTEM unresolved.');
    OUT.source_sha=file_hash(pR);
    OUT.source_exact=strcmp(OUT.source_sha,EXPECTED_SHA);

    R=read_utf8(pR);
    OUT.sensor_contract=struct();
    OUT.sensor_contract.generator_detected_only= ...
        contains(R,'The generator receives ONLY sensor-known obstacles:');
    OUT.sensor_contract.detected_only_context_flag= ...
        contains(R,'''detected_only_obstacle_contract'',true');
    OUT.sensor_contract.initially_unknown= ...
        contains(R,'Initial known set is empty.');
    OUT.sensor_contract.raw_sensor_only_admission= ...
        contains(R,'raw_sensor_gate alone may admit a previously unknown obstacle.');
    OUT.sensor_contract.closed= ...
        OUT.sensor_contract.generator_detected_only && ...
        OUT.sensor_contract.detected_only_context_flag && ...
        OUT.sensor_contract.initially_unknown && ...
        OUT.sensor_contract.raw_sensor_only_admission;

    assert(evalin('base','exist(''AF7R58ZZB_RUNTIME_PROFILE'',''var'')')==1, ...
        '[AF7R58ZZI] run the same AF7R58ZZB real finite-sensing scenario first.');
    P=evalin('base','AF7R58ZZB_RUNTIME_PROFILE');

    reqMask=(P.phase=='f') & isfinite(P.reference_s) & P.gen_request_delta>0;
    idx=find(reqMask);
    OUT.request_count=numel(idx);
    OUT.request_t_s=double(P.t(idx));
    OUT.hard_ms=double(P.gen_last_cycle_critical_ms(idx));
    OUT.callback_ms=double(P.gen_last_callback_ms(idx));
    OUT.deadline_fail=double(P.gen_deadline_delta(idx));
    OUT.commit=double(P.gen_commit_delta(idx));
    OUT.reference_ms=1e3*double(P.reference_s(idx));

    OUT.fail_count=sum(OUT.deadline_fail);
    OUT.commit_count=sum(OUT.commit);
    OUT.hard_max_ms=safe_max(OUT.hard_ms);
    OUT.callback_max_ms=safe_max(OUT.callback_ms);
    OUT.reference_max_ms=safe_max(OUT.reference_ms);
    OUT.first_request_t_s=NaN;
    OUT.first_commit_t_s=NaN;
    OUT.first_commit_request_index=NaN;
    OUT.first_request_to_commit_ms=NaN;
    if OUT.request_count>0
        OUT.first_request_t_s=OUT.request_t_s(1);
        jc=find(OUT.commit>0,1,'first');
        if ~isempty(jc)
            OUT.first_commit_request_index=jc;
            OUT.first_commit_t_s=OUT.request_t_s(jc);
            OUT.first_request_to_commit_ms=1e3*(OUT.first_commit_t_s-OUT.first_request_t_s);
        end
    end

    % Optional live-object evidence. Absence does not make the audit fail.
    OUT.object_found=false;
    OUT.object=struct( ...
        'compact_native_proposal_count',NaN, ...
        'reference_only_evaluation_count',NaN, ...
        'reference_only_commit_count',NaN, ...
        'generator_validation_pass_count',NaN, ...
        'generator_validation_fail_count',NaN, ...
        'replan_commit_count',NaN, ...
        'active_reference_mode','', ...
        'active_reference_version',NaN, ...
        'prepared_cache_valid',false, ...
        'prepared_cache_store_count',NaN, ...
        'prepared_cache_hit_count',NaN, ...
        'prepared_cache_retarget_count',NaN, ...
        'prepared_cache_commit_count',NaN, ...
        'active_displacement_norm_m',NaN, ...
        'sensor_memory_known_count',NaN, ...
        'sensor_memory_initially_unknown_contract',false);

    if evalin('base','exist(''replanner_instance'',''var'')')==1
        obj=evalin('base','replanner_instance');
        if isa(obj,'REPLANNING_SYSTEM')
            OUT.object_found=true;
            OUT.object.compact_native_proposal_count=get_prop_num(obj,'compact_native_proposal_count');
            OUT.object.reference_only_evaluation_count=get_prop_num(obj,'reference_only_evaluation_count');
            OUT.object.reference_only_commit_count=get_prop_num(obj,'reference_only_commit_count');
            OUT.object.generator_validation_pass_count=get_prop_num(obj,'generator_validation_pass_count');
            OUT.object.generator_validation_fail_count=get_prop_num(obj,'generator_validation_fail_count');
            OUT.object.replan_commit_count=get_prop_num(obj,'replan_commit_count');
            OUT.object.active_reference_mode=get_prop_char(obj,'active_reference_mode');
            OUT.object.active_reference_version=get_prop_num(obj,'active_reference_version');
            OUT.object.prepared_cache_valid=get_prop_logical(obj,'reference_only_prepared_packet_cache_valid');
            OUT.object.prepared_cache_store_count=get_prop_num(obj,'reference_only_prepared_packet_cache_store_count');
            OUT.object.prepared_cache_hit_count=get_prop_num(obj,'reference_only_prepared_packet_cache_hit_count');
            OUT.object.prepared_cache_retarget_count=get_prop_num(obj,'reference_only_prepared_packet_cache_retarget_count');
            OUT.object.prepared_cache_commit_count=get_prop_num(obj,'reference_only_prepared_packet_cache_commit_count');

            if isprop(obj,'active_repaired_reference')
                A=obj.active_repaired_reference;
                if isstruct(A) && isfield(A,'representation') && ...
                        isstruct(A.representation) && isfield(A.representation,'displacement')
                    d=double(A.representation.displacement(:));
                    if ~isempty(d) && all(isfinite(d))
                        OUT.object.active_displacement_norm_m=norm(d);
                    end
                end
            end

            if ismethod(obj,'get_simulated_sensor_memory_status')
                try
                    SM=obj.get_simulated_sensor_memory_status();
                    if isstruct(SM)
                        if isfield(SM,'known_count')
                            OUT.object.sensor_memory_known_count=double(SM.known_count);
                        end
                        if isfield(SM,'initially_unknown_contract')
                            OUT.object.sensor_memory_initially_unknown_contract= ...
                                logical(SM.initially_unknown_contract);
                        end
                    end
                catch
                end
            end
        end
    end

    proposalEvidence=false;
    if OUT.object_found
        proposalEvidence= ...
            finite_positive(OUT.object.compact_native_proposal_count) || ...
            finite_positive(OUT.object.reference_only_evaluation_count) || ...
            finite_positive(OUT.object.generator_validation_pass_count);
    end
    OUT.proposal_evidence=proposalEvidence;
    OUT.deadline_rejection_evidence=(OUT.fail_count>0);
    OUT.no_solution_proven=false; % no-solution requires direct generator/certificate evidence, not timing failure.

    if ~OUT.source_exact
        OUT.status='SOURCE_NOT_EXACT_ZZH';
    elseif ~OUT.sensor_contract.closed
        OUT.status='FINITE_SENSING_CONTRACT_NOT_CLOSED';
    elseif OUT.request_count==0
        OUT.status='NO_REPLAN_REQUEST_OBSERVED';
    elseif OUT.commit_count>0 && OUT.first_commit_request_index==1 && OUT.deadline_fail(1)==0
        OUT.status='FIRST_REQUEST_COMMIT_OBSERVED__CHECK_FIRST_DETECTION_4MS_LEDGER';
    elseif OUT.commit_count>0 && OUT.first_commit_request_index>1
        OUT.status='LATE_RETRY_COMMIT__FIRST_DETECTION_DEADLINE_OPEN';
    elseif OUT.commit_count==0 && proposalEvidence && OUT.fail_count>0
        OUT.status='PROPOSALS_EXIST__COMMIT_DEADLINE_BLOCKING';
    elseif OUT.commit_count==0 && proposalEvidence
        OUT.status='PROPOSALS_EXIST__NONDEADLINE_COMMIT_REJECT__INSPECT_CERT_REASON';
    elseif OUT.commit_count==0 && OUT.fail_count>0
        OUT.status='DEADLINE_BLOCKING__PROPOSAL_EVIDENCE_NOT_EXPORTED';
    else
        OUT.status='NO_PROPOSAL_EVIDENCE__GENERATOR_DIAGNOSTIC_REQUIRED';
    end

    fprintf(' source exact                         = %d | %s\n',OUT.source_exact,OUT.source_sha);
    fprintf(' finite-sensing contract closed       = %d\n',OUT.sensor_contract.closed);
    fprintf(' requests / deadline fails / commits = %d / %.0f / %.0f\n', ...
        OUT.request_count,OUT.fail_count,OUT.commit_count);
    fprintf(' first request [s]                    = %.9g\n',OUT.first_request_t_s);
    fprintf(' first commit [s]                     = %.9g\n',OUT.first_commit_t_s);
    fprintf(' first request -> commit [ms]         = %.9g\n',OUT.first_request_to_commit_ms);
    fprintf(' max hard-cycle [ms]                  = %.9g\n',OUT.hard_max_ms);
    fprintf(' max callback [ms]                    = %.9g\n',OUT.callback_max_ms);
    fprintf(' max reference [ms]                   = %.9g\n',OUT.reference_max_ms);
    fprintf(' proposal evidence                    = %d\n',OUT.proposal_evidence);
    fprintf(' no-solution proven                   = %d\n',OUT.no_solution_proven);

    if OUT.object_found
        fprintf('\n[LIVE OBJECT]\n');
        fprintf(' compact native proposals             = %.0f\n',OUT.object.compact_native_proposal_count);
        fprintf(' reference-only evaluations           = %.0f\n',OUT.object.reference_only_evaluation_count);
        fprintf(' validation pass / fail               = %.0f / %.0f\n', ...
            OUT.object.generator_validation_pass_count,OUT.object.generator_validation_fail_count);
        fprintf(' replan commits                       = %.0f\n',OUT.object.replan_commit_count);
        fprintf(' active mode / version                = %s / %.0f\n', ...
            OUT.object.active_reference_mode,OUT.object.active_reference_version);
        fprintf(' active displacement norm [m]         = %.9g\n',OUT.object.active_displacement_norm_m);
        fprintf(' prepared cache store/hit/retarget    = %.0f / %.0f / %.0f\n', ...
            OUT.object.prepared_cache_store_count,OUT.object.prepared_cache_hit_count, ...
            OUT.object.prepared_cache_retarget_count);
        fprintf(' prepared cache commits               = %.0f\n',OUT.object.prepared_cache_commit_count);
    end

    fprintf('\n request table:\n');
    fprintf(' i  t[s]    hard    callback   ref     fail commit\n');
    for k=1:OUT.request_count
        fprintf('%2d %7.3f %8.3f %8.3f %8.3f %4.0f %6.0f\n', ...
            k,OUT.request_t_s(k),OUT.hard_ms(k),OUT.callback_ms(k), ...
            OUT.reference_ms(k),OUT.deadline_fail(k),OUT.commit(k));
    end
    fprintf('\nstatus = %s\n',OUT.status);
    fprintf('NOTE: a late retry commit does not close first-detection -> modification <= 4 ms.\n');
    fprintf('NOTE: unseen obstacles remain unavailable to the generator until sensor detection.\n');

catch ME
    OUT.status='AUDIT_ERROR__NO_PRODUCTION_CHANGE';
    OUT.error=struct('identifier',ME.identifier,'message',ME.message,'stack',{ME.stack});
    fprintf('[AUDIT ERROR] %s | %s\n',ME.identifier,ME.message);
end

root=fileparts(which('REPLANNING_SYSTEM'));
outdir=tempname(root); [ok,msg]=mkdir(outdir); assert(ok,'%s',msg);
report=fullfile(outdir,'AF7R58ZZI_REPORT.txt');
matp=fullfile(outdir,'AF7R58ZZI_OUT.mat');
fid=fopen(report,'wt','n','UTF-8'); assert(fid>=0);
fprintf(fid,'status=%s\n',OUT.status);
if isfield(OUT,'source_sha'), fprintf(fid,'sourceSHA=%s\n',OUT.source_sha); end
if isfield(OUT,'request_count')
    fprintf(fid,'requests=%d\nfails=%.0f\ncommits=%.0f\n', ...
        OUT.request_count,OUT.fail_count,OUT.commit_count);
    fprintf(fid,'firstRequestS=%.17g\nfirstCommitS=%.17g\nfirstRequestToCommitMs=%.17g\n', ...
        OUT.first_request_t_s,OUT.first_commit_t_s,OUT.first_request_to_commit_ms);
    fprintf(fid,'hardMaxMs=%.17g\ncallbackMaxMs=%.17g\nreferenceMaxMs=%.17g\n', ...
        OUT.hard_max_ms,OUT.callback_max_ms,OUT.reference_max_ms);
    fprintf(fid,'proposalEvidence=%d\nnoSolutionProven=%d\n', ...
        OUT.proposal_evidence,OUT.no_solution_proven);
end
fclose(fid);
OUT.report_path=report; OUT.mat_path=matp;
save(matp,'OUT','-v7');
assignin('base','AF7R58ZZI_OUT',OUT);
fprintf('RETURN: %s\nRETURN: %s\n',report,matp);
end

function tf=finite_positive(x)
tf=isnumeric(x)&&isscalar(x)&&isfinite(x)&&x>0;
end

function x=get_prop_num(obj,name)
x=NaN;
try
    if isprop(obj,name)
        v=obj.(name);
        if isnumeric(v)&&isscalar(v)&&isfinite(double(v)), x=double(v); end
    end
catch
end
end

function s=get_prop_char(obj,name)
s='';
try
    if isprop(obj,name), s=char(string(obj.(name))); end
catch
end
end

function tf=get_prop_logical(obj,name)
tf=false;
try
    if isprop(obj,name)
        v=obj.(name);
        if (islogical(v)||isnumeric(v))&&isscalar(v)&&isfinite(double(v))
            tf=logical(v);
        end
    end
catch
end
end

function m=safe_max(x)
q=double(x(:)); q=q(isfinite(q));
if isempty(q), m=NaN; else, m=max(q); end
end

function s=read_utf8(p)
fid=fopen(p,'rb'); assert(fid>=0); c=onCleanup(@()fclose(fid)); %#ok<NASGU>
b=fread(fid,Inf,'*uint8'); s=native2unicode(b(:).','UTF-8');
end

function h=file_hash(p)
fid=fopen(p,'rb'); assert(fid>=0); c=onCleanup(@()fclose(fid)); %#ok<NASGU>
b=fread(fid,Inf,'*uint8');
m=java.security.MessageDigest.getInstance('SHA-256'); m.update(b);
d=typecast(m.digest(),'uint8'); h=lower(reshape(dec2hex(d,2).',1,[]));
end
