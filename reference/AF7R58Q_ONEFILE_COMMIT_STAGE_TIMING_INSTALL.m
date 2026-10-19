function OUT = AF7R58Q_ONEFILE_COMMIT_STAGE_TIMING_INSTALL()
% AF7R58Q
% Transactional diagnostic instrumentation for try_commit_reference_only_proposal.
%
% It measures stage costs only. It DOES NOT alter:
%   - candidate mathematics,
%   - continuous-time geometry certificate,
%   - resource/spacing gates,
%   - commit acceptance logic,
%   - controller/plant/actuator logic,
%   - payload-mass authority.
%
% IMPORTANT:
%   Timing is diagnostic instrumentation and may add small overhead.
%   Do NOT use instrumented end-to-end timing as the final 4-ms proof.
%
% RETURN:
%   AF7R58Q_REPORT.txt
%   AF7R58Q_OUT.mat

root=fileparts(mfilename('fullpath'));
outdir=tempname(root);
[ok,msg]=mkdir(outdir); assert(ok,'%s',msg);

f=fopen(fullfile(outdir,'AF7R58Q_REPORT.txt'),'wt','n','UTF-8');
assert(f>=0,'[AF7R58Q] cannot open report.');
cleanupF=onCleanup(@()fclose(f)); %#ok<NASGU>

say(f,'====================================================================================================');
say(f,' AF7R58Q | REFERENCE-ONLY COMMIT STAGE TIMING INSTALL');
say(f,'====================================================================================================');
say(f,'[AUTHORITY] diagnosticOnly=1 | boundedRing=1 | rollbackOnFailure=1 | productionWriteAfterParseOnly=1');
say(f,'[SAFETY] trajectoryMathChanged=0 | certificateLogicChanged=0 | commitDecisionChanged=0 | mLTouched=0');
say(f,' root = %s',root);

OUT=struct();
OUT.schema='AF7R58Q_COMMIT_STAGE_TIMING_INSTALL_V1';
OUT.production_modified=false;
OUT.rollback_performed=false;
OUT.status='OPEN';
OUT.output_directory=outdir;
OUT.required_return_files={'AF7R58Q_REPORT.txt','AF7R58Q_OUT.mat'};
OUT.error=struct();

pR=fullfile(root,'REPLANNING_SYSTEM.m');
expectedR='051b2ee71448d51532f1f260e5141476a955e91eb205d406a083260dc4583f31';

backupDir=tempname(root);
backupReady=false;
writesStarted=false;

try
    assert(isfile(pR),'[AF7R58Q] REPLANNING_SYSTEM.m missing.');
    h0=file_hash(pR);

    say(f,'');
    say(f,'[A. EXACT PRE-INSTALL SOURCE BINDING]');
    say(f,' REPLANNING_SYSTEM AF7R58O SHA exact = %d | %s',strcmp(h0,expectedR),h0);
    assert(strcmp(h0,expectedR), ...
        '[AF7R58Q] current REPLANNING_SYSTEM differs from AF7R58O authority; no install.');

    [R,eolR]=read_text_norm(pR);

    % ------------------------------------------------------------------
    % 1. Diagnostic properties.
    % ------------------------------------------------------------------
    propAnchor='        af7r58m_max_capture_overhead_us      = 0;';
    propInsert=blk({ ...
        propAnchor; ...
        ''; ...
        '        % AF7R58Q bounded commit-stage timing diagnostics.'; ...
        '        ENABLE_AF7R58Q_COMMIT_TIMING          = true;'; ...
        '        af7r58q_commit_ring_capacity          = 64;'; ...
        '        af7r58q_commit_ring_width             = 16;'; ...
        '        af7r58q_commit_ring                   = nan(64,16);'; ...
        '        af7r58q_commit_ring_head              = 0;'; ...
        '        af7r58q_commit_ring_count             = 0;'; ...
        '        af7r58q_last_stage_ms                 = nan(1,12);'; ...
        '        af7r58q_last_stage_code               = 0;'; ...
        '        af7r58q_last_capture_overhead_us      = NaN;'; ...
        '        af7r58q_max_capture_overhead_us       = 0;'});
    R=patch_once(R,propAnchor,propInsert,'58Q properties');

    % ------------------------------------------------------------------
    % 2. Internal bounded ring writer.
    % ------------------------------------------------------------------
    fnMarker='        function [ok,reason] = try_commit_reference_only_proposal(obj,P,t_commit,t_hard_cycle)';
    idx=strfind(R,fnMarker);
    assert(numel(idx)==1,'[AF7R58Q] try_commit function marker count=%d',numel(idx));

    captureMethod=blk({ ...
        '        function af7r58q_capture_commit_timing_internal(obj,totalTic)'; ...
        '            % Bounded no-throw diagnostic writer; never a safety decision.'; ...
        '            tcap=tic;'; ...
        '            try'; ...
        '                if ~logical(obj.ENABLE_AF7R58Q_COMMIT_TIMING)'; ...
        '                    return;'; ...
        '                end'; ...
        '                if ~(obj.af7r58q_commit_ring_capacity==64 && ...'; ...
        '                        obj.af7r58q_commit_ring_width==16 && ...'; ...
        '                        isequal(size(obj.af7r58q_commit_ring),[64 16]))'; ...
        '                    return;'; ...
        '                end'; ...
        '                r=nan(1,16);'; ...
        '                r(1)=double(obj.generator_request_count);'; ...
        '                r(2)=1e3*toc(totalTic);'; ...
        '                r(3)=double(obj.af7r58q_last_stage_code);'; ...
        '                r(4:15)=double(obj.af7r58q_last_stage_ms);'; ...
        '                r(16)=double(obj.active_reference_version);'; ...
        '                head=double(obj.af7r58q_commit_ring_head);'; ...
        '                cap=double(obj.af7r58q_commit_ring_capacity);'; ...
        '                j=mod(head,cap)+1;'; ...
        '                obj.af7r58q_commit_ring(j,:)=r;'; ...
        '                obj.af7r58q_commit_ring_head=j;'; ...
        '                obj.af7r58q_commit_ring_count= ...'; ...
        '                    min(cap,double(obj.af7r58q_commit_ring_count)+1);'; ...
        '                us=1e6*toc(tcap);'; ...
        '                obj.af7r58q_last_capture_overhead_us=us;'; ...
        '                if isfinite(us)'; ...
        '                    obj.af7r58q_max_capture_overhead_us= ...'; ...
        '                        max(double(obj.af7r58q_max_capture_overhead_us),us);'; ...
        '                end'; ...
        '            catch'; ...
        '                % Diagnostic instrumentation must never affect control.'; ...
        '            end'; ...
        '        end'; ...
        ''; ...
        ''});

    R=[R(1:idx-1) captureMethod R(idx:end)];

    % ------------------------------------------------------------------
    % 3. Function entry total timer + cleanup.
    % ------------------------------------------------------------------
    entryOld=blk({ ...
        fnMarker; ...
        '            ok=false;'; ...
        '            reason='''';'});
    entryNew=blk({ ...
        fnMarker; ...
        '            ok=false;'; ...
        '            reason='''';'; ...
        '            af7r58q_total_tic=tic;'; ...
        '            obj.af7r58q_last_stage_ms=nan(1,12);'; ...
        '            obj.af7r58q_last_stage_code=0;'; ...
        '            af7r58q_cleanup=onCleanup( ...'; ...
        '                @()obj.af7r58q_capture_commit_timing_internal(af7r58q_total_tic)); %#ok<NASGU>'});
    R=patch_once(R,entryOld,entryNew,'58Q function entry');

    % ------------------------------------------------------------------
    % 4. Commit spacing.
    % ------------------------------------------------------------------
    old=blk({ ...
        '            [spacingOK,~,~,spacingReason]= ...'; ...
        '                af7r58i_commit_spacing_guard(obj,double(t_commit));'});
    new=blk({ ...
        '            af7r58q_t=tic;'; ...
        '            [spacingOK,~,~,spacingReason]= ...'; ...
        '                af7r58i_commit_spacing_guard(obj,double(t_commit));'; ...
        '            obj.af7r58q_last_stage_ms(1)=1e3*toc(af7r58q_t);'; ...
        '            obj.af7r58q_last_stage_code=1;'});
    R=patch_once_in_function(R,fnMarker,old,new,'spacing timer');

    % ------------------------------------------------------------------
    % 5. Resource canonical prepare.
    % ------------------------------------------------------------------
    old=blk({ ...
        '            [resourceOK,resourceReason,batchModel,resourceMeta]= ...'; ...
        '                af7r58i_prepare_batch_model( ...'; ...
        '                    obj,P,targetVersion,double(t_commit));'});
    new=blk({ ...
        '            af7r58q_t=tic;'; ...
        '            [resourceOK,resourceReason,batchModel,resourceMeta]= ...'; ...
        '                af7r58i_prepare_batch_model( ...'; ...
        '                    obj,P,targetVersion,double(t_commit));'; ...
        '            obj.af7r58q_last_stage_ms(2)=1e3*toc(af7r58q_t);'; ...
        '            obj.af7r58q_last_stage_code=2;'});
    R=patch_once_in_function(R,fnMarker,old,new,'resource prepare timer');

    % ------------------------------------------------------------------
    % 6. Canonical evaluator handles.
    % ------------------------------------------------------------------
    old=blk({ ...
        '            [evalFn,flatEvalFn]= ...'; ...
        '                af7r58i_make_canonical_eval_handles(obj,batchModel);'});
    new=blk({ ...
        '            af7r58q_t=tic;'; ...
        '            [evalFn,flatEvalFn]= ...'; ...
        '                af7r58i_make_canonical_eval_handles(obj,batchModel);'; ...
        '            obj.af7r58q_last_stage_ms(3)=1e3*toc(af7r58q_t);'; ...
        '            obj.af7r58q_last_stage_code=3;'});
    R=patch_once_in_function(R,fnMarker,old,new,'canonical handles timer');

    % ------------------------------------------------------------------
    % 7. Boundary pre-evaluation.
    % ------------------------------------------------------------------
    old='            % Pre-evaluate the exact HLC packet at commit/horizon boundary.';
    new=blk({ ...
        old; ...
        '            af7r58q_t=tic;'});
    R=patch_once_in_function(R,fnMarker,old,new,'preeval timer start');

    old=blk({ ...
        '            old=struct( ...'; ...
        '                ''mode'',obj.active_reference_mode, ...'});
    new=blk({ ...
        '            obj.af7r58q_last_stage_ms(4)=1e3*toc(af7r58q_t);'; ...
        '            obj.af7r58q_last_stage_code=4;'; ...
        '            old=struct( ...'; ...
        '                ''mode'',obj.active_reference_mode, ...'});
    R=patch_once_in_function(R,fnMarker,old,new,'preeval timer end');

    % ------------------------------------------------------------------
    % 8. R build + active-state assignment up to speed validation.
    % ------------------------------------------------------------------
    old='            try';
    new=blk({ ...
        '            try'; ...
        '                af7r58q_t=tic;'});
    R=patch_once_in_function(R,fnMarker,old,new,'try timer start');

    old='                obj.validate_active_speed_certificate(SC);';
    new=blk({ ...
        '                obj.af7r58q_last_stage_ms(5)=1e3*toc(af7r58q_t);'; ...
        '                obj.af7r58q_last_stage_code=5;'; ...
        '                af7r58q_t=tic;'; ...
        '                obj.validate_active_speed_certificate(SC);'});
    R=patch_once_in_function(R,fnMarker,old,new,'active-state timer end');

    % speed validation + trigger sweep
    old=blk({ ...
        '                obj.trigger_sweep_inflation= ...'; ...
        '                    obj.build_trigger_sweep_inflation(SC);'});
    new=blk({ ...
        '                obj.trigger_sweep_inflation= ...'; ...
        '                    obj.build_trigger_sweep_inflation(SC);'; ...
        '                obj.af7r58q_last_stage_ms(6)=1e3*toc(af7r58q_t);'; ...
        '                obj.af7r58q_last_stage_code=6;'});
    R=patch_once_in_function(R,fnMarker,old,new,'speed sweep timer');

    % ------------------------------------------------------------------
    % 9. Obstacle packet.
    % ------------------------------------------------------------------
    old=blk({ ...
        '                if ~isempty(obj.static_obstacle_map)'; ...
        '                    obj.obstacle_packet=obj.prepare_obstacle_packet(obj.static_obstacle_map);'; ...
        '                end'});
    new=blk({ ...
        '                af7r58q_t=tic;'; ...
        '                if ~isempty(obj.static_obstacle_map)'; ...
        '                    obj.obstacle_packet=obj.prepare_obstacle_packet(obj.static_obstacle_map);'; ...
        '                end'; ...
        '                obj.af7r58q_last_stage_ms(7)=1e3*toc(af7r58q_t);'; ...
        '                obj.af7r58q_last_stage_code=7;'});
    R=patch_once_in_function(R,fnMarker,old,new,'obstacle packet timer');

    % ------------------------------------------------------------------
    % 10. Sensor horizon.
    % ------------------------------------------------------------------
    old='                obj.sensor_horizon_certificate=obj.build_sensor_horizon_certificate();';
    new=blk({ ...
        '                af7r58q_t=tic;'; ...
        '                obj.sensor_horizon_certificate=obj.build_sensor_horizon_certificate();'});
    R=patch_once_in_function(R,fnMarker,old,new,'sensor timer start');

    old=blk({ ...
        '                if ~(isfield(obj.sensor_horizon_certificate,''certified'') && ...'; ...
        '                        logical(obj.sensor_horizon_certificate.certified))'; ...
        '                    error(''[REPLANNER ERROR] reference-only repaired sensor horizon is not closed.'');'; ...
        '                end'});
    new=blk({ ...
        old; ...
        '                obj.af7r58q_last_stage_ms(8)=1e3*toc(af7r58q_t);'; ...
        '                obj.af7r58q_last_stage_code=8;'});
    R=patch_once_in_function(R,fnMarker,old,new,'sensor timer end');

    % ------------------------------------------------------------------
    % 11. Runtime dependency signature + stale state.
    % ------------------------------------------------------------------
    old=blk({ ...
        '                obj.runtime_dependency_signature= ...'; ...
        '                    obj.capture_runtime_dependency_signature_fast();'});
    new=blk({ ...
        '                af7r58q_t=tic;'; ...
        '                obj.runtime_dependency_signature= ...'; ...
        '                    obj.capture_runtime_dependency_signature_fast();'});
    R=patch_once_in_function(R,fnMarker,old,new,'dependency timer start');

    old=blk({ ...
        '                obj.certificate_stale.HLC_ROBUST_CERT=true;'});
    new=blk({ ...
        '                obj.certificate_stale.HLC_ROBUST_CERT=true;'; ...
        '                obj.af7r58q_last_stage_ms(9)=1e3*toc(af7r58q_t);'; ...
        '                obj.af7r58q_last_stage_code=9;'});
    R=patch_once_in_function(R,fnMarker,old,new,'dependency timer end');

    % ------------------------------------------------------------------
    % 12. Counters/status + assertions.
    % ------------------------------------------------------------------
    old='                obj.replan_commit_count=obj.replan_commit_count+1;';
    new=blk({ ...
        '                af7r58q_t=tic;'; ...
        '                obj.replan_commit_count=obj.replan_commit_count+1;'});
    R=patch_once_in_function(R,fnMarker,old,new,'commit state timer start');

    old=blk({ ...
        '                obj.replan_request_active=false;'; ...
        ''; ...
        '                obj.assert_trigger_packet_certificate_current();'; ...
        '                obj.assert_repaired_reference_current();'});
    new=blk({ ...
        '                obj.replan_request_active=false;'; ...
        '                obj.af7r58q_last_stage_ms(10)=1e3*toc(af7r58q_t);'; ...
        '                obj.af7r58q_last_stage_code=10;'; ...
        ''; ...
        '                af7r58q_t=tic;'; ...
        '                obj.assert_trigger_packet_certificate_current();'; ...
        '                obj.assert_repaired_reference_current();'; ...
        '                obj.af7r58q_last_stage_ms(11)=1e3*toc(af7r58q_t);'; ...
        '                obj.af7r58q_last_stage_code=11;'});
    R=patch_once_in_function(R,fnMarker,old,new,'assert timer');

    % ------------------------------------------------------------------
    % 13. Rollback timing.
    % ------------------------------------------------------------------
    old=blk({ ...
        '            catch ME'; ...
        '                obj.active_reference_mode=old.mode;'});
    new=blk({ ...
        '            catch ME'; ...
        '                af7r58q_t=tic;'; ...
        '                obj.active_reference_mode=old.mode;'});
    R=patch_once_in_function(R,fnMarker,old,new,'rollback timer start');

    old=blk({ ...
        '                obj.last_replan_commit_time=old.time;'; ...
        '                reason=ME.message;'});
    new=blk({ ...
        '                obj.last_replan_commit_time=old.time;'; ...
        '                obj.af7r58q_last_stage_ms(12)=1e3*toc(af7r58q_t);'; ...
        '                obj.af7r58q_last_stage_code=12;'; ...
        '                reason=ME.message;'});
    R=patch_once_in_function(R,fnMarker,old,new,'rollback timer end');

    % ------------------------------------------------------------------
    % Source invariants.
    % ------------------------------------------------------------------
    cR=compact_ws(R);
    inv=struct();
    inv.internal_writer=contains(cR,'functionaf7r58q_capture_commit_timing_internal(obj,totalTic)');
    inv.function_cleanup=contains(cR,'af7r58q_capture_commit_timing_internal(af7r58q_total_tic)');
    inv.resource_timer=contains(cR,'af7r58q_last_stage_ms(2)=1e3*toc(af7r58q_t);');
    inv.rollback_timer=contains(cR,'af7r58q_last_stage_ms(12)=1e3*toc(af7r58q_t);');
    inv.ring64x16=contains(cR,'af7r58q_commit_ring=nan(64,16);');
    inv.all=all(structfun(@logical,inv));
    assert(inv.all,'[AF7R58Q] prewrite source invariants failed.');

    % ------------------------------------------------------------------
    % Preflight class parse before production write.
    % ------------------------------------------------------------------
    preflightDir=fullfile(outdir,'AF7R58Q_PREFLIGHT');
    [ok,msg]=mkdir(preflightDir); assert(ok,'%s',msg);
    parseName='AF7R58Q_PARSE_REPLANNING_SYSTEM';
    parseText=strrep(R,'classdef REPLANNING_SYSTEM < handle', ...
        ['classdef ' parseName ' < handle']);
    assert(~strcmp(parseText,R),'[AF7R58Q] class rename anchor missing.');
    parseFile=fullfile(preflightDir,[parseName '.m']);
    write_utf8(parseFile,parseText);

    addpath(preflightDir,'-begin');
    eval(['clear ' parseName]);
    mc=meta.class.fromName(parseName);
    assert(~isempty(mc),'[AF7R58Q] patched class preflight parse failed.');
    safe_rmpath_if_present(preflightDir);

    say(f,'');
    say(f,'[B. PREFLIGHT PARSE -- PRODUCTION WRITES STILL ZERO]');
    say(f,' patched REPLANNING_SYSTEM class PASS=1');

    % ------------------------------------------------------------------
    % Backup + production write.
    % ------------------------------------------------------------------
    [ok,msg]=mkdir(backupDir); assert(ok,'%s',msg);
    backupReady=true;
    OUT.backup_directory=backupDir;
    copyfile(pR,fullfile(backupDir,'REPLANNING_SYSTEM.m'));

    writesStarted=true;
    write_text_eol(pR,R,eolR);
    rehash;

    % Post-write parse.
    postName='AF7R58Q_POST_PARSE_REPLANNING_SYSTEM';
    postText=strrep(fileread(pR),'classdef REPLANNING_SYSTEM < handle', ...
        ['classdef ' postName ' < handle']);
    postFile=fullfile(outdir,[postName '.m']);
    write_utf8(postFile,postText);
    addpath(outdir,'-begin');
    eval(['clear ' postName]);
    mc2=meta.class.fromName(postName);
    assert(~isempty(mc2),'[AF7R58Q] installed class post-parse failed.');
    safe_rmpath_if_present(outdir);

    h1=file_hash(pR);
    OUT.source_hash_before=h0;
    OUT.source_hash_after=h1;
    OUT.invariants=inv;
    OUT.production_modified=true;
    OUT.status='COMMIT_STAGE_TIMING_INSTALL_COMPLETE__CLEAN_RERUN_REQUIRED';

    labels={ ...
        'request_seq','commit_total_ms','last_stage_code', ...
        'spacing_ms','resource_prepare_ms','canonical_handles_ms', ...
        'boundary_preeval_ms','R_and_active_assign_ms','speed_sweep_ms', ...
        'obstacle_packet_ms','sensor_horizon_ms','dependency_stale_ms', ...
        'commit_state_ms','post_assert_ms','rollback_ms','active_reference_version'};
    OUT.column_labels=labels;

    say(f,'');
    say(f,'[C. INSTALL RESULT]');
    say(f,' bounded commit ring                     = 64 x 16');
    say(f,' commit acceptance logic changed         = 0');
    say(f,' trajectory/certificate logic changed   = 0');
    say(f,' production modified                     = 1');
    say(f,' new REPLANNING_SYSTEM SHA               = %s',h1);

    say(f,'');
    say(f,'[D. COMMIT TIMING COLUMNS]');
    for k=1:numel(labels)
        say(f,' %2d  %s',k,labels{k});
    end

    say(f,'');
    say(f,'[E. NEXT STEP]');
    say(f,' Recreate REPLANNING_SYSTEM from a clean run.');
    say(f,' Run the same scenario until the real replan requests occur.');
    say(f,' Keep the live object. The next read-only audit will correlate');
    say(f,' request status (-20/-30/30) with exact commit substage timing.');
    say(f,' Instrumented end-to-end cycle time is NOT the final 4-ms validation.');

catch ME
    if backupReady&&writesStarted
        try
            copyfile(fullfile(backupDir,'REPLANNING_SYSTEM.m'),pR,'f');
            rehash;
            OUT.rollback_performed=true;
        catch
        end
    end
    OUT.production_modified=false;
    OUT.status='INSTALL_FAILED__ROLLBACK_COMPLETE';
    OUT.error=struct('identifier',ME.identifier,'message',ME.message,'stack',{ME.stack});

    say(f,'');
    say(f,'[INSTALL FAILURE]');
    say(f,' identifier          = %s',ME.identifier);
    say(f,' message             = %s',ME.message);
    say(f,' rollback performed  = %d',OUT.rollback_performed);
    say(f,' production modified = 0');
end

save(fullfile(outdir,'AF7R58Q_OUT.mat'),'OUT','-v7');
assignin('base','AF7R58Q_OUT',OUT);

say(f,'');
say(f,'[RETURN THESE TWO FILES]');
say(f,' RETURN: %s',fullfile(outdir,'AF7R58Q_REPORT.txt'));
say(f,' RETURN: %s',fullfile(outdir,'AF7R58Q_OUT.mat'));
say(f,' status = %s',OUT.status);

end


function s=patch_once_in_function(s,fnMarker,old,new,label)
a=strfind(s,fnMarker);
assert(numel(a)==1,'[AF7R58Q] function marker ambiguous for %s.',label);
tail=s(a(1):end);
nextFn=strfind(tail(2:end),'        function ');
if isempty(nextFn)
    b=numel(s)+1;
else
    b=a(1)+nextFn(1);
end
block=s(a(1):b-1);
idx=strfind(block,old);
assert(numel(idx)==1, ...
    '[AF7R58Q] function-local patch anchor must occur once (%s), found %d.', ...
    label,numel(idx));
k=a(1)+idx(1)-1;
s=[s(1:k-1) new s(k+numel(old):end)];
end


function s=patch_once(s,old,new,label)
idx=strfind(s,old);
assert(numel(idx)==1, ...
    '[AF7R58Q] patch anchor must occur exactly once (%s), found %d.',label,numel(idx));
k=idx(1);
s=[s(1:k-1) new s(k+numel(old):end)];
end


function [s,eol]=read_text_norm(p)
f=fopen(p,'rb'); assert(f>=0,'cannot read %s',p);
c=onCleanup(@()fclose(f)); %#ok<NASGU>
b=fread(f,Inf,'*uint8');
s=native2unicode(b.','UTF-8');
if contains(s,sprintf('\r\n'))
    eol='CRLF';
elseif contains(s,sprintf('\r'))
    eol='CR';
else
    eol='LF';
end
s=strrep(s,sprintf('\r\n'),sprintf('\n'));
s=strrep(s,sprintf('\r'),sprintf('\n'));
end


function write_text_eol(p,s,eol)
switch eol
    case 'CRLF'
        s=strrep(s,sprintf('\n'),sprintf('\r\n'));
    case 'CR'
        s=strrep(s,sprintf('\n'),sprintf('\r'));
end
write_utf8(p,s);
end


function write_utf8(p,s)
f=fopen(p,'wb'); assert(f>=0,'[AF7R58Q] cannot write %s',p);
c=onCleanup(@()fclose(f)); %#ok<NASGU>
b=unicode2native(s,'UTF-8');
n=fwrite(f,b,'uint8');
assert(n==numel(b),'[AF7R58Q] short write: %s',p);
end


function safe_rmpath_if_present(p)
try
    parts=strsplit(path,pathsep);
    if any(strcmpi(parts,p))
        rmpath(p);
    end
catch
end
end


function s=compact_ws(s)
s=regexprep(s,'\s+','');
end


function s=blk(c)
s=strjoin(c,newline);
end


function h=file_hash(p)
f=fopen(p,'rb'); assert(f>=0,'cannot read %s',p);
c=onCleanup(@()fclose(f)); %#ok<NASGU>
b=fread(f,Inf,'*uint8');
m=java.security.MessageDigest.getInstance('SHA-256');
m.update(b);
d=typecast(m.digest(),'uint8');
h=lower(reshape(dec2hex(d,2).',1,[]));
end


function say(f,fmt,varargin)
s=sprintf(fmt,varargin{:});
fprintf(1,'%s\n',s);
fprintf(f,'%s\n',s);
end
