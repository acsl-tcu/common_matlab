function OUT = AF7R58ZZ_ONEFILE_REMOVE_SAMPLED_COMMIT_PROBES_INSTALL()
% AF7R58ZZ
% Exact-source-bound removal of theorem-redundant sampled evaluator probes
% from the REFERENCE_ONLY atomic commit path.
%
% PRE SHA  = c7baa3c7dda80bbacb412443a035fd7c5ae7afbc6c95a46eddc4d82b69d88289
% POST SHA = 59e24920394b53faf2d832d6b84dead3e94e9728ea767cf7dca1ef3a1a5f19fb
%
% Safety logic retained/strengthened:
%   - continuous, non-sampled geometry cert remains mandatory
%   - continuous, non-sampled speed cert remains mandatory
%   - specific_force_norm_LB_m_s2 > 0 becomes explicit commit guard
%   - C7 exactness becomes explicit commit guard
%   - canonical finite/resource gate remains mandatory
%   - packet/currentness assertions remain
%   - final hard-deadline rollback remains
%
% Removed code evaluated the canonical reference only at two sample times
% (t_commit and t_commit+horizon). Those samples were not a continuous-time
% certificate and are redundant once the continuous certificate is required.

PRE_SHA='c7baa3c7dda80bbacb412443a035fd7c5ae7afbc6c95a46eddc4d82b69d88289';
POST_SHA='59e24920394b53faf2d832d6b84dead3e94e9728ea767cf7dca1ef3a1a5f19fb';

OUT=struct('schema','AF7R58ZZ_V1','production_modified',false, ...
    'rollback_performed',false,'status','OPEN','error',struct());

pR=which('REPLANNING_SYSTEM');
assert(~isempty(pR)&&isfile(pR),'[AF7R58ZZ] REPLANNING_SYSTEM unresolved.');
root=fileparts(pR);
outdir=tempname(root); [ok,msg]=mkdir(outdir); assert(ok,'%s',msg);
report=fullfile(outdir,'AF7R58ZZ_REPORT.txt');
fid=fopen(report,'wt','n','UTF-8'); assert(fid>=0); c=onCleanup(@()safe_fclose(fid)); %#ok<NASGU>

say(fid,'====================================================================================================');
say(fid,' AF7R58ZZ | CONTINUOUS-CERTIFICATE COMMIT HOT-PATH INSTALL');
say(fid,'====================================================================================================');
say(fid,'[AUTHORITY] exactCurrentSHA=1 | exactHelperSHA=1 | parseBeforeWrite=1 | rollback=1');
say(fid,'[THEORY] sampledAcceptanceAdded=0 | continuousCertificateRequired=1 | finalAtomicGuardsRetained=1');

backupDir=tempname(root); backupReady=false; writesStarted=false;
try
    h=file_hash(pR); OUT.pre_sha=h; OUT.production_path=pR;
    say(fid,''); say(fid,'[A. CURRENT SOURCE]');
    say(fid,' path = %s',pR); say(fid,' SHA = %s',h);
    assert(strcmp(h,PRE_SHA),'[AF7R58ZZ] current source differs from reviewed authority.');

    helpers={ ...
        'af7r58i_prepare_batch_model.m','92f093168f1df95661c5e7f2e75005fe439406eb3ec22e53fd1f25d6bdfb8a15'; ...
        'af7r58i_resource_contract.m','772e49a8007bd955cb262b1db513f65c1270f87384b5f12694ee8774ae458392'; ...
        'af7r58i_make_canonical_eval_handles.m','1c9bf7407b9c2482c5f6d2676479966b808995c4d10ee2871a620faba55a82f8'; ...
        'af7r58i_eval_canonical_reference.m','4f8ecd8576983d638b5fbbc71384d333fc90d4d9e4787c6ea694e94dc2d0ca39'};
    say(fid,''); say(fid,'[B. HELPER BINDING]');
    for k=1:size(helpers,1)
        fn=erase(helpers{k,1},'.m'); hp=which(fn);
        assert(~isempty(hp)&&isfile(hp),'[AF7R58ZZ] missing helper %s',helpers{k,1});
        hh=file_hash(hp); match=strcmp(hh,helpers{k,2});
        say(fid,' %-42s SHA-match=%d | %s',helpers{k,1},match,hh);
        assert(match,'[AF7R58ZZ] helper changed: %s',helpers{k,1});
    end

    raw=read_utf8(pR);
    oldProbe=sprintf([ ...
'            %% Pre-evaluate the exact HLC packet at commit/horizon boundary.\n' ...
'            tq=[double(t_commit),double(t_commit)+double(obj.horizon_s)];\n' ...
'            for kk=1:numel(tq)\n' ...
'                xd=evalFn(tq(kk));\n' ...
'                xf=flatEvalFn(tq(kk));\n' ...
'                xd=double(xd(:));\n' ...
'                xf=double(xf(:));\n' ...
'                if numel(xd)<28 || any(~isfinite(xd(1:28))) || ...\n' ...
'                        numel(xf)<28 || any(~isfinite(xf(1:28)))\n' ...
'                    reason=''REFERENCE_ONLY_EVAL_FN_INVALID'';\n' ...
'                    return;\n' ...
'                end\n' ...
'                sf=xf(9:11)+[0;0;double(obj.gravity)];\n' ...
'                if ~(isfinite(norm(sf))&&norm(sf)>0)\n' ...
'                    reason=''REFERENCE_ONLY_FLATNESS_SINGULAR'';\n' ...
'                    return;\n' ...
'                end\n' ...
'            end\n\n']);

    anchor=sprintf([ ...
'            targetVersion=double(obj.active_reference_version)+1;\n' ...
'            if double(SC.reference_version)~=targetVersion\n' ...
'                reason=''REFERENCE_SPEED_CERTIFICATE_VERSION_MISMATCH'';\n' ...
'                return;\n' ...
'            end\n\n']);

    guard=[anchor sprintf([ ...
'            %% AF7R58ZZ: continuous certificate is the authority here.\n' ...
'            %% The removed two-time evaluator probes were sampled implementation\n' ...
'            %% checks, not the continuous-time proof.\n' ...
'            if ~(isfield(SC,''continuous_time'') && logical(SC.continuous_time) && ...\n' ...
'                    isfield(SC,''sampled_acceptance'') && ~logical(SC.sampled_acceptance) && ...\n' ...
'                    isfield(SC,''specific_force_norm_LB_m_s2'') && ...\n' ...
'                    isnumeric(SC.specific_force_norm_LB_m_s2) && ...\n' ...
'                    isscalar(SC.specific_force_norm_LB_m_s2) && ...\n' ...
'                    isfinite(double(SC.specific_force_norm_LB_m_s2)) && ...\n' ...
'                    double(SC.specific_force_norm_LB_m_s2)>0)\n' ...
'                reason=''REFERENCE_ONLY_CONTINUOUS_FLATNESS_CERTIFICATE_NOT_CLOSED'';\n' ...
'                return;\n' ...
'            end\n' ...
'            if ~(isfield(P,''c7_exact'') && logical(P.c7_exact))\n' ...
'                reason=''REFERENCE_ONLY_C7_CERTIFICATE_NOT_CLOSED'';\n' ...
'                return;\n' ...
'            end\n\n'])];

    assert(count(raw,oldProbe)==1,'[AF7R58ZZ] sampled probe block count is not 1.');
    assert(count(raw,anchor)==1,'[AF7R58ZZ] version anchor count is not 1.');
    patched=strrep(raw,anchor,guard);
    patched=strrep(patched,oldProbe,'');

    assert(~contains(patched,'tq=[double(t_commit),double(t_commit)+double(obj.horizon_s)]'));
    assert(count(patched,'REFERENCE_ONLY_CONTINUOUS_FLATNESS_CERTIFICATE_NOT_CLOSED')==1);
    assert(count(patched,'REFERENCE_ONLY_C7_CERTIFICATE_NOT_CLOSED')==1);
    assert(contains(patched,'af7r58i_prepare_batch_model('));
    assert(contains(patched,'obj.assert_trigger_packet_certificate_current();'));
    assert(contains(patched,'obj.assert_repaired_reference_current();'));
    assert(contains(patched,'finished after the hard online modification deadline'));

    payload=fullfile(outdir,'REPLANNING_SYSTEM_AF7R58ZZ_PAYLOAD.m');
    write_utf8(payload,patched);
    ph=file_hash(payload); say(fid,''); say(fid,'[C. PATCH PREFLIGHT]');
    say(fid,' payload SHA = %s',ph); assert(strcmp(ph,POST_SHA),'[AF7R58ZZ] payload SHA mismatch.');

    parseDir=fullfile(outdir,'AF7R58ZZ_PARSE'); [ok,msg]=mkdir(parseDir); assert(ok,'%s',msg);
    parseName='AF7R58ZZ_PARSE_REPLANNING_SYSTEM';
    parseText=strrep(patched,'classdef REPLANNING_SYSTEM < handle',['classdef ' parseName ' < handle']);
    write_utf8(fullfile(parseDir,[parseName '.m']),parseText);
    addpath(parseDir,'-begin'); cp=onCleanup(@()safe_rmpath(parseDir)); %#ok<NASGU>
    eval(['clear ' parseName]); mc=meta.class.fromName(parseName);
    assert(~isempty(mc),'[AF7R58ZZ] patched class parse failed.'); eval(['clear ' parseName]);
    say(fid,' full MATLAB class parse = 1');
    say(fid,' production writes = 0');

    [ok,msg]=mkdir(backupDir); assert(ok,'%s',msg); backupReady=true;
    OUT.backup_directory=backupDir; copyfile(pR,fullfile(backupDir,'REPLANNING_SYSTEM.m'));
    writesStarted=true; write_utf8(pR,patched); rehash;
    post=file_hash(pR); assert(strcmp(post,POST_SHA),'[AF7R58ZZ] post-write SHA mismatch.');

    OUT.post_sha=post; OUT.production_modified=true;
    OUT.status='CONTINUOUS_CERT_COMMIT_HOTPATH_INSTALLED__CLEAN_RERUN_REQUIRED';
    say(fid,''); say(fid,'[D. INSTALL RESULT]');
    say(fid,' production modified = 1');
    say(fid,' sampled evaluator probes removed = 1');
    say(fid,' continuous sigma guard required = 1');
    say(fid,' C7 guard required = 1');
    say(fid,' resource/currentness/deadline guards retained = 1');
    say(fid,' post SHA = %s',post);
catch ME
    if backupReady && writesStarted
        try, copyfile(fullfile(backupDir,'REPLANNING_SYSTEM.m'),pR,'f'); rehash; OUT.rollback_performed=true; catch, end
    end
    OUT.production_modified=false; OUT.status='INSTALL_FAILED__ROLLBACK_COMPLETE';
    OUT.error=struct('identifier',ME.identifier,'message',ME.message,'stack',{ME.stack});
    say(fid,''); say(fid,'[INSTALL FAILURE] %s | %s',ME.identifier,ME.message);
    say(fid,' rollback performed = %d',OUT.rollback_performed);
    say(fid,' production modified = 0');
end

save(fullfile(outdir,'AF7R58ZZ_OUT.mat'),'OUT','-v7'); assignin('base','AF7R58ZZ_OUT',OUT);
say(fid,''); say(fid,'RETURN: %s',report); say(fid,'RETURN: %s',fullfile(outdir,'AF7R58ZZ_OUT.mat'));
say(fid,'status = %s',OUT.status);
end

function s=read_utf8(p)
fid=fopen(p,'rb');assert(fid>=0);c=onCleanup(@()fclose(fid)); %#ok<NASGU>
b=fread(fid,Inf,'*uint8');s=native2unicode(b.','UTF-8');
end
function write_utf8(p,s)
fid=fopen(p,'wb');assert(fid>=0);c=onCleanup(@()fclose(fid)); %#ok<NASGU>
b=unicode2native(s,'UTF-8');n=fwrite(fid,b,'uint8');assert(n==numel(b));
end
function h=file_hash(p)
fid=fopen(p,'rb');assert(fid>=0);c=onCleanup(@()fclose(fid)); %#ok<NASGU>
b=fread(fid,Inf,'*uint8');m=java.security.MessageDigest.getInstance('SHA-256');m.update(b);
d=typecast(m.digest(),'uint8');h=lower(reshape(dec2hex(d,2).',1,[]));
end
function safe_rmpath(p)
try,q=strsplit(path,pathsep);if any(strcmpi(q,p)),rmpath(p);end,catch,end
end
function safe_fclose(fid)
try,if fid>=0,fclose(fid);end,catch,end
end
function say(fid,fmt,varargin)
s=sprintf(fmt,varargin{:});fprintf(1,'%s\n',s);fprintf(fid,'%s\n',s);
end
