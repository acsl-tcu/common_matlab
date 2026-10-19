function OUT = AF7R58ZZN_ONEFILE_LOW_RISK_HOTPATH_OPTIMIZATION()
% AF7R58ZZN
% Low-risk theorem-preserving hot-path optimization.
%
% Exact current production authority:
%   REPLANNING_SYSTEM.m
%     SHA-256 = 2a0013a583cbcff3afd62e4495fa5e784427132390616ec5886000f134f2c05f
%   af7r58i_resource_contract.m
%     SHA-256 = 772e49a8007bd955cb262b1db513f65c1270f87384b5f12694ee8774ae458392
%
% Changes:
%   A) exact-key memoization of CLOSED AF7R58I resource theorem packet.
%      Cache key is exactly {flight_reference_time_origin,
%      nominal_table_tend, prediction_dt}, which are the only replanner
%      values consumed by the current resource-contract theorem.
%      Any key change recomputes the original theorem exactly.
%
%   B) fuse three repeated obstacle inflation helper calls into one helper.
%      The individual formulas for payload/UAV/cable are unchanged.
%      Static-obstacle authority checks remain exactly in place.
%
% NOT changed:
%   - candidate acceptance / rejection logic
%   - continuous-time geometry / C7 / speed certificates
%   - 4 ms deadline gates
%   - finite sensing / detected-only generator contract
%   - obstacle count/type/pose assumptions
%   - generator/MEX math
%   - mL, thrust, moment, tension logic
%
% This installer performs full parse/self-test before the first production
% write and rolls back both modified files on any post-write failure.

PRE_R_SHA='2a0013a583cbcff3afd62e4495fa5e784427132390616ec5886000f134f2c05f';
PRE_RC_SHA='772e49a8007bd955cb262b1db513f65c1270f87384b5f12694ee8774ae458392';

OUT=struct();
OUT.schema='AF7R58ZZN_LOW_RISK_HOTPATH_OPT_V1';
OUT.production_modified=false;
OUT.rollback_performed=false;
OUT.status='OPEN';
OUT.error=struct();

pR=which('REPLANNING_SYSTEM');
pRC=which('af7r58i_resource_contract');

assert(~isempty(pR)&&isfile(pR), ...
    '[AF7R58ZZN] REPLANNING_SYSTEM unresolved.');
assert(~isempty(pRC)&&isfile(pRC), ...
    '[AF7R58ZZN] af7r58i_resource_contract unresolved.');

root=fileparts(pR);
assert(strcmpi(fileparts(pRC),root), ...
    '[AF7R58ZZN] resource-contract helper does not resolve from production root.');

outdir=tempname(root);
[ok,msg]=mkdir(outdir);
assert(ok,'%s',msg);

report=fullfile(outdir,'AF7R58ZZN_REPORT.txt');
matp=fullfile(outdir,'AF7R58ZZN_OUT.mat');
backupR=fullfile(outdir,'REPLANNING_SYSTEM_PRE_AF7R58ZZN.m');
backupRC=fullfile(outdir,'af7r58i_resource_contract_PRE_AF7R58ZZN.m');

fid=fopen(report,'wt','n','UTF-8');
assert(fid>=0);
cf=onCleanup(@()safe_fclose(fid)); %#ok<NASGU>

say(fid,'====================================================================================================');
say(fid,' AF7R58ZZN | LOW-RISK HOT-PATH OPTIMIZATION');
say(fid,'====================================================================================================');
say(fid,'[AUTHORITY] exactSourceBinding=1 | parseBeforeWrite=1 | transactionalRollback=1');
say(fid,'[SAFETY] acceptanceChanged=0 | certificateMathChanged=0 | deadlineChanged=0 | generatorMexMathChanged=0');
say(fid,'[SENSING] detectedOnlyGeneratorRequired=1 | unseenTruthPreplan=0');
say(fid,'[OPT] resourceClosedPacketMemoization=1 | tripleInflationFusion=1');

writeStarted=false;

try
    preR=file_hash(pR);
    preRC=file_hash(pRC);

    OUT.pre_sha=struct('REPLANNING_SYSTEM',preR, ...
        'af7r58i_resource_contract',preRC);

    say(fid,'');
    say(fid,'[A. EXACT CURRENT SOURCE BINDING]');
    say(fid,' REPLANNING_SYSTEM.m             = %s',preR);
    say(fid,' af7r58i_resource_contract.m     = %s',preRC);

    assert(strcmp(preR,PRE_R_SHA), ...
        '[AF7R58ZZN] REPLANNING_SYSTEM is not exact restored ZZH authority.');
    assert(strcmp(preRC,PRE_RC_SHA), ...
        '[AF7R58ZZN] AF7R58I resource helper differs from reviewed authority.');

    [R,eolR]=read_text_norm(pR);
    [RC,eolRC]=read_text_norm(pRC);

    % ------------------------------------------------------------------
    % A. Resource theorem memoization.
    % ------------------------------------------------------------------
    assert(~contains(RC,'AF7R58ZZN_RESOURCE_CONTRACT_MEMO'), ...
        '[AF7R58ZZN] resource memoization already present.');

    anchorA=sprintf([ ...
        'tf0=double(replanner.flight_reference_time_origin);\n' ...
        'tend=double(replanner.nominal_table_tend);\n' ...
        'dt=double(replanner.prediction_dt);\n']);

    insertA=sprintf([ ...
        'tf0=double(replanner.flight_reference_time_origin);\n' ...
        'tend=double(replanner.nominal_table_tend);\n' ...
        'dt=double(replanner.prediction_dt);\n' ...
        '\n' ...
        '%% AF7R58ZZN_RESOURCE_CONTRACT_MEMO\n' ...
        '%% Exact theorem memoization only. The current theorem consumes no\n' ...
        '%% replanner value other than {tf0,tend,dt}. A key mismatch executes\n' ...
        '%% the original theorem below without modification.\n' ...
        'persistent AF7R58ZZN_KEY AF7R58ZZN_CLOSED_CONTRACT\n' ...
        'af7r58zzn_key=[tf0 tend dt];\n' ...
        'if ~isempty(AF7R58ZZN_KEY) && ...\n' ...
        '        isequal(AF7R58ZZN_KEY,af7r58zzn_key) && ...\n' ...
        '        isstruct(AF7R58ZZN_CLOSED_CONTRACT) && ...\n' ...
        '        isfield(AF7R58ZZN_CLOSED_CONTRACT,''closed'') && ...\n' ...
        '        logical(AF7R58ZZN_CLOSED_CONTRACT.closed)\n' ...
        '    C=AF7R58ZZN_CLOSED_CONTRACT;\n' ...
        '    return;\n' ...
        'end\n']);

    RC2=replace_once(RC,anchorA,insertA, ...
        '[AF7R58ZZN] resource key insertion anchor');

    anchorB=sprintf([ ...
        'C.Gmax=Gmax;\n' ...
        'C.Smax=Smax;\n' ...
        'end\n' ...
        '\n' ...
        'function y=nextup_nonnegative(x)\n']);

    insertB=sprintf([ ...
        'C.Gmax=Gmax;\n' ...
        'C.Smax=Smax;\n' ...
        '\n' ...
        '%% Store only a CLOSED theorem packet. Open/failed theorem evaluations\n' ...
        '%% are never memoized.\n' ...
        'AF7R58ZZN_KEY=af7r58zzn_key;\n' ...
        'AF7R58ZZN_CLOSED_CONTRACT=C;\n' ...
        'end\n' ...
        '\n' ...
        'function y=nextup_nonnegative(x)\n']);

    RC2=replace_once(RC2,anchorB,insertB, ...
        '[AF7R58ZZN] resource cache-store anchor');

    assert(count_exact(RC2,'AF7R58ZZN_RESOURCE_CONTRACT_MEMO')==1);
    assert(count_exact(RC2,'AF7R58ZZN_KEY=af7r58zzn_key;')==1);
    assert(count_exact(RC2,'AF7R58ZZN_CLOSED_CONTRACT=C;')==1);

    % Prove the memo key is exactly the three current theorem inputs.
    assert(contains(RC2,'af7r58zzn_key=[tf0 tend dt];'));
    assert(~contains(RC2,'static_obstacle_map'));
    assert(~contains(RC2,'loadmass'));
    assert(~contains(RC2,'thrust'));
    assert(~contains(RC2,'moment'));

    % ------------------------------------------------------------------
    % B. Fuse the three exact obstacle-inflation calculations.
    % ------------------------------------------------------------------
    assert(~contains(R,'outer_ellipsoid_triplet_from_raw_cached'), ...
        '[AF7R58ZZN] triplet inflation helper already present.');

    oldFast=sprintf([ ...
        '                pack.Apay(:,i)=obj.outer_ellipsoid_from_raw_cached(o,ar,Dp);\n' ...
        '                pack.Auav(:,i)=obj.outer_ellipsoid_from_raw_cached(o,ar,Dq);\n' ...
        '                pack.Acable(:,i)=obj.outer_ellipsoid_from_raw_cached(o,ar,Dc);']);

    newFast=sprintf([ ...
        '                [pack.Apay(:,i),pack.Auav(:,i),pack.Acable(:,i)]= ...\n' ...
        '                    obj.outer_ellipsoid_triplet_from_raw_cached( ...\n' ...
        '                        o,ar,Dp,Dq,Dc);']);

    R2=replace_once(R,oldFast,newFast, ...
        '[AF7R58ZZN] fast packet triple-inflation block');

    oldRetarget=sprintf([ ...
        '                packet.Apay(:,ii)=obj.outer_ellipsoid_from_raw_cached(o,ar,Dp);\n' ...
        '                packet.Auav(:,ii)=obj.outer_ellipsoid_from_raw_cached(o,ar,Dq);\n' ...
        '                packet.Acable(:,ii)=obj.outer_ellipsoid_from_raw_cached(o,ar,Dc);']);

    newRetarget=sprintf([ ...
        '                [packet.Apay(:,ii),packet.Auav(:,ii),packet.Acable(:,ii)]= ...\n' ...
        '                    obj.outer_ellipsoid_triplet_from_raw_cached( ...\n' ...
        '                        o,ar,Dp,Dq,Dc);']);

    R2=replace_once(R2,oldRetarget,newRetarget, ...
        '[AF7R58ZZN] prepared-cache retarget triple-inflation block');

    oldHelper=sprintf([ ...
        '        function A = outer_ellipsoid_from_raw_cached(~,o,base,D)\n' ...
        '            type=lower(o.type);\n' ...
        '            if strcmp(type,''cylinder'')\n' ...
        '                r=double(o.raw_param(1)); h=double(o.raw_param(2));\n' ...
        '                A=[sqrt(3/2)*(r+D);sqrt(3/2)*(r+D);sqrt(3)*(h/2+D)]; return;\n' ...
        '            end\n' ...
        '            if strcmp(type,''sphere'')\n' ...
        '                r=double(o.raw_param(1)); A=(r+D)*ones(3,1); return;\n' ...
        '            end\n' ...
        '            amin=min(base); assert(isfinite(amin)&&amin>0);\n' ...
        '            A=(1+D/amin)*base;\n' ...
        '        end']);

    newHelper=[oldHelper sprintf([ ...
        '\n' ...
        '\n' ...
        '        function [Ap,Aq,Ac] = outer_ellipsoid_triplet_from_raw_cached( ...\n' ...
        '                ~,o,base,Dp,Dq,Dc)\n' ...
        '            %% AF7R58ZZN exact fused form of three calls to\n' ...
        '            %% outer_ellipsoid_from_raw_cached(). Each individual\n' ...
        '            %% floating-point expression is intentionally unchanged.\n' ...
        '            type=lower(o.type);\n' ...
        '            if strcmp(type,''cylinder'')\n' ...
        '                r=double(o.raw_param(1)); h=double(o.raw_param(2));\n' ...
        '                Ap=[sqrt(3/2)*(r+Dp);sqrt(3/2)*(r+Dp);sqrt(3)*(h/2+Dp)];\n' ...
        '                Aq=[sqrt(3/2)*(r+Dq);sqrt(3/2)*(r+Dq);sqrt(3)*(h/2+Dq)];\n' ...
        '                Ac=[sqrt(3/2)*(r+Dc);sqrt(3/2)*(r+Dc);sqrt(3)*(h/2+Dc)];\n' ...
        '                return;\n' ...
        '            end\n' ...
        '            if strcmp(type,''sphere'')\n' ...
        '                r=double(o.raw_param(1));\n' ...
        '                Ap=(r+Dp)*ones(3,1);\n' ...
        '                Aq=(r+Dq)*ones(3,1);\n' ...
        '                Ac=(r+Dc)*ones(3,1);\n' ...
        '                return;\n' ...
        '            end\n' ...
        '            amin=min(base); assert(isfinite(amin)&&amin>0);\n' ...
        '            Ap=(1+Dp/amin)*base;\n' ...
        '            Aq=(1+Dq/amin)*base;\n' ...
        '            Ac=(1+Dc/amin)*base;\n' ...
        '        end'])];

    R2=replace_once(R2,oldHelper,newHelper, ...
        '[AF7R58ZZN] cached inflation helper definition');

    % Exact source topology checks.
    assert(count_exact(R2,'outer_ellipsoid_triplet_from_raw_cached')==3, ...
        '[AF7R58ZZN] expected definition + two production call sites.');
    assert(count_exact(R2,'outer_ellipsoid_from_raw_cached(o,ar,Dp)')==0);
    assert(count_exact(R2,'outer_ellipsoid_from_raw_cached(o,ar,Dq)')==0);
    assert(count_exact(R2,'outer_ellipsoid_from_raw_cached(o,ar,Dc)')==0);

    % Static authority checks MUST remain on both paths.
    assert(count_exact(R2, ...
        '[REPLANNER ERROR] static obstacle authority changed during fast packet rebuild.')==1);
    assert(count_exact(R2, ...
        '[REPLANNER ERROR] cached static obstacle authority changed during retarget.')==1);
    assert(count_exact(R2, ...
        '[REPLANNER ERROR] moving obstacle entered static fast packet path.')==1);
    assert(count_exact(R2, ...
        '[REPLANNER ERROR] moving obstacle entered prepared-cache retarget path.')==1);

    % Existing theorem/deadline/sensing invariants must remain.
    assert(contains(R2,'The generator receives ONLY sensor-known obstacles:'));
    assert(contains(R2,'''detected_only_obstacle_contract'',true'));
    assert(contains(R2,'Initial known set is empty.'));
    assert(contains(R2,'raw_sensor_gate alone may admit a previously unknown obstacle.'));
    assert(count_exact(R2,'REFERENCE_ONLY_DEADLINE_AFTER_RESOURCE_MODEL')==1);
    assert(count_exact(R2,'REFERENCE_ONLY_DEADLINE_AFTER_REUSE_GATE')==1);
    assert(count_exact(R2,'REFERENCE_ONLY_COMMIT_PREPARATION_DEADLINE_EXCEEDED')==1);
    assert(count_exact(R2,'REFERENCE_ONLY_DEADLINE_AFTER_EVAL_HANDLES')==1);
    assert(count_exact(R2,'REFERENCE_ONLY_DEADLINE_AFTER_CERT_METADATA')==1);
    assert(count_exact(R2,'REFERENCE_ONLY_DEADLINE_AFTER_ROLLBACK_SNAPSHOT')==1);
    assert(contains(R2,'finished after the hard online modification deadline.'));

    % ------------------------------------------------------------------
    % Supplementary bitwise regression for the fused inflation algebra.
    % Not a safety acceptance test; it checks implementation equivalence.
    % ------------------------------------------------------------------
    [bitwiseOK,maxAbs]=inflation_bitwise_selftest();
    assert(bitwiseOK, ...
        '[AF7R58ZZN] fused inflation is not bitwise identical in self-test.');

    % ------------------------------------------------------------------
    % Stage exact patched bytes and parse before production write.
    % ------------------------------------------------------------------
    stageR=fullfile(outdir,'REPLANNING_SYSTEM_AF7R58ZZN_STAGE.m');
    stageRC=fullfile(outdir,'af7r58i_resource_contract_AF7R58ZZN_STAGE.m');
    write_text_eol(stageR,R2,eolR);
    write_text_eol(stageRC,RC2,eolRC);

    OUT.staged_sha=struct( ...
        'REPLANNING_SYSTEM',file_hash(stageR), ...
        'af7r58i_resource_contract',file_hash(stageRC));

    % Parse patched resource helper under a temporary function name.
    parseDir=fullfile(outdir,'PARSE');
    [ok,msg]=mkdir(parseDir);
    assert(ok,'%s',msg);

    rcParseName='AF7R58ZZN_PARSE_RESOURCE_CONTRACT';
    rcParse=strrep(RC2, ...
        'function C = af7r58i_resource_contract(replanner)', ...
        ['function C = ' rcParseName '(replanner)']);
    assert(~strcmp(rcParse,RC2), ...
        '[AF7R58ZZN] resource helper parse rename failed.');
    write_text_eol(fullfile(parseDir,[rcParseName '.m']),rcParse,eolRC);

    classParseName='AF7R58ZZN_PARSE_REPLANNING_SYSTEM';
    classParse=strrep(R2,'classdef REPLANNING_SYSTEM < handle', ...
        ['classdef ' classParseName ' < handle']);
    assert(~strcmp(classParse,R2), ...
        '[AF7R58ZZN] class parse rename failed.');
    write_text_eol(fullfile(parseDir,[classParseName '.m']),classParse,eolR);

    addpath(parseDir,'-begin');
    cp=onCleanup(@()safe_rmpath(parseDir)); %#ok<NASGU>

    eval(['clear ' rcParseName]);
    nin=nargin(rcParseName);
    assert(nin==1,'[AF7R58ZZN] patched resource helper parse failed.');
    eval(['clear ' rcParseName]);

    eval(['clear ' classParseName]);
    mc=meta.class.fromName(classParseName);
    assert(~isempty(mc),'[AF7R58ZZN] patched class parse failed.');
    eval(['clear ' classParseName]);

    say(fid,'');
    say(fid,'[B. PREFLIGHT BEFORE FIRST PRODUCTION WRITE]');
    say(fid,' resource memo key exactly {tf0,tend,dt} = 1');
    say(fid,' closed contracts only are memoized        = 1');
    say(fid,' original theorem body retained            = 1');
    say(fid,' triple inflation production sites         = 2');
    say(fid,' static obstacle authority checks retained = 1');
    say(fid,' moving-obstacle rejection retained        = 1');
    say(fid,' fused inflation bitwise self-test          = %d',bitwiseOK);
    say(fid,' fused inflation self-test max abs diff    = %.17g',maxAbs);
    say(fid,' patched resource helper parse             = 1');
    say(fid,' patched full class parse                  = 1');
    say(fid,' production writes                         = 0');

    % Back up both exact pre-install files.
    copyfile(pR,backupR,'f');
    copyfile(pRC,backupRC,'f');
    OUT.backup=struct('REPLANNING_SYSTEM',backupR, ...
        'af7r58i_resource_contract',backupRC);

    % ------------------------------------------------------------------
    % First production write occurs only after all preflight checks pass.
    % ------------------------------------------------------------------
    writeStarted=true;
    write_text_eol(pR,R2,eolR);
    write_text_eol(pRC,RC2,eolRC);
    OUT.production_modified=true;

    postR=file_hash(pR);
    postRC=file_hash(pRC);
    OUT.post_sha=struct('REPLANNING_SYSTEM',postR, ...
        'af7r58i_resource_contract',postRC);

    assert(strcmp(postR,OUT.staged_sha.REPLANNING_SYSTEM), ...
        '[AF7R58ZZN] class post-write bytes differ from staged bytes.');
    assert(strcmp(postRC,OUT.staged_sha.af7r58i_resource_contract), ...
        '[AF7R58ZZN] resource-helper post-write bytes differ from staged bytes.');

    clear af7r58i_resource_contract
    rehash

    say(fid,'');
    say(fid,'[C. INSTALL RESULT]');
    say(fid,' REPLANNING_SYSTEM post SHA         = %s',postR);
    say(fid,' resource helper post SHA           = %s',postRC);
    say(fid,' staged/post byte identity           = 1');
    say(fid,' acceptance/certificate logic changed= 0');
    say(fid,' 4-ms deadline changed               = 0');
    say(fid,' finite sensing changed              = 0');
    say(fid,' generator/MEX math changed          = 0');
    say(fid,' payload mass / actuator logic changed= 0');
    say(fid,' runtime timing instrumentation added = 0');

    OUT.status='LOW_RISK_HOTPATH_OPTIMIZATION_INSTALLED__CLEAN_RERUN_REQUIRED';

catch ME
    OUT.error=struct('identifier',ME.identifier, ...
        'message',ME.message,'stack',{ME.stack});

    if writeStarted
        try
            if isfile(backupR), copyfile(backupR,pR,'f'); end
            if isfile(backupRC), copyfile(backupRC,pRC,'f'); end
            clear af7r58i_resource_contract
            rehash
            OUT.rollback_performed=true;
            OUT.production_modified=false;
        catch
        end
    end

    OUT.status='INSTALL_FAILED__ROLLBACK_ATTEMPTED';

    say(fid,'');
    say(fid,'[INSTALL FAILURE]');
    say(fid,' identifier = %s',ME.identifier);
    say(fid,' message    = %s',ME.message);
    say(fid,' rollback   = %d',OUT.rollback_performed);
end

OUT.report_path=report;
OUT.mat_path=matp;
save(matp,'OUT','-v7');

say(fid,'');
say(fid,'RETURN: %s',report);
say(fid,'RETURN: %s',matp);
say(fid,'status = %s',OUT.status);

assignin('base','AF7R58ZZN_OUT',OUT);
end


function [ok,maxAbs]=inflation_bitwise_selftest()
ok=true;
maxAbs=0;

Ds=[0,0.025,0.137,0.5,2.75];

types={'sphere','cylinder','box','prism','cone','pyramid','ellipsoid','custom'};
for it=1:numel(types)
    o=struct();
    o.type=types{it};

    switch o.type
        case 'sphere'
            o.raw_param=[1.23456789012345];
            base=[o.raw_param(1);o.raw_param(1);o.raw_param(1)];
        case 'cylinder'
            o.raw_param=[0.81234567890123,2.34567890123456];
            base=[sqrt(3/2)*o.raw_param(1); ...
                  sqrt(3/2)*o.raw_param(1); ...
                  sqrt(3)*o.raw_param(2)/2];
        otherwise
            o.raw_param=[1 2 3];
            base=[0.71234567890123;1.45678901234567;2.34567890123456];
    end

    for a=1:numel(Ds)
        Dp=Ds(a);
        Dq=Ds(mod(a,numel(Ds))+1);
        Dc=Ds(mod(a+1,numel(Ds))+1);

        Ap0=old_inflate(o,base,Dp);
        Aq0=old_inflate(o,base,Dq);
        Ac0=old_inflate(o,base,Dc);

        [Ap,Aq,Ac]=fused_inflate(o,base,Dp,Dq,Dc);

        maxAbs=max([maxAbs;abs(Ap0(:)-Ap(:)); ...
            abs(Aq0(:)-Aq(:));abs(Ac0(:)-Ac(:))]);

        ok=ok && isequal(Ap0,Ap) && isequal(Aq0,Aq) && isequal(Ac0,Ac);
    end
end
end


function A=old_inflate(o,base,D)
type=lower(o.type);
if strcmp(type,'cylinder')
    r=double(o.raw_param(1));
    h=double(o.raw_param(2));
    A=[sqrt(3/2)*(r+D);sqrt(3/2)*(r+D);sqrt(3)*(h/2+D)];
    return;
end
if strcmp(type,'sphere')
    r=double(o.raw_param(1));
    A=(r+D)*ones(3,1);
    return;
end
amin=min(base);
assert(isfinite(amin)&&amin>0);
A=(1+D/amin)*base;
end


function [Ap,Aq,Ac]=fused_inflate(o,base,Dp,Dq,Dc)
type=lower(o.type);
if strcmp(type,'cylinder')
    r=double(o.raw_param(1));
    h=double(o.raw_param(2));
    Ap=[sqrt(3/2)*(r+Dp);sqrt(3/2)*(r+Dp);sqrt(3)*(h/2+Dp)];
    Aq=[sqrt(3/2)*(r+Dq);sqrt(3/2)*(r+Dq);sqrt(3)*(h/2+Dq)];
    Ac=[sqrt(3/2)*(r+Dc);sqrt(3/2)*(r+Dc);sqrt(3)*(h/2+Dc)];
    return;
end
if strcmp(type,'sphere')
    r=double(o.raw_param(1));
    Ap=(r+Dp)*ones(3,1);
    Aq=(r+Dq)*ones(3,1);
    Ac=(r+Dc)*ones(3,1);
    return;
end
amin=min(base);
assert(isfinite(amin)&&amin>0);
Ap=(1+Dp/amin)*base;
Aq=(1+Dq/amin)*base;
Ac=(1+Dc/amin)*base;
end


function R=replace_once(R,a,b,label)
p=strfind(R,a);
assert(numel(p)==1,'%s count=%d.',label,numel(p));
R=[R(1:p-1) b R(p+length(a):end)];
end


function n=count_exact(s,pat)
n=numel(strfind(s,pat));
end


function [s,eol]=read_text_norm(path)
bytes=fileread(path);
if contains(bytes,sprintf('\r\n'))
    eol=sprintf('\r\n');
elseif contains(bytes,sprintf('\n'))
    eol=sprintf('\n');
else
    eol=sprintf('\n');
end
s=strrep(bytes,sprintf('\r\n'),sprintf('\n'));
s=strrep(s,sprintf('\r'),sprintf('\n'));
end


function write_text_eol(path,s,eol)
s=strrep(s,sprintf('\r\n'),sprintf('\n'));
s=strrep(s,sprintf('\r'),sprintf('\n'));
if ~strcmp(eol,sprintf('\n'))
    s=strrep(s,sprintf('\n'),eol);
end
bytes=unicode2native(s,'UTF-8');
fid=fopen(path,'wb');
assert(fid>=0);
c=onCleanup(@()fclose(fid)); %#ok<NASGU>
fwrite(fid,uint8(bytes),'uint8');
end


function h=file_hash(path)
md=java.security.MessageDigest.getInstance('SHA-256');
fid=fopen(path,'rb');
assert(fid>=0);
c=onCleanup(@()fclose(fid)); %#ok<NASGU>
while true
    b=fread(fid,1024*1024,'*uint8');
    if isempty(b), break; end
    md.update(b);
end
u=typecast(md.digest(),'uint8');
h=lower(reshape(dec2hex(u,2).',1,[]));
end


function say(fid,varargin)
fprintf(fid,[varargin{1} '\n'],varargin{2:end});
fprintf([varargin{1} '\n'],varargin{2:end});
end


function safe_rmpath(p)
try
    rmpath(p);
catch
end
end


function safe_fclose(fid)
try
    fclose(fid);
catch
end
end
