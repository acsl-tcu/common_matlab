function OUT = AF7R57B_AUTHORITY_RECOVERY_AUDIT()
% AF7R57B_AUTHORITY_RECOVERY_AUDIT
% -----------------------------------------------------------------------------
% Read-only recovery audit for the AF7R49B -> AF7R50A -> AF7R56B authority
% chain after the live replanner no longer exposes the original true trigger.
%
% This audit DOES NOT:
%   - rerun flight,
%   - synthesize B49/R50A,
%   - modify production source,
%   - select production t1,
%   - accept sampled evidence.
%
% It DOES:
%   1) inspect current base-workspace authority objects;
%   2) inspect the exact AF7R50_THEORY_SNAPSHOT.mat if present;
%   3) recursively locate schema-identifiable B49 / R50A / AF7R56B records;
%   4) inspect AF7R56B source dependencies by line number;
%   5) report the safest recovery path without guessing.
% -----------------------------------------------------------------------------

clc;
fprintf('\n====================================================================================================\n');
fprintf(' AF7R57B | SAME-RUN AUTHORITY RECOVERY AUDIT\n');
fprintf('====================================================================================================\n');
fprintf('[AUTHORITY] readOnly=1 | sameRunOnly=1 | noReflight=1 | noRecompute=1 | sampledAcceptance=0 | productionModified=0\n');

OUT=struct();
OUT.schema='AF7R57B_AUTHORITY_RECOVERY_AUDIT_V1';
OUT.closed=false;
OUT.production_modified=false;
OUT.production_t1_selected=false;
OUT.continuous_terminal_boundary_proof=false;

%% A. Current replanner authority
fprintf('\n[A. CURRENT REPLANNER AUTHORITY]\n');
rep=[];
repName='';

if evalin('base','exist(''replanner_instance'',''var'')==1')
    q=evalin('base','replanner_instance');
    if isa(q,'REPLANNING_SYSTEM')
        rep=q;
        repName='replanner_instance';
    end
end

if isempty(rep)
    W=evalin('base','whos');
    ids=find(strcmp({W.class},'REPLANNING_SYSTEM'));
    if numel(ids)==1
        repName=W(ids).name;
        rep=evalin('base',repName);
    end
end

repPresent=isa(rep,'REPLANNING_SYSTEM');
triggerValid=false;
triggerError='';

if repPresent
    try
        E=rep.get_first_trigger_event();
        triggerValid=isstruct(E)&&isfield(E,'valid')&&logical(E.valid);
    catch ME
        triggerError=sprintf('%s | %s',ME.identifier,ME.message);
    end
end

fprintf(' replanner present                       = %d\n',repPresent);
fprintf(' replanner variable                      = %s\n',repName);
fprintf(' first true trigger valid                = %d\n',triggerValid);
if ~isempty(triggerError)
    fprintf(' trigger query error                     = %s\n',triggerError);
end

%% B. Base-workspace schema recovery
fprintf('\n[B. BASE-WORKSPACE SCHEMA RECOVERY]\n');

W=evalin('base','whos');
baseHits=struct('variable',{},'path',{},'kind',{},'schema',{});

for i=1:numel(W)
    nm=W(i).name;
    try
        x=evalin('base',nm);
        H=af7r57b_scan_value(x,nm,0,7);
        if ~isempty(H)
            baseHits=[baseHits H]; %#ok<AGROW>
        end
    catch
    end
end

fprintf(' schema-identifiable base hits           = %d\n',numel(baseHits));
for i=1:numel(baseHits)
    fprintf('  [%d] %-7s | %s | schema=%s\n', ...
        i,baseHits(i).kind,baseHits(i).path,baseHits(i).schema);
end

baseB49=find(strcmp({baseHits.kind},'B49'));
baseR50A=find(strcmp({baseHits.kind},'R50A'));
baseR56B=find(strcmp({baseHits.kind},'R56B'));

fprintf(' base B49 hits                           = %d\n',numel(baseB49));
fprintf(' base R50A hits                          = %d\n',numel(baseR50A));
fprintf(' base AF7R56B hits                       = %d\n',numel(baseR56B));

%% C. Locate snapshot
fprintf('\n[C. AF7R50 THEORY SNAPSHOT]\n');

snap=af7r57b_find_snapshot();
snapPresent=~isempty(snap) && isfile(snap);

fprintf(' snapshot present                        = %d\n',snapPresent);
fprintf(' snapshot                                = %s\n',snap);

snapHash='';
snapHits=struct('variable',{},'path',{},'kind',{},'schema',{});
snapVars=struct([]);

if snapPresent
    snapHash=af7r57b_sha256_file(snap);
    fprintf(' snapshot SHA-256                        = %s\n',snapHash);

    snapVars=whos('-file',snap);
    fprintf(' top-level variable count                = %d\n',numel(snapVars));

    for i=1:numel(snapVars)
        fprintf('  %-36s | class=%-14s | size=%s\n', ...
            snapVars(i).name,snapVars(i).class,mat2str(snapVars(i).size));
    end

    S=load(snap);
    fn=fieldnames(S);

    for i=1:numel(fn)
        H=af7r57b_scan_value(S.(fn{i}),fn{i},0,9);
        if ~isempty(H)
            snapHits=[snapHits H]; %#ok<AGROW>
        end
    end

    fprintf('\n snapshot schema-identifiable hits        = %d\n',numel(snapHits));
    for i=1:numel(snapHits)
        fprintf('  [%d] %-7s | %s | schema=%s\n', ...
            i,snapHits(i).kind,snapHits(i).path,snapHits(i).schema);
    end
end

snapB49=find(strcmp({snapHits.kind},'B49'));
snapR50A=find(strcmp({snapHits.kind},'R50A'));
snapR56B=find(strcmp({snapHits.kind},'R56B'));

fprintf(' snapshot B49 hits                       = %d\n',numel(snapB49));
fprintf(' snapshot R50A hits                      = %d\n',numel(snapR50A));
fprintf(' snapshot AF7R56B hits                   = %d\n',numel(snapR56B));

%% D. AF7R56B exact source dependency audit
fprintf('\n[D. AF7R56B SOURCE DEPENDENCY AUDIT]\n');

src=af7r57b_find_source();
srcPresent=~isempty(src)&&isfile(src);
fprintf(' AF7R56B source present                  = %d\n',srcPresent);
fprintf(' source                                  = %s\n',src);

sourceHash='';
dep=struct();

if srcPresent
    sourceHash=af7r57b_sha256_file(src);
    fprintf(' source SHA-256                          = %s\n',sourceHash);

    raw=fileread(src);
    L=splitlines(string(af7r57b_normalize_newlines(raw)));

    keys={ ...
        'replanner', ...
        'B49', ...
        'R50A', ...
        'AF7R50_THEORY_SNAPSHOT', ...
        'get_first_trigger_event', ...
        'load(', ...
        'af7r56_trace_mex', ...
        'af7r56_interval_kkt_mex'};

    for k=1:numel(keys)
        key=keys{k};
        rows=find(contains(L,key));
        dep.(af7r57b_field_key(key))=double(rows(:).');

        fprintf('\n key "%s" | lines=%s\n',key,mat2str(rows(:).'));
        for j=1:numel(rows)
            ln=rows(j);
            fprintf('  L%-4d %s\n',ln,char(L(ln)));
        end
    end
end

%% E. Exact recoverability classification
fprintf('\n[E. RECOVERABILITY CLASSIFICATION]\n');

sameRunR56Recovered=(numel(baseR56B)==1)||(numel(snapR56B)==1);
sameRunPredRecovered=((numel(baseB49)==1)&&(numel(baseR50A)==1)) || ...
                     ((numel(snapB49)==1)&&(numel(snapR50A)==1));

fprintf(' unique same-run AF7R56B result recovered = %d\n',sameRunR56Recovered);
fprintf(' unique same-run B49/R50A pair recovered  = %d\n',sameRunPredRecovered);
fprintf(' current live trigger authority valid     = %d\n',triggerValid);

if sameRunR56Recovered
    mode='USE_SAVED_AF7R56B_RESULT_DIRECTLY';
elseif sameRunPredRecovered && triggerValid
    mode='REPLAY_AF7R56B_WITH_SAVED_PREDECESSORS';
elseif sameRunPredRecovered && ~triggerValid
    mode='SAVED_PREDECESSORS_FOUND__LIVE_REPLANNER_STALE__DO_NOT_RECOMPUTE';
elseif snapPresent
    mode='SNAPSHOT_PRESENT__PREDECESSOR_SCHEMA_NOT_YET_LOCATED';
else
    mode='SAME_RUN_AUTHORITY_NOT_RECOVERED';
end

fprintf(' recovery mode                           = %s\n',mode);
fprintf(' rerun flight allowed by this audit      = 0\n');
fprintf(' regenerate B49 from stale replanner     = 0\n');
fprintf(' regenerate R50A from stale replanner    = 0\n');

%% F. Formal gate
fprintf('\n[F. FORMAL GATE]\n');

sourceStable=true;
snapshotStable=true;

if srcPresent
    sourceStable=strcmpi(sourceHash,af7r57b_sha256_file(src));
end
if snapPresent
    snapshotStable=strcmpi(snapHash,af7r57b_sha256_file(snap));
end

closed=srcPresent && sourceStable && (~snapPresent || snapshotStable);

fprintf(' source audit closed                     = %d\n',srcPresent);
fprintf(' source SHA unchanged                    = %d\n',sourceStable);
fprintf(' snapshot SHA unchanged                  = %d\n',snapshotStable);
fprintf(' recovery audit closed                   = %d\n',closed);
fprintf(' production source modified              = 0\n');
fprintf(' production t1 selected                  = 0\n');
fprintf(' continuous terminal-boundary proof      = 0\n');
fprintf(' unresolved recovery policy              = FAIL CLOSED\n');
fprintf(' next action                              = %s\n',mode);
fprintf('====================================================================================================\n');

OUT.closed=closed;
OUT.replanner_present=repPresent;
OUT.replanner_variable=repName;
OUT.first_true_trigger_valid=triggerValid;
OUT.base_hits=baseHits;
OUT.snapshot=snap;
OUT.snapshot_present=snapPresent;
OUT.snapshot_sha256=snapHash;
OUT.snapshot_hits=snapHits;
OUT.source=src;
OUT.source_sha256=sourceHash;
OUT.source_dependencies=dep;
OUT.unique_saved_R56B=sameRunR56Recovered;
OUT.unique_saved_B49_R50A=sameRunPredRecovered;
OUT.recovery_mode=mode;

assignin('base','AF7R57B_AUTHORITY_RECOVERY_AUDIT_OUT',OUT);
end


function snap=af7r57b_find_snapshot()
snap='';

p=which('AF7R50_THEORY_SNAPSHOT.mat');
if ~isempty(p) && isfile(p)
    snap=p;
    return;
end

roots={pwd,fileparts(mfilename('fullpath'))};

p=which('AF7R56B_STAGEB_BOUNDARY_TRACE');
if ~isempty(p)
    roots{end+1}=fileparts(p); %#ok<AGROW>
end

p=which('af7r56_trace_mex');
if ~isempty(p)
    roots{end+1}=fileparts(p); %#ok<AGROW>
end

u=getenv('USERPROFILE');
if ~isempty(u)
    roots{end+1}=fullfile(u,'Documents','GitHub','common_matlab','reference'); %#ok<AGROW>
end

for i=1:numel(roots)
    f=fullfile(roots{i},'AF7R50_THEORY_SNAPSHOT.mat');
    if isfile(f)
        snap=f;
        return;
    end
end
end


function src=af7r57b_find_source()
src='';

p=which('AF7R56B_STAGEB_BOUNDARY_TRACE');
if ~isempty(p) && isfile(p)
    src=p;
    return;
end

roots={pwd,fileparts(mfilename('fullpath'))};

p=which('af7r56_trace_mex');
if ~isempty(p)
    roots{end+1}=fileparts(p); %#ok<AGROW>
end

u=getenv('USERPROFILE');
if ~isempty(u)
    roots{end+1}=fullfile(u,'Documents','GitHub','common_matlab','reference'); %#ok<AGROW>
end

for i=1:numel(roots)
    f=fullfile(roots{i},'AF7R56B_STAGEB_BOUNDARY_TRACE.m');
    if isfile(f)
        src=f;
        return;
    end
end
end


function H=af7r57b_scan_value(x,path,depth,maxDepth)
H=struct('variable',{},'path',{},'kind',{},'schema',{});

if depth>maxDepth
    return;
end

if isstruct(x)
    if isscalar(x) && isfield(x,'schema')
        try
            sc=char(string(x.schema));
        catch
            sc='';
        end

        kind=af7r57b_schema_kind(sc);

        if ~isempty(kind)
            H(end+1)=struct( ... %#ok<AGROW>
                'variable',af7r57b_root_name(path), ...
                'path',path, ...
                'kind',kind, ...
                'schema',sc);
        end
    end

    if numel(x)==1
        f=fieldnames(x);

        for i=1:numel(f)
            try
                q=af7r57b_scan_value(x.(f{i}),[path '.' f{i}],depth+1,maxDepth);
                if ~isempty(q)
                    H=[H q]; %#ok<AGROW>
                end
            catch
            end
        end
    else
        lim=min(numel(x),32);

        for j=1:lim
            q=af7r57b_scan_value(x(j),sprintf('%s(%d)',path,j),depth+1,maxDepth);
            if ~isempty(q)
                H=[H q]; %#ok<AGROW>
            end
        end
    end

elseif iscell(x)
    lim=min(numel(x),64);

    for j=1:lim
        try
            q=af7r57b_scan_value(x{j},sprintf('%s{%d}',path,j),depth+1,maxDepth);
            if ~isempty(q)
                H=[H q]; %#ok<AGROW>
            end
        catch
        end
    end
end
end


function kind=af7r57b_schema_kind(sc)
kind='';

if contains(sc,'AF7R49B_REJOIN_DOMAIN_FORENSIC')
    kind='B49';
elseif contains(sc,'AF7R50A_TERMINAL_COMPONENT_SPEED_THEOREM')
    kind='R50A';
elseif contains(sc,'AF7R56B')
    kind='R56B';
end
end


function r=af7r57b_root_name(path)
k=regexp(path,'[.\(\{]','once');

if isempty(k)
    r=path;
else
    r=path(1:k-1);
end
end


function s=af7r57b_normalize_newlines(s)
s=strrep(s,sprintf('\r\n'),sprintf('\n'));
s=strrep(s,sprintf('\r'),sprintf('\n'));
end


function f=af7r57b_field_key(s)
f=regexprep(s,'[^A-Za-z0-9]','_');
if isempty(f) || ~isletter(f(1))
    f=['k_' f];
end
f=matlab.lang.makeValidName(f);
end


function h=af7r57b_sha256_file(fn)
md=java.security.MessageDigest.getInstance('SHA-256');

fid=fopen(fn,'rb');
if fid<0
    error('AF7R57B:Open','Cannot open %s',fn);
end

c=onCleanup(@() fclose(fid));

while true
    b=fread(fid,1024*1024,'*uint8');
    if isempty(b)
        break;
    end
    md.update(b);
end

raw=typecast(md.digest(),'uint8');
h=lower(reshape(dec2hex(raw,2).',1,[]));
end
