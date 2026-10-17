function OUT = AF7R57C_SNAPSHOT_BOUND_SCHEMA_CAPTURE(varargin)
% AF7R57C_SNAPSHOT_BOUND_SCHEMA_CAPTURE
% -----------------------------------------------------------------------------
% Same-run, snapshot-bound schema capture for AF7R56B.
%
% Key point:
%   AF7R56B itself calls
%       AF7R52_LOAD_TRUE_TRIGGER_AUTHORITY(replanner)
%   and, when B49/R50A are empty, recovers them from AUX.
%
% Therefore this audit does NOT rerun AF7R49B/AF7R50A.
% It executes an instrumented SHADOW copy of AF7R56B with:
%       shadow(replanner,[],[])
% exactly so the existing AF7R52 authority loader remains authoritative.
%
% No production source is modified.
% No production t1 is selected.
% No sampled acceptance is introduced.
% -----------------------------------------------------------------------------

clc;
fprintf('\n====================================================================================================\n');
fprintf(' AF7R57C | SNAPSHOT-BOUND AF7R56B STAGE-B / KKT SCHEMA CAPTURE\n');
fprintf('====================================================================================================\n');
fprintf('[AUTHORITY] shadowOnly=1 | AF7R52Loader=1 | sameRunSnapshot=1 | sampledAcceptance=0 | productionModified=0\n');

OUT=struct();
OUT.schema='AF7R57C_SNAPSHOT_BOUND_SCHEMA_CAPTURE_V1';
OUT.closed=false;
OUT.production_modified=false;
OUT.production_t1_selected=false;
OUT.continuous_terminal_boundary_proof=false;

%% A. Bind replanner
fprintf('\n[A. REPLANNER BINDING]\n');

if nargin>=1 && ~isempty(varargin{1})
    replanner=varargin{1};
    replannerSource='CALLER_EXPLICIT';
elseif evalin('base','exist(''replanner_instance'',''var'')==1')
    replanner=evalin('base','replanner_instance');
    replannerSource='BASE_ALIAS:replanner_instance';
else
    error('AF7R57C:ReplannerUnavailable', ...
        'Pass replanner_instance or keep it in the base workspace.');
end

assert(isa(replanner,'REPLANNING_SYSTEM'), ...
    '[AF7R57C] replanner is not REPLANNING_SYSTEM.');

fprintf(' replanner source                        = %s\n',replannerSource);
fprintf(' replanner class authority               = 1\n');

try
    E_live=replanner.get_first_trigger_event();
    liveTriggerValid=isstruct(E_live)&&isfield(E_live,'valid')&&logical(E_live.valid);
catch
    liveTriggerValid=false;
end

fprintf(' current live first-trigger valid        = %d\n',liveTriggerValid);
fprintf(' live trigger required for this audit    = 0\n');

%% B. Source and snapshot authority
fprintf('\n[B. SOURCE / SNAPSHOT AUTHORITY]\n');

src=which('AF7R56B_STAGEB_BOUNDARY_TRACE');
assert(~isempty(src)&&isfile(src),'[AF7R57C] AF7R56B source missing.');

srcHash0=af7r57c_sha256_file(src);
raw=fileread(src);

requiredSource={ ...
    'AF7R52_LOAD_TRUE_TRIGGER_AUTHORITY(replanner)', ...
    'if nargin<2||isempty(B49)', ...
    'assert(isfield(AUX,''B49'')', ...
    'if nargin<3||isempty(R50A)', ...
    'if isfield(AUX,''R50A'')', ...
    'TL{c}=af7r56_trace_mex(5,pkt,tU,lo,c);', ...
    'TU{c}=af7r56_trace_mex(5,pkt,tU,hi,c);', ...
    'af7r56_interval_kkt_mex('};

sourceContract=true;
for i=1:numel(requiredSource)
    sourceContract=sourceContract && contains(raw,requiredSource{i});
end

fprintf(' source                                  = %s\n',src);
fprintf(' source SHA-256                          = %s\n',srcHash0);
fprintf(' AF7R52 fallback source contract         = %d\n',sourceContract);
assert(sourceContract,'[AF7R57C] AF7R56B fallback source contract changed.');

loader=which('AF7R52_LOAD_TRUE_TRIGGER_AUTHORITY');
assert(~isempty(loader)&&isfile(loader),'[AF7R57C] AF7R52 loader missing.');

loaderHash0=af7r57c_sha256_file(loader);

fprintf(' loader                                  = %s\n',loader);
fprintf(' loader SHA-256                          = %s\n',loaderHash0);

snap=af7r57c_find_snapshot(src);
assert(~isempty(snap)&&isfile(snap),'[AF7R57C] AF7R50 theory snapshot missing.');
snapHash0=af7r57c_sha256_file(snap);

fprintf(' snapshot                                = %s\n',snap);
fprintf(' snapshot SHA-256                        = %s\n',snapHash0);

S=load(snap);
assert(isfield(S,'B49')&&isfield(S,'R50A')&&isfield(S,'E')&&isfield(S,'M'), ...
    '[AF7R57C] snapshot top-level authority incomplete.');

snapB49=S.B49;
snapR50A=S.R50A;
snapE=S.E;
snapM=S.M;

fprintf(' snapshot B49 schema                     = %s\n',af7r57c_schema(snapB49));
fprintf(' snapshot R50A schema                    = %s\n',af7r57c_schema(snapR50A));
fprintf(' snapshot E valid                        = %d\n', ...
    isstruct(snapE)&&isfield(snapE,'valid')&&logical(snapE.valid));
fprintf(' snapshot M certified                    = %d\n', ...
    isstruct(snapM)&&isfield(snapM,'certified')&&logical(snapM.certified));

%% C. Exercise AF7R52 loader directly
fprintf('\n[C. AF7R52 SAME-RUN LOADER]\n');

[E,M,AUX,authoritySource]=AF7R52_LOAD_TRUE_TRIGGER_AUTHORITY(replanner);

loaderEValid=isstruct(E)&&isfield(E,'valid')&&logical(E.valid);
loaderMCert=isstruct(M)&&isfield(M,'certified')&&logical(M.certified);
auxB49=isstruct(AUX)&&isfield(AUX,'B49');
auxR50A=isstruct(AUX)&&isfield(AUX,'R50A');

fprintf(' authority source                         = %s\n',char(string(authoritySource)));
fprintf(' loader E valid                          = %d\n',loaderEValid);
fprintf(' loader M certified                      = %d\n',loaderMCert);
fprintf(' AUX.B49 present                         = %d\n',auxB49);
fprintf(' AUX.R50A present                        = %d\n',auxR50A);

assert(loaderEValid&&loaderMCert&&auxB49&&auxR50A, ...
    '[AF7R57C] AF7R52 loader did not recover complete same-run authority.');

sameB49=isequaln(AUX.B49,snapB49);
sameR50A=isequaln(AUX.R50A,snapR50A);
sameE=isequaln(E,snapE);
sameM=isequaln(M,snapM);

fprintf(' loader B49 == snapshot B49              = %d\n',sameB49);
fprintf(' loader R50A == snapshot R50A            = %d\n',sameR50A);
fprintf(' loader E == snapshot E                  = %d\n',sameE);
fprintf(' loader M == snapshot M                  = %d\n',sameM);

snapshotBindingClosed=sameB49&&sameR50A&&sameE&&sameM;
fprintf(' snapshot binding closed                 = %d\n',snapshotBindingClosed);

assert(snapshotBindingClosed, ...
    '[AF7R57C] AF7R52 loader authority differs from AF7R50 snapshot.');

%% D. Build instrumented AF7R56B shadow
fprintf('\n[D. SHADOW CONSTRUCTION]\n');

root=fileparts(mfilename('fullpath'));
shadowDir=fullfile(root,'AF7R57C_SHADOW');
if ~isfolder(shadowDir), mkdir(shadowDir); end

shadowName='AF7R57C_AF7R56B_SHADOW';
shadowFile=fullfile(shadowDir,[shadowName '.m']);
traceFile=fullfile(shadowDir,'AF7R57C_TRACE_CAPTURE.mat');
kktFile=fullfile(shadowDir,'AF7R57C_KKT_CAPTURE.mat');

if isfile(traceFile), delete(traceFile); end
if isfile(kktFile), delete(kktFile); end

% Rename only the primary function declaration.
oldDecl='function OUT = AF7R56B_STAGEB_BOUNDARY_TRACE(replanner,B49,R50A)';
newDecl=['function OUT = ' shadowName '(replanner,B49,R50A)'];

assert(numel(strfind(raw,oldDecl))==1, ...
    '[AF7R57C] exact AF7R56B primary declaration not unique.');
shadow=strrep(raw,oldDecl,newDecl);

% Inject after exact KKT call statement.
pat=['K\{c\}=af7r56_interval_kkt_mex\(\s*\.\.\.\s*' ...
     'lo,hi,double\(U\.t0\),double\(U\.component_start\(1\)\),\s*\.\.\.\s*' ...
     'double\(U\.component_end\(1\)\),A,b,ac0\(1\)\+1,signTag,c\);'];
[a,b]=regexp(shadow,pat,'start','end');

fprintf(' interval-KKT injection count            = %d\n',numel(a));
assert(numel(a)==1,'[AF7R57C] interval-KKT call not uniquely bound.');

orig=shadow(a(1):b(1));
inj=[orig newline 'af7r57c_capture_kkt(K,U,pkt,lo,hi,tU,A,b,ac0,signTag,c);'];
shadow=[shadow(1:a(1)-1) inj shadow(b(1)+1:end)];

% Inject after exact upper endpoint trace call.
needle='TU{c}=af7r56_trace_mex(5,pkt,tU,hi,c);';
nTrace=numel(strfind(shadow,needle));

fprintf(' mode-5 trace injection count            = %d\n',nTrace);
assert(nTrace==1,'[AF7R57C] mode-5 trace call not uniquely bound.');

shadow=strrep(shadow,needle, ...
    [needle newline 'af7r57c_capture_trace(TL,TU,U,pkt,lo,hi,tU,c);']);

% Append helper functions. Each computes its own shadow-directory path,
% avoiding generated absolute-path literals.
shadow=[shadow newline af7r57c_trace_helper_text() newline ...
    af7r57c_kkt_helper_text() newline];

fid=fopen(shadowFile,'w');
assert(fid>=0,'[AF7R57C] cannot create shadow source.');
cc=onCleanup(@() fclose(fid));
fwrite(fid,shadow,'char');
clear cc

fprintf(' shadow source                           = %s\n',shadowFile);
fprintf(' production source modified              = 0\n');

%% E. Execute using AF7R56B's own fallback contract
fprintf('\n[E. EXECUTE SNAPSHOT-BOUND SHADOW]\n');
fprintf(' invocation                              = shadow(replanner,[],[])\n');

addpath(shadowDir,'-begin');
pc=onCleanup(@() rmpath(shadowDir)); %#ok<NASGU>
rehash;

R56=feval(shadowName,replanner,[],[]);

tracePresent=isfile(traceFile);
kktPresent=isfile(kktFile);

fprintf(' shadow execution completed              = 1\n');
fprintf(' trace capture present                   = %d\n',tracePresent);
fprintf(' KKT capture present                     = %d\n',kktPresent);

assert(tracePresent&&kktPresent,'[AF7R57C] shadow captures missing.');

TC=load(traceFile);
KC=load(kktFile);

traceRecords=TC.records;
kktRecords=KC.records;

fprintf(' trace append records                    = %d\n',numel(traceRecords));
fprintf(' KKT append records                      = %d\n',numel(kktRecords));

%% F. Actual runtime schema
fprintf('\n[F. MODE-5 TRACE RUNTIME SCHEMA]\n');
af7r57c_print_schema('TRACE',TC,0,6);

fprintf('\n[G. INTERVAL-KKT RUNTIME SCHEMA]\n');
af7r57c_print_schema('KKT',KC,0,6);

%% H. Candidate/leaf summary using real field names
fprintf('\n[H. TRACE/KKT COMPACT SUMMARY]\n');

for i=1:numel(traceRecords)
    Q=traceRecords{i};
    fprintf(' trace record %d | candidate=%g | lo=%.17g | hi=%.17g\n', ...
        i,double(Q.c),double(Q.lo),double(Q.hi));

    if iscell(Q.TL) && numel(Q.TL)>=Q.c
        af7r57c_print_struct_scalars(sprintf(' TL{%d}',Q.c),Q.TL{Q.c},2);
    end
    if iscell(Q.TU) && numel(Q.TU)>=Q.c
        af7r57c_print_struct_scalars(sprintf(' TU{%d}',Q.c),Q.TU{Q.c},2);
    end
end

for i=1:numel(kktRecords)
    Q=kktRecords{i};
    fprintf(' KKT record %d | candidate=%g | lo=%.17g | hi=%.17g | signTag=%g\n', ...
        i,double(Q.c),double(Q.lo),double(Q.hi),double(Q.signTag));
    if iscell(Q.K) && numel(Q.K)>=Q.c
        af7r57c_print_struct_scalars(sprintf(' K{%d}',Q.c),Q.K{Q.c},2);
    end
end

%% I. Final immutable gate
fprintf('\n[I. FORMAL GATE]\n');

srcSame=strcmpi(srcHash0,af7r57c_sha256_file(src));
loaderSame=strcmpi(loaderHash0,af7r57c_sha256_file(loader));
snapSame=strcmpi(snapHash0,af7r57c_sha256_file(snap));

schemaClosed=tracePresent&&kktPresent&& ...
    ~isempty(traceRecords)&&~isempty(kktRecords);

fprintf(' AF7R52 snapshot binding closed          = %d\n',snapshotBindingClosed);
fprintf(' runtime trace schema captured           = %d\n',tracePresent);
fprintf(' runtime interval-KKT schema captured    = %d\n',kktPresent);
fprintf(' schema capture closed                   = %d\n',schemaClosed);
fprintf(' production source SHA unchanged         = %d\n',srcSame);
fprintf(' loader source SHA unchanged             = %d\n',loaderSame);
fprintf(' snapshot SHA unchanged                  = %d\n',snapSame);
fprintf(' production source modified              = 0\n');
fprintf(' production t1 selected                  = 0\n');
fprintf(' continuous terminal-boundary proof      = 0\n');
fprintf(' next theorem                            = COMMON_DYADIC_REFINEMENT_INTERVAL_STAGEB\n');
fprintf('====================================================================================================\n');

OUT.closed=snapshotBindingClosed&&schemaClosed&&srcSame&&loaderSame&&snapSame;
OUT.authority_source=authoritySource;
OUT.snapshot=snap;
OUT.snapshot_sha256=snapHash0;
OUT.source=src;
OUT.source_sha256=srcHash0;
OUT.loader=loader;
OUT.loader_sha256=loaderHash0;
OUT.R56=R56;
OUT.trace_records=traceRecords;
OUT.kkt_records=kktRecords;
OUT.snapshot_binding_closed=snapshotBindingClosed;
OUT.schema_capture_closed=schemaClosed;
OUT.production_modified=false;
OUT.production_t1_selected=false;
OUT.continuous_terminal_boundary_proof=false;

assignin('base','AF7R57C_SNAPSHOT_BOUND_SCHEMA_CAPTURE_OUT',OUT);
save(fullfile(root,'AF7R57C_SNAPSHOT_BOUND_SCHEMA_CAPTURE_OUT.mat'),'OUT','-v7.3');
end


function snap=af7r57c_find_snapshot(src)
snap='';

p=which('AF7R50_THEORY_SNAPSHOT.mat');
if ~isempty(p)&&isfile(p)
    snap=p;
    return;
end

cand=fullfile(fileparts(src),'AF7R50_THEORY_SNAPSHOT.mat');
if isfile(cand)
    snap=cand;
end
end


function s=af7r57c_schema(x)
s='';

if isstruct(x)&&isfield(x,'schema')
    try
        s=char(string(x.schema));
    catch
        s='';
    end
end
end


function t=af7r57c_trace_helper_text()
q={ ...
'function af7r57c_capture_trace(TL,TU,U,pkt,lo,hi,tU,c)', ...
'root=fileparts(mfilename(''fullpath''));', ...
'fn=fullfile(root,''AF7R57C_TRACE_CAPTURE.mat'');', ...
'Q=struct(''TL'',{TL},''TU'',{TU},''U'',U,''pkt'',pkt,''lo'',lo,''hi'',hi,''tU'',tU,''c'',c);', ...
'if isfile(fn)', ...
'    S=load(fn,''records''); records=S.records;', ...
'else', ...
'    records={};', ...
'end', ...
'records{end+1}=Q;', ...
'save(fn,''records'',''-v7.3'');', ...
'end'};
t=strjoin(q,newline);
end


function t=af7r57c_kkt_helper_text()
q={ ...
'function af7r57c_capture_kkt(K,U,pkt,lo,hi,tU,A,b,ac0,signTag,c)', ...
'root=fileparts(mfilename(''fullpath''));', ...
'fn=fullfile(root,''AF7R57C_KKT_CAPTURE.mat'');', ...
'Q=struct(''K'',{K},''U'',U,''pkt'',pkt,''lo'',lo,''hi'',hi,''tU'',tU,''A'',A,''b'',b,''ac0'',ac0,''signTag'',signTag,''c'',c);', ...
'if isfile(fn)', ...
'    S=load(fn,''records''); records=S.records;', ...
'else', ...
'    records={};', ...
'end', ...
'records{end+1}=Q;', ...
'save(fn,''records'',''-v7.3'');', ...
'end'};
t=strjoin(q,newline);
end


function af7r57c_print_schema(name,x,depth,maxDepth)
indent=repmat(' ',1,2*depth);

if depth>maxDepth
    fprintf('%s%s : <max depth>\n',indent,name);
    return;
end

if isstruct(x)
    fprintf('%s%s : struct %s\n',indent,name,mat2str(size(x)));
    f=fieldnames(x);

    for i=1:numel(f)
        if numel(x)==1
            af7r57c_print_schema(f{i},x.(f{i}),depth+1,maxDepth);
        else
            fprintf('%s  %s : field over struct array\n',indent,f{i});
        end
    end

elseif iscell(x)
    fprintf('%s%s : cell %s\n',indent,name,mat2str(size(x)));
    n=min(numel(x),6);

    for i=1:n
        af7r57c_print_schema(sprintf('{%d}',i),x{i},depth+1,maxDepth);
    end

    if numel(x)>n
        fprintf('%s  ... %d more cells\n',indent,numel(x)-n);
    end

else
    fprintf('%s%s : %s %s\n',indent,name,class(x),mat2str(size(x)));
end
end


function af7r57c_print_struct_scalars(name,x,depth)
indent=repmat(' ',1,2*depth);

if ~isstruct(x)||numel(x)~=1
    fprintf('%s%s | class=%s size=%s\n',indent,name,class(x),mat2str(size(x)));
    return;
end

fprintf('%s%s | fields=%d\n',indent,name,numel(fieldnames(x)));
f=fieldnames(x);

for i=1:numel(f)
    v=x.(f{i});

    if (isnumeric(v)||islogical(v))&&isscalar(v)
        fprintf('%s  %-32s = %.17g\n',indent,f{i},double(v));
    elseif ischar(v)||isstring(v)
        fprintf('%s  %-32s = %s\n',indent,f{i},char(string(v)));
    elseif isnumeric(v)&&numel(v)<=12
        fprintf('%s  %-32s = %s\n',indent,f{i},mat2str(double(v),17));
    end
end
end


function h=af7r57c_sha256_file(fn)
md=java.security.MessageDigest.getInstance('SHA-256');

fid=fopen(fn,'rb');
assert(fid>=0,'[AF7R57C] cannot open file for SHA.');

c=onCleanup(@() fclose(fid));

while true
    b=fread(fid,1024*1024,'*uint8');
    if isempty(b), break; end
    md.update(b);
end

raw=typecast(md.digest(),'uint8');
h=lower(reshape(dec2hex(raw,2).',1,[]));
end
