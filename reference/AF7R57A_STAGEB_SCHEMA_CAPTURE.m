function R = AF7R57A_STAGEB_SCHEMA_CAPTURE(varargin)
% AF7R57A_STAGEB_SCHEMA_CAPTURE
% -----------------------------------------------------------------------------
% V7 -- canonical predecessor-bound schema capture for AF7R56B_STAGEB_BOUNDARY_TRACE.
%
% Main corrections:
%   * parses multiline MATLAB function declarations with "..." continuation;
%   * binds the actual formal inputs without inventing defaults;
%   * for formal input "replanner", recognizes the established
%       replanner_instance
%     base-workspace alias used by the existing run_flight_headless flow;
%   * if inputs are unavailable, FAILS CLOSED WITHOUT executing the shadow;
%   * preserves all candidate-loop trace/KKT captures append-only;
%   * verifies shadow interface identity and production SHA invariance;
%   * never selects production t1 and never claims the terminal theorem.
%
% Preferred:
%   R57A = AF7R57A_STAGEB_SCHEMA_CAPTURE();
%
% Explicit:
%   R57A = AF7R57A_STAGEB_SCHEMA_CAPTURE(replanner_instance);
% -----------------------------------------------------------------------------

clc;
af7r57a_line('');
af7r57a_line('====================================================================================================');
af7r57a_line(' AF7R57A V7 | REAL AF7R56B STAGE-B / INTERVAL-KKT SCHEMA CAPTURE');
af7r57a_line('====================================================================================================');
af7r57a_line('[AUTHORITY] shadowOnly=1 | sourceBoundInputs=1 | appendOnlyCapture=1 | sampledAcceptance=0 | productionModified=0');

R=struct();
R.version='AF7R57A_V7';
R.shadowOnly=true;
R.sampledAcceptance=false;
R.productionModified=false;
R.production_t1_selected=false;
R.continuous_terminal_boundary_proof=false;
R.schema_capture_closed=false;

srcFile=af7r57a_find_source();
src0=fileread(srcFile);
sha0=af7r57a_sha256_file(srcFile);

[DECL,declClosed]=af7r57a_parse_main_declaration(src0);

af7r57a_line('');
af7r57a_line('[A. SOURCE / FUNCTION AUTHORITY]');
af7r57a_line(' source                                  = %s',srcFile);
af7r57a_line(' source SHA-256                          = %s',sha0);
af7r57a_line(' declaration parsed                      = %d',declClosed);

if ~declClosed
    af7r57a_line(' production source modified              = 0');
    af7r57a_line(' production t1 selected                  = 0');
    af7r57a_line(' continuous terminal-boundary proof      = 0');
    af7r57a_line(' status                                  = FAIL_CLOSED_DECLARATION_UNRESOLVED');
    R.source=srcFile;
    R.source_sha256=sha0;
    R.status='FAIL_CLOSED_DECLARATION_UNRESOLVED';
    return;
end

af7r57a_line(' main declaration                        = %s',DECL.declaration_one_line);
af7r57a_line(' declaration physical lines              = %d',DECL.physical_line_count);
af7r57a_line(' formal input count                      = %d',numel(DECL.inputs));
af7r57a_line(' formal output count                     = %d',numel(DECL.outputs));

for i=1:numel(DECL.inputs)
    af7r57a_line(' formal input %d                          = %s',i,DECL.inputs{i});
end
for i=1:numel(DECL.outputs)
    af7r57a_line(' formal output %d                         = %s',i,DECL.outputs{i});
end

[callArgs,argSource,argClosed,BIND]=af7r57a_resolve_call_args(DECL.inputs,varargin);

af7r57a_line('');
af7r57a_line('[B. INPUT BINDING]');
af7r57a_line(' caller supplied input count             = %d',numel(varargin));
af7r57a_line(' required input count                    = %d',numel(DECL.inputs));
af7r57a_line(' base replanner variable exists          = %d',BIND.base_replanner_exists);
af7r57a_line(' base replanner_instance exists          = %d',BIND.base_replanner_instance_exists);
af7r57a_line(' base B49 exact variable exists          = %d',BIND.base_B49_exact_exists);
af7r57a_line(' base B49 canonical output exists        = %d',BIND.base_B49_canonical_exists);
af7r57a_line(' base R50A exact variable exists         = %d',BIND.base_R50A_exact_exists);
af7r57a_line(' base R50A canonical output exists       = %d',BIND.base_R50A_canonical_exists);
af7r57a_line(' REPLANNING_SYSTEM objects in base       = %d',BIND.replanning_system_object_count);

for i=1:numel(DECL.inputs)
    af7r57a_line(' %-38s = %-36s | class=%s | size=%s', ...
        DECL.inputs{i},argSource{i},class(callArgs{i}),mat2str(size(callArgs{i})));
end

af7r57a_line(' exact formal-input binding closed       = %d',argClosed);
af7r57a_line(' predecessor cross-consistency closed    = %d',BIND.predecessor_consistency_closed);

R.source=srcFile;
R.source_sha256=sha0;
R.main_declaration=DECL;
R.input_binding_sources=argSource;
R.input_binding_diagnostics=BIND;
R.input_binding_closed=argClosed;

if ~argClosed
    af7r57a_line('');
    af7r57a_line('[B2. FAIL-CLOSED INVOCATION GUIDANCE]');
    af7r57a_print_required_call(DECL.inputs);
    if any(strcmp(DECL.inputs,'replanner'))
        af7r57a_line(' established runtime object name         = replanner_instance');
    end

    if any(strcmp(DECL.inputs,'B49'))
        af7r57a_line(' canonical B49 producer                  = B49 = AF7R49B_REJOIN_DOMAIN_FORENSIC(replanner_instance)');
        af7r57a_line(' canonical B49 base output               = AF7R49B_REJOIN_DOMAIN_FORENSIC_OUT');
    end

    if any(strcmp(DECL.inputs,'R50A'))
        af7r57a_line(' canonical R50A producer                 = R50A = AF7R50A_TERMINAL_COMPONENT_SPEED_THEOREM(replanner_instance,B49)');
        af7r57a_line(' canonical R50A base output              = AF7R50A_TERMINAL_COMPONENT_SPEED_THEOREM_OUT');
    end
    af7r57a_line(' shadow executed                         = 0');
    af7r57a_line(' production source modified              = 0');
    af7r57a_line(' production t1 selected                  = 0');
    af7r57a_line(' continuous terminal-boundary proof      = 0');
    af7r57a_line(' status                                  = FAIL_CLOSED_INPUT_UNAVAILABLE');
    af7r57a_line('====================================================================================================');
    R.status='FAIL_CLOSED_INPUT_UNAVAILABLE';
    return;
end

idxRep=find(strcmp(DECL.inputs,'replanner'),1);
if ~isempty(idxRep)
    repOK=isa(callArgs{idxRep},'REPLANNING_SYSTEM');
    af7r57a_line(' replanner class authority               = %d',repOK);
    if ~repOK
        af7r57a_line(' shadow executed                         = 0');
        af7r57a_line(' production source modified              = 0');
        af7r57a_line(' production t1 selected                  = 0');
        af7r57a_line(' continuous terminal-boundary proof      = 0');
        af7r57a_line(' status                                  = FAIL_CLOSED_REPLANNER_TYPE');
        R.status='FAIL_CLOSED_REPLANNER_TYPE';
        return;
    end
end

baseDir=fileparts(mfilename('fullpath'));
shadowDir=fullfile(baseDir,'AF7R57A_SHADOW_V7');
if ~isfolder(shadowDir)
    mkdir(shadowDir);
end

shadowName='AF7R57A_AF7R56B_SHADOW_V7';
shadowFile=fullfile(shadowDir,[shadowName '.m']);
traceMat=fullfile(shadowDir,'AF7R57A_TRACE_CAPTURE_V7.mat');
kktMat=fullfile(shadowDir,'AF7R57A_KKT_CAPTURE_V7.mat');

if exist(traceMat,'file')
    delete(traceMat);
end
if exist(kktMat,'file')
    delete(kktMat);
end

[srcShadow,renameClosed]=af7r57a_rename_main_exact(src0,DECL,shadowName);

af7r57a_line('');
af7r57a_line('[C. SHADOW INSTRUMENTATION PRECHECK]');
af7r57a_line(' declaration rename closed               = %d',renameClosed);

if ~renameClosed
    af7r57a_line(' status                                  = FAIL_CLOSED_RENAME');
    R.status='FAIL_CLOSED_RENAME';
    return;
end

helperCollision=contains(src0,'af7r57a_v7_shadow_save_trace') || ...
                contains(src0,'af7r57a_v7_shadow_save_kkt');
af7r57a_line(' helper-name collision                   = %d',helperCollision);

if helperCollision
    af7r57a_line(' status                                  = FAIL_CLOSED_HELPER_COLLISION');
    R.status='FAIL_CLOSED_HELPER_COLLISION';
    return;
end

traceNeedle='TU{c}=af7r56_trace_mex(5,pkt,tU,hi,c);';
traceCount=numel(strfind(srcShadow,traceNeedle));
af7r57a_line(' exact upper mode-5 call count            = %d',traceCount);

if traceCount~=1
    af7r57a_line(' status                                  = FAIL_CLOSED_TRACE_CALL_COUNT');
    R.status='FAIL_CLOSED_TRACE_CALL_COUNT';
    return;
end

traceInject=[traceNeedle newline ...
    'af7r57a_v7_shadow_save_trace(TL,TU,U,pkt,lo,hi,tU,c);'];
srcShadow=strrep(srcShadow,traceNeedle,traceInject);

pat=['K\{c\}=af7r56_interval_kkt_mex\(\s*\.\.\.\s*' ...
     'lo,hi,double\(U\.t0\),double\(U\.component_start\(1\)\),\s*\.\.\.\s*' ...
     'double\(U\.component_end\(1\)\),A,b,ac0\(1\)\+1,signTag,c\);'];
[m0,m1]=regexp(srcShadow,pat,'start','end');

af7r57a_line(' exact interval-KKT assignment count      = %d',numel(m0));

if numel(m0)~=1
    af7r57a_line(' status                                  = FAIL_CLOSED_KKT_CALL_COUNT');
    R.status='FAIL_CLOSED_KKT_CALL_COUNT';
    return;
end

orig=srcShadow(m0(1):m1(1));
kktInject=[orig newline ...
    'af7r57a_v7_shadow_save_kkt(K,U,pkt,lo,hi,tU,A,b,ac0,signTag,c);'];
srcShadow=[srcShadow(1:m0(1)-1) kktInject srcShadow(m1(1)+1:end)];

srcShadow=[srcShadow newline ...
    af7r57a_trace_helper_text() newline ...
    af7r57a_kkt_helper_text() newline];

[SHDECL,shadowDeclClosed]=af7r57a_parse_named_declaration(srcShadow,shadowName);
sameInputs=shadowDeclClosed && isequal(SHDECL.inputs,DECL.inputs);
sameOutputs=shadowDeclClosed && isequal(SHDECL.outputs,DECL.outputs);

af7r57a_line(' shadow declaration parsed               = %d',shadowDeclClosed);
af7r57a_line(' shadow formal inputs identical          = %d',sameInputs);
af7r57a_line(' shadow formal outputs identical         = %d',sameOutputs);

if ~(shadowDeclClosed && sameInputs && sameOutputs)
    af7r57a_line(' status                                  = FAIL_CLOSED_SHADOW_INTERFACE');
    R.status='FAIL_CLOSED_SHADOW_INTERFACE';
    return;
end

fid=fopen(shadowFile,'w');
if fid<0
    error('AF7R57A:Write','Cannot create shadow source.');
end
cc=onCleanup(@() fclose(fid));
fwrite(fid,srcShadow,'char');
clear cc

shadowHash=af7r57a_sha256_file(shadowFile);

af7r57a_line(' shadow source                           = %s',shadowFile);
af7r57a_line(' shadow SHA-256                          = %s',shadowHash);
af7r57a_line(' production source modified              = 0');

addpath(shadowDir,'-begin');
pathCleanup=onCleanup(@() rmpath(shadowDir)); %#ok<NASGU>
rehash;

af7r57a_line('');
af7r57a_line('[D. EXECUTE INSTRUMENTED SHADOW]');
af7r57a_line(' forwarded input count                   = %d',numel(callArgs));
af7r57a_line(' mirrored output count                   = %d',numel(DECL.outputs));

try
    if isempty(DECL.outputs)
        feval(shadowName,callArgs{:});
        outputMeta=cell(1,0);
    else
        shadowOutputs=cell(1,numel(DECL.outputs));
        [shadowOutputs{:}]=feval(shadowName,callArgs{:});
        outputMeta=af7r57a_value_meta(shadowOutputs);
    end
    shadowExecClosed=true;
catch ME
    shadowExecClosed=false;
    af7r57a_line(' shadow execution closed                 = 0');
    af7r57a_line(' identifier                              = %s',ME.identifier);
    af7r57a_line(' message                                 = %s',ME.message);

    shaFail=af7r57a_sha256_file(srcFile);
    af7r57a_line(' production source SHA unchanged         = %d',strcmpi(sha0,shaFail));
    af7r57a_line(' production source modified              = %d',~strcmpi(sha0,shaFail));
    af7r57a_line(' production t1 selected                  = 0');
    af7r57a_line(' continuous terminal-boundary proof      = 0');
    af7r57a_line(' status                                  = FAIL_CLOSED_SHADOW_RUNTIME');

    R.status='FAIL_CLOSED_SHADOW_RUNTIME';
    R.shadow_runtime_identifier=ME.identifier;
    R.shadow_runtime_message=ME.message;
    R.production_source_unchanged=strcmpi(sha0,shaFail);
    R.productionModified=~strcmpi(sha0,shaFail);
    return;
end

af7r57a_line(' shadow execution closed                 = %d',shadowExecClosed);

traceCaptured=exist(traceMat,'file')==2;
kktCaptured=exist(kktMat,'file')==2;

af7r57a_line(' trace capture present                   = %d',traceCaptured);
af7r57a_line(' KKT capture present                     = %d',kktCaptured);

if ~(traceCaptured && kktCaptured)
    af7r57a_line(' status                                  = FAIL_CLOSED_CAPTURE_MISSING');
    R.status='FAIL_CLOSED_CAPTURE_MISSING';
    return;
end

T=load(traceMat);
K=load(kktMat);

traceRecords=af7r57a_extract_records(T,'records');
kktRecords=af7r57a_extract_records(K,'records');

af7r57a_line('');
af7r57a_line('[E. CAPTURE COMPLETENESS]');
af7r57a_line(' trace append records                    = %d',numel(traceRecords));
af7r57a_line(' KKT append records                      = %d',numel(kktRecords));

traceCandidates=af7r57a_record_candidate_ids(traceRecords);
kktCandidates=af7r57a_record_candidate_ids(kktRecords);

af7r57a_line(' trace candidate ids                     = %s',mat2str(traceCandidates));
af7r57a_line(' KKT candidate ids                       = %s',mat2str(kktCandidates));
af7r57a_line(' trace candidate ids unique              = %d', ...
    numel(unique(traceCandidates))==numel(traceCandidates));
af7r57a_line(' KKT candidate ids unique                = %d', ...
    numel(unique(kktCandidates))==numel(kktCandidates));

af7r57a_line('');
af7r57a_line('[F. MODE-5 TRACE SCHEMA]');
af7r57a_print_schema('TRACE_CAPTURE',T,0,5);

af7r57a_line('');
af7r57a_line('[G. INTERVAL-KKT SCHEMA]');
af7r57a_print_schema('KKT_CAPTURE',K,0,5);

af7r57a_line('');
af7r57a_line('[H. MODE-5 TRACE NUMERIC LEAF CANDIDATES]');
leafCandidates={};
leafCandidates=af7r57a_find_leaf_candidates(T,'TRACE_CAPTURE',leafCandidates,0,7);

af7r57a_line(' leaf-like numeric containers found      = %d',numel(leafCandidates));
for i=1:numel(leafCandidates)
    af7r57a_line('  %s',leafCandidates{i});
end

af7r57a_line('');
af7r57a_line('[I. SMALL NUMERIC VALUES RELEVANT TO STAGE-B]');
af7r57a_print_small_numeric(T,'TRACE_CAPTURE',0,6,48);
af7r57a_print_small_numeric(K,'KKT_CAPTURE',0,5,48);

sha1=af7r57a_sha256_file(srcFile);
prodSame=strcmpi(sha0,sha1);

schemaClosed=shadowExecClosed && traceCaptured && kktCaptured && ...
    ~isempty(traceRecords) && ~isempty(kktRecords);

af7r57a_line('');
af7r57a_line('[J. FORMAL GATE]');
af7r57a_line(' multiline declaration binding closed    = %d',declClosed);
af7r57a_line(' exact formal-input binding closed       = %d',argClosed);
af7r57a_line(' source-bound shadow interface closed    = %d', ...
    shadowDeclClosed && sameInputs && sameOutputs);
af7r57a_line(' runtime trace schema captured           = %d',traceCaptured);
af7r57a_line(' runtime interval-KKT schema captured    = %d',kktCaptured);
af7r57a_line(' append-only trace history nonempty      = %d',~isempty(traceRecords));
af7r57a_line(' append-only KKT history nonempty        = %d',~isempty(kktRecords));
af7r57a_line(' schema capture closed                   = %d',schemaClosed);
af7r57a_line(' production source SHA unchanged         = %d',prodSame);
af7r57a_line(' production source modified              = %d',~prodSame);
af7r57a_line(' production t1 selected                  = 0');
af7r57a_line(' continuous terminal-boundary proof      = 0');
af7r57a_line(' next theorem                            = COMMON_DYADIC_REFINEMENT_INTERVAL_STAGEB');
af7r57a_line('====================================================================================================');

R.status='CLOSED';
R.shadow_source=shadowFile;
R.shadow_sha256=shadowHash;
R.shadow_output_meta=outputMeta;
R.trace_capture=traceMat;
R.kkt_capture=kktMat;
R.trace_record_count=numel(traceRecords);
R.kkt_record_count=numel(kktRecords);
R.trace_candidate_ids=traceCandidates;
R.kkt_candidate_ids=kktCandidates;
R.trace_schema_captured=traceCaptured;
R.kkt_schema_captured=kktCaptured;
R.schema_capture_closed=schemaClosed;
R.production_source_unchanged=prodSame;
R.productionModified=~prodSame;
R.leaf_candidates=leafCandidates;

save(fullfile(baseDir,'AF7R57A_SCHEMA_CAPTURE_V7_REPORT.mat'),'R');
end


function af7r57a_line(fmt,varargin)
if isempty(fmt)
    fprintf('%c',10);
else
    fprintf([fmt '%c'],varargin{:},10);
end
end


function srcFile=af7r57a_find_source()
srcFile='';

w=which('AF7R56B_STAGEB_BOUNDARY_TRACE');
if ~isempty(w)
    srcFile=w;
    return;
end

roots={pwd,fileparts(mfilename('fullpath'))};

p=which('AF7R56_PRECHECK');
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

for ir=1:numel(roots)
    if ~isfolder(roots{ir})
        continue;
    end

    d=dir(fullfile(roots{ir},'AF7R56B_STAGEB_BOUNDARY_TRACE.m'));
    if ~isempty(d)
        srcFile=fullfile(d(1).folder,d(1).name);
        return;
    end
end

error('AF7R57A:SourceNotFound','AF7R56B_STAGEB_BOUNDARY_TRACE.m not found.');
end


function [D,closed]=af7r57a_parse_main_declaration(src)
[D,closed]=af7r57a_parse_named_declaration(src,'AF7R56B_STAGEB_BOUNDARY_TRACE');
end


function [D,closed]=af7r57a_parse_named_declaration(src,name)
D=struct( ...
    'raw','','declaration_one_line','','inputs',{{}},'outputs',{{}}, ...
    'start_index',NaN,'stop_index',NaN,'name_start',NaN,'name_stop',NaN, ...
    'physical_line_count',0);
closed=false;

normSrc=strrep(src,sprintf('\r\n'),sprintf('\n'));
normSrc=strrep(normSrc,sprintf('\r'),sprintf('\n'));
lines=regexp(normSrc,sprintf('\n'),'split');

startLine=NaN;
for i=1:numel(lines)
    s=strtrim(lines{i});
    if startsWith(s,'function') && contains(s,name)
        startLine=i;
        break;
    end
end

if isnan(startLine)
    return;
end

parts={};
paren=0;
brack=0;
brace=0;
seenName=false;

for i=startLine:min(numel(lines),startLine+30)
    ln=lines{i};
    parts{end+1}=ln; %#ok<AGROW>

    codePart=af7r57a_strip_comment_preserve_strings(ln);
    seenName=seenName || contains(codePart,name);

    [dp,db,dc]=af7r57a_delimiter_delta(codePart);
    paren=paren+dp;
    brack=brack+db;
    brace=brace+dc;

    continued=endsWith(strtrim(codePart),'...');

    if seenName && ~continued && paren==0 && brack==0 && brace==0
        break;
    end
end

if isempty(parts)
    return;
end

rawNorm=strjoin(parts,sprintf('\n'));

lastCode=af7r57a_strip_comment_preserve_strings(parts{end});
if endsWith(strtrim(lastCode),'...') || paren~=0 || brack~=0 || brace~=0
    return;
end

one=regexprep(rawNorm,'\.\.\.\s*\n',' ');
one=regexprep(one,'\s+',' ');
one=strtrim(one);

if ~startsWith(one,'function')
    return;
end

body=char(strtrim(extractAfter(string(one),'function')));
eqPos=af7r57a_find_top_level_equal(body);

if isempty(eqPos)
    outPart='';
    callPart=strtrim(body);
else
    outPart=strtrim(body(1:eqPos-1));
    callPart=strtrim(body(eqPos+1:end));
end

outs=af7r57a_parse_outputs(outPart);
[fnName,ins,callClosed]=af7r57a_parse_call_signature(callPart);

if ~callClosed || ~strcmp(fnName,name)
    return;
end

for i=1:numel(ins)
    if isempty(regexp(ins{i},'^[A-Za-z]\w*$','once'))
        return;
    end
end

for i=1:numel(outs)
    if strcmp(outs{i},'~')
        continue;
    end
    if isempty(regexp(outs{i},'^[A-Za-z]\w*$','once'))
        return;
    end
end

nameHits=strfind(src,name);
if isempty(nameHits)
    return;
end

chosen=NaN;
for k=1:numel(nameHits)
    ns=nameHits(k);
    ls=ns;

    while ls>1 && src(ls-1)~=char(10) && src(ls-1)~=char(13)
        ls=ls-1;
    end

    le=ns;
    while le<=numel(src) && src(le)~=char(10) && src(le)~=char(13)
        le=le+1;
    end

    head=strtrim(src(ls:le-1));
    if startsWith(head,'function')
        chosen=ns;
        break;
    end
end

if isnan(chosen)
    return;
end

declStart=chosen;
while declStart>1 && src(declStart-1)~=char(10) && src(declStart-1)~=char(13)
    declStart=declStart-1;
end

idx=declStart;
origParts={};
paren=0;
brack=0;
brace=0;
seenName=false;
declStop=NaN;

while idx<=numel(src) && numel(origParts)<40
    j=idx;

    while j<=numel(src) && src(j)~=char(10) && src(j)~=char(13)
        j=j+1;
    end

    ln=src(idx:j-1);
    origParts{end+1}=ln; %#ok<AGROW>

    codePart=af7r57a_strip_comment_preserve_strings(ln);
    seenName=seenName || contains(codePart,name);

    [dp,db,dc]=af7r57a_delimiter_delta(codePart);
    paren=paren+dp;
    brack=brack+db;
    brace=brace+dc;

    continued=endsWith(strtrim(codePart),'...');

    if seenName && ~continued && paren==0 && brack==0 && brace==0
        declStop=j-1;
        break;
    end

    if j>numel(src)
        return;
    end

    if src(j)==char(13) && j<numel(src) && src(j+1)==char(10)
        idx=j+2;
    else
        idx=j+1;
    end
end

if isnan(declStop)
    return;
end

localName=strfind(src(declStart:declStop),name);
if numel(localName)~=1
    return;
end

nameStart=declStart+localName(1)-1;
nameStop=nameStart+numel(name)-1;

D.raw=src(declStart:declStop);
D.declaration_one_line=one;
D.inputs=ins;
D.outputs=outs;
D.start_index=declStart;
D.stop_index=declStop;
D.name_start=nameStart;
D.name_stop=nameStop;
D.physical_line_count=numel(origParts);
closed=true;
end


function s=af7r57a_strip_comment_preserve_strings(s)
out='';
inString=false;
i=1;

while i<=numel(s)
    ch=s(i);

    if inString
        out(end+1)=ch; %#ok<AGROW>

        if ch==''''
            if i<numel(s) && s(i+1)==''''
                out(end+1)=s(i+1); %#ok<AGROW>
                i=i+2;
                continue;
            else
                inString=false;
            end
        end

        i=i+1;
        continue;
    end

    if ch=='%'
        break;
    end

    if ch==''''
        inString=true;
    end

    out(end+1)=ch; %#ok<AGROW>
    i=i+1;
end

s=out;
end


function [dp,db,dc]=af7r57a_delimiter_delta(s)
dp=0;
db=0;
dc=0;
inString=false;
i=1;

while i<=numel(s)
    ch=s(i);

    if inString
        if ch==''''
            if i<numel(s) && s(i+1)==''''
                i=i+2;
                continue;
            else
                inString=false;
            end
        end

        i=i+1;
        continue;
    end

    if ch==''''
        inString=true;
        i=i+1;
        continue;
    end

    if ch=='('
        dp=dp+1;
    elseif ch==')'
        dp=dp-1;
    elseif ch=='['
        db=db+1;
    elseif ch==']'
        db=db-1;
    elseif ch=='{'
        dc=dc+1;
    elseif ch=='}'
        dc=dc-1;
    end

    i=i+1;
end
end


function eqPos=af7r57a_find_top_level_equal(s)
eqPos=[];
paren=0;
brack=0;
brace=0;

for i=1:numel(s)
    ch=s(i);

    if ch=='('
        paren=paren+1;
    elseif ch==')'
        paren=paren-1;
    elseif ch=='['
        brack=brack+1;
    elseif ch==']'
        brack=brack-1;
    elseif ch=='{'
        brace=brace+1;
    elseif ch=='}'
        brace=brace-1;
    elseif ch=='=' && paren==0 && brack==0 && brace==0
        eqPos=i;
        return;
    end
end
end


function outs=af7r57a_parse_outputs(s)
outs={};

if isempty(s)
    return;
end

s=strtrim(s);

if startsWith(s,'[')
    if ~endsWith(s,']')
        return;
    end

    body=strtrim(s(2:end-1));

    if isempty(body)
        return;
    end

    outs=regexp(body,',','split');
    outs=cellfun(@strtrim,outs,'UniformOutput',false);
else
    outs={s};
end
end


function [name,ins,closed]=af7r57a_parse_call_signature(s)
name='';
ins={};
closed=false;
s=strtrim(s);

p=strfind(s,'(');

if isempty(p)
    if isempty(regexp(s,'^[A-Za-z]\w*$','once'))
        return;
    end

    name=s;
    closed=true;
    return;
end

if numel(p)~=1 || ~endsWith(s,')')
    return;
end

name=strtrim(s(1:p(1)-1));

if isempty(regexp(name,'^[A-Za-z]\w*$','once'))
    return;
end

body=strtrim(s(p(1)+1:end-1));

if isempty(body)
    closed=true;
    return;
end

ins=regexp(body,',','split');
ins=cellfun(@strtrim,ins,'UniformOutput',false);
closed=true;
end


function [args,sources,closed,B]=af7r57a_resolve_call_args(formals,userArgs)
n=numel(formals);
args=cell(1,n);
sources=repmat({'UNRESOLVED'},1,n);
closed=false;

B=struct();
B.base_replanner_exists=false;
B.base_replanner_instance_exists=false;
B.base_B49_exact_exists=false;
B.base_B49_canonical_exists=false;
B.base_R50A_exact_exists=false;
B.base_R50A_canonical_exists=false;
B.predecessor_consistency_closed=false;
B.replanning_system_object_count=0;
B.replanning_system_object_names={};

if evalin('base','exist(''replanner'',''var'')==1')
    B.base_replanner_exists=true;
end

if evalin('base','exist(''replanner_instance'',''var'')==1')
    B.base_replanner_instance_exists=true;
end

B.base_B49_exact_exists=evalin('base','exist(''B49'',''var'')==1');
B.base_B49_canonical_exists=evalin('base', ...
    'exist(''AF7R49B_REJOIN_DOMAIN_FORENSIC_OUT'',''var'')==1');
B.base_R50A_exact_exists=evalin('base','exist(''R50A'',''var'')==1');
B.base_R50A_canonical_exists=evalin('base', ...
    'exist(''AF7R50A_TERMINAL_COMPONENT_SPEED_THEOREM_OUT'',''var'')==1');

W=evalin('base','whos');

for k=1:numel(W)
    if strcmp(W(k).class,'REPLANNING_SYSTEM')
        B.replanning_system_object_count=B.replanning_system_object_count+1;
        B.replanning_system_object_names{end+1}=W(k).name; %#ok<AGROW>
    end
end

if ~isempty(userArgs)
    if numel(userArgs)~=n
        return;
    end

    args=userArgs;

    for i=1:n
        sources{i}='CALLER_EXPLICIT';
    end

    closed=true;
    return;
end

for i=1:n
    name=formals{i};

    exactExists=evalin('base',sprintf('exist(''%s'',''var'')==1',name));

    if exactExists
        args{i}=evalin('base',name);
        sources{i}='BASE_EXACT_FORMAL_NAME';
        continue;
    end

    if strcmp(name,'replanner')
        if evalin('base','exist(''replanner_instance'',''var'')==1')
            x=evalin('base','replanner_instance');

            if isa(x,'REPLANNING_SYSTEM')
                args{i}=x;
                sources{i}='BASE_ALIAS:replanner_instance';
                continue;
            end
        end

        if B.replanning_system_object_count==1
            nm=B.replanning_system_object_names{1};
            x=evalin('base',nm);

            if isa(x,'REPLANNING_SYSTEM')
                args{i}=x;
                sources{i}=['BASE_UNIQUE_CLASS:' nm];
                continue;
            end
        end
    end

    if strcmp(name,'B49')
        if evalin('base', ...
                'exist(''AF7R49B_REJOIN_DOMAIN_FORENSIC_OUT'',''var'')==1')
            x=evalin('base','AF7R49B_REJOIN_DOMAIN_FORENSIC_OUT');

            if af7r57a_valid_B49(x)
                args{i}=x;
                sources{i}='BASE_CANONICAL:AF7R49B_REJOIN_DOMAIN_FORENSIC_OUT';
                continue;
            end
        end
    end

    if strcmp(name,'R50A')
        if evalin('base', ...
                'exist(''AF7R50A_TERMINAL_COMPONENT_SPEED_THEOREM_OUT'',''var'')==1')
            x=evalin('base','AF7R50A_TERMINAL_COMPONENT_SPEED_THEOREM_OUT');

            if af7r57a_valid_R50A(x)
                args{i}=x;
                sources{i}='BASE_CANONICAL:AF7R50A_TERMINAL_COMPONENT_SPEED_THEOREM_OUT';
                continue;
            end
        end
    end

    args{i}=[];
    return;
end

% Cross-check predecessor semantics when the canonical three-input contract is used.
ir=find(strcmp(formals,'replanner'),1);
ib=find(strcmp(formals,'B49'),1);
ia=find(strcmp(formals,'R50A'),1);

if ~isempty(ir) && ~isempty(ib) && ~isempty(ia)
    B.predecessor_consistency_closed=af7r57a_predecessor_consistent( ...
        args{ir},args{ib},args{ia});

    if ~B.predecessor_consistency_closed
        closed=false;
        return;
    end
else
    B.predecessor_consistency_closed=true;
end

closed=true;
end


function tf=af7r57a_valid_B49(x)
tf=isstruct(x) && isscalar(x) && ...
    isfield(x,'schema') && ...
    contains(char(x.schema),'AF7R49B_REJOIN_DOMAIN_FORENSIC') && ...
    isfield(x,'t1') && isfield(x,'success') && isfield(x,'reason') && ...
    isfield(x,'current_t1') && isfield(x,'production_modified') && ...
    ~logical(x.production_modified);

if ~tf
    return;
end

t=double(x.t1(:));
p=logical(x.success(:));
r=double(x.reason(:));

tf=~isempty(t) && numel(t)==numel(p) && numel(p)==numel(r) && ...
    all(isfinite(t)) && all(diff(t)>0) && ...
    isfinite(double(x.current_t1)) && ...
    double(x.current_t1)==t(end);
end


function tf=af7r57a_valid_R50A(x)
tf=isstruct(x) && isscalar(x) && ...
    isfield(x,'schema') && ...
    contains(char(x.schema),'AF7R50A_TERMINAL_COMPONENT_SPEED_THEOREM') && ...
    isfield(x,'current_upper_t1') && ...
    isfield(x,'earliest_grid_certified_t1') && ...
    isfield(x,'VQ_limit') && ...
    isfield(x,'production_modified') && ...
    ~logical(x.production_modified);

if ~tf
    return;
end

tf=isfinite(double(x.current_upper_t1)) && ...
   isfinite(double(x.earliest_grid_certified_t1)) && ...
   isfinite(double(x.VQ_limit));
end


function tf=af7r57a_predecessor_consistent(replanner,B49,R50A)
tf=isa(replanner,'REPLANNING_SYSTEM') && ...
    af7r57a_valid_B49(B49) && af7r57a_valid_R50A(R50A);

if ~tf
    return;
end

t=double(B49.t1(:));
p=logical(B49.success(:));

if isempty(t) || ~any(p)
    tf=false;
    return;
end

first=t(find(p,1,'first'));

tf = double(R50A.current_upper_t1)==double(B49.current_t1) && ...
     double(R50A.current_upper_t1)==t(end) && ...
     double(R50A.earliest_grid_certified_t1)==first;
end


function af7r57a_print_required_call(formals)
if isempty(formals)
    af7r57a_line(' required invocation                     = AF7R57A_STAGEB_SCHEMA_CAPTURE()');
else
    af7r57a_line(' required invocation                     = AF7R57A_STAGEB_SCHEMA_CAPTURE(%s)', ...
        strjoin(formals,','));
end
end


function [s,closed]=af7r57a_rename_main_exact(src,D,newName)
s=src;
closed=false;

if ~(isfinite(D.name_start) && isfinite(D.name_stop))
    return;
end

old='AF7R56B_STAGEB_BOUNDARY_TRACE';

if ~strcmp(src(D.name_start:D.name_stop),old)
    return;
end

s=[src(1:D.name_start-1) newName src(D.name_stop+1:end)];
closed=true;
end


function t=af7r57a_trace_helper_text()
q={ ...
'function af7r57a_v7_shadow_save_trace(TL,TU,U,pkt,lo,hi,tU,c)', ...
'root=fileparts(mfilename(''fullpath''));', ...
'fn=fullfile(root,''AF7R57A_TRACE_CAPTURE_V7.mat'');', ...
'R=struct();', ...
'R.TL=TL; R.TU=TU; R.U=U; R.pkt=pkt; R.lo=lo; R.hi=hi; R.tU=tU; R.c=c;', ...
'if exist(fn,''file'')', ...
'    S=load(fn,''records'');', ...
'    if isfield(S,''records''), records=S.records; else, records={}; end', ...
'else', ...
'    records={};', ...
'end', ...
'records{end+1}=R;', ...
'save(fn,''records'',''-v7.3'');', ...
'end'};
t=strjoin(q,newline);
end


function t=af7r57a_kkt_helper_text()
q={ ...
'function af7r57a_v7_shadow_save_kkt(K,U,pkt,lo,hi,tU,A,b,ac0,signTag,c)', ...
'root=fileparts(mfilename(''fullpath''));', ...
'fn=fullfile(root,''AF7R57A_KKT_CAPTURE_V7.mat'');', ...
'R=struct();', ...
'R.K=K; R.U=U; R.pkt=pkt; R.lo=lo; R.hi=hi; R.tU=tU;', ...
'R.A=A; R.b=b; R.ac0=ac0; R.signTag=signTag; R.c=c;', ...
'if exist(fn,''file'')', ...
'    S=load(fn,''records'');', ...
'    if isfield(S,''records''), records=S.records; else, records={}; end', ...
'else', ...
'    records={};', ...
'end', ...
'records{end+1}=R;', ...
'save(fn,''records'',''-v7.3'');', ...
'end'};
t=strjoin(q,newline);
end


function M=af7r57a_value_meta(C)
M=cell(size(C));

for i=1:numel(C)
    M{i}=struct('class',class(C{i}),'size',size(C{i}));
end
end


function records=af7r57a_extract_records(S,name)
records={};

if isstruct(S) && isfield(S,name) && iscell(S.(name))
    records=S.(name);
end
end


function ids=af7r57a_record_candidate_ids(records)
ids=zeros(1,0);

for i=1:numel(records)
    r=records{i};

    if isstruct(r) && isfield(r,'c') && ...
            isnumeric(r.c) && isscalar(r.c) && isfinite(r.c)
        ids(end+1)=double(r.c); %#ok<AGROW>
    end
end
end


function af7r57a_print_schema(name,x,depth,maxDepth)
indent=repmat(' ',1,2*depth);

if depth>maxDepth
    af7r57a_line('%s%s : <max depth>',indent,name);
    return;
end

if isstruct(x)
    af7r57a_line('%s%s : struct %s',indent,name,mat2str(size(x)));
    f=fieldnames(x);

    for i=1:numel(f)
        if numel(x)==1
            af7r57a_print_schema(f{i},x.(f{i}),depth+1,maxDepth);
        else
            af7r57a_line('%s  %s : field over struct array',indent,f{i});
        end
    end

elseif iscell(x)
    af7r57a_line('%s%s : cell %s',indent,name,mat2str(size(x)));
    n=min(numel(x),6);

    for i=1:n
        af7r57a_print_schema(sprintf('{%d}',i),x{i},depth+1,maxDepth);
    end

    if numel(x)>n
        af7r57a_line('%s  ... %d more cells',indent,numel(x)-n);
    end

elseif isnumeric(x) || islogical(x)
    af7r57a_line('%s%s : %s %s',indent,name,class(x),mat2str(size(x)));

elseif ischar(x)
    af7r57a_line('%s%s : char %s',indent,name,mat2str(size(x)));

elseif isstring(x)
    af7r57a_line('%s%s : string %s',indent,name,mat2str(size(x)));

else
    af7r57a_line('%s%s : %s %s',indent,name,class(x),mat2str(size(x)));
end
end


function out=af7r57a_find_leaf_candidates(x,path,out,depth,maxDepth)
if depth>maxDepth
    return;
end

if isnumeric(x) && ismatrix(x) && ~isempty(x)
    sz=size(x);

    if sz(2)>=2 && sz(2)<=16
        v=double(x);
        finite=all(isfinite(v(:)));
        in01=finite && all(v(:)>=0 & v(:)<=1);

        if in01
            out{end+1}=sprintf('%s : numeric %s all values in [0,1]', ...
                path,mat2str(sz)); %#ok<AGROW>

        elseif sz(2)>=3
            for a=1:sz(2)
                for b=1:sz(2)
                    for c=1:sz(2)
                        if numel(unique([a b c]))<3
                            continue;
                        end

                        seg=v(:,a);
                        lo=v(:,b);
                        hi=v(:,c);

                        ok=all(isfinite(seg)) && all(seg==fix(seg)) && all(seg>=1) && ...
                           all(isfinite(lo)) && all(isfinite(hi)) && ...
                           all(lo>=0 & hi<=1 & lo<hi);

                        if ok
                            out{end+1}=sprintf( ...
                                '%s : numeric %s possible [segment,u0,u1] columns=[%d %d %d]', ...
                                path,mat2str(sz),a,b,c); %#ok<AGROW>
                            return;
                        end
                    end
                end
            end
        end
    end

elseif isstruct(x)
    if numel(x)==1
        f=fieldnames(x);

        for i=1:numel(f)
            out=af7r57a_find_leaf_candidates(x.(f{i}), ...
                [path '.' f{i}],out,depth+1,maxDepth);
        end

    else
        f=fieldnames(x);

        for i=1:min(numel(f),16)
            try
                vals={x.(f{i})};

                if all(cellfun(@(z)isnumeric(z)&&isscalar(z),vals))
                    vv=cell2mat(vals(:));

                    if all(isfinite(vv)) && all(vv>=0 & vv<=1)
                        out{end+1}=sprintf( ...
                            '%s.%s : scalar numeric field over struct %s in [0,1]', ...
                            path,f{i},mat2str(size(x))); %#ok<AGROW>
                    end
                end
            catch
            end
        end
    end

elseif iscell(x)
    for i=1:min(numel(x),12)
        out=af7r57a_find_leaf_candidates(x{i}, ...
            sprintf('%s{%d}',path,i),out,depth+1,maxDepth);
    end
end
end


function af7r57a_print_small_numeric(x,path,depth,maxDepth,maxElems)
if depth>maxDepth
    return;
end

if isnumeric(x) || islogical(x)
    if ~isempty(x) && numel(x)<=maxElems
        af7r57a_line(' %s = %s',path,mat2str(double(x),17));
    end

elseif isstruct(x)
    if numel(x)==1
        f=fieldnames(x);

        for i=1:numel(f)
            af7r57a_print_small_numeric(x.(f{i}), ...
                [path '.' f{i}],depth+1,maxDepth,maxElems);
        end

    else
        f=fieldnames(x);

        for i=1:numel(f)
            try
                vals={x.(f{i})};

                if all(cellfun(@(z)isnumeric(z)&&isscalar(z),vals)) && ...
                        numel(vals)<=maxElems
                    af7r57a_line(' %s.%s[:] = %s',path,f{i}, ...
                        mat2str(cell2mat(vals),17));
                end
            catch
            end
        end
    end

elseif iscell(x)
    for i=1:min(numel(x),12)
        af7r57a_print_small_numeric(x{i}, ...
            sprintf('%s{%d}',path,i),depth+1,maxDepth,maxElems);
    end
end
end


function h=af7r57a_sha256_file(fn)
md=java.security.MessageDigest.getInstance('SHA-256');
fid=fopen(fn,'rb');

if fid<0
    error('AF7R57A:Open','Cannot open %s',fn);
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
