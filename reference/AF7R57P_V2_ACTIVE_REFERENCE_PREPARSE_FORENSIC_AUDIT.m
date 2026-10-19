function OUT = AF7R57P_V2_ACTIVE_REFERENCE_PREPARSE_FORENSIC_AUDIT()
clc;
fprintf('\n====================================================================================================\n');
fprintf(' AF7R57P V2 | ACTIVE-REFERENCE PRE-PARSE / EVAL FORENSIC AUDIT\n');
fprintf('====================================================================================================\n');
fprintf('[AUTHORITY] readOnly=1 | sourceBound=1 | noGuardInstall=1 | noTimingClaimUpgrade=1 | productionModified=0\n');

root=fileparts(mfilename('fullpath'));
selfFile=[mfilename('fullpath') '.m'];
selfSHA=sha256_file(selfFile);
fprintf(' immutable entrypoint                    = %s\n',selfFile);
fprintf(' immutable entrypoint SHA-256             = %s\n',selfSHA);

coreK=fullfile(root,'af7r57k_trace_core.hpp');
serK=fullfile(root,'af7r57k_corrected_core_mex.cpp');
rps=fullfile(root,'REPLANNING_SYSTEM.m');
assert(isfile(coreK)&&isfile(serK)&&isfile(rps),'[AF7R57P] bound source missing.');

expectedCore='4f99d3ba7bdd5645a851a1c2236a8239753631d47998760cc6939a0655a344e1';
expectedSer ='3b774a4aa1b18646182d1f7fa6d01bd2c6669e5ad2e6a8200f702f2f8e6f8597';
expectedRps ='223d34c49d7148b8ea14ff4ab9c420d409560c0650ac19e9b0f99318887090c9';
shaCore=sha256_file(coreK); shaSer=sha256_file(serK); shaRps=sha256_file(rps);

fprintf('\n[A. EXACT SOURCE AUTHORITY]\n');
fprintf(' R57K corrected core current             = %d\n',strcmp(shaCore,expectedCore));
fprintf(' R57K corrected serializer current       = %d\n',strcmp(shaSer,expectedSer));
fprintf(' REPLANNING_SYSTEM current               = %d\n',strcmp(shaRps,expectedRps));
assert(strcmp(shaCore,expectedCore)&&strcmp(shaSer,expectedSer)&&strcmp(shaRps,expectedRps), ...
    '[AF7R57P] source differs from bound authority.');

core=normnl(fileread(coreK));
ser=normnl(fileread(serK));
sys=normnl(fileread(rps));

parser_selftest();
fprintf(' parser lexical self-test                = 1\n');

[parseBody,parseL0,parseL1,parseDecl]=cpp_function_body(ser,'parse_ref');
[evalBody,evalL0,evalL1]=active_eval_body(core);

fprintf('\n[B. EXACT parse_ref BODY]\n');
fprintf(' parse_ref declaration                   = %s\n',strtrim(parseDecl));
fprintf(' parse_ref lines                         = [%d %d]\n',parseL0,parseL1);
fprintf('%s\n',numbered(parseBody,parseL0));

fprintf('\n[C. EXACT ActiveReference::eval BODY]\n');
fprintf(' ActiveReference::eval lines             = [%d %d]\n',evalL0,evalL1);
fprintf('%s\n',numbered(evalBody,evalL0));

fields={'repair_t0','repair_t1','repair_offsets','repair_seg0','repair_seg1','repair_cp','flight_t0'};
pres=false(size(fields));
for k=1:numel(fields), pres(k)=contains(parseBody,fields{k}); end
loopR=contains(evalBody,'for(const auto& C:repairs)');
loopS=contains(evalBody,'for(const auto& s:C.seg)');
deg15=contains(evalBody,'int deg=15');
dloop=contains(evalBody,'for(int r=1;r<=6;++r)');
copyCP=contains(evalBody,'Poly D=sp->cp');

fprintf('\n[D. REPAIR PACKET / EVAL WORK SEMANTICS]\n');
for k=1:numel(fields), fprintf(' parser field %-18s present = %d\n',fields{k},pres(k)); end
fprintf(' eval loops over repairs                 = %d\n',loopR);
fprintf(' eval loops over segments                = %d\n',loopS);
fprintf(' selected segment CP copied              = %d\n',copyCP);
fprintf(' fixed polynomial degree 15              = %d\n',deg15);
fprintf(' derivative recursion r=1..6             = %d\n',dloop);

op4=cpp_if_block(ser,'if(op==4)');
pRef=strfind(op4,'parse_ref(P)');
pLive=strfind(op4,'parse_live(P)');
pObs=strfind(op4,'parse_obs(P)');
preRepairGuard=contains(op4,'max_repair')||contains(op4,'repair_count')||contains(op4,'repair_segment')||contains(op4,'preparse');

fprintf('\n[E. op==4 PRE-PARSE ORDERING]\n');
fprintf(' parse_ref present                        = %d\n',~isempty(pRef));
fprintf(' parse_live present                       = %d\n',~isempty(pLive));
fprintf(' parse_obs present                        = %d\n',~isempty(pObs));
fprintf(' parse_ref before parse_live              = %d\n',~isempty(pRef)&&~isempty(pLive)&&pRef(1)<pLive(1));
fprintf(' explicit repair pre-admission            = %d\n',preRepairGuard);

fprintf('\n[F. REPLANNING_SYSTEM PACKET-PRODUCER VISIBILITY]\n');
hits=zeros(size(fields));
for k=1:numel(fields)
    hits(k)=numel(strfind(sys,fields{k}));
    fprintf(' %-18s source hit count = %d\n',fields{k},hits(k));
end

workModel=loopR&&loopS&&deg15&&dloop&&copyCP;
fprintf('\n[G. SOURCE-BOUND EVAL WORK MODEL]\n');
fprintf(' structural upper model                  = O(Nrepair + Nseg_total + Nselected*C15x7)\n');
fprintf(' selected correction arithmetic constant = %d\n',deg15&&dloop);
fprintf(' eval work model source-bound            = %d\n',workModel);

nonemptyRepairProof=contains(parseBody,'b<=a')||contains(parseBody,'b>a')||contains(parseBody,'segment count');
fprintf('\n[H. MINIMUM PRE-PARSE RESOURCE COORDINATES]\n');
fprintf(' nonempty-repair parser proof visible    = %d\n',nonemptyRepairProof);
if nonemptyRepairProof
    coords={'total_repair_segments'};
else
    coords={'repair_count','total_repair_segments'};
end
fprintf(' coordinate count                        = %d\n',numel(coords));
for k=1:numel(coords), fprintf(' coordinate %d                            = %s\n',k,coords{k}); end
fprintf(' current zero-repair values used as caps = 0\n');

fprintf('\n[I. FORENSIC GATE]\n');
fprintf(' exact parser body captured              = %d\n',~isempty(parseBody)&&all(pres));
fprintf(' exact eval body captured                = %d\n',workModel);
fprintf(' pre-parse admission currently absent    = %d\n',~preRepairGuard);
fprintf(' repaired-reference work theorem closed = 0\n');
fprintf(' production source modified              = 0\n');
fprintf(' production timing claim upgraded        = 0\n');
fprintf(' next action                              = DERIVE_SOURCE_BOUND_PREPARSE_SCHEMA__THEN_NONZERO_REPAIR_TESTS__THEN_QPC_ENVELOPE\n');
fprintf('====================================================================================================\n');

OUT=struct('schema','AF7R57P_V2_ACTIVE_REFERENCE_PREPARSE_FORENSIC_V2', ...
    'closed',true,'parse_lines',[parseL0 parseL1],'eval_lines',[evalL0 evalL1], ...
    'minimal_coordinates',{coords},'preparse_admission_absent',~preRepairGuard, ...
    'production_modified',false,'timing_claim_upgraded',false, ...
    'entrypoint',selfFile,'entrypoint_sha256',selfSHA);
assignin('base','AF7R57P_V2_ACTIVE_REFERENCE_PREPARSE_FORENSIC_AUDIT_OUT',OUT);
save(fullfile(root,'AF7R57P_V2_ACTIVE_REFERENCE_PREPARSE_FORENSIC_AUDIT_OUT.mat'),'OUT');
end

function [body,l0,l1,decl]=cpp_function_body(src,name)
L=splitlines(string(src)); hit=[];
for i=1:numel(L)
    q=char(L(i));
    if contains(q,[name '(']) && (contains(q,'static')||contains(q,'inline')) && ~contains(q,';')
        hit=i; break;
    end
end
assert(~isempty(hit),'[AF7R57P] definition %s not found.',name);
startPos=line_start_pos(src,hit); open=find_from(src,startPos,'{'); close=brace_match(src,open);
body=src(startPos:close); l0=hit; l1=line_no(src,close); decl=src(startPos:open-1);
end

function [body,l0,l1]=active_eval_body(src)
pS=strfind(src,'struct ActiveReference'); assert(~isempty(pS));
pE=strfind(src(pS(1):end),'eval(double t) const'); assert(~isempty(pE));
p=pS(1)+pE(1)-1; l0=line_no(src,p); startPos=line_start_pos(src,l0);
open=find_from(src,p,'{'); close=brace_match(src,open); body=src(startPos:close); l1=line_no(src,close);
end

function b=cpp_if_block(src,tok)
p=strfind(src,tok); assert(~isempty(p)); open=find_from(src,p(1),'{'); close=brace_match(src,open);
l0=line_no(src,p(1)); s=line_start_pos(src,l0); b=src(s:close);
end

function p=find_from(src,p0,ch)
q=strfind(src(p0:end),ch); assert(~isempty(q)); p=p0+q(1)-1;
end

function q=brace_match(src,p)
% C++ lexical brace matcher. Braces inside comments or literals are ignored.
% Character codes avoid MATLAB backslash-literal ambiguity.
BS=char(92); SQ=char(39); DQ=char(34); NL=char(10);
d=0; inSQ=false; inDQ=false; inLine=false; inBlock=false; esc=false;
i=p;
while i<=numel(src)
    c=src(i);
    n=char(0); if i<numel(src), n=src(i+1); end

    if inLine
        if c==NL, inLine=false; end
        i=i+1; continue;
    end
    if inBlock
        if c=='*' && n=='/', inBlock=false; i=i+2; else, i=i+1; end
        continue;
    end
    if inSQ || inDQ
        if esc, esc=false; i=i+1; continue; end
        if c==BS, esc=true; i=i+1; continue; end
        if inSQ && c==SQ, inSQ=false; i=i+1; continue; end
        if inDQ && c==DQ, inDQ=false; i=i+1; continue; end
        i=i+1; continue;
    end

    if c=='/' && n=='/', inLine=true; i=i+2; continue; end
    if c=='/' && n=='*', inBlock=true; i=i+2; continue; end
    if c==SQ, inSQ=true; i=i+1; continue; end
    if c==DQ, inDQ=true; i=i+1; continue; end

    if c=='{'
        d=d+1;
    elseif c=='}'
        d=d-1;
        if d==0, q=i; return; end
        assert(d>=0,'[AF7R57P V2] brace depth became negative.');
    end
    i=i+1;
end
error('[AF7R57P V2] unmatched brace.');
end

function parser_selftest()
% Braces inside C++ strings/comments must not affect matching.
BS=char(92); DQ=char(34);
x=['void f(){ ' ...
   'const char* a=' DQ 'text { } ' BS DQ ' quoted' BS DQ DQ '; ' ...
   '// comment { }' char(10) ...
   '/* block { } */ if(true){ int z=0; } }'];
p=strfind(x,'{'); assert(~isempty(p));
q=brace_match(x,p(1));
assert(q==numel(x),'[AF7R57P V2] lexical brace matcher self-test failed.');
end

function p=line_start_pos(src,n)
if n<=1, p=1; return; end
ix=find(src==newline,n-1,'first'); p=ix(end)+1;
end

function n=line_no(src,p)
n=1+sum(src(1:max(1,p)-1)==newline);
end

function s=numbered(block,l0)
L=splitlines(string(block)); O=strings(numel(L),1);
for i=1:numel(L), O(i)=sprintf(' L%-5d %s',l0+i-1,L(i)); end
s=strjoin(O,newline);
end

function s=normnl(s)
s=strrep(s,sprintf('\r\n'),sprintf('\n')); s=strrep(s,sprintf('\r'),sprintf('\n'));
end

function h=sha256_file(f)
fid=fopen(f,'r'); assert(fid>=0); c=onCleanup(@()fclose(fid));
b=fread(fid,Inf,'*uint8'); md=java.security.MessageDigest.getInstance('SHA-256');
md.update(b); d=typecast(md.digest(),'uint8'); h=lower(reshape(dec2hex(d,2).',1,[]));
end
