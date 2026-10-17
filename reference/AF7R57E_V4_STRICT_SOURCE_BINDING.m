function OUT = AF7R57E_V4_STRICT_SOURCE_BINDING()
% AF7R57E_V3_STRICT_SOURCE_BINDING
% -----------------------------------------------------------------------------
% Strict source binding before the final outward interval Stage-B theorem.
%
% V4 correction: ActiveReference::eval performs correction derivatives
% inline rather than through deriv_poly(). The gate checks the actual inline
% degree-reduction/difference recursion.
%
% Fixes the V2 false-positive MATLAB extractor by requiring that a method
% name appears in the LOGICAL FUNCTION DECLARATION itself (continuations
% included), never merely in a call inside a preceding function.
%
% Also binds:
%   - scalar_control_from_jets
%   - vector_control_from_jets
%   - deriv_poly
%   - build_candidate
%   - make_vleaf
%   - ActiveReference::eval
%   - AF7R56B_PRIME_TRACE_REFERENCE exact source
%   - all af7r56_trace_mex call sites in the prime function
%
% No production modification. No sampled acceptance. No t1 selection.
% -----------------------------------------------------------------------------

clc;
fprintf('\n====================================================================================================\n');
fprintf(' AF7R57E V4 | STRICT SOURCE BINDING FOR OUTWARD INTERVAL STAGE-B\n');
fprintf('====================================================================================================\n');
fprintf('[AUTHORITY] readOnly=1 | strictDeclarations=1 | nativeSource=1 | primeSource=1 | sampledAcceptance=0 | productionModified=0\n');

OUT=struct();
OUT.schema='AF7R57E_V4_STRICT_SOURCE_BINDING_V1';
OUT.closed=false;
OUT.production_modified=false;
OUT.production_t1_selected=false;
OUT.continuous_terminal_boundary_proof=false;

%% A. Locate authority sources
fprintf('\n[A. SOURCE AUTHORITY]\n');

hpp=af7r57e3_find_source('af7r56_trace_core.hpp');
mexcpp=af7r57e3_find_source('af7r56_trace_mex.cpp');
rps=which('REPLANNING_SYSTEM');
prime=which('AF7R56B_PRIME_TRACE_REFERENCE');

assert(~isempty(hpp)&&isfile(hpp),'[AF7R57E-V4] trace core header missing.');
assert(~isempty(mexcpp)&&isfile(mexcpp),'[AF7R57E-V4] trace MEX source missing.');
assert(~isempty(rps)&&isfile(rps),'[AF7R57E-V4] active REPLANNING_SYSTEM missing.');
assert(~isempty(prime)&&isfile(prime),'[AF7R57E-V4] AF7R56B prime source missing.');

H0=af7r57e3_sha256_file(hpp);
X0=af7r57e3_sha256_file(mexcpp);
R0=af7r57e3_sha256_file(rps);
P0=af7r57e3_sha256_file(prime);

fprintf(' trace core header                       = %s\n',hpp);
fprintf(' trace core SHA-256                      = %s\n',H0);
fprintf(' trace MEX source                        = %s\n',mexcpp);
fprintf(' trace MEX SHA-256                       = %s\n',X0);
fprintf(' active REPLANNING_SYSTEM                = %s\n',rps);
fprintf(' replanner SHA-256                       = %s\n',R0);
fprintf(' prime source                            = %s\n',prime);
fprintf(' prime SHA-256                           = %s\n',P0);

%% B. Strict C++ exact bodies
fprintf('\n[B. STRICT C++ BODY BINDING]\n');

cppNames={ ...
    'scalar_control_from_jets', ...
    'vector_control_from_jets', ...
    'deriv_poly', ...
    'split_half', ...
    'decasteljau', ...
    'poly_norm_ub', ...
    'build_candidate', ...
    'make_vleaf', ...
    'stageB_speed', ...
    'stageB_trace', ...
    'candidate_eval'};

CPP=struct();
cppClosed=true;

for i=1:numel(cppNames)
    nm=cppNames{i};
    [txt,range,ok]=af7r57e3_extract_cpp_function(hpp,nm);
    CPP.(nm)=struct('closed',ok,'line_range',range,'text',txt);
    cppClosed=cppClosed&&ok;

    fprintf('\n ---- %s | closed=%d | lines=%s ----\n',nm,ok,mat2str(range));
    fprintf('%s\n',txt);
end

%% C. ActiveReference exact struct / eval body
fprintf('\n[C. ACTIVE REFERENCE EXACT BODY]\n');

[activeTxt,activeRange,activeClosed0]= ...
    af7r57e3_extract_cpp_struct(hpp,'ActiveReference');

fprintf(' ActiveReference closed                  = %d\n',activeClosed0);
fprintf(' ActiveReference lines                   = %s\n',mat2str(activeRange));
fprintf('%s\n',activeTxt);

activeHasNominal=contains(activeTxt,'nominal->eval');
activeHasRepairs=contains(activeTxt,'repairs');

% ActiveReference::eval differentiates the degree-15 correction Bezier
% inline. It does NOT call deriv_poly(). Bind the actual implementation
% semantics instead of a helper-function spelling.
activeHasDerivativeLoop = ...
    contains(activeTxt,'for(int r=1;r<=6;++r)') && ...
    contains(activeTxt,'E[i]=(D[i+1]-D[i])*(double(deg)/T)') && ...
    contains(activeTxt,'D.swap(E);--deg;y[r]+=decasteljau(D,u)');

activeHasDeCasteljau=contains(activeTxt,'decasteljau');
activeHasClamp=contains(activeTxt,'std::max(0.0,std::min(1.0,u))');
activeHasStrictRepairWindow= ...
    contains(activeTxt,'if(!(t>C.t0&&t<C.t1))continue;');

fprintf(' nominal evaluation source-bound         = %d\n',activeHasNominal);
fprintf(' immutable repair traversal source-bound = %d\n',activeHasRepairs);
fprintf(' correction derivative recursion bound   = %d\n',activeHasDerivativeLoop);
fprintf(' correction de Casteljau source-bound    = %d\n',activeHasDeCasteljau);
fprintf(' correction local-u clamp source-bound   = %d\n',activeHasClamp);
fprintf(' strict repair window source-bound       = %d\n',activeHasStrictRepairWindow);

%% D. Strict MATLAB declaration binding
fprintf('\n[D. STRICT MATLAB METHOD DECLARATION BINDING]\n');

matNames={ ...
    'piecewise_transition_jet_maps', ...
    'bezier15_control_from_endpoint_jets_scalar', ...
    'bezier15_control_map_from_jet_maps', ...
    'evaluate_control_map_xyz', ...
    'build_piecewise_bezier_control_points'};

MAT=struct();
matClosed=true;

for i=1:numel(matNames)
    nm=matNames{i};
    [txt,range,decl,ok]=af7r57e3_extract_matlab_method_strict(rps,nm);
    MAT.(nm)=struct('closed',ok,'line_range',range,'declaration',decl,'text',txt);
    matClosed=matClosed&&ok;

    fprintf('\n ---- %s | closed=%d | lines=%s ----\n',nm,ok,mat2str(range));
    fprintf(' declaration = %s\n',decl);
    fprintf('%s\n',txt);
end

scalarTxt=MAT.bezier15_control_from_endpoint_jets_scalar.text;
mapTxt=MAT.bezier15_control_map_from_jet_maps.text;
buildTxt=MAT.build_piecewise_bezier_control_points.text;

matSemantics = ...
    contains(scalarTxt,'Degree 15 is exactly determined by endpoint derivatives 0..7') && ...
    contains(scalarTxt,'T^r') && ...
    contains(mapTxt,'bezier15_control_from_endpoint_jets_scalar') && ...
    contains(buildTxt,'UNSAFE_AFFINE_TOWING_CORRIDOR') && ...
    contains(buildTxt,'EXIT_TO_END') && ...
    contains(buildTxt,'tb=double(t1)');

fprintf('\n strict MATLAB semantic binding          = %d\n',matSemantics);

%% E. Prime source exact capture
fprintf('\n[E. AF7R56B PRIME SOURCE]\n');

primeRaw=af7r57e3_normnl(fileread(prime));
fprintf('%s\n',primeRaw);

PL=splitlines(string(primeRaw));
rows=find(contains(PL,'af7r56_trace_mex'));

fprintf('\n trace-MEX call lines in prime            = %s\n',mat2str(double(rows(:).')));
for i=1:numel(rows)
    a=max(1,rows(i)-6);
    b=min(numel(PL),rows(i)+10);
    fprintf('\n [prime context around L%d]\n',rows(i));
    for q=a:b
        fprintf('  L%-4d %s\n',q,char(PL(q)));
    end
end

primeHas21 = contains(primeRaw,'21') || ...
             contains(lower(primeRaw),'scalar expressions') || ...
             contains(lower(primeRaw),'nominal');
primeHasTraceCall = ~isempty(rows);
primeHasSignature = contains(lower(primeRaw),'signature');

fprintf('\n prime nominal-expression source present = %d\n',primeHas21);
fprintf(' prime trace-MEX call source present     = %d\n',primeHasTraceCall);
fprintf(' prime signature binding present         = %d\n',primeHasSignature);

%% F. Current runtime nominal identity
fprintf('\n[F. CURRENT RUNTIME NOMINAL IDENTITY]\n');

rep=[];
if evalin('base','exist(''replanner_instance'',''var'')==1')
    q=evalin('base','replanner_instance');
    if isa(q,'REPLANNING_SYSTEM'), rep=q; end
end

repPresent=isa(rep,'REPLANNING_SYSTEM');
fprintf(' replanner_instance present              = %d\n',repPresent);

fstr='';
if repPresent
    try
        if isprop(rep,'base_ref') && ~isempty(rep.base_ref) && ...
                isprop(rep.base_ref,'func') && isa(rep.base_ref.func,'function_handle')
            fstr=func2str(rep.base_ref.func);
        end
    catch
    end
end

fprintf(' base_ref.func available                 = %d\n',~isempty(fstr));
if ~isempty(fstr)
    fprintf(' base_ref.func                           = %s\n',fstr);
end

nomSig='';
if evalin('base','exist(''AF7R57C_SNAPSHOT_BOUND_SCHEMA_CAPTURE_OUT'',''var'')==1')
    C=evalin('base','AF7R57C_SNAPSHOT_BOUND_SCHEMA_CAPTURE_OUT');
    if isfield(C,'trace_records')&&iscell(C.trace_records)&&~isempty(C.trace_records)
        Q=C.trace_records{1};
        if isfield(Q,'pkt')&&isfield(Q.pkt,'nominal_signature')
            nomSig=char(Q.pkt.nominal_signature);
        end
    end
end

fprintf(' AF7R57C nominal signature present       = %d\n',~isempty(nomSig));
if ~isempty(nomSig)
    fprintf(' AF7R57C nominal signature               = %s\n',nomSig);
end

%% G. Formal gate
fprintf('\n[G. STRICT SOURCE-BINDING GATE]\n');

shaStable=strcmpi(H0,af7r57e3_sha256_file(hpp)) && ...
          strcmpi(X0,af7r57e3_sha256_file(mexcpp)) && ...
          strcmpi(R0,af7r57e3_sha256_file(rps)) && ...
          strcmpi(P0,af7r57e3_sha256_file(prime));

activeClosed = activeClosed0 && activeHasNominal && activeHasRepairs && ...
    activeHasDerivativeLoop && activeHasDeCasteljau && ...
    activeHasClamp && activeHasStrictRepairWindow;

primeClosed = primeHas21 && primeHasTraceCall && primeHasSignature;

closed=cppClosed&&activeClosed&&matClosed&&matSemantics&&primeClosed&&shaStable;

fprintf(' strict C++ body binding closed          = %d\n',cppClosed);
fprintf(' ActiveReference binding closed          = %d\n',activeClosed);
fprintf(' strict MATLAB declaration binding       = %d\n',matClosed);
fprintf(' strict MATLAB semantic binding          = %d\n',matSemantics);
fprintf(' prime-source binding closed             = %d\n',primeClosed);
fprintf(' source SHA unchanged                    = %d\n',shaStable);
fprintf(' strict source binding closed            = %d\n',closed);
fprintf(' outward interval theorem implemented    = 0\n');
fprintf(' continuous terminal-boundary proof      = 0\n');
fprintf(' production t1 selected                  = 0\n');
fprintf(' production source modified              = 0\n');
fprintf(' unresolved policy                       = FAIL CLOSED\n');
fprintf(' next action                              = BUILD_OUTWARD_INTERVAL_MEX_USING_BOUND_NATIVE_SOURCE\n');
fprintf('====================================================================================================\n');

OUT.closed=closed;
OUT.cpp=CPP;
OUT.active_reference=struct('closed',activeClosed,'line_range',activeRange,'text',activeTxt);
OUT.matlab=MAT;
OUT.prime_source=prime;
OUT.prime_sha256=P0;
OUT.prime_text=primeRaw;
OUT.nominal_signature=nomSig;
OUT.base_ref_func=fstr;
OUT.production_modified=false;
OUT.production_t1_selected=false;
OUT.continuous_terminal_boundary_proof=false;

assignin('base','AF7R57E_V3_STRICT_SOURCE_BINDING_OUT',OUT);
save('AF7R57E_V3_STRICT_SOURCE_BINDING_OUT.mat','OUT','-v7.3');
end


function [txt,range,ok]=af7r57e3_extract_cpp_function(fn,name)
raw=af7r57e3_normnl(fileread(fn));
L=splitlines(string(raw));
txt='';range=[NaN NaN];ok=false;

hits=find(contains(L,[name '(']));

for hh=hits(:).'
    s0=hh;
    for j=hh:-1:max(1,hh-4)
        x=strtrim(char(L(j)));
        if startsWith(x,'static ') || startsWith(x,'inline ') || ...
                contains(x,' static ') || contains(x,' inline ')
            s0=j;
            break;
        end
    end

    decl=char(strjoin(L(s0:hh),newline));
    if ~(contains(decl,name) && contains(decl,'('))
        continue;
    end

    if ~contains(decl,'{') && ...
            ~(contains(decl,'static')||contains(decl,'inline'))
        continue;
    end

    [stop,bodyok]=af7r57e3_cpp_brace_stop(L,s0);
    if bodyok
        txt=char(strjoin(L(s0:stop),newline));
        range=[s0 stop];
        ok=true;
        return;
    end
end
end


function [txt,range,ok]=af7r57e3_extract_cpp_struct(fn,name)
raw=af7r57e3_normnl(fileread(fn));
L=splitlines(string(raw));
txt='';range=[NaN NaN];ok=false;

hits=find(contains(L,['struct ' name]));
if isempty(hits), return; end

s0=hits(1);
[stop,bodyok]=af7r57e3_cpp_brace_stop(L,s0);

if bodyok
    txt=char(strjoin(L(s0:stop),newline));
    range=[s0 stop];
    ok=true;
end
end


function [stop,ok]=af7r57e3_cpp_brace_stop(L,start)
depth=0;started=false;stop=NaN;ok=false;
inBlock=false;inString=false;inChar=false;

for j=start:numel(L)
    s=char(L(j));k=1;
    while k<=numel(s)
        ch=s(k);nx=char(0);
        if k<numel(s), nx=s(k+1); end

        if inBlock
            if ch=='*'&&nx=='/'
                inBlock=false;k=k+2;continue;
            end
            k=k+1;continue;
        end
        if inString
            if ch=='\'
                k=k+2;continue;
            elseif ch=='"'
                inString=false;
            end
            k=k+1;continue;
        end
        if inChar
            if ch=='\'
                k=k+2;continue;
            elseif double(ch)==39
                inChar=false;
            end
            k=k+1;continue;
        end

        if ch=='/'&&nx=='*'
            inBlock=true;k=k+2;continue;
        elseif ch=='/'&&nx=='/'
            break;
        elseif ch=='"'
            inString=true;
        elseif double(ch)==39
            inChar=true;
        elseif ch=='{'
            depth=depth+1;started=true;
        elseif ch=='}'&&started
            depth=depth-1;
            if depth==0
                stop=j;ok=true;return;
            end
        end
        k=k+1;
    end
end
end


function [txt,range,decl,ok]=af7r57e3_extract_matlab_method_strict(fn,name)
raw=af7r57e3_normnl(fileread(fn));
L=splitlines(string(raw));
txt='';range=[NaN NaN];decl='';ok=false;

starts=find(startsWith(strtrim(L),'function '));

for ii=1:numel(starts)
    s0=starts(ii);
    j=s0;
    D=strtrim(char(L(j)));

    while endsWith(strtrim(D),'...')
        if j>=numel(L), break; end
        j=j+1;
        nxt=strtrim(char(L(j)));
        D=[D newline nxt]; %#ok<AGROW>
    end

    if ~contains(D,name)
        continue;
    end

    q=regexprep(D,'\s+',' ');
    if isempty(regexp(q,[regexptranslate('escape',name) '\s*\('],'once'))
        continue;
    end

    if ii<numel(starts)
        stop=starts(ii+1)-1;
    else
        stop=numel(L);
    end

    body=char(strjoin(L(s0:stop),newline));

    if contains(body,name) && contains(body,'end')
        txt=body;
        range=[s0 stop];
        decl=q;
        ok=true;
        return;
    end
end
end


function fn=af7r57e3_find_source(name)
fn='';
p=which(name);
if ~isempty(p)&&isfile(p),fn=p;return;end

roots={pwd,fileparts(mfilename('fullpath'))};
p=which('af7r56_trace_mex');
if ~isempty(p),roots{end+1}=fileparts(p);end %#ok<AGROW>
p=which('AF7R56B_STAGEB_BOUNDARY_TRACE');
if ~isempty(p),roots{end+1}=fileparts(p);end %#ok<AGROW>
u=getenv('USERPROFILE');
if ~isempty(u)
    roots{end+1}=fullfile(u,'Documents','GitHub','common_matlab','reference'); %#ok<AGROW>
end

for i=1:numel(roots)
    q=fullfile(roots{i},name);
    if isfile(q),fn=q;return;end
end
end


function s=af7r57e3_normnl(s)
s=strrep(s,sprintf('\r\n'),sprintf('\n'));
s=strrep(s,sprintf('\r'),sprintf('\n'));
end


function h=af7r57e3_sha256_file(fn)
md=java.security.MessageDigest.getInstance('SHA-256');
fid=fopen(fn,'rb');
assert(fid>=0,'[AF7R57E-V4] cannot open source.');
c=onCleanup(@() fclose(fid));
while true
    b=fread(fid,1024*1024,'*uint8');
    if isempty(b),break;end
    md.update(b);
end
raw=typecast(md.digest(),'uint8');
h=lower(reshape(dec2hex(raw,2).',1,[]));
end
