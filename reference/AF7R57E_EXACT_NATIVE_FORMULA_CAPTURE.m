function OUT = AF7R57E_EXACT_NATIVE_FORMULA_CAPTURE()
% AF7R57E_EXACT_NATIVE_FORMULA_CAPTURE
% -----------------------------------------------------------------------------
% Read-only exact source extraction needed to implement the outward
% [t1_L,t1_U] x common-leaf Stage-B theorem without algebraic guessing.
%
% Extracts from af7r56_trace_core.hpp:
%   split_half
%   decasteljau
%   poly_norm_ub
%   deriv_poly
%   vector_control_from_jets
%   build_candidate
%   make_vleaf
%   stageB_speed
%   stageB_trace
%   candidate_eval
%
% Extracts from the active REPLANNING_SYSTEM source:
%   piecewise_transition_jet_maps
%   bezier15_control_from_endpoint_jets_scalar
%   bezier15_control_map_from_jet_maps
%   evaluate_control_map_xyz
%   build_piecewise_bezier_control_points
%
% Also prints the exact KKT z-interval authority from AF7R57C.
%
% No production modification. No sampled acceptance. No t1 selection.
% -----------------------------------------------------------------------------

clc;
fprintf('\n====================================================================================================\n');
fprintf(' AF7R57E V2 | EXACT NATIVE FORMULA CAPTURE FOR INTERVAL STAGE-B\n');
fprintf('====================================================================================================\n');
fprintf('[AUTHORITY] readOnly=1 | exactSource=1 | noAlgebraGuess=1 | sampledAcceptance=0 | productionModified=0\n');

OUT=struct();
OUT.schema='AF7R57E_EXACT_NATIVE_FORMULA_CAPTURE_V2';
OUT.closed=false;
OUT.production_modified=false;
OUT.production_t1_selected=false;
OUT.continuous_terminal_boundary_proof=false;

%% A. AF7R57C authority
fprintf('\n[A. AF7R57C / KKT AUTHORITY]\n');

C=[];
if evalin('base','exist(''AF7R57C_SNAPSHOT_BOUND_SCHEMA_CAPTURE_OUT'',''var'')==1')
    C=evalin('base','AF7R57C_SNAPSHOT_BOUND_SCHEMA_CAPTURE_OUT');
    csrc='BASE:AF7R57C_SNAPSHOT_BOUND_SCHEMA_CAPTURE_OUT';
else
    f=af7r57e_find_nearby('AF7R57C_SNAPSHOT_BOUND_SCHEMA_CAPTURE_OUT.mat');
    assert(~isempty(f),'[AF7R57E] AF7R57C output unavailable.');
    S=load(f,'OUT');
    C=S.OUT;
    csrc=f;
end

assert(isstruct(C)&&isfield(C,'closed')&&logical(C.closed), ...
    '[AF7R57E] AF7R57C authority is not closed.');

fprintf(' authority source                        = %s\n',csrc);
fprintf(' AF7R57C closed                         = 1\n');

TR=C.trace_records;
KR=C.kkt_records;
ids=cellfun(@(q)double(q.c),KR);

for c=unique(ids(:)).'
    j=find(ids==c,1,'last');
    K=KR{j}.K{c};

    fprintf('\n candidate %g\n',c);
    fprintf('  t1 interval                            = [%.17g %.17g]\n', ...
        double(K.t_interval(1)),double(K.t_interval(2)));
    fprintf('  sign tag                               = %.17g\n',double(K.sign_tag));
    fprintf('  active index                           = %.17g\n',double(K.active_index));
    fprintf('  KKT closed                             = %d\n',logical(K.closed));
    fprintf('  z interval =\n');
    disp(double(K.z_interval));
end

%% B. Locate exact native/build sources
fprintf('\n[B. SOURCE LOCATION / SHA]\n');

mexcpp=af7r57e_find_source('af7r56_trace_mex.cpp');
hpp=af7r57e_find_source('af7r56_trace_core.hpp');
rps=which('REPLANNING_SYSTEM');

assert(~isempty(mexcpp)&&isfile(mexcpp),'[AF7R57E] af7r56_trace_mex.cpp missing.');
assert(~isempty(hpp)&&isfile(hpp),'[AF7R57E] af7r56_trace_core.hpp missing.');
assert(~isempty(rps)&&isfile(rps),'[AF7R57E] active REPLANNING_SYSTEM source missing.');

hashCpp=af7r57e_sha256_file(mexcpp);
hashHpp=af7r57e_sha256_file(hpp);
hashRps=af7r57e_sha256_file(rps);

fprintf(' trace MEX source                        = %s\n',mexcpp);
fprintf(' trace MEX source SHA-256                = %s\n',hashCpp);
fprintf(' trace core header                       = %s\n',hpp);
fprintf(' trace core header SHA-256               = %s\n',hashHpp);
fprintf(' active REPLANNING_SYSTEM                = %s\n',rps);
fprintf(' active REPLANNING_SYSTEM SHA-256        = %s\n',hashRps);

%% C. Exact C++ function bodies
fprintf('\n[C. EXACT C++ FUNCTION BODIES]\n');

cppNames={ ...
    'split_half', ...
    'decasteljau', ...
    'poly_norm_ub', ...
    'deriv_poly', ...
    'vector_control_from_jets', ...
    'build_candidate', ...
    'make_vleaf', ...
    'stageB_speed', ...
    'stageB_trace', ...
    'candidate_eval'};

CPP=struct();
for i=1:numel(cppNames)
    nm=cppNames{i};
    [txt,range,ok]=af7r57e_extract_cpp_function(hpp,nm);
    CPP.(nm)=struct('text',txt,'line_range',range,'closed',ok);

    fprintf('\n ---- %s | closed=%d | lines=%s ----\n',nm,ok,mat2str(range));
    fprintf('%s\n',txt);
end

cppTokens={ ...
    'differentiate(', ...
    'derivative(', ...
    'C1=', ...
    'C2=', ...
    'C3=', ...
    'candidate_eval(', ...
    'build_candidate', ...
    'Candidate', ...
    'struct ActiveReference', ...
    'ref.eval(', ...
    'eval(double', ...
    'seg0', ...
    'seg1'};

fprintf('\n[C2. C++ TOKEN CONTEXTS]\n');
CPP_CONTEXT=af7r57e_print_token_contexts(hpp,cppTokens,5);

%% D. Exact MATLAB method blocks
fprintf('\n[D. ACTIVE REPLANNING_SYSTEM EXACT METHOD BLOCKS]\n');

matNames={ ...
    'piecewise_transition_jet_maps', ...
    'bezier15_control_from_endpoint_jets_scalar', ...
    'bezier15_control_map_from_jet_maps', ...
    'evaluate_control_map_xyz', ...
    'build_piecewise_bezier_control_points'};

MAT=struct();
for i=1:numel(matNames)
    nm=matNames{i};
    [txt,range,ok]=af7r57e_extract_matlab_method(rps,nm);
    MAT.(nm)=struct('text',txt,'line_range',range,'closed',ok);

    fprintf('\n ---- %s | closed=%d | lines=%s ----\n',nm,ok,mat2str(range));
    fprintf('%s\n',txt);
end

%% E. Exact endpoint runtime reconstruction inputs
fprintf('\n[E. ENDPOINT RUNTIME INPUTS]\n');

tids=cellfun(@(q)double(q.c),TR);
for c=unique(tids(:)).'
    j=find(tids==c,1,'last');
    Q=TR{j};

    fprintf('\n candidate %g\n',c);
    fprintf('  lo / hi                                = %.17g / %.17g\n',double(Q.lo),double(Q.hi));
    fprintf('  packet t0 / tm / t1                   = %.17g / %.17g / %.17g\n', ...
        double(Q.pkt.t0),double(Q.pkt.tm),double(Q.pkt.t1));
    fprintf('  base VL/A/J/sigma                     = %.17g / %.17g / %.17g / %.17g\n', ...
        double(Q.pkt.base_payload_speed),double(Q.pkt.base_payload_accel), ...
        double(Q.pkt.base_payload_jerk),double(Q.pkt.base_sigma));
    fprintf('  cable length / gravity                 = %.17g / %.17g\n', ...
        double(Q.pkt.cable_length),double(Q.pkt.gravity));

    L=Q.TL{c};
    U=Q.TU{c};

    fprintf('  lower / upper trace limit             = %.17g / %.17g\n', ...
        double(L.limit),double(U.limit));
    fprintf('  trace limits identical                = %d\n',double(L.limit)==double(U.limit));
    fprintf('  lower candidate seg0                  = %s\n',mat2str(double(L.candidate_seg0),17));
    fprintf('  lower candidate seg1                  = %s\n',mat2str(double(L.candidate_seg1),17));
    fprintf('  upper candidate seg0                  = %s\n',mat2str(double(U.candidate_seg0),17));
    fprintf('  upper candidate seg1                  = %s\n',mat2str(double(U.candidate_seg1),17));

    % Do not invent component_start/component_end fields: pkt does not carry
    % them.  The exact candidate segment boundaries above are the runtime
    % authority actually returned by the Stage-B trace.
    fprintf('  pkt component_start field present     = %d\n',isfield(Q.pkt,'component_start'));
    fprintf('  pkt component_end field present       = %d\n',isfield(Q.pkt,'component_end'));
end

%% F. Source completeness gate
fprintf('\n[F. FORMULA-CAPTURE GATE]\n');

cppClosed=true;
for i=1:numel(cppNames)
    cppClosed=cppClosed&&CPP.(cppNames{i}).closed;
end

matClosed=true;
for i=1:numel(matNames)
    matClosed=matClosed&&MAT.(matNames{i}).closed;
end

shaStable=strcmpi(hashCpp,af7r57e_sha256_file(mexcpp)) && ...
          strcmpi(hashHpp,af7r57e_sha256_file(hpp)) && ...
          strcmpi(hashRps,af7r57e_sha256_file(rps));

closed=cppClosed&&matClosed&&shaStable;

fprintf(' exact C++ body capture closed           = %d\n',cppClosed);
fprintf(' exact MATLAB map capture closed         = %d\n',matClosed);
fprintf(' source SHA unchanged                    = %d\n',shaStable);
fprintf(' exact formula capture closed            = %d\n',closed);
fprintf(' interval Stage-B theorem implemented    = 0\n');
fprintf(' outward Stage-B leaf interval proof     = 0\n');
fprintf(' continuous terminal-boundary proof      = 0\n');
fprintf(' production t1 selected                  = 0\n');
fprintf(' production source modified              = 0\n');
fprintf(' unresolved interval policy              = FAIL CLOSED\n');
fprintf(' next action                              = BUILD_SOURCE_EXACT_OUTWARD_INTERVAL_STAGEB\n');
fprintf('====================================================================================================\n');

OUT.closed=closed;
OUT.cpp_source=mexcpp;
OUT.cpp_sha256=hashCpp;
OUT.hpp_source=hpp;
OUT.hpp_sha256=hashHpp;
OUT.replanner_source=rps;
OUT.replanner_sha256=hashRps;
OUT.cpp_functions=CPP;
OUT.cpp_token_contexts=CPP_CONTEXT;
OUT.matlab_methods=MAT;
OUT.production_modified=false;
OUT.production_t1_selected=false;
OUT.continuous_terminal_boundary_proof=false;

assignin('base','AF7R57E_EXACT_NATIVE_FORMULA_CAPTURE_OUT',OUT);
save('AF7R57E_EXACT_NATIVE_FORMULA_CAPTURE_OUT.mat','OUT','-v7.3');
end


function [txt,range,ok]=af7r57e_extract_cpp_function(fn,name)
raw=af7r57e_normnl(fileread(fn));
L=splitlines(string(raw));

hits=find(contains(L,[name '(']));
txt='';
range=[NaN NaN];
ok=false;

for h=hits(:).'
    start=h;
    depth=0;
    started=false;
    stop=NaN;
    inBlock=false;
    inString=false;
    inChar=false;

    for j=start:numel(L)
        s=char(L(j));
        k=1;

        while k<=numel(s)
            ch=s(k);
            nx=char(0);
            if k<numel(s)
                nx=s(k+1);
            end

            if inBlock
                if ch=='*' && nx=='/'
                    inBlock=false;
                    k=k+2;
                    continue;
                end
                k=k+1;
                continue;
            end

            if inString
                if ch=='\'
                    k=k+2;
                    continue;
                elseif ch=='"'
                    inString=false;
                end
                k=k+1;
                continue;
            end

            if inChar
                if ch=='\'
                    k=k+2;
                    continue;
                elseif ch==''''
                    inChar=false;
                end
                k=k+1;
                continue;
            end

            if ch=='/' && nx=='*'
                inBlock=true;
                k=k+2;
                continue;
            elseif ch=='/' && nx=='/'
                break;
            elseif ch=='"'
                inString=true;
            elseif ch==''''
                inChar=true;
            elseif ch=='{'
                depth=depth+1;
                started=true;
            elseif ch=='}' && started
                depth=depth-1;
                if depth==0
                    stop=j;
                    break;
                end
            end

            k=k+1;
        end

        if ~isnan(stop)
            break;
        end
    end

    if ~isnan(stop)
        txt=char(strjoin(L(start:stop),newline));
        range=[start stop];
        ok=true;
        return;
    end
end
end


function [txt,range,ok]=af7r57e_extract_matlab_method(fn,name)
raw=af7r57e_normnl(fileread(fn));
L=splitlines(string(raw));

txt='';
range=[NaN NaN];
ok=false;

% Method names may appear on a continuation line, e.g.
%   function [a,b] = ...
%           build_piecewise_bezier_control_points( ...
% Search the name first, then walk upward to its enclosing function line.
hits=find(contains(L,name));

for hh=hits(:).'
    start=NaN;

    for j=hh:-1:max(1,hh-8)
        if contains(L(j),'function ')
            start=j;
            break;
        end
    end

    if isnan(start)
        continue;
    end

    % The declaration window from the function line through the hit must
    % actually contain the requested method name.
    decl=char(strjoin(L(start:hh),newline));
    if ~contains(decl,name)
        continue;
    end

    stop=numel(L);

    for j=hh+1:numel(L)
        if startsWith(strtrim(L(j)),'function ')
            stop=j-1;
            break;
        end
    end

    body=char(strjoin(L(start:stop),newline));

    if contains(body,name) && contains(body,'end')
        txt=body;
        range=[start stop];
        ok=true;
        return;
    end
end
end


function C=af7r57e_print_token_contexts(fn,tokens,pad)
raw=af7r57e_normnl(fileread(fn));
L=splitlines(string(raw));

C=struct('token',{},'lines',{});

for i=1:numel(tokens)
    rows=find(contains(L,tokens{i}));
    C(end+1)=struct('token',tokens{i},'lines',double(rows(:).')); %#ok<AGROW>

    fprintf('\n token "%s" | lines=%s\n',tokens{i},mat2str(rows(:).'));
    for j=1:numel(rows)
        a=max(1,rows(j)-pad);
        b=min(numel(L),rows(j)+pad);

        fprintf('  [context around L%d]\n',rows(j));
        for q=a:b
            fprintf('   L%-5d %s\n',q,char(L(q)));
        end
    end
end
end


function fn=af7r57e_find_source(name)
fn='';

p=which(name);
if ~isempty(p)&&isfile(p)
    fn=p;
    return;
end

roots={pwd,fileparts(mfilename('fullpath'))};

p=which('af7r56_trace_mex');
if ~isempty(p)
    roots{end+1}=fileparts(p); %#ok<AGROW>
end

p=which('AF7R56B_STAGEB_BOUNDARY_TRACE');
if ~isempty(p)
    roots{end+1}=fileparts(p); %#ok<AGROW>
end

u=getenv('USERPROFILE');
if ~isempty(u)
    roots{end+1}=fullfile(u,'Documents','GitHub','common_matlab','reference'); %#ok<AGROW>
end

for i=1:numel(roots)
    q=fullfile(roots{i},name);
    if isfile(q)
        fn=q;
        return;
    end
end
end


function f=af7r57e_find_nearby(name)
f='';

p=which(name);
if ~isempty(p)&&isfile(p)
    f=p;
    return;
end

roots={pwd,fileparts(mfilename('fullpath'))};

for i=1:numel(roots)
    q=fullfile(roots{i},name);
    if isfile(q)
        f=q;
        return;
    end
end
end


function s=af7r57e_normnl(s)
s=strrep(s,sprintf('\r\n'),sprintf('\n'));
s=strrep(s,sprintf('\r'),sprintf('\n'));
end


function h=af7r57e_sha256_file(fn)
md=java.security.MessageDigest.getInstance('SHA-256');
fid=fopen(fn,'rb');
assert(fid>=0,'[AF7R57E] cannot open source file.');
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
