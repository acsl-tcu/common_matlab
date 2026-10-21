function R40 = AF7R40_STAGEAB_SHADOW_TRACE_AUDIT()
% AF7R40_STAGEAB_SHADOW_TRACE_AUDIT
% DIAGNOSTIC ONLY -- production REPLANNING_SYSTEM.m is never edited.
%
% Purpose
% -------
% Trace the exact R37/R38 candidate data through:
%   CANDS(ic) -> Q -> derivative-bound tightening -> Stage-A -> Stage-B
% and identify the first point at which finite candidate quantities become
% Inf/NaN or fail to produce a finite certificate.
%
% Method
% ------
% 1) Copy the currently resolved REPLANNING_SYSTEM.m to a temporary shadow dir.
% 2) Add diagnostic-only trace statements to the shadow copy.
% 3) Put the shadow dir first on path and rerun AF7R38_SPEED_ROOTCAUSE_AUDIT.
% 4) Restore the original MATLAB path and class resolution.
% 5) Delete the temporary shadow dir.
%
% Safety contract
% ---------------
% - production source modified                = 0
% - acceptance semantics modified            = 0
% - finite-sensing theorem modified           = 0
% - Stage-A / Stage-B gate modified           = 0
% - C7 / yaw-C6 requirements modified         = 0
% - rejoin / geometry certificates modified   = 0
% - no Inf/NaN replacement                    = 1
% - no sampled value may grant acceptance     = 1
%
% Outputs are written beside THIS .m file:
%   AF7R40_STAGEAB_SHADOW_TRACE_REPORT.txt
%   AF7R40_STAGEAB_SHADOW_TRACE_CAPTURE.mat
%
% Run:
%   R40 = AF7R40_STAGEAB_SHADOW_TRACE_AUDIT;

fprintf('\n');
fprintf('====================================================================================================\n');
fprintf(' AF7R40 STAGE-A/B SHADOW TRACE AUDIT (DIAGNOSTIC ONLY)\n');
fprintf(' production modified = 0 | acceptance modified = 0 | safety theorem modified = 0\n');
fprintf('====================================================================================================\n');

R40 = struct();
R40.audit_name = 'AF7R40_STAGEAB_SHADOW_TRACE_AUDIT';
R40.timestamp = char(datetime('now','Format','yyyy-MM-dd HH:mm:ss.SSS'));
R40.production_modified = false;
R40.acceptance_modified = false;
R40.certificate_semantics_modified = false;

auditDir = fileparts(mfilename('fullpath'));
if isempty(auditDir), auditDir = pwd; end
reportFile  = fullfile(auditDir,'AF7R40_STAGEAB_SHADOW_TRACE_REPORT.txt');
captureFile = fullfile(auditDir,'AF7R40_STAGEAB_SHADOW_TRACE_CAPTURE.mat');
shadowDir   = fullfile(auditDir,'AF7R40_shadow_runtime');

R40.audit_directory = auditDir;
R40.report_file = reportFile;
R40.capture_file = captureFile;

origPath = path;
origPwd  = pwd;
origSource = which('REPLANNING_SYSTEM');

if isempty(origSource) || ~isfile(origSource)
    error('AF7R40:NoSource','REPLANNING_SYSTEM.m could not be resolved.');
end

R40.original_source = origSource;
R40.original_sha256 = local_sha256(origSource);

% Require the source structure actually audited by R38.
src = fileread(origSource);

requiredPatterns = { ...
    'for ic=1:numel(CANDS)', ...
    'Q=CANDS(ic);', ...
    'tighten_piecewise_derivative_bounds_for_speed', ...
    'Stage A', ...
    'Stage B', ...
    'build_piecewise_local_vector_speed_certificate'};

R40.source_pattern_present = false(size(requiredPatterns));
for k=1:numel(requiredPatterns)
    R40.source_pattern_present(k) = contains(src,requiredPatterns{k});
end

if ~all(R40.source_pattern_present)
    fprintf('[R40] Required current-source markers are missing. Aborting without shadow execution.\n');
    R40.execution_status = 'SOURCE_PATTERN_MISMATCH';
    local_write_report(reportFile,R40);
    save(captureFile,'R40','-v7.3');
    return;
end

% Never leave stale shadow content.
if isfolder(shadowDir)
    try, rmdir(shadowDir,'s'); catch, end
end
mkdir(shadowDir);

shadowSource = fullfile(shadowDir,'REPLANNING_SYSTEM.m');
copyfile(origSource,shadowSource);

cleanupObj = onCleanup(@() local_restore(origPath,origPwd,shadowDir)); %#ok<NASGU>

shadow = fileread(shadowSource);

%% -----------------------------------------------------------------------------------------------
% Injection 1: immediately after Q=CANDS(ic)
% Capture the exact constructive candidate before any speed-certificate logic.
% ------------------------------------------------------------------------------------------------
anchor1 = 'Q=CANDS(ic);';
inject1 = [anchor1 newline ...
'                try' newline ...
'                    T40=evalin(''base'',''AF7R40_TRACE'');' newline ...
'                    if ~isfield(T40,''candidate''), T40.candidate=struct([]); end' newline ...
'                    T40.candidate(ic).candidate_index=ic;' newline ...
'                    if isfield(Q,''corridor_signs''), T40.candidate(ic).corridor_signs=Q.corridor_signs; end' newline ...
'                    if isfield(Q,''segment_t0''), T40.candidate(ic).segment_t0=Q.segment_t0; end' newline ...
'                    if isfield(Q,''segment_t1''), T40.candidate(ic).segment_t1=Q.segment_t1; end' newline ...
'                    if isfield(Q,''segment_kind''), T40.candidate(ic).segment_kind=Q.segment_kind; end' newline ...
'                    if isfield(Q,''control_points''), T40.candidate(ic).control_points=Q.control_points; end' newline ...
'                    if isfield(Q,''component_start''), T40.candidate(ic).component_start=Q.component_start; end' newline ...
'                    if isfield(Q,''component_end''), T40.candidate(ic).component_end=Q.component_end; end' newline ...
'                    if isfield(Q,''component_position''), T40.candidate(ic).component_position=Q.component_position; end' newline ...
'                    if isfield(Q,''component_velocity''), T40.candidate(ic).component_velocity=Q.component_velocity; end' newline ...
'                    if isfield(Q,''derivative_ub''), T40.candidate(ic).Q_derivative_ub=Q.derivative_ub; end' newline ...
'                    if isfield(Q,''max_control_norm_m''), T40.candidate(ic).max_control_norm_m=Q.max_control_norm_m; end' newline ...
'                    if isfield(Q,''qp_cost''), T40.candidate(ic).qp_cost=Q.qp_cost; end' newline ...
'                    assignin(''base'',''AF7R40_TRACE'',T40);' newline ...
'                catch ME40' newline ...
'                    assignin(''base'',''AF7R40_LAST_TRACE_ERROR'',ME40);' newline ...
'                end'];

if count(shadow,anchor1) < 1
    error('AF7R40:Anchor1','Candidate anchor not found.');
end
shadow = local_replace_first(shadow,anchor1,inject1);

%% -----------------------------------------------------------------------------------------------
% Injection 2: after tightening call completes.
% Match the exact current R9 call as exposed in the authoritative source.
% ------------------------------------------------------------------------------------------------
needle2 = 'tighten_piecewise_derivative_bounds_for_speed';
inject2 = [newline ...
'                try' newline ...
'                    T40=evalin(''base'',''AF7R40_TRACE'');' newline ...
'                    T40.candidate(ic).VQlimit=VQlimit;' newline ...
'                    T40.candidate(ic).dubTight=dubTight;' newline ...
'                    T40.candidate(ic).DB=DB;' newline ...
'                    T40.candidate(ic).baseSC=baseSC;' newline ...
'                    assignin(''base'',''AF7R40_TRACE'',T40);' newline ...
'                catch ME40' newline ...
'                    assignin(''base'',''AF7R40_LAST_TRACE_ERROR'',ME40);' newline ...
'                end'];
[shadow,ok2] = local_insert_after_statement_after(shadow,anchor1,needle2,inject2);
if ~ok2
    error('AF7R40:Anchor2','Could not locate tightening call statement.');
end

%% -----------------------------------------------------------------------------------------------
% Injection 3: immediately before Stage-B.
% This is the Stage-A exit snapshot, whether Stage-A closed or not.
% ------------------------------------------------------------------------------------------------
anchor3 = '% Stage B: local vector-coupled suspended-load theorem.';
inject3 = [ ...
'                try' newline ...
'                    T40=evalin(''base'',''AF7R40_TRACE'');' newline ...
'                    T40.candidate(ic).stageA_speedClosed=speedClosed;' newline ...
'                    T40.candidate(ic).stageA_SCtry=SCtry;' newline ...
'                    T40.candidate(ic).stageA_Ptry=Ptry;' newline ...
'                    assignin(''base'',''AF7R40_TRACE'',T40);' newline ...
'                catch ME40' newline ...
'                    assignin(''base'',''AF7R40_LAST_TRACE_ERROR'',ME40);' newline ...
'                end' newline ...
'                ' anchor3];

if count(shadow,anchor3) < 1
    error('AF7R40:Anchor3','Stage-B marker not found.');
end
shadow = local_replace_first(shadow,anchor3,inject3);

%% -----------------------------------------------------------------------------------------------
% Injection 4: immediately after Stage-B local-vector call.
% Capture raw SClocal and LV, including any internal leaves/witnesses they expose.
% ------------------------------------------------------------------------------------------------
needle4 = 'build_piecewise_local_vector_speed_certificate';
inject4 = [newline ...
'                        try' newline ...
'                            T40=evalin(''base'',''AF7R40_TRACE'');' newline ...
'                            T40.candidate(ic).stageB_called=true;' newline ...
'                            T40.candidate(ic).stageB_SClocal=SClocal;' newline ...
'                            T40.candidate(ic).stageB_LV=LV;' newline ...
'                            assignin(''base'',''AF7R40_TRACE'',T40);' newline ...
'                        catch ME40' newline ...
'                            assignin(''base'',''AF7R40_LAST_TRACE_ERROR'',ME40);' newline ...
'                        end'];
[shadow,ok4] = local_insert_after_statement_after(shadow,anchor1,needle4,inject4);
if ~ok4
    error('AF7R40:Anchor4','Could not locate Stage-B call statement.');
end

fid=fopen(shadowSource,'w');
if fid<0, error('AF7R40:WriteShadow','Cannot write shadow source.'); end
fwrite(fid,shadow,'char');
fclose(fid);

R40.shadow_sha256 = local_sha256(shadowSource);
R40.instrumentation = struct( ...
    'candidate_anchor',true, ...
    'tightening_statement_anchor',ok2, ...
    'stageA_exit_anchor',true, ...
    'stageB_statement_anchor',ok4);

%% -----------------------------------------------------------------------------------------------
% Execute R38 with shadow source first on path.
% ------------------------------------------------------------------------------------------------
assignin('base','AF7R40_TRACE',struct('candidate',struct([])));
evalin('base','clear AF7R40_LAST_TRACE_ERROR');

addpath(shadowDir,'-begin');
rehash;

% Clear only this class/function resolution. Production file itself is untouched.
try
    evalin('base','clear REPLANNING_SYSTEM');
catch
end
clear REPLANNING_SYSTEM
rehash;

resolvedDuring = which('REPLANNING_SYSTEM');
R40.shadow_resolved_source = resolvedDuring;
R40.shadow_resolution_ok = strcmpi(local_normpath(resolvedDuring),local_normpath(shadowSource));

if ~R40.shadow_resolution_ok
    error('AF7R40:ShadowResolution','Shadow REPLANNING_SYSTEM did not become authoritative.');
end

fprintf('[R40] Shadow source active: %s\n',resolvedDuring);
fprintf('[R40] Running AF7R38 once under diagnostic-only source shadow...\n');

try
    D38_shadow = AF7R38_SPEED_ROOTCAUSE_AUDIT;
    R40.r38_execution_error = '';
catch ME
    D38_shadow = struct();
    R40.r38_execution_error = getReport(ME,'extended','hyperlinks','off');
end

R40.D38_shadow = D38_shadow;

if evalin('base','exist(''AF7R40_TRACE'',''var'')==1')
    TRACE = evalin('base','AF7R40_TRACE');
else
    TRACE = struct('candidate',struct([]));
end
R40.trace = TRACE;

if evalin('base','exist(''AF7R40_LAST_TRACE_ERROR'',''var'')==1')
    E40=evalin('base','AF7R40_LAST_TRACE_ERROR');
    try
        R40.last_trace_error=getReport(E40,'extended','hyperlinks','off');
    catch
        R40.last_trace_error='TRACE ERROR PRESENT';
    end
else
    R40.last_trace_error='';
end

%% -----------------------------------------------------------------------------------------------
% Restore production class resolution BEFORE analysis/save.
% ------------------------------------------------------------------------------------------------
path(origPath);
cd(origPwd);
try
    evalin('base','clear REPLANNING_SYSTEM');
catch
end
clear REPLANNING_SYSTEM
rehash;
R40.restored_source = which('REPLANNING_SYSTEM');
R40.production_resolution_restored = strcmpi(local_normpath(R40.restored_source),local_normpath(origSource));
R40.production_sha256_after = local_sha256(origSource);
R40.production_hash_unchanged = strcmpi(R40.original_sha256,R40.production_sha256_after);

%% -----------------------------------------------------------------------------------------------
% Analyze exact candidate data.
% ------------------------------------------------------------------------------------------------
R40.analysis = struct();
R40.analysis.candidate_count = 0;
R40.analysis.candidates = struct([]);

if isstruct(TRACE) && isfield(TRACE,'candidate') && ~isempty(TRACE.candidate)
    C=TRACE.candidate;
    R40.analysis.candidate_count=numel(C);

    for ic=1:numel(C)
        A=struct();
        A.index=ic;

        A.corridor_signs = local_get(C(ic),'corridor_signs',[]);
        A.segment_t0 = double(local_get(C(ic),'segment_t0',[]));
        A.segment_t1 = double(local_get(C(ic),'segment_t1',[]));
        A.segment_duration = A.segment_t1 - A.segment_t0;
        A.Q_derivative_ub = local_get(C(ic),'Q_derivative_ub',[]);
        A.max_control_norm_m = local_get(C(ic),'max_control_norm_m',NaN);
        A.qp_cost = local_get(C(ic),'qp_cost',NaN);
        A.VQlimit = local_get(C(ic),'VQlimit',NaN);
        A.dubTight = local_get(C(ic),'dubTight',[]);
        A.DB = local_get(C(ic),'DB',struct());
        A.stageA_speedClosed = local_get(C(ic),'stageA_speedClosed',false);
        A.stageA_SCtry = local_get(C(ic),'stageA_SCtry',struct());
        A.stageB_called = local_get(C(ic),'stageB_called',false);
        A.stageB_SClocal = local_get(C(ic),'stageB_SClocal',struct());
        A.stageB_LV = local_get(C(ic),'stageB_LV',struct());

        CP=local_get(C(ic),'control_points',[]);
        A.control_point_summary=local_cp_summary(CP,A.segment_duration);

        % Recursive finite/nonfinite diagnostics on the actual certificate structs.
        A.DB_nonfinite = local_nonfinite_scan(A.DB,'DB',0,12);
        A.stageA_nonfinite = local_nonfinite_scan(A.stageA_SCtry,'SCtry',0,12);
        A.stageB_SC_nonfinite = local_nonfinite_scan(A.stageB_SClocal,'SClocal',0,12);
        A.stageB_LV_nonfinite = local_nonfinite_scan(A.stageB_LV,'LV',0,14);

        % Search raw Stage-B structures for the requested semantic fields.
        A.stageB_leaf_fields = local_semantic_scan(A.stageB_LV,'LV', ...
            {'leaf','segment','ta','tb','vl','sigma','qdot','vq','witness','violation'},0,14);

        A.first_break = local_first_break(A);

        R40.analysis.candidates(ic)=A; %#ok<AGROW>
    end
end

% Global classification.
if ~isempty(R40.r38_execution_error)
    R40.analysis.classification='SHADOW_EXECUTION_FAILED';
elseif R40.analysis.candidate_count==0
    R40.analysis.classification='NO_CANDIDATE_TRACE_CAPTURED';
else
    breaks={R40.analysis.candidates.first_break};
    if any(strcmp(breaks,'CANDIDATE_ALREADY_NONFINITE'))
        R40.analysis.classification='NONFINITE_BEFORE_SPEED_TIGHTENING';
    elseif any(strcmp(breaks,'TIGHTENING_INTRODUCES_NONFINITE'))
        R40.analysis.classification='DERIVATIVE_TIGHTENING_ROOT_CAUSE';
    elseif any(strcmp(breaks,'STAGEA_INTRODUCES_NONFINITE'))
        R40.analysis.classification='STAGEA_FORMULA_ROOT_CAUSE';
    elseif any(strcmp(breaks,'STAGEB_FINITE_HARD_VIOLATION'))
        R40.analysis.classification='PHYSICAL_SPEED_CERTIFICATE_VIOLATION';
    elseif any(strcmp(breaks,'STAGEB_NONFINITE_OR_INCONCLUSIVE'))
        R40.analysis.classification='STAGEB_NONFINITE_OR_INCONCLUSIVE';
    else
        R40.analysis.classification='TRACE_REQUIRES_MANUAL_FIELD_INSPECTION';
    end
end

%% -----------------------------------------------------------------------------------------------
% Report and capture beside this audit file.
% ------------------------------------------------------------------------------------------------
local_write_report(reportFile,R40);
save(captureFile,'R40','-v7.3');

fprintf('\n[A. SOURCE INTEGRITY]\n');
fprintf('production source                             = %s\n',R40.original_source);
fprintf('production SHA-256 before                     = %s\n',R40.original_sha256);
fprintf('production SHA-256 after                      = %s\n',R40.production_sha256_after);
fprintf('production hash unchanged                     = %d\n',R40.production_hash_unchanged);
fprintf('production resolution restored                = %d\n',R40.production_resolution_restored);

fprintf('\n[B. TRACE]\n');
fprintf('candidate count                               = %d\n',R40.analysis.candidate_count);
fprintf('classification                                = %s\n',R40.analysis.classification);
for ic=1:R40.analysis.candidate_count
    A=R40.analysis.candidates(ic);
    fprintf('candidate %d first break                      = %s\n',ic,A.first_break);
    fprintf('  signs                                       = %s\n',local_short(A.corridor_signs));
    fprintf('  segment t0                                  = %s\n',local_short(A.segment_t0));
    fprintf('  segment t1                                  = %s\n',local_short(A.segment_t1));
    fprintf('  durations                                   = %s\n',local_short(A.segment_duration));
    fprintf('  VQlimit                                     = %s\n',local_short(A.VQlimit));
    fprintf('  Q derivative UB                             = %s\n',local_short(A.Q_derivative_ub));
    fprintf('  tightened derivative UB                     = %s\n',local_short(A.dubTight));
    fprintf('  Stage-A closed                              = %d\n',logical(A.stageA_speedClosed));
    fprintf('  Stage-B called                              = %d\n',logical(A.stageB_called));
end

fprintf('\n[C. OUTPUT]\n');
fprintf('report                                        = %s\n',reportFile);
fprintf('capture                                       = %s\n',captureFile);
fprintf('====================================================================================================\n');

assignin('base','R40',R40);

% Remove temporary shadow now that production resolution is restored.
try
    if isfolder(shadowDir), rmdir(shadowDir,'s'); end
catch
end

end

% =================================================================================================
% Helpers
% =================================================================================================
function [out,ok]=local_insert_after_statement_after(in,afterNeedle,needle,injection)
% Insert immediately after the semicolon terminating the first MATLAB
% statement containing NEEDLE that occurs AFTER afterNeedle.
% This avoids matching the method definition elsewhere in the class and
% intentionally ignores whitespace, CR/LF layout and continuation tokens (...).
out=in;
ok=false;

a=strfind(in,afterNeedle);
if isempty(a), return; end
searchStart=a(1)+length(afterNeedle);

krel=strfind(in(searchStart:end),needle);
if isempty(krel), return; end
p=searchStart+krel(1)-1;

qrel=find(in(p:end)==';',1,'first');
if isempty(qrel), return; end
q=p+qrel-1;

out=[in(1:q) injection in(q+1:end)];
ok=true;
end

function out=local_replace_first(in,old,new)
k=strfind(in,old);
if isempty(k), error('AF7R40:Replace','Anchor not found.'); end
p=k(1);
out=[in(1:p-1) new in(p+length(old):end)];
end

function p=local_normpath(p)
p=strrep(char(p),'/','\');
p=lower(p);
end

function v=local_get(S,f,d)
if isstruct(S) && isfield(S,f), v=S.(f); else, v=d; end
end

function S=local_cp_summary(CP,durations)
S=struct('available',false,'layout','','segment',struct([]));
if isempty(CP), return; end
S.available=true;

segments={};
if iscell(CP)
    segments=CP(:).';
    S.layout=sprintf('cell %s',mat2str(size(CP)));
elseif isnumeric(CP)
    sz=size(CP);
    S.layout=sprintf('numeric %s',mat2str(sz));
    if ndims(CP)==3
        % Prefer 3 x (n+1) x nSeg. Also support (n+1) x 3 x nSeg.
        for j=1:sz(3)
            X=CP(:,:,j);
            if size(X,1)==3
                segments{j}=double(X); %#ok<AGROW>
            elseif size(X,2)==3
                segments{j}=double(X.'); %#ok<AGROW>
            else
                segments{j}=double(X); %#ok<AGROW>
            end
        end
    elseif ismatrix(CP)
        if size(CP,1)==3
            segments={double(CP)};
        elseif size(CP,2)==3
            segments={double(CP.')};
        else
            segments={double(CP)};
        end
    end
else
    return;
end

for j=1:numel(segments)
    P=segments{j};
    Z=struct();
    Z.index=j;
    Z.size=size(P);
    if size(P,1)==3
        Z.control_norms=sqrt(sum(P.^2,1));
        Z.max_control_norm=max(Z.control_norms);
        if j<=numel(durations), T=durations(j); else, T=NaN; end
        Z.duration=T;
        n=size(P,2)-1;
        if isfinite(T) && T>0 && n>=1
            V=n*diff(P,1,2)/T;
            Z.correction_velocity_bernstein_ub=max(sqrt(sum(V.^2,1)));
        else
            Z.correction_velocity_bernstein_ub=NaN;
        end
        if isfinite(T) && T>0 && n>=2
            A=n*(n-1)*diff(P,2,2)/(T^2);
            Z.correction_accel_bernstein_ub=max(sqrt(sum(A.^2,1)));
        else
            Z.correction_accel_bernstein_ub=NaN;
        end
        if isfinite(T) && T>0 && n>=3
            J=n*(n-1)*(n-2)*diff(P,3,2)/(T^3);
            Z.correction_jerk_bernstein_ub=max(sqrt(sum(J.^2,1)));
        else
            Z.correction_jerk_bernstein_ub=NaN;
        end
    else
        Z.control_norms=[];
        Z.max_control_norm=NaN;
        Z.duration=NaN;
        Z.correction_velocity_bernstein_ub=NaN;
        Z.correction_accel_bernstein_ub=NaN;
        Z.correction_jerk_bernstein_ub=NaN;
    end
    S.segment(j)=Z; %#ok<AGROW>
end
end

function H=local_nonfinite_scan(x,path,depth,maxDepth)
H=repmat(struct('path','','value',''),0,1);
if depth>maxDepth, return; end
try
    if isnumeric(x)
        if any(~isfinite(x(:)))
            H(end+1,1)=struct('path',path,'value',local_short(x)); %#ok<AGROW>
        end
    elseif isstruct(x)
        for k=1:numel(x)
            fn=fieldnames(x(k));
            for i=1:numel(fn)
                p=sprintf('%s(%d).%s',path,k,fn{i});
                H=[H;local_nonfinite_scan(x(k).(fn{i}),p,depth+1,maxDepth)]; %#ok<AGROW>
            end
        end
    elseif iscell(x)
        for k=1:numel(x)
            H=[H;local_nonfinite_scan(x{k},sprintf('%s{%d}',path,k),depth+1,maxDepth)]; %#ok<AGROW>
        end
    end
catch
end
end

function H=local_semantic_scan(x,path,keys,depth,maxDepth)
H=repmat(struct('path','','summary',''),0,1);
if depth>maxDepth, return; end
try
    if isstruct(x)
        for k=1:numel(x)
            fn=fieldnames(x(k));
            for i=1:numel(fn)
                f=fn{i}; v=x(k).(f);
                p=sprintf('%s(%d).%s',path,k,f);
                low=lower(f);
                hit=false;
                for q=1:numel(keys)
                    if contains(low,lower(keys{q})), hit=true; break; end
                end
                if hit
                    H(end+1,1)=struct('path',p,'summary',local_short(v)); %#ok<AGROW>
                end
                if isstruct(v)||iscell(v)
                    H=[H;local_semantic_scan(v,p,keys,depth+1,maxDepth)]; %#ok<AGROW>
                end
            end
        end
    elseif iscell(x)
        for k=1:numel(x)
            H=[H;local_semantic_scan(x{k},sprintf('%s{%d}',path,k),keys,depth+1,maxDepth)]; %#ok<AGROW>
        end
    end
catch
end
end

function c=local_first_break(A)
% Conservative classification only. Never turns a diagnostic into acceptance.
candFinite = local_all_finite(A.segment_t0) && local_all_finite(A.segment_t1) ...
    && local_all_finite(A.Q_derivative_ub);

if ~candFinite
    c='CANDIDATE_ALREADY_NONFINITE';
    return;
end

if ~local_all_finite(A.dubTight) || ~isempty(A.DB_nonfinite)
    c='TIGHTENING_INTRODUCES_NONFINITE';
    return;
end

if ~isempty(A.stageA_nonfinite)
    c='STAGEA_INTRODUCES_NONFINITE';
    return;
end

if logical(A.stageB_called)
    finiteWitness = false;
    hardViolation = false;
    F=A.stageB_leaf_fields;
    for i=1:numel(F)
        low=lower(F(i).path);
        if contains(low,'witness') || contains(low,'violation')
            if ~contains(lower(F(i).summary),'nan') && ~contains(lower(F(i).summary),'inf')
                finiteWitness=true;
            end
        end
    end
    if finiteWitness, hardViolation=true; end
    if hardViolation
        c='STAGEB_FINITE_HARD_VIOLATION';
    elseif ~isempty(A.stageB_SC_nonfinite) || ~isempty(A.stageB_LV_nonfinite)
        c='STAGEB_NONFINITE_OR_INCONCLUSIVE';
    else
        c='NO_NONFINITE_BREAK_EXPOSED';
    end
else
    if logical(A.stageA_speedClosed)
        c='STAGEA_CERTIFIED';
    else
        c='STAGEA_FINITE_BUT_NOT_CLOSED';
    end
end
end

function tf=local_all_finite(x)
if isempty(x)
    tf=false;
elseif isnumeric(x)
    tf=all(isfinite(x(:)));
else
    tf=true;
end
end

function h=local_sha256(file)
h='';
try
    md=java.security.MessageDigest.getInstance('SHA-256');
    fid=fopen(file,'rb');
    c=onCleanup(@() fclose(fid));
    while true
        b=fread(fid,1024*1024,'*uint8');
        if isempty(b), break; end
        md.update(b);
    end
    d=typecast(md.digest(),'uint8');
    h=lower(reshape(dec2hex(d,2).',1,[]));
catch
end
end

function s=local_short(v)
try
    if isempty(v), s='[]'; return; end
    if islogical(v) && isscalar(v), s=sprintf('%d',v); return; end
    if isnumeric(v)
        if isscalar(v), s=sprintf('%.17g',double(v)); return; end
        if numel(v)<=64, s=mat2str(v,17); return; end
        z=double(v(:));
        nf=sum(~isfinite(z));
        fin=z(isfinite(z));
        if isempty(fin)
            s=sprintf('%s %s all-nonfinite',class(v),mat2str(size(v)));
        else
            s=sprintf('%s %s min=%.17g max=%.17g nonfinite=%d', ...
                class(v),mat2str(size(v)),min(fin),max(fin),nf);
        end
        return;
    end
    if ischar(v), s=v; return; end
    if isstring(v)&&isscalar(v), s=char(v); return; end
    if isstruct(v), s=sprintf('struct %s fields=%d',mat2str(size(v)),numel(fieldnames(v))); return; end
    if iscell(v), s=sprintf('cell %s',mat2str(size(v))); return; end
    s=class(v);
catch
    s='<unprintable>';
end
end

function local_write_report(file,R)
fid=fopen(file,'w');
if fid<0, return; end
c=onCleanup(@() fclose(fid));

fprintf(fid,'AF7R40 STAGE-A/B SHADOW TRACE AUDIT\n');
fprintf(fid,'timestamp=%s\n',R.timestamp);
fprintf(fid,'production_modified=0\n');
fprintf(fid,'acceptance_modified=0\n');
fprintf(fid,'certificate_semantics_modified=0\n');
fprintf(fid,'original_source=%s\n',R.original_source);
fprintf(fid,'original_sha256=%s\n',R.original_sha256);

if isfield(R,'production_sha256_after')
    fprintf(fid,'production_sha256_after=%s\n',R.production_sha256_after);
    fprintf(fid,'production_hash_unchanged=%d\n',R.production_hash_unchanged);
    fprintf(fid,'production_resolution_restored=%d\n',R.production_resolution_restored);
end

if isfield(R,'r38_execution_error') && ~isempty(R.r38_execution_error)
    fprintf(fid,'\nR38_EXECUTION_ERROR\n%s\n',R.r38_execution_error);
end
if isfield(R,'last_trace_error') && ~isempty(R.last_trace_error)
    fprintf(fid,'\nLAST_TRACE_ERROR\n%s\n',R.last_trace_error);
end

if ~isfield(R,'analysis'), return; end
fprintf(fid,'\nCLASSIFICATION=%s\n',R.analysis.classification);
fprintf(fid,'candidate_count=%d\n',R.analysis.candidate_count);

for ic=1:R.analysis.candidate_count
    A=R.analysis.candidates(ic);
    fprintf(fid,'\n====================================================================================================\n');
    fprintf(fid,'CANDIDATE %d\n',ic);
    fprintf(fid,'first_break=%s\n',A.first_break);
    fprintf(fid,'corridor_signs=%s\n',local_short(A.corridor_signs));
    fprintf(fid,'segment_t0=%s\n',local_short(A.segment_t0));
    fprintf(fid,'segment_t1=%s\n',local_short(A.segment_t1));
    fprintf(fid,'segment_duration=%s\n',local_short(A.segment_duration));
    fprintf(fid,'VQlimit=%s\n',local_short(A.VQlimit));
    fprintf(fid,'Q_derivative_ub=%s\n',local_short(A.Q_derivative_ub));
    fprintf(fid,'dubTight=%s\n',local_short(A.dubTight));
    fprintf(fid,'stageA_speedClosed=%d\n',logical(A.stageA_speedClosed));
    fprintf(fid,'stageB_called=%d\n',logical(A.stageB_called));

    fprintf(fid,'\nCONTROL-POINT / CORRECTION BERNSTEIN SUMMARY\n');
    C=A.control_point_summary;
    fprintf(fid,'available=%d layout=%s\n',C.available,C.layout);
    if isfield(C,'segment')
        for j=1:numel(C.segment)
            Z=C.segment(j);
            fprintf(fid,' segment %d duration=%s\n',j,local_short(Z.duration));
            fprintf(fid,'   control_norms=%s\n',local_short(Z.control_norms));
            fprintf(fid,'   correction_velocity_bernstein_ub=%s\n',local_short(Z.correction_velocity_bernstein_ub));
            fprintf(fid,'   correction_accel_bernstein_ub=%s\n',local_short(Z.correction_accel_bernstein_ub));
            fprintf(fid,'   correction_jerk_bernstein_ub=%s\n',local_short(Z.correction_jerk_bernstein_ub));
        end
    end

    fprintf(fid,'\nDB NONFINITE FIELDS\n');
    local_print_hits(fid,A.DB_nonfinite);
    fprintf(fid,'\nSTAGE-A NONFINITE FIELDS\n');
    local_print_hits(fid,A.stageA_nonfinite);
    fprintf(fid,'\nSTAGE-B SC NONFINITE FIELDS\n');
    local_print_hits(fid,A.stageB_SC_nonfinite);
    fprintf(fid,'\nSTAGE-B LV NONFINITE FIELDS\n');
    local_print_hits(fid,A.stageB_LV_nonfinite);

    fprintf(fid,'\nSTAGE-B LEAF / WITNESS SEMANTIC FIELDS\n');
    for j=1:numel(A.stageB_leaf_fields)
        fprintf(fid,'%s = %s\n',A.stageB_leaf_fields(j).path,A.stageB_leaf_fields(j).summary);
    end
end
end

function local_print_hits(fid,H)
if isempty(H)
    fprintf(fid,'<none>\n');
    return;
end
for i=1:numel(H)
    fprintf(fid,'%s = %s\n',H(i).path,H(i).value);
end
end

function local_restore(origPath,origPwd,shadowDir)
try, path(origPath); catch, end
try, cd(origPwd); catch, end
try, evalin('base','clear REPLANNING_SYSTEM'); catch, end
try, clear REPLANNING_SYSTEM; catch, end
try, rehash; catch, end
try
    if isfolder(shadowDir), rmdir(shadowDir,'s'); end
catch
end
end
