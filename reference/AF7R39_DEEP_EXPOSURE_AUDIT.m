function R = AF7R39_DEEP_EXPOSURE_AUDIT()
% AF7R39_DEEP_EXPOSURE_AUDIT
% DIAGNOSTIC ONLY.
%
% Purpose:
%   Recover every already-exposed scalar/array/struct related to the R38
%   speed-root-cause investigation WITHOUT modifying production code,
%   changing any certificate, or granting acceptance.
%
% Usage:
%   1) Run:
%          D38 = AF7R38_SPEED_ROOTCAUSE_AUDIT;
%   2) Immediately run:
%          R39 = AF7R39_DEEP_EXPOSURE_AUDIT;
%
% If D38 is not present, this audit will run R38 once.
%
% Outputs:
%   R39  : structured diagnostic result
%   AF7R39_DEEP_EXPOSURE_REPORT.txt
%   AF7R39_DEEP_EXPOSURE_CAPTURE.mat
%
% This audit NEVER changes:
%   - finite-sensing speed limit
%   - Stage-A / Stage-B acceptance
%   - C7 / yaw-C6 requirements
%   - rejoin certificate
%   - whole-body geometry certificate
%   - fallback semantics
%
% It only exposes data that already exists after R38.

fprintf('\n');
fprintf('====================================================================================================\n');
fprintf(' AF7R39 DEEP EXPOSURE AUDIT (DIAGNOSTIC ONLY)\n');
fprintf(' production modified = 0 | acceptance modified = 0 | certificate semantics modified = 0\n');
fprintf('====================================================================================================\n');

R = struct();
R.audit_name = 'AF7R39_DEEP_EXPOSURE_AUDIT';
R.production_modified = false;
R.acceptance_modified = false;
R.certificate_semantics_modified = false;
R.timestamp = char(datetime('now','Format','yyyy-MM-dd HH:mm:ss.SSS'));

%% -----------------------------------------------------------------------------------------------
%  A. Acquire D38
% ------------------------------------------------------------------------------------------------
haveD38 = evalin('base','exist(''D38'',''var'')==1');
if haveD38
    D38 = evalin('base','D38');
    R.d38_source = 'EXISTING_BASE_WORKSPACE';
else
    fprintf('[R39] D38 not found in base workspace; running AF7R38_SPEED_ROOTCAUSE_AUDIT once.\n');
    D38 = AF7R38_SPEED_ROOTCAUSE_AUDIT;
    assignin('base','D38',D38);
    R.d38_source = 'RERUN_BY_R39';
end
R.D38 = D38;

%% -----------------------------------------------------------------------------------------------
%  B. Capture relevant base-workspace variables without changing them
% ------------------------------------------------------------------------------------------------
W = evalin('base','whos');
R.base_whos = W;

nameHints = { ...
    'D38','D37','R37','R38','CANDS','CAND','candidate','candidates', ...
    'Q','Ptry','SCtry','SClocal','SC','LV','DB','dubTight', ...
    'segment','knot','control','piecewise','speed','velocity','accel','jerk', ...
    'sigma','qdot','VL','VQ','witness','stage','rejoin','geometry','yaw','c7'};

CAP = struct();
capturedNames = {};
for i = 1:numel(W)
    nm = W(i).name;
    low = lower(nm);
    hit = false;
    for j = 1:numel(nameHints)
        if contains(low, lower(nameHints{j}))
            hit = true;
            break;
        end
    end
    if hit
        try
            v = evalin('base',nm);
            safeName = matlab.lang.makeValidName(nm);
            CAP.(safeName) = v;
            capturedNames{end+1,1} = nm; %#ok<AGROW>
        catch ME
            safeName = matlab.lang.makeValidName([nm '_CAPTURE_ERROR']);
            CAP.(safeName) = ME.message;
        end
    end
end
R.base_capture = CAP;
R.base_captured_names = capturedNames;

%% -----------------------------------------------------------------------------------------------
%  C. Search recursively for the exact R38-next-diagnostic quantities
% ------------------------------------------------------------------------------------------------
targets = { ...
    'side','sign','corridor_sign', ...
    'knot','segment_t0','segment_t1','segment','duration', ...
    'control_point','control_points','piecewise_control_points', ...
    'position','velocity','derivative','derivative_ub', ...
    'speed','uav_speed','total_velocity', ...
    'accel','acceleration','jerk', ...
    'stagea','stage_a','stageb','stage_b', ...
    'leaf','worst','ta','tb', ...
    'vl','sigma','qdot','vq', ...
    'witness','violation', ...
    'representability','certified','reason'};

hits = local_recursive_collect(D38,'D38',targets,0,14);
if ~isempty(fieldnames(CAP))
    hits2 = local_recursive_collect(CAP,'BASE',targets,0,14);
    hits = [hits; hits2]; %#ok<AGROW>
end
R.hits = hits;

%% -----------------------------------------------------------------------------------------------
%  D. Build semantic buckets corresponding exactly to R38 section [F]
% ------------------------------------------------------------------------------------------------
R.required = struct();
R.required.per_candidate_side_assignment = local_filter_hits(hits, ...
    {'side','sign','corridor_sign'});
R.required.knot_times_and_segment_durations = local_filter_hits(hits, ...
    {'knot','segment_t0','segment_t1','duration'});
R.required.knot_position_velocity_norm_inputs = local_filter_hits(hits, ...
    {'control_point','control_points','position','velocity'});
R.required.segment_velocity_accel_jerk_bounds = local_filter_hits(hits, ...
    {'speed','velocity','accel','acceleration','jerk','derivative_ub'});
R.required.stageB_worst_leaf = local_filter_hits(hits, ...
    {'leaf','worst','ta','tb','vl','sigma','qdot','vq'});
R.required.finite_violation_witness = local_filter_hits(hits, ...
    {'witness','violation'});

R.required_counts = struct( ...
    'side_assignment', numel(R.required.per_candidate_side_assignment), ...
    'knot_segment', numel(R.required.knot_times_and_segment_durations), ...
    'knot_state', numel(R.required.knot_position_velocity_norm_inputs), ...
    'segment_bounds', numel(R.required.segment_velocity_accel_jerk_bounds), ...
    'stageB_leaf', numel(R.required.stageB_worst_leaf), ...
    'finite_witness', numel(R.required.finite_violation_witness));

%% -----------------------------------------------------------------------------------------------
%  E. Classify whether existing exposed state is sufficient
% ------------------------------------------------------------------------------------------------
haveSide = R.required_counts.side_assignment > 0;
haveSeg  = R.required_counts.knot_segment > 0;
haveBnd  = R.required_counts.segment_bounds > 0;
haveLeaf = R.required_counts.stageB_leaf > 0;
haveWit  = R.required_counts.finite_witness > 0;

if haveSide && haveSeg && haveBnd && (haveLeaf || haveWit)
    R.exposure_classification = 'EXISTING_EXPOSURE_MAY_BE_SUFFICIENT';
else
    R.exposure_classification = 'SHADOW_INSTRUMENTATION_REQUIRED';
end

%% -----------------------------------------------------------------------------------------------
%  F. Locate authoritative current source only; do not edit it
% ------------------------------------------------------------------------------------------------
src = which('REPLANNING_SYSTEM');
R.replanning_system_source = src;
if ~isempty(src) && isfile(src)
    R.replanning_system_sha256 = local_sha256(src);
    txt = fileread(src);
    R.source_markers = struct();
    R.source_markers.candidate_loop = local_line_of(txt,'for ic=1:numel(CANDS)');
    R.source_markers.tighten_bounds = local_line_of(txt,'tighten_piecewise_derivative_bounds_for_speed');
    R.source_markers.stageA = local_line_of(txt,'Stage A');
    R.source_markers.stageB = local_line_of(txt,'Stage B');
    R.source_markers.local_vector = local_line_of(txt,'build_piecewise_local_vector_speed_certificate');
else
    R.replanning_system_sha256 = '';
    R.source_markers = struct();
end

%% -----------------------------------------------------------------------------------------------
%  G. Console report
% ------------------------------------------------------------------------------------------------
fprintf('\n[A. SOURCE]\n');
fprintf('D38 source                                  = %s\n',R.d38_source);
fprintf('REPLANNING_SYSTEM                           = %s\n',local_dispstr(R.replanning_system_source));
fprintf('SHA-256                                     = %s\n',local_dispstr(R.replanning_system_sha256));

fprintf('\n[B. BASE-WORKSPACE RECOVERY]\n');
fprintf('relevant variables captured                 = %d\n',numel(capturedNames));
for i=1:numel(capturedNames)
    fprintf('  %s\n',capturedNames{i});
end

fprintf('\n[C. REQUIRED R38->R39 EXPOSURE COUNTS]\n');
fprintf('1) per-candidate side/sign fields            = %d\n',R.required_counts.side_assignment);
fprintf('2) knot/segment timing fields                = %d\n',R.required_counts.knot_segment);
fprintf('3) knot/control-point state fields           = %d\n',R.required_counts.knot_state);
fprintf('4) velocity/accel/jerk bound fields          = %d\n',R.required_counts.segment_bounds);
fprintf('5) Stage-B worst-leaf fields                 = %d\n',R.required_counts.stageB_leaf);
fprintf('6) finite violation witness fields           = %d\n',R.required_counts.finite_witness);

fprintf('\n[D. EXPOSURE CLASSIFICATION]\n');
fprintf('classification                               = %s\n',R.exposure_classification);

fprintf('\n[E. SOURCE MARKERS]\n');
fn = fieldnames(R.source_markers);
for i=1:numel(fn)
    fprintf('%-44s = %g\n',fn{i},R.source_markers.(fn{i}));
end

fprintf('\n[F. IMPORTANT]\n');
fprintf('This audit does not infer safety from sampled values.\n');
fprintf('This audit does not replace Inf/NaN with finite values.\n');
fprintf('This audit does not relax C7, yaw-C6, speed, rejoin, or geometry gates.\n');
fprintf('If classification = SHADOW_INSTRUMENTATION_REQUIRED, the missing values are function-local\n');
fprintf('and the next audit must instrument a renamed shadow copy of REPLANNING_SYSTEM, not production.\n');
fprintf('====================================================================================================\n');

%% -----------------------------------------------------------------------------------------------
%  H. Save full machine-readable capture and text report
% ------------------------------------------------------------------------------------------------
save('AF7R39_DEEP_EXPOSURE_CAPTURE.mat','R','-v7.3');
local_write_report('AF7R39_DEEP_EXPOSURE_REPORT.txt',R);

assignin('base','R39',R);

end

% =================================================================================================
% Local helpers
% =================================================================================================

function H = local_recursive_collect(x,path,targets,depth,maxDepth)
H = repmat(struct('path','','class','','size',[],'summary',''),0,1);
if depth > maxDepth
    return;
end

try
    if isstruct(x)
        if numel(x) > 1
            for k=1:numel(x)
                H = [H; local_recursive_collect(x(k),sprintf('%s(%d)',path,k),targets,depth+1,maxDepth)]; %#ok<AGROW>
            end
            return;
        end
        fn = fieldnames(x);
        for i=1:numel(fn)
            f = fn{i};
            v = x.(f);
            p = [path '.' f];
            if local_name_hit(f,targets)
                H(end+1,1) = local_make_hit(p,v); %#ok<AGROW>
            end
            if isstruct(v) || iscell(v) || isobject(v)
                H = [H; local_recursive_collect(v,p,targets,depth+1,maxDepth)]; %#ok<AGROW>
            end
        end
    elseif iscell(x)
        for k=1:numel(x)
            v=x{k};
            p=sprintf('%s{%d}',path,k);
            if isstruct(v) || iscell(v) || isobject(v)
                H=[H; local_recursive_collect(v,p,targets,depth+1,maxDepth)]; %#ok<AGROW>
            end
        end
    elseif isobject(x)
        pr=properties(x);
        for i=1:numel(pr)
            f=pr{i};
            try
                v=x.(f);
            catch
                continue;
            end
            p=[path '.' f];
            if local_name_hit(f,targets)
                H(end+1,1)=local_make_hit(p,v); %#ok<AGROW>
            end
            if isstruct(v) || iscell(v)
                H=[H; local_recursive_collect(v,p,targets,depth+1,maxDepth)]; %#ok<AGROW>
            end
        end
    end
catch
end
end

function tf = local_name_hit(name,targets)
low=lower(name);
tf=false;
for i=1:numel(targets)
    if contains(low,lower(targets{i}))
        tf=true;
        return;
    end
end
end

function h = local_make_hit(path,v)
h=struct();
h.path=path;
h.class=class(v);
try
    h.size=size(v);
catch
    h.size=[];
end
h.summary=local_value_summary(v);
end

function s = local_value_summary(v)
try
    if isnumeric(v) || islogical(v)
        if isempty(v)
            s='[]';
        elseif isscalar(v)
            if islogical(v)
                s=sprintf('%d',v);
            else
                s=sprintf('%.17g',double(v));
            end
        elseif numel(v)<=64
            s=mat2str(v,17);
        else
            vv=double(v(:));
            finite=vv(isfinite(vv));
            if isempty(finite)
                s=sprintf('%s %s all-nonfinite',class(v),mat2str(size(v)));
            else
                s=sprintf('%s %s min=%.17g max=%.17g finite=%d/%d', ...
                    class(v),mat2str(size(v)),min(finite),max(finite),numel(finite),numel(vv));
            end
        end
    elseif ischar(v)
        s=v;
    elseif isstring(v) && isscalar(v)
        s=char(v);
    elseif iscell(v)
        s=sprintf('cell %s',mat2str(size(v)));
    elseif isstruct(v)
        s=sprintf('struct %s fields=%d',mat2str(size(v)),numel(fieldnames(v)));
    else
        s=sprintf('%s %s',class(v),mat2str(size(v)));
    end
catch
    s='<unprintable>';
end
end

function F = local_filter_hits(H,keys)
if isempty(H)
    F=H;
    return;
end
keep=false(numel(H),1);
for i=1:numel(H)
    low=lower(H(i).path);
    for j=1:numel(keys)
        if contains(low,lower(keys{j}))
            keep(i)=true;
            break;
        end
    end
end
F=H(keep);
end

function h = local_sha256(file)
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

function n = local_line_of(txt,needle)
n=NaN;
k=strfind(txt,needle);
if isempty(k), return; end
n=1+sum(txt(1:k(1))==newline);
end

function s = local_dispstr(v)
if isempty(v), s=''; else, s=char(string(v)); end
end

function local_write_report(file,R)
fid=fopen(file,'w');
if fid<0, return; end
c=onCleanup(@() fclose(fid));

fprintf(fid,'AF7R39 DEEP EXPOSURE AUDIT\n');
fprintf(fid,'timestamp=%s\n',R.timestamp);
fprintf(fid,'production_modified=0\n');
fprintf(fid,'acceptance_modified=0\n');
fprintf(fid,'certificate_semantics_modified=0\n');
fprintf(fid,'D38_source=%s\n',R.d38_source);
fprintf(fid,'REPLANNING_SYSTEM=%s\n',local_dispstr(R.replanning_system_source));
fprintf(fid,'SHA256=%s\n',local_dispstr(R.replanning_system_sha256));
fprintf(fid,'classification=%s\n\n',R.exposure_classification);

fprintf(fid,'REQUIRED_COUNTS\n');
fprintf(fid,'side_assignment=%d\n',R.required_counts.side_assignment);
fprintf(fid,'knot_segment=%d\n',R.required_counts.knot_segment);
fprintf(fid,'knot_state=%d\n',R.required_counts.knot_state);
fprintf(fid,'segment_bounds=%d\n',R.required_counts.segment_bounds);
fprintf(fid,'stageB_leaf=%d\n',R.required_counts.stageB_leaf);
fprintf(fid,'finite_witness=%d\n\n',R.required_counts.finite_witness);

fprintf(fid,'ALL_RELEVANT_HITS\n');
for i=1:numel(R.hits)
    fprintf(fid,'%s | %s | %s | %s\n', ...
        R.hits(i).path,R.hits(i).class,mat2str(R.hits(i).size),R.hits(i).summary);
end
end
