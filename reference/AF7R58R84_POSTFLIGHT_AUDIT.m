function R=AF7R58R84_POSTFLIGHT_AUDIT(root)
% R84 postflight check of duplicated-obstacle reports vs real frame mismatch.
% Does NOT modify current replanner, reference, sensor, HLC, EKF or plant.
% Uses EXISTING R71R2 event captures; one snapshot per version, <=3.
obj=r84_find_replanner(root,5);
assert(~isempty(obj),'R84:LiveReplannerNotFound');
assert(isprop(obj,'r71r2_event_snapshots'),'R84:EventCapturePropertyMissing');
S=obj.r71r2_event_snapshots;
assert(isstruct(S)&&~isempty(S),'R84:NoHistoricalEventSnapshotsInAgent');
assert(exist('g26_future_trigger_kernel_mex','file')==3, ...
    'R84:ProductionTriggerMEXNotFound');
E=struct([]);
for j=1:numel(S)
    X=AF7R58R84_PACKET_FRAME_COMPARE(S(j));
    if isempty(E),E=X;else,E(end+1)=X;end %#ok<AGROW>
end
pairs=struct('first_event',{},'second_event',{},'obstacle_id',{}, ...
    'first_version',{},'second_version',{},'same_obstacle',{}, ...
    'second_hlc_collision',{},'post_first_commit_unsafe_witness',{});
for j=1:numel(E)
    for k=j+1:numel(E)
        a=E(j); b=E(k);
        sameId=isfinite(a.original_obstacle_id)&& ...
            a.original_obstacle_id==b.original_obstacle_id;
        if sameId && b.version>a.version
            z=struct('first_event',j,'second_event',k, ...
                'obstacle_id',a.original_obstacle_id, ...
                'first_version',a.version,'second_version',b.version, ...
                'same_obstacle',true, ...
                'second_hlc_collision',b.hlc_hit, ...
                'post_first_commit_unsafe_witness',b.hlc_hit && ...
                    b.hlc_obstacle_id==a.original_obstacle_id);
            pairs(end+1)=z; %#ok<AGROW>
        end
    end
end
R=struct('status','R84_POSTFLIGHT_REFERENCE_ONLY_FRAME_AUDIT__NOT_CERTIFIED', ...
   'event_count',numel(E),'events',E,'same_obstacle_pairs',pairs, ...
   'same_obstacle_new_version_count',numel(pairs), ...
   'captured_only_one_event_per_version',true, ...
   'snapshot_capture_capacity',3, ...
   'suppresses_future_triggers',false, ...
   'continuous_time_safety_proved',false, ...
   'actual_HLC_plant_rollout_performed',false, ...
   'production_modified',false, ...
   'source_snapshots_for_optional_full_interval_scan',S);
% Save only portable DATA (no event closures or mutable object handles).
folder=fileparts(mfilename('fullpath'));
record=rmfield(R,'source_snapshots_for_optional_full_interval_scan');
mat=fullfile(folder,'AF7R58R84_POSTFLIGHT_AUDIT.mat');
txt=fullfile(folder,'AF7R58R84_POSTFLIGHT_AUDIT.txt');
save(mat,'record');
fid=fopen(txt,'wt');
assert(fid>=0,'R84:CannotOpenReport');
c=onCleanup(@()fclose(fid)); %#ok<NASGU>
fprintf(fid,'R84 SOURCE-BOUND SAME-OBSTACLE / HLC FRAME POSTFLIGHT AUDIT\n');
fprintf(fid,'Reference ONLY; sampled no-hit does NOT imply safety.\n');
fprintf(fid,'Event snapshots one/reference version, at most 3; NOT all attempts.\n');
for j=1:numel(E)
    a=E(j);
    fprintf(fid,'event=%d t=%.9f ver=%d originalID=%g knownIDs=%s\n', ...
        j,a.event_time_s,a.version,a.original_obstacle_id,mat2str(a.known_obstacle_ids));
    fprintf(fid,'  RAW  hit=%d id=%g comp=%g ttc=%.9f\n', ...
        a.raw_hit,a.raw_obstacle_id,a.raw_component,a.raw_ttc_s);
    fprintf(fid,'  HLC  hit=%d id=%g comp=%g ttc=%.9f frameShiftZ=%.9f\n', ...
        a.hlc_hit,a.hlc_obstacle_id,a.hlc_component,a.hlc_ttc_s, ...
        a.flight_to_HLC_z_shift_m);
end
for j=1:numel(pairs)
    p=pairs(j);
    fprintf(fid,'SAME-ID NEW-VERSION first=%d second=%d id=%g HLC_rehit=%d\n', ...
        p.first_event,p.second_event,p.obstacle_id,p.post_first_commit_unsafe_witness);
end
fprintf(fid,'PROVED_FULL_CLOSED_LOOP=0 | PRODUCTION_MODIFIED=0\n');
R.report_file=txt;R.mat_file=mat;
end

function obj=r84_find_replanner(x,depth)
obj=[];
if isa(x,'REPLANNING_SYSTEM')&&isscalar(x),obj=x;return;end
if depth<=0||isempty(x)||numel(x)~=1,return;end
if iscell(x)
    for j=1:min(numel(x),32)
        obj=r84_find_replanner(x{j},depth-1);
        if ~isempty(obj),return;end
    end
    return;
end
if isstruct(x)
    names=fieldnames(x);
elseif isobject(x)
    try,names=properties(x);catch,return;end
else
    return;
end
skip={'self','result','parameter','plant','sensor','estimator', ...
    'controller','logger','time','state','parent'};
for j=1:min(numel(names),96)
    nm=names{j};if any(strcmpi(nm,skip)),continue;end
    try
        v=x.(nm);
        obj=r84_find_replanner(v,depth-1);
        if ~isempty(obj),return;end
    catch
    end
end
end
