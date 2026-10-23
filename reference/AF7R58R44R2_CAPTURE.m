function R = AF7R58R44R2_CAPTURE(P, saveFile)
% Read-only, post-run capture from an ACTUAL REPLANNING_SYSTEM instance.
% No Flight I/O, reference mutation, invented control points or safety claim.
if nargin < 1 || isempty(P)
    if evalin('base', 'exist(''replanner_instance'',''var'') == 1')
        P = evalin('base', 'replanner_instance');
    else
        R = local_empty('NO_LIVE_INSTANCE');
        fprintf('[R44R2] No live replanner. Use AF7R58R44R2_RUN_AND_CAPTURE.\n');
        return;
    end
end
if nargin < 2 || isempty(saveFile)
    saveFile = fullfile(fileparts(mfilename('fullpath')), ...
        'AF7R58R44R2_ACTIVE_SNAPSHOT.mat');
end
assert(isa(P,'REPLANNING_SYSTEM'), ...
    '[R44R2] An actual REPLANNING_SYSTEM instance is required.');
saveFile = char(saveFile);
R = local_empty('LIVE_REPLANNER_READ');
R.active_mode = char(P.active_reference_mode);
R.active_version = double(P.active_reference_version);
R.commit_count = double(P.replan_commit_count);
R.snapshot_file = saveFile;
R.trigger_source = 'ACTIVE_REFERENCE_FUTURE_NOT_ESTIMATED_ROLLOUT';
R.estimated_state_available = false;
R.latest_estimated_mass_kg = NaN;
R.estimated_cable_geometry_residual_m = NaN;
try
    Sest = P.self.estimator.result.state;
    if isprop(Sest,'mL') && isprop(Sest,'p') && ...
            isprop(Sest,'pL') && isprop(Sest,'pT')
        R.latest_estimated_mass_kg = double(Sest.mL);
        pQ = double(Sest.p(:));
        pL = double(Sest.pL(:));
        q = double(Sest.pT(:));
        L = double(P.L_cable);
        R.estimated_cable_geometry_residual_m = norm(pL-pQ-L*q);
        R.estimated_state_available = true;
    end
catch ME
    R.estimated_state_readout_issue = ME.message;
end
Snapshot = struct('schema','AF7R58R44R2_ACTIVE_SNAPSHOT_V1', ...
    'source','READ_ONLY_ACTUAL_REPLANNER_POSTRUN', ...
    'mode',R.active_mode, ...
    'version',R.active_version, ...
    'commit_count',R.commit_count, ...
    'latest_estimated_mass_kg',R.latest_estimated_mass_kg, ...
    'corrections',struct([]), 'phase_layers',struct([]), ...
    'contains_committed_c7_control_points',false, ...
    'continuous_combined_return_certified',false, ...
    'wholebody_collision_certified_by_this_audit',false);
if strcmpi(R.active_mode,'REPAIRED')
    A = P.active_repaired_reference;
    if isstruct(A) && isfield(A,'batch_flat_model') && ...
            isstruct(A.batch_flat_model) && ...
            isfield(A.batch_flat_model,'corrections') && ...
            isstruct(A.batch_flat_model.corrections) && ...
            ~isempty(A.batch_flat_model.corrections)
        Snapshot.corrections = A.batch_flat_model.corrections;
        if isfield(A.batch_flat_model,'phase_layers')
            Snapshot.phase_layers = A.batch_flat_model.phase_layers;
        end
        Snapshot.contains_committed_c7_control_points = true;
        if isfield(A,'reference_version')
            assert(double(A.reference_version)==R.active_version, ...
                '[R44R2] Active polygon version does not match active reference.');
        end
    else
        R.status = 'REPAIRED_BUT_NO_CANONICAL_POLYGONS';
    end
else
    R.status = 'NOT_IN_REPAIRED_MODE__NO_ACTIVE_POLYGON';
end
if Snapshot.contains_committed_c7_control_points
    H = AF7R58R44R2_RETURN_HULL(Snapshot);
    R.return_hull = H;
    if H.actual_polygons_available
        R.return_hull_available = true;
        R.status = 'ACTUAL_ACTIVE_CONTROL_POINTS_CAPTURED';
    else
        R.return_hull_available = false;
        R.status = ['POLYGON_LAYOUT_REJECTED__' char(H.status)];
        Snapshot.contains_committed_c7_control_points = false;
    end
else
    R.return_hull = AF7R58R44R2_RETURN_HULL(Snapshot);
end
R.snapshot_saved = true;
save(saveFile,'Snapshot','R');
assert(isfile(saveFile), '[R44R2] Failed to write snapshot file.');
fprintf('[R44R2] saved=%s | mode=%s | commit=%d | real_polygon=%d | status=%s\n', ...
    saveFile,R.active_mode,R.commit_count,R.return_hull_available,R.status);
end

function R=local_empty(status)
R=struct('schema','AF7R58R44R2_CAPTURE_V1', ...
    'status',status,'snapshot_saved',false, ...
    'return_hull_available',false, ...
    'actual_reference_changed',false, ...
    'production_trigger_switched',false, ...
    'wholebody_safety_certified_by_audit',false);
end
