function [E,M,AUX,source] = AF7R52_LOAD_TRUE_TRIGGER_AUTHORITY(replanner)
% Load the exact true-trigger authority used by the R49/R50 theory audits.
% Priority:
%   1) valid event still present in the supplied replanner object
%   2) AF7R50_THEORY_SNAPSHOT.mat saved beside REPLANNING_SYSTEM
%   3) AF7R50_THEORY_SNAPSHOT.mat in current folder
%
% No runtime authority is modified.

assert(isa(replanner,'REPLANNING_SYSTEM'),'[AF7R52] replanner type mismatch.');
AUX=struct(); source='';

E=[];
try
    q=replanner.get_first_trigger_event();
    if isstruct(q)&&isfield(q,'valid')&&logical(q.valid)
        E=q; source='LIVE_REPLANNER_FIRST_TRIGGER_EVENT';
    end
catch
end

M=[];
if ~isempty(E)
    if replanner.nominal_batch_model_cache_valid && ...
            isstruct(replanner.nominal_batch_model_cache) && ...
            isfield(replanner.nominal_batch_model_cache,'certified') && ...
            logical(replanner.nominal_batch_model_cache.certified)
        M=replanner.nominal_batch_model_cache;
    end
end

if isempty(E) || isempty(M)
    root=fileparts(which('REPLANNING_SYSTEM'));
    cand={fullfile(root,'AF7R50_THEORY_SNAPSHOT.mat'), ...
          fullfile(pwd,'AF7R50_THEORY_SNAPSHOT.mat')};
    hit='';
    for k=1:numel(cand)
        if isfile(cand{k}), hit=cand{k}; break; end
    end
    assert(~isempty(hit), ...
        ['[AF7R52] true-trigger authority unavailable. Expected ' ...
         'AF7R50_THEORY_SNAPSHOT.mat from AF7R50C_SAVE_THEORY_SNAPSHOT.']);
    S=load(hit);
    assert(isfield(S,'E')&&isfield(S,'M'), ...
        '[AF7R52] theory snapshot lacks E/M.');
    E=S.E; M=S.M; source=hit;
    if isfield(S,'B49'), AUX.B49=S.B49; end
    if isfield(S,'R50A'), AUX.R50A=S.R50A; end
end

assert(isstruct(E)&&isfield(E,'valid')&&logical(E.valid));
assert(isfield(E,'reference')&&strcmpi(char(E.reference.mode),'NOMINAL'), ...
    '[AF7R52] the saved true trigger is not NOMINAL authority.');
assert(isstruct(M)&&isfield(M,'certified')&&logical(M.certified));

% Source/signature freshness: the exact AF7R33 evaluator checks the current
% nominal derivative signature against the saved model.
af7r33_flat_jet_batch_model(replanner,double(E.event_time),M);

% Geometric/live quantities used by AF7R36 must still match the saved event.
D=E.dependency_signature;
check_exact('dt',double(replanner.prediction_dt),double(D.dt));
check_exact('sensor_range',double(replanner.trigger_dist),double(D.sensor_range));
check_exact('horizon',double(replanner.horizon_s),double(D.horizon));
check_exact('r_drone',double(replanner.r_drone),double(D.r_drone));
check_exact('r_load',double(replanner.r_load),double(D.r_load));
check_exact('r_cable',double(replanner.r_cable),double(D.r_cable));
check_exact('d_margin',double(replanner.d_margin),double(D.d_margin));
check_exact('cable_length',double(replanner.L_cable),double(D.cable_length));
check_exact('gravity',double(replanner.gravity),double(D.gravity));
check_exact('hard_modify_deadline_s',double(replanner.hard_modify_deadline_s),double(D.hard_modify_deadline_s));
end

function check_exact(name,a,b)
tol=64*eps(max([1,abs(a),abs(b)]));
assert(isfinite(a)&&isfinite(b)&&abs(a-b)<=tol, ...
    '[AF7R52] saved/current %s mismatch: %.17g vs %.17g.',name,a,b);
end
