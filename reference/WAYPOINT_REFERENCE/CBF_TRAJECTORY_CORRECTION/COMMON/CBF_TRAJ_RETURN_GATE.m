function [ok,reason]=CBF_TRAJ_RETURN_GATE(ctrl,physical,id)
% Previous sample only. Not a certificate of future return safety.
ok=false;reason='CBF_INTERVENTION';
if physical.active,return;end
reason='RAW_INPUT_OUT_OF_BOX';raw=ctrl.tmp(:);
if any(raw<[0;-1;-1;-1]-1e-9 | raw>[20;1;1;1]+1e-9),return;end
reason='CBF_STATUS_UNKNOWN';
if ~isfield(ctrl,'cbf') || ~isstruct(ctrl.cbf),return;end
c=ctrl.cbf;
if ~isfield(c,'success') || ~isequal(c.success,true),reason='CBF_NOT_SUCCESS';return;end
if isfield(c,'appliedWithoutCertificate') && c.appliedWithoutCertificate
    reason='UNCERTIFIED_FALLBACK';return
end
if id==0,ok=true;reason='CBF_OFF';return;end
if ~isfield(c,'nominalFeasible') || ~isequal(c.nominalFeasible,true)
    reason='NOMINAL_NOT_FEASIBLE';return
end
if ~isfield(c,'maxViolation') || ~isscalar(c.maxViolation) || ...
        ~isfinite(c.maxViolation) || c.maxViolation>1e-8
    reason='CBF_RESIDUAL';return
end
if isfield(c,'stageValues') && any(~isfinite(c.stageValues(:)) | c.stageValues(:)<-1e-8)
    reason='CBF_STAGE_NEGATIVE';return
end
ok=true;reason='PREVIOUS_SAMPLE_FEASIBLE';
end
