function [st,status,keepComparison]=CBF_TRAJ_CORRECTION_STEP(st,prev,E)
% REV38 (REV37 D_OUTWARD) pure causal mode/jet update. E is the exact 7th-order homogeneous
% return transition expm(A*dt). CARRY overwrites the TOTAL original-reference offset;
% it does not integrate it, filter it or add it twice.
prior=st.mode;status=prev.status;keepComparison=false;
% Only a previously selected RETURN law is advanced. A valid new CARRY
% snapshot already describes the end of the completed interval.
if strcmp(prior,'RETURN')
    st.correction=st.correction*E.';
end
if any(~isfinite(st.correction(:)))
    error('CBFTraj:CorrectionNonfinite','Internal return jet failed');
end
if ~prev.valid
    st=hold_invalid(st);return
end
if strcmp(prior,'DEGRADED')
    % Do not re-seed/accumulate an invalid comparison in this episode.
    status='COMPARISON_LATCHED_OFF';
elseif prev.active || strcmp(prior,'CARRY')
    if ~strcmp(prior,'CARRY')
        % An original-reference comparison is a TOTAL offset, not an
        % increment relative to the remaining RETURN offset.
        % Never reseed from the actual state during RETURN.
        if strcmp(prior,'RETURN') && prev.comparisonStarted
            st=hold_invalid(st);status='RETURN_BASELINE_RESTART_REJECTED';return
        end
        st.anchor=zeros(3,1);st.mode='CARRY';
    end
    if ~prev.comparisonValid
        st=hold_invalid(st);status=['COMPARISON_INVALID:' prev.comparisonStatus];return
    end
    pv=prev.modelOffsetPV(:);
    if numel(pv)~=6 || ~isreal(pv) || any(~isfinite(pv))
        st=hold_invalid(st);status='COMPARISON_TARGET_INVALID';return
    end
    candidate=pv(1:3);
    st.anchor=zeros(3,1);
    % Retain REV20's diagnostic domain (not a physical safety constraint).
    if norm(candidate,Inf)>5 || norm(pv(4:6),Inf)>5
        st=hold_invalid(st);status='COMPARISON_TARGET_DOMAIN';return
    end
    % Preserve all original derivatives of orders 1..6.
    st.correction=[candidate,zeros(3,6)];
    st.center=candidate;keepComparison=true;
elseif strcmp(prior,'RETURN')
    % Maintain the same comparison baseline without using its offset for
    % the analytic return. If invalid, discard it and prohibit reseeding
    % until NOMINAL. A later intervention then enters DEGRADED.
    keepComparison=prev.comparisonValid && ~prev.comparisonStarted;
    if ~keepComparison && ~strcmp(prev.comparisonStatus,'NOT_USED')
        status=['RETURN_BASELINE_LOST:' prev.comparisonStatus];
    end
end
if prev.active || ~prev.returnGate
    st.quietCount=0;
    if strcmp(st.mode,'RETURN'),st=hold_invalid(st);keepComparison=false;end
    return
end
if strcmp(st.mode,'NOMINAL'),st.quietCount=0;return;end
st.quietCount=st.quietCount+1;
if st.quietCount>=8 && any(strcmp(st.mode,{'CARRY','DEGRADED'}))
    % Same gate as REV20: do not discard a still-growing episode effect.
    if strcmp(st.mode,'CARRY') && prev.comparisonValid && ...
            dot(prev.modelOffsetPV(1:3),prev.modelOffsetPV(4:6))>0
        status='QUIET_WAIT_OUTWARD_MODEL_MOTION';return
    end
    st.mode='RETURN';st.center=zeros(3,1);
    % CARRY retains the valid baseline for a possible re-intervention.
    % DEGRADED has no valid baseline to retain.
    % No return propagation at this switch: current jet is its initial jet.
end
if strcmp(st.mode,'RETURN') && max(abs(st.correction),[],'all')<=1e-5
    st.mode='NOMINAL';st.correction=zeros(3,7);st.center=zeros(3,1);
    st.anchor=zeros(3,1);st.quietCount=0;keepComparison=false;
end
end

function st=hold_invalid(st)
st.quietCount=0;
if strcmp(st.mode,'NOMINAL'),return;end
% Position remains at the last accepted value (or current analytic RETURN
% endpoint). No stale velocity extrapolation and no invalid displacement.
% Derivative reset is deliberate; inter-sample C6 is not required here.
st.correction(:,2:7)=0;st.center=st.correction(:,1);st.mode='DEGRADED';
end
