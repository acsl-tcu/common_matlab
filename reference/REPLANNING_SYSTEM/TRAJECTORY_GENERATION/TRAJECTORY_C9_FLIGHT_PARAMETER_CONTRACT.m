classdef TRAJECTORY_C9_FLIGHT_PARAMETER_CONTRACT
% TRAJECTORY_C9_FLIGHT_PARAMETER_CONTRACT
% One Flight: freeze external *non-payload-mass* physical authority and
% per-flight collision/tracking envelopes. EACH candidate uses current EKF
% payload mass, and cannot inherit a previous Flight's certificate.
% No Flight runner / EKF / HLC modifications and no background work.
% Important: the external future-mass envelope is an ASSUMPTION that must
% be justified by the estimator/plant authority. This class cannot infer it.
methods (Static)
    function C=begin(binding,agent,time,req)
        C=struct('valid',false,'status','UNKNOWN_FLIGHT_NOT_BOUND', ...
            'safe_to_commit',false,'binding',binding, ...
            'flight_time_origin_s',NaN,'flight_session_id','','nonmass_authority',struct(), ...
            'geometry_contract',struct(), ...
            'mass_envelope',struct(),'has_future_mass_envelope',false, ...
            'first_estimated_payload_mass_kg',NaN);
        S=TRAJECTORY_C9_HLC_FORCE_AUTHORITY.current(binding,agent,time);
        if ~S.valid, C.status=S.status;return;end
        if ~isstruct(req)||~isfield(req,'flight_session_id')|| ...
                ~(ischar(req.flight_session_id)||isstring(req.flight_session_id))|| ...
                isempty(char(req.flight_session_id))
            C.status='UNKNOWN_MISSING_FRESH_FLIGHT_SESSION_ID';return
        end
        [ok,GC,reason]=TRAJECTORY_C9_FLIGHT_PARAMETER_CONTRACT.geometry(req);
        if ~ok, C.status=reason;return;end
        C.flight_session_id=char(req.flight_session_id);
        C.nonmass_authority=rmfield(S.authority,'payload_mass_kg');
        C.geometry_contract=GC;
        C.first_estimated_payload_mass_kg=S.estimated_payload_mass_kg;
        C.flight_time_origin_s=S.tick_time_s;
        if isfield(req,'future_mass_envelope')
            M=req.future_mass_envelope;
            if ~TRAJECTORY_C9_FLIGHT_PARAMETER_CONTRACT.valid_envelope(M)
                C.status='UNKNOWN_INVALID_EXTERNAL_FUTURE_MASS_ENVELOPE';return
            end
            if S.tick_time_s<M.valid_from_s||S.tick_time_s>M.valid_until_s|| ...
                    S.estimated_payload_mass_kg<M.lower_kg|| ...
                    S.estimated_payload_mass_kg>M.upper_kg
                C.status='UNKNOWN_LIVE_MASS_OUTSIDE_EXTERNAL_ENVELOPE';return
            end
            C.mass_envelope=M;
            C.has_future_mass_envelope=true;
        end
        C.status='FLIGHT_NONMASS_PARAMETERS_BOUND__NO_HLC_COMMIT';
        C.valid=true;
    end
    function S=check(C,binding,agent,time,req,current_snapshot)
        S=struct('valid',false,'status','UNKNOWN_FLIGHT_CONTRACT', ...
            'safe_to_commit',false,'estimated_payload_mass_kg',NaN, ...
            'future_mass_envelope_available',false,'mass_envelope',struct(), ...
            'current_snapshot',struct());
        if ~isstruct(C)||~isscalar(C)||~isfield(C,'valid')||~C.valid
            S.status='UNKNOWN_FLIGHT_CONTRACT_NOT_INITIALIZED';return
        end
        if ~isstruct(req)||~isfield(req,'flight_session_id')|| ...
                ~(ischar(req.flight_session_id)||isstring(req.flight_session_id))|| ...
                ~strcmp(char(req.flight_session_id),C.flight_session_id)
            S.status='UNKNOWN_FLIGHT_SESSION_ID_CHANGED';return
        end
        if ~isequal(binding,C.binding)
            S.status='UNKNOWN_HLC_BINDING_CHANGED_SINCE_FLIGHT_START';return
        end
        % Caller may pass a current() snapshot that it JUST obtained in
        % this same synchronous replanning call. Do not capture the same
        % HLC/EKF state a second time. Public/legacy 5-arg calls continue
        % to capture their own live state, so this is backward compatible.
        if nargin>=6
            now=current_snapshot;
            if ~isstruct(now)||~isscalar(now)|| ...
                    ~all(isfield(now,{'valid','status','tick_time_s', ...
                        'authority','estimated_payload_mass_kg'}))|| ...
                    ~isequal(now.tick_time_s,double(time.t))
                S.status='UNKNOWN_INVALID_OR_STALE_SHARED_FLIGHT_SNAPSHOT';return
            end
        else
            now=TRAJECTORY_C9_HLC_FORCE_AUTHORITY.current(binding,agent,time);
        end
        S.current_snapshot=now;
        if ~now.valid,S.status=now.status;return;end
        if now.tick_time_s<C.flight_time_origin_s
            S.status='UNKNOWN_PREVIOUS_FLIGHT_CONTRACT_REUSE';return
        end
        if ~isequaln(rmfield(now.authority,'payload_mass_kg'), ...
                C.nonmass_authority)
            S.status='UNKNOWN_NONMASS_PHYSICS_CHANGED_WITHIN_FLIGHT';return
        end
        [ok,GC,reason]=TRAJECTORY_C9_FLIGHT_PARAMETER_CONTRACT.geometry(req);
        if ~ok,S.status=reason;return;end
        if ~isequaln(GC,C.geometry_contract)
            S.status='UNKNOWN_GEOMETRY_OR_TUBE_CHANGED_WITHIN_FLIGHT';return
        end
        S.estimated_payload_mass_kg=now.estimated_payload_mass_kg;
        if C.has_future_mass_envelope
            M=C.mass_envelope;
            if ~isfield(req,'future_mass_envelope')|| ...
                    ~isequaln(M,req.future_mass_envelope)
                S.status='UNKNOWN_FUTURE_MASS_ENVELOPE_REPLACED';return
            end
            if now.tick_time_s<M.valid_from_s||now.tick_time_s>M.valid_until_s|| ...
                    now.estimated_payload_mass_kg<M.lower_kg|| ...
                    now.estimated_payload_mass_kg>M.upper_kg
                S.status='UNKNOWN_LIVE_MASS_OUTSIDE_EXTERNAL_ENVELOPE';return
            end
            S.future_mass_envelope_available=true;
            S.mass_envelope=M;
        elseif isfield(req,'future_mass_envelope')
            S.status='UNKNOWN_FUTURE_MASS_ENVELOPE_INTRODUCED_MIDFLIGHT';return
        end
        S.valid=true;
        S.status='CURRENT_FLIGHT_NONMASS_CONSTANT__EKF_MASS_FRESH';
    end
    function [ok,GC,reason]=geometry(req)
        ok=false;GC=struct();reason='UNKNOWN_MISSING_FLIGHT_GEOMETRY_AUTHORITY';
        if ~isstruct(req)||~isscalar(req)||~all(isfield(req, ...
                {'radii','geometry_options','force_options'})),return;end
        g=req.geometry_options;f=req.force_options;r=req.radii;
        if ~isstruct(g)||~isscalar(g)||~isstruct(f)||~isscalar(f)|| ...
                ~isstruct(r)||~isscalar(r),return;end
        if ~all(isfield(r,{'uav_m','cable_m','payload_m'}))|| ...
                ~all(isfield(g,{'robust_tube_enabled','numerical_guard_m','trajectory_phase'}))|| ...
                ~isfield(f,'force_guard_N'),return;end
        if ~strcmp(char(g.trajectory_phase),'f')
            reason='UNKNOWN_NOT_FLIGHT_GROUND_SCOPE';return
        end
        if ~isscalar(g.robust_tube_enabled)|| ...
                ~(isnumeric(g.robust_tube_enabled)||islogical(g.robust_tube_enabled))|| ...
                ~ismember(double(g.robust_tube_enabled),[0 1])
            reason='UNKNOWN_INVALID_TUBE_SWITCH';return
        end
        tube=struct();
        if logical(g.robust_tube_enabled)
            if ~isfield(g,'uncertainty_bounds_m')
                reason='UNKNOWN_MISSING_PER_FLIGHT_TRACKING_TUBE';return
            end
            tube=g.uncertainty_bounds_m;
            if ~isstruct(tube)||~isscalar(tube)|| ...
                    ~all(isfield(tube,{'uav_m','cable_m','payload_m'}))
                reason='UNKNOWN_INVALID_PER_FLIGHT_TRACKING_TUBE';return
            end
        end
        vals=[double(r.uav_m),double(r.cable_m),double(r.payload_m), ...
            double(g.numerical_guard_m),double(f.force_guard_N)];
        if logical(g.robust_tube_enabled)
            vals=[vals,double(tube.uav_m),double(tube.cable_m),double(tube.payload_m)];
        end
        if ~all(isfinite(vals))||any(vals<0)||~isreal(vals)
            reason='UNKNOWN_INVALID_PER_FLIGHT_GEOMETRY_VALUES';return
        end
        % Only physical envelopes/guards are frozen; search/proof iteration
        % budgets may vary and are NOT physical system parameters.
        GC=struct('radii',r,'robust_tube_enabled',logical(g.robust_tube_enabled), ...
            'uncertainty_bounds_m',tube,'numerical_guard_m',g.numerical_guard_m, ...
            'force_guard_N',f.force_guard_N,'phase','f');
        ok=true;reason='VALID_FLIGHT_GEOMETRY_AUTHORITY';
    end
    function ok=valid_envelope(M)
        ok=false;
        if ~isstruct(M)||~isscalar(M)||~all(isfield(M, ...
                {'lower_kg','upper_kg','valid_from_s','valid_until_s','source'}))
            return
        end
        parts={M.lower_kg,M.upper_kg,M.valid_from_s,M.valid_until_s};
        for i=1:4
            x=parts{i};
            if ~(isnumeric(x)&&isreal(x)&&isscalar(x)&&isfinite(x))
                return
            end
        end
        v=[parts{:}];
        if v(1)<=0||v(2)<v(1)||v(4)<v(3),return;end
        if ~(ischar(M.source)||isstring(M.source))||isempty(char(M.source)),return;end
        ok=true;
    end
end
end
