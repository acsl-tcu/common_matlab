function B=AF7R58R122_BIND_LIVE_C7(replanner,tGlobal)
% Read-only bridge from the ACTUAL current REPLANNING_SYSTEM to R121 planner.
% Does NOT modify nominal order 6, the 28-vector controller interface,
% existing HLC, Flight state, or any algorithm's commit policy.
% Only valid for a known same-version, Flight-time source authority.
assert(isa(replanner,'REPLANNING_SYSTEM'), ...
    'R122:LiveReplannerRequired', ...
    'Use the live REPLANNING_SYSTEM instance, not an obsolete capture.');
assert(isnumeric(tGlobal)&&isscalar(tGlobal)&&isfinite(tGlobal), ...
    'R122:FiniteGlobalTimeRequired','Pass global physical Flight time.');
assert(isa(replanner.base_ref,'TIME_VARYING_REFERENCE') && ...
    isa(replanner.base_ref.func,'function_handle'), ...
    'R122:SourceBoundNominalRequired', ...
    'Original TIME_VARYING_REFERENCE source must be available.');
fns=replanner.nominal_flat_derivative_fn;
assert(iscell(fns)&&numel(fns)>=8&& ...
    all(cellfun(@(x)isa(x,'function_handle'),fns(1:8))), ...
    'R122:ExistingSourceC7Required', ...
    'R115 did not construct eight source-derived position derivatives.');
fns=fns(1:8);  % Freeze *source handles* for this planning event.
flightT0=double(replanner.flight_reference_time_origin);
assert(isscalar(flightT0)&&isfinite(flightT0) && tGlobal>=flightT0, ...
    'R122:FlightOriginRequired', ...
    'Flight origin is required. No guessed time offset is allowed.');
L=double(replanner.self.parameter.get("cableL"));
assert(isscalar(L)&&isfinite(L)&&L>0, ...
    'R122:RuntimeCableLengthRequired', ...
    'Runtime cableL authority is missing.');
% Read exactly the ACTIVE controller-bound C7 authority, not a stale
% nominal sample or a fresh reconstructed approximation.
% R125: In REPAIRED mode the old R63 analytical reconstruction may
% disagree with the *actual HLC-bound* MEX evaluator (observed previously
% at order one). R86 reads orders 0..6 from the actual native evaluator
% and derives order seven with the native serialized correction op=25;
% it independently compares 0..6 against active flat_eval_fn. Never
% reconstruct a repaired C7 boundary from an unverified R63 fallback.
if strcmp(upper(char(replanner.active_reference_mode)),'REPAIRED')
    A=replanner.r86_live_serialized_c7_authority(double(tGlobal));
else
    A=replanner.r63_live_c7_authority(double(tGlobal));
end
assert(isstruct(A)&&isfield(A,'valid') && isequal(A.valid,true) && ...
    isfield(A,'active_jets_0_to_7')&&isfield(A,'phase_jets_0_to_7')&& ...
    isfield(A,'reference_version')&&isfield(A,'nominal_jets_0_to_7'), ...
    'R122:ActiveC7AuthorityUnavailable', ...
    'Exact 0..7 active source consistency is not established.');
J0=double(A.active_jets_0_to_7);
JN=double(A.nominal_jets_0_to_7);
P=double(A.phase_jets_0_to_7);
assert(isequal(size(J0),[3 8])&&isequal(size(JN),[3 8])&& ...
    isequal(size(P),[1 8]) && ...
    all(isfinite(J0(:)))&&all(isfinite(JN(:)))&&all(isfinite(P(:))), ...
    'R122:InvalidActiveC7Shape','C7 jets/phase have invalid shapes.');
% The source binding is the SAME nominal law already used by replanner;
% it is NOT independent new gen_ref_line/circle/saddle source logic.
% For naturally retimed repaired references, the canonical nominal jet
% is evaluated at the active PHASE, not at the wall-clock Flight time.
% R122 previously compared different physical times and could reject a
% perfectly valid non-identity phase. This never relaxes the equality test.
phaseStart=double(P(1));
assert(isscalar(phaseStart)&&isfinite(phaseStart) && ...
    phaseStart>=flightT0 && ...
    phaseStart<=tGlobal+64*eps(max(1,abs(tGlobal))), ...
    'R122:InvalidNaturalPhase', ...
    'Live phase must be inside the nominal Flight source and not ahead.');
N=AF7R58R122_NOMINAL_PAYLOAD_JETS(fns,flightT0,L,phaseStart);
Nraw=N;Nraw(3,1)=Nraw(3,1)+L;
scale=max(1,max(abs(JN),abs(Nraw)));
tol=1024*eps(scale);
assert(all(abs(JN(:)-Nraw(:))<=tol(:)), ...
    'R122:SourceMismatch', ...
    'Frozen nominal 7-jets disagree with live R63 source at this event.');
J0(3,1)=J0(3,1)-L;
B=struct();
B.status='SOURCE_BOUND_C7_PAYLOAD_INTERNAL__NO_COMMIT';
B.initial_reference_jets07=J0;
B.nominal_jets07_fn=@(tg)AF7R58R122_NOMINAL_PAYLOAD_JETS( ...
    fns,flightT0,L,tg);
B.nominal_phase_start_s=phaseStart;
B.t0_s=double(tGlobal);
B.nominal_flight_origin_s=flightT0;
B.live_cable_length_m=L;
B.reference_mode=char(A.reference_mode);
B.reference_version=double(A.reference_version);
B.nominal_source_signature=func2str(replanner.base_ref.func);
B.controller_packet_width=28;
B.planner_position_derivative_order=7;
B.controller_position_derivative_order=6;
B.source_model_exact_seventh_available=true;
B.active_c7_authority_source='R63_EXACT_NOMINAL';
if strcmp(upper(char(A.reference_mode)),'REPAIRED')
    B.active_c7_authority_source=char(A.r86_source);
end
B.active_flat_eval_0_to_6_verified=logical(A.valid);
B.active_flat_eval_max_abs_jet_residual=double(A.flat_active_jet_consistency_error);
B.position_payload_world=true;
B.yaw_is_not_certified_by_this_binding=true;
B.tail_is_not_certified_by_this_binding=true;
B.production_commit_authorized=false;
B.revalidate_reference_version_at_commit=true;
end
