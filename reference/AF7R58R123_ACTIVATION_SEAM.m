function S=AF7R58R123_ACTIVATION_SEAM(obj,route,eventVersion,ta)
% Bind the GENERATED C7 start to the exact active source at native Stage3
% activation (two nominal control cycles after trigger), not to EKF now.
S=struct('certified',false,'status','SOURCE_MISMATCH', ...
    'reference_version_same',false,'same_0_to_7_real_jets',false, ...
    'maximum_absolute_jet_residual',NaN, ...
    'activation_time_s',ta,'production_commit_authorized',false);
assert(isa(obj,'REPLANNING_SYSTEM')&&isstruct(route)&& ...
    isfield(route,'knot_jets')&&isequal(size(route.knot_jets,1),3)&& ...
    size(route.knot_jets,2)==8 && isfinite(ta), ...
    'R123:ActivationSeamInputs');
if double(obj.active_reference_version)~=double(eventVersion)
    S.status='STALE_ACTIVE_REFERENCE_VERSION';return;
end
S.reference_version_same=true;
B=AF7R58R122_BIND_LIVE_C7(obj,double(ta));
J=double(route.knot_jets(:,:,1));Jtrue=double(B.initial_reference_jets07);
assert(isequal(size(J),[3 8])&&all(isfinite(J(:))), ...
    'R123:InvalidGeneratedStartJets');
err=max(abs(J(:)-Jtrue(:)));
S.maximum_absolute_jet_residual=err;
% Use a recorded diagnostic tolerance; neither sampling nor this
% floating-point comparison is an outward-rounded IEEE C7 proof.
scale=max([1;abs(J(:));abs(Jtrue(:))]);
bound=1024*eps(scale);
S.same_0_to_7_real_jets=err<=bound;
S.certified=S.reference_version_same&&S.same_0_to_7_real_jets;
if S.certified
    S.status='SAME_SOURCE_ACTIVATION_C7_JETS_NUMERIC_MATCH__NOT_EXECUTION_PROOF';
else
    S.status='BLOCKED_C7_ACTIVATION_JETS_DIFFER';
end
end
