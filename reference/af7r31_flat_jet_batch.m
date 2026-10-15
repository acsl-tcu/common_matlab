function [Y,valid] = af7r31_flat_jet_batch(replanner,tg)
tg=double(tg(:).');
tf0=double(replanner.flight_reference_time_origin);
assert(isfinite(tf0),'[AF7R31] flight time origin unavailable.');
tau=max(0,tg-tf0);
Y=double(af7r31_nominal_flat_batch_mex(tau));
valid=size(Y,1)==21 && size(Y,2)==numel(tg) && all(isfinite(Y),1);
end
