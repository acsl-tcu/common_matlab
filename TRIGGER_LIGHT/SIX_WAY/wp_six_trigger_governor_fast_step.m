function [zNew,tauNew,tauDer] = wp_six_trigger_governor_fast_step(z,tau,cmd,C)
%WP_SIX_TRIGGER_GOVERNOR_FAST_STEP Precomputed linear-algebra equivalent
% of original wp_governor_step. Input is PREVIOUS held command (not future
% command, not an assumed constant command over the whole 4 seconds).
if ~(isequal(size(z),[6,1]) && all(isfinite(z)) && ...
        isscalar(tau) && isfinite(tau) && isscalar(cmd) && ...
        isfinite(cmd) && cmd>=0)
    error('LightTrigger:GovernorState','Invalid governor inputs');
end
zNew=C.E*z+C.Bc*cmd;
tauNew=tau+C.W*z+C.tauCmd*cmd;
tauDer=[tauNew;C.D*zNew];
end
