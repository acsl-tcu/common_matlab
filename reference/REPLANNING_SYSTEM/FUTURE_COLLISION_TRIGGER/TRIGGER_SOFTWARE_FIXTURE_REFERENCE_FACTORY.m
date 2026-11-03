function source=TRIGGER_SOFTWARE_FIXTURE_REFERENCE_FACTORY()
% Isolated reference FACTORY for code-only preflight; not a trajectory.
% Never reads the actual Flight epoch, nominal reference, EKF or obstacle.
source=@(t)zeros(28,1); %#ok<NASGU> % Pure function of a scalar argument.
end
