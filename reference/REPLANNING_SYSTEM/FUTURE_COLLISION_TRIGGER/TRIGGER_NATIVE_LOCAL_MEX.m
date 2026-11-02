function B=TRIGGER_NATIVE_LOCAL_MEX(command)
% TRIGGER_NATIVE_LOCAL_MEX: build the LOCAL native kernel iff missing.
% Only call from REPLANNING_SYSTEM.preflight_update, before arming/Flight.
% Never sample sensors, EKF, HLC, plant, obstacle geometry, or reference.
% The generated MEX is NOT trusted until a separate exact-SHA numerical
% parity audit passes; TRIGGER_NATIVE_SOURCE_PIN_CACHE enforces this.
if nargin~=1 || ~strcmp(char(command),'ensure')
    error('TRIGGER_NATIVE_LOCAL_MEX:Command','Only ensure is supported.');
end
root=fileparts(mfilename('fullpath'));
base='TRIGGER_SYMBOLIC_NATIVE_CORE';
source=fullfile(root,[base '.cpp']);
native=fullfile(root,[base '.' mexext]);
B=struct('ready',false,'built',false,'status','NOT_READY', ...
    'reason','','source_path',source,'native_path',native);
if ~isfile(source)
    B.status='MISSING_CPP_SOURCE';
    B.reason='The native C++ source is not present in the trigger folder.';
    return
end
if ~isfile(native)
    % This branch MUST execute only during software preflight; no hot path.
    % No calls that access the simulated or real obstacle environment.
    try
        mex('-R2018a','-O','-outdir',root,'-output',base,source);
        rehash
        B.built=true;
    catch ME
        B.status='MEX_BUILD_FAILED';
        B.reason=sprintf('%s: %s',ME.identifier,ME.message);
        return
    end
end
if ~isfile(native)
    B.status='MEX_OUTPUT_NOT_FOUND';
    B.reason='Build completed but the local native binary was not found.';
    return
end
resolved=which(base);
if isempty(resolved) || ~strcmpi(char(resolved),char(native))
    B.status='MEX_SOURCE_SHADOWED';
    B.reason=['MATLAB resolves a different native kernel: ' char(resolved)];
    return
end
B.ready=true;
if B.built
    B.status='LOCAL_MEX_BUILT__PARITY_AUDIT_REQUIRED';
else
    B.status='EXISTING_LOCAL_MEX__PARITY_PIN_STILL_REQUIRED';
end
end
