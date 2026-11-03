function R=TRIGGER_ORIGINAL_HLC_ACTUATOR_SCOPE()
% Read-only source-bound actuator envelope for the ORIGINAL HLC.
% No Flight, obstacle, future reference, EKF or sensor data is accessed.
% The original HLC clamps its FINITE computed control input to these limits.
% This proves neither that the applied plant input is the HLC output nor
% that the HLC was successfully evaluated for every uncertain EKF state.
% It is a source-level obligation and NEVER an execution/safety certificate.
R=struct('status','UNKNOWN_HLC_SOURCE_NOT_PINNED', ...
    'source_sha256','','source_verified',false, ...
    'conditional_finite_HLC_output_bounds',false, ...
    'lower',[0;-1;-1;-1],'upper',[20;1;1;1], ...
    'actual_applied_input_wiring_verified',false, ...
    'singularity_and_finite_output_verified',false, ...
    'safe_to_commit',false,'reason','');
try
    file=which('HLC_SUSPENDED_LOAD');
    assert(~isempty(file) && isfile(file), ...
        'TRIGGER_ORIGINAL_HLC_ACTUATOR_SCOPE:Missing', ...
        'The actual HLC_SUSPENDED_LOAD.m must be on MATLAB path.');
    f=fopen(file,'rb');
    assert(f>=0,'TRIGGER_ORIGINAL_HLC_ACTUATOR_SCOPE:Open','Cannot read HLC source.');
    c=onCleanup(@()fclose(f)); %#ok<NASGU>
    bytes=fread(f,inf,'*uint8');
    md=java.security.MessageDigest.getInstance('SHA-256');
    md.update(typecast(bytes,'int8'));
    raw=typecast(int8(md.digest()),'uint8');
    actual=lower(reshape(dec2hex(raw,2).',1,[]));
    R.source_sha256=actual;
    expected='09b38e167a43979a41660fe32088f44255bb6d3e9591093701b4a41712bced5a';
    R.source_verified=strcmpi(actual,expected);
    if ~R.source_verified
        R.reason='Original HLC source checksum mismatched; no actuator-source claim.';
        return
    end
    R.conditional_finite_HLC_output_bounds=true;
    R.status='ORIGINAL_HLC_SATURATION_SOURCE_PINNED__APPLIED_INPUT_NOT_PROVED';
    R.reason=['The byte-identified source clamps finite tmp to [0,20] and [-1,1]^3. ' ...
        'Proving the actual controller-to-plant connection, finite HLC output, ' ...
        'saturation timing, hybrid modes and uncertainty remains mandatory.'];
catch ME
    R.reason=[ME.identifier ' | ' ME.message];
end
end
