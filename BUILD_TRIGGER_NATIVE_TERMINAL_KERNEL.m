function R=BUILD_TRIGGER_NATIVE_TERMINAL_KERNEL()
% OFFLINE ONLY. Compile the modified ORIGINAL HLC/plant kernel.  The
% rebuilt binary is NOT trusted until the full numeric parity audit passes.
% Source / binary / parity artifacts are checked separately. No auto-install.
R=struct('status','FAIL_NATIVE_TERMINAL_KERNEL_BUILD', ...
    'reason','','source_path','','mex_path','','source_pin_status','');
try
    base=fileparts(mfilename('fullpath'));
    dir0=fullfile(base,'reference','REPLANNING_SYSTEM', ...
        'FUTURE_COLLISION_TRIGGER');
    src=fullfile(dir0,'TRIGGER_SYMBOLIC_NATIVE_CORE.cpp');
    assert(isfile(src),'Missing production native source file.');
    % Ensure old MEX memory mapping is released before replacing it.
    clear TRIGGER_SYMBOLIC_NATIVE_CORE
    mex('-R2018a','-O','-outdir',dir0, ...
        '-output','TRIGGER_SYMBOLIC_NATIVE_CORE',src);
    rehash;
    TRIGGER_NATIVE_SOURCE_PIN_CACHE('reset');
    P=TRIGGER_NATIVE_MODEL_SOURCE_CHECK(false);
    R.source_path=src;
    R.mex_path=P.native_path;
    R.source_pin_status=P.status;
    assert(P.cpp_source_matches_verified && P.native_exists, ...
        'Source digest mismatch or rebuilt MEX is not visible.');
    R.status='BUILT__REQUIRES_FRESH_HLC_PLANT_PARITY_BEFORE_FLIGHT';
    R.reason=['Build succeeded. Run CHECK_TRIGGER_NATIVE_TERMINAL_PARITY ' ...
        'against a completed existing real Flight logger and agent before ' ...
        'enabling this kernel in the Flight path.'];
catch ME
    R.reason=[ME.identifier ' | ' ME.message];
end
save(fullfile(fileparts(mfilename('fullpath')), ...
    'TRIGGER_NATIVE_TERMINAL_BUILD_RESULT.mat'),'R');
fid=fopen(fullfile(fileparts(mfilename('fullpath')), ...
    'TRIGGER_NATIVE_TERMINAL_BUILD_RESULT.txt'),'w');
if fid>0
    fprintf(fid,'status=%s\nreason=%s\nsource=%s\nmex=%s\n', ...
        R.status,R.reason,R.source_path,R.mex_path);
    fclose(fid);
end
end
