function V=AF7R58R53_SOURCE_CHECK()
% Run before simulation to verify source resolves from this folder.
root=fileparts(mfilename('fullpath'));
expected=fullfile(root,'REPLANNING_SYSTEM.m');
resolved=which('REPLANNING_SYSTEM');
V=struct('expected',expected,'resolved',resolved, ...
    'path_match',strcmpi(expected,resolved),'class_loaded_before_clear',false);
assert(V.path_match,'R53:Path','MATLAB resolves another REPLANNING_SYSTEM: %s',resolved);
txt=fileread(expected);
assert(contains(txt,'r53_native_tangent_shared_count') && ...
    contains(txt,'r53NativeTangent='),'R53:Version', ...
    'Complete R53 main source not found or incomplete.');
V.source_verified=true;
disp('[R53] COMPLETE replanner source selected.');
end
