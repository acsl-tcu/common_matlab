function V=AF7R58R54_SOURCE_CHECK()
root=fileparts(mfilename('fullpath'));
expected=fullfile(root,'REPLANNING_SYSTEM.m');
resolved=which('REPLANNING_SYSTEM');
assert(strcmpi(expected,resolved),'R54:SourcePath', ...
    'Wrong active replanner: %s',resolved);
txt=fileread(expected);
assert(contains(txt,'AF7R54-R1 SOURCE-BOUND THREE-TRACK REVISION') && ...
    contains(txt,'r54_stage_ring') && ...
    contains(txt,'yawAnchorEvalFn=[];'), ...
    'R54:Version','R54 source not installed.');
V=struct('path_match',true,'source',resolved,'source_verified',true);
disp('[R54] REPLANNING_SYSTEM.m SOURCE VERIFIED');
end
