function AF7R58R57_SOURCE_CHECK()
% Report which code is active. No modification.
p=which('REPLANNING_SYSTEM');
assert(~isempty(p),'R57:MissingSource','REPLANNING_SYSTEM not on path');
myDir=fileparts(mfilename('fullpath'));
assert(strcmpi(fileparts(p),myDir),'R57:ShadowedSource', ...
    'REPLANNING_SYSTEM is shadowed by %s',p);
s=fileread(p);
assert(contains(s,'ENABLE_AF7R57_DETAIL_RECORD = false') && ...
    contains(s,'ENABLE_AF7R58R16_REFERENCE_BRIDGE = 1') && ...
    contains(s,'if obj.ENABLE_AF7R57_DETAIL_RECORD'),'R57:SourceMismatch', ...
    'R57 source signature missing');
fprintf('[R57 SOURCE] path=%s\n',p);
fprintf('[R57 SOURCE] Detailed captures default OFF; R58 production bridge unchanged\n');
fprintf('[R57 SOURCE] R17 research deadline-continuation unchanged; do NOT interpret as WCET PASS\n');
end
