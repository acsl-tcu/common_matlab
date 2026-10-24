function AF7R58R55_SOURCE_CHECK()
root=fileparts(mfilename('fullpath'));
main=fullfile(root,'REPLANNING_SYSTEM.m');
assert(strcmpi(which('REPLANNING_SYSTEM'),main), ...
    'R55:SOURCE','REPLANNING_SYSTEM is shadowed by another directory.');
S=fileread(main);
assert(contains(S,'r55_trigger_ring') && contains(S,'r55_last_mex_split'), ...
    'R55:SOURCE','R55 MATLAB source markers not found.');
cpp=fullfile(root,'af7r36_single_core_mex.cpp');
assert(isfile(cpp) && contains(fileread(cpp),'r55_mex_result_ms'), ...
    'R55:SOURCE','R55 native C++ source marker not found.');
assert(isfile(fullfile(root,'af7r36_core.hpp')), ...
    'R55:SOURCE','The exact native header is missing.');
fprintf('R55 MATLAB/C++ source paths verified: %s\n',root);
assert(exist('af7r36_single_core_mex','file')==3, ...
    'R55:MEX','Rebuild native MEX with mex -R2018a af7r36_single_core_mex.cpp');
fprintf('Compiled MEX found: %s\n',which('af7r36_single_core_mex'));
fprintf('Run normal Flight, then AF7R58R55_COLLECT. The native timing fields prove whether MEX was rebuilt.\n');
end
