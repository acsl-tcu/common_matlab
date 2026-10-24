function AF7R58R56_SOURCE_CHECK()
% Source presence/binding check; no simulation modifications.
root=fileparts(mfilename('fullpath'));
main=fullfile(root,'REPLANNING_SYSTEM.m');
assert(strcmpi(which('REPLANNING_SYSTEM'),main), ...
    'R56:SourceShadowed','REPLANNING_SYSTEM is loaded from another directory.');
s=fileread(main);
assert(contains(s,'r56_r58_ring') && contains(s,"'xd_active_now',double(xd_active_now(:))"), ...
    'R56:SourceMissing','R56 source marker absent.');
hpp=fullfile(root,'af7r36_core.hpp');
assert(isfile(hpp),'R56:MissingHeader','Native header missing.');
h=fileread(hpp);
assert(contains(h,'r56RowScales') && contains(h,'R56: row-wise scaling/equilibration'), ...
    'R56:HeaderMissing','R56 native code missing.');
assert(isfile(fullfile(root,'af7r36_single_core_mex.cpp')), ...
    'R56:MissingMexSource','MEX source missing.');
assert(exist('af7r36_single_core_mex','file')==3, ...
    'R56:MissingMexBinary','Compile the MEX with C++17 before simulation.');
fprintf('R56 source verified: %s\n',root);
fprintf('C++17 MEX found: %s\n',which('af7r36_single_core_mex'));
fprintf('Please verify MEX was rebuilt; this check cannot infer compiled source hash.\n');
end
