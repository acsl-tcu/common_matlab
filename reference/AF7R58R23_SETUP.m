function Q=AF7R58R23_SETUP()
% One command for the preconfigured R22->R23 research diagnostic.
% Builds MEX with C++17, proves opcode/source revision, patches only the
% currently resolved headless runner with exact source anchors and backup.
source=which('REPLANNING_SYSTEM');
assert(~isempty(source),'[R23] REPLANNING_SYSTEM not found on path.');
folder=fileparts(source);
cpp=fullfile(folder,'af7r36_single_core_mex.cpp');
assert(isfile(cpp),'[R23] Native MEX source missing.');
clear('af7r36_single_core_mex');
mex('-R2018a','COMPFLAGS=$COMPFLAGS /std:c++17',cpp, ...
    '-outdir',folder,'-output','af7r36_single_core_mex');
rehash;
Q=AF7R58R23_PREFLIGHT();
Q.loop_probe=AF7R58R23_INSTALL_SPIKE_PROBE();
fprintf('AF7R58R23 SETUP COMPLETE | no flag changes needed | NO COMMIT\n');
end
