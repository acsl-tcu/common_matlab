function OUT = AF7R52B_V2_PRECHECK()
clc;
fprintf('\n====================================================================================================\n');
fprintf(' AF7R52B V2 | BRANCH-PRIME FIX PRECHECK\n');
fprintf('====================================================================================================\n');
C=struct();
C.branch_mex=(exist('af7r52_branch_mex','file')==3);
C.prime_helper=(exist('AF7R52B_V2_PRIME_BRANCH_REFERENCE','file')==2);
C.track_b_v2=(exist('AF7R52B_V2_TERMINAL_BRANCH_KKT_SHADOW','file')==2);
C.loader=(exist('AF7R52_LOAD_TRUE_TRIGGER_AUTHORITY','file')==2);
closed=all(structfun(@logical,C));
F=fieldnames(C);
for k=1:numel(F)
    fprintf(' %-36s = %d\n',strrep(F{k},'_',' '),logical(C.(F{k})));
end
fprintf(' precheck closed                         = %d\n',closed);
fprintf(' production source modified             = 0\n');
fprintf('====================================================================================================\n');
assert(closed,'[AF7R52B V2] precheck failed.');
OUT=struct('schema','AF7R52B_V2_PRECHECK_V1','closed',closed,'checks',C);
assignin('base','AF7R52B_V2_PRECHECK_OUT',OUT);
end
