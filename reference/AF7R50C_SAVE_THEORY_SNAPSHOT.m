function OUT = AF7R50C_SAVE_THEORY_SNAPSHOT(replanner,B49,R50A)
% Save the current true-trigger theorem data before the clean timing A/B run.
% This is read-only with respect to REPLANNING_SYSTEM authority.
clc;
assert(isa(replanner,'REPLANNING_SYSTEM'));
E=replanner.get_first_trigger_event();
assert(isstruct(E)&&isfield(E,'valid')&&logical(E.valid), ...
    '[AF7R50C snapshot] true trigger event unavailable.');
M=replanner.nominal_batch_model_cache;
assert(isstruct(M)&&isfield(M,'certified')&&logical(M.certified), ...
    '[AF7R50C snapshot] nominal batch model unavailable.');
src=which('REPLANNING_SYSTEM');
fp=fullfile(fileparts(src),'AF7R50_THEORY_SNAPSHOT.mat');
save(fp,'E','M','B49','R50A');
fprintf(' saved theory snapshot                   = %s\n',fp);
OUT=struct('schema','AF7R50C_SAVE_THEORY_SNAPSHOT_V1','file',fp);
assignin('base','AF7R50_THEORY_SNAPSHOT_OUT',OUT);
end
