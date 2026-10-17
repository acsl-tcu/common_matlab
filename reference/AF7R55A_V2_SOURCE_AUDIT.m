function OUT = AF7R55A_V2_SOURCE_AUDIT()
clc;
fprintf('\n====================================================================================================\n');
fprintf(' AF7R55A V2 | CHAIN SHADOW SOURCE AUDIT\n');
fprintf('====================================================================================================\n');
fp=which('AF7R55A_V2_IMMUTABLE_CHAIN_SHADOW');
assert(~isempty(fp)&&isfile(fp),'[AF7R55A V2] script unresolved.');
raw=fileread(fp);

C=struct();
C.legacy_t0=contains(raw,'P.t0=double(R.t0);');
C.legacy_t1=contains(raw,'P.t1=double(R.t1);');
C.segment_t0=contains(raw,'P.piecewise_segment_t0=double(R.segment_t0');
C.segment_t1=contains(raw,'P.piecewise_segment_t1=double(R.segment_t1');
C.control_points=contains(raw,'P.piecewise_control_points=R.control_points;');
C.no_production_install=contains(raw,'productionModified=0');
C.chain_mex=(exist('af7r55_chain_mex','file')==3);

closed=all(structfun(@logical,C));
F=fieldnames(C);
for k=1:numel(F)
    fprintf(' %-38s = %d\n',strrep(F{k},'_',' '),logical(C.(F{k})));
end
fprintf(' source audit closed                    = %d\n',closed);
fprintf(' production source modified            = 0\n');
fprintf('====================================================================================================\n');
assert(closed,'[AF7R55A V2] source audit failed.');

OUT=struct('schema','AF7R55A_V2_SOURCE_AUDIT_V1','closed',closed,'checks',C);
assignin('base','AF7R55A_V2_SOURCE_AUDIT_OUT',OUT);
end
