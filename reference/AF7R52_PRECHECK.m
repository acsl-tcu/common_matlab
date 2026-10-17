function OUT = AF7R52_PRECHECK()
clc;
fprintf('\n====================================================================================================\n');
fprintf(' AF7R52 | TWO-TRACK SHADOW PRECHECK\n');
fprintf('====================================================================================================\n');

src=which('REPLANNING_SYSTEM');
assert(~isempty(src)&&isfile(src),'[AF7R52] REPLANNING_SYSTEM unresolved.');
expected='223d34c49d7148b8ea14ff4ab9c420d409560c0650ac19e9b0f99318887090c9';
h=sha256_file(src);
assert(strcmp(h,expected), ...
    '[AF7R52] exact AF7R50C source required. Current SHA-256=%s',h);

C=struct();
C.r50c_hash=strcmp(h,expected);
C.af7r36_mex=(exist('af7r36_single_core_mex','file')==3);
C.prepared_mex=(exist('af7r52_prepared_commit_mex','file')==3);
C.branch_mex=(exist('af7r52_branch_mex','file')==3);
C.replay_helper=(exist('af7r49_replay_context','file')==2);
C.packet_helper=(exist('af7r34_make_core_packet','file')==2);
C.batch_helper=(exist('af7r33_flat_jet_batch_model','file')==2);
closed=all(structfun(@logical,C));

F=fieldnames(C);
for k=1:numel(F)
    fprintf(' %-38s = %d\n',strrep(F{k},'_',' '),logical(C.(F{k})));
end
fprintf(' source SHA-256                         = %s\n',h);
fprintf(' AF7R52 precheck closed                 = %d\n',closed);
fprintf(' production source modified            = 0\n');
fprintf('====================================================================================================\n');
assert(closed,'[AF7R52] precheck failed.');
OUT=struct('schema','AF7R52_PRECHECK_V1','closed',closed,'checks',C,'sha256',h);
assignin('base','AF7R52_PRECHECK_OUT',OUT);
end

function h=sha256_file(fp)
fid=fopen(fp,'rb');assert(fid>=0);c=onCleanup(@()fclose(fid));
b=fread(fid,Inf,'*uint8');clear c;
md=java.security.MessageDigest.getInstance('SHA-256');md.update(b);
d=typecast(md.digest(),'uint8');h=lower(reshape(dec2hex(d,2).',1,[]));
end
