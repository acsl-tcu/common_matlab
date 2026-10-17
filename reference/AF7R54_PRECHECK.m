function OUT = AF7R54_PRECHECK()
clc;
fprintf('\n====================================================================================================\n');
fprintf(' AF7R54 | THEOREM-FIRST PRECHECK\n');
fprintf('====================================================================================================\n');
src=which('REPLANNING_SYSTEM');assert(~isempty(src)&&isfile(src));
expected='223d34c49d7148b8ea14ff4ab9c420d409560c0650ac19e9b0f99318887090c9';
h=sha256_file(src);
C=struct();
C.exact_r50c=strcmp(h,expected);
C.af7r36_mex=(exist('af7r36_single_core_mex','file')==3);
C.snapshot=isfile(fullfile(fileparts(src),'AF7R50_THEORY_SNAPSHOT.mat'));
C.helpers=(exist('af7r49_replay_context','file')==2)&& ...
          (exist('af7r34_make_core_packet','file')==2)&& ...
          (exist('af7r33_flat_jet_batch_model','file')==2);
closed=all(structfun(@logical,C));
F=fieldnames(C);
for k=1:numel(F)
    fprintf(' %-38s = %d\n',strrep(F{k},'_',' '),logical(C.(F{k})));
end
fprintf(' source SHA-256                         = %s\n',h);
fprintf(' precheck closed                        = %d\n',closed);
fprintf(' production modifications in AF7R54     = 0\n');
fprintf('====================================================================================================\n');
assert(closed,'[AF7R54] precheck failed.');
OUT=struct('schema','AF7R54_PRECHECK_V1','closed',closed,'checks',C);
assignin('base','AF7R54_PRECHECK_OUT',OUT);
end
function h=sha256_file(fp)
fid=fopen(fp,'rb');assert(fid>=0);c=onCleanup(@()fclose(fid));
b=fread(fid,Inf,'*uint8');clear c;
md=java.security.MessageDigest.getInstance('SHA-256');md.update(b);
d=typecast(md.digest(),'uint8');h=lower(reshape(dec2hex(d,2).',1,[]));
end
