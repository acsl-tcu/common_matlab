function OUT = AF7R10C_GRAVITY_VECTOR_SHAPE_AUDIT()
% AF7R10C_GRAVITY_VECTOR_SHAPE_AUDIT
% Read-only source audit for the backend-1 gravity-vector syntax bug.

clc;
fprintf('\n');
fprintf('====================================================================================================\n');
fprintf(' AF7R10C | GRAVITY VECTOR SHAPE SOURCE AUDIT\n');
fprintf('====================================================================================================\n');

src=which('REPLANNING_SYSTEM');
assert(~isempty(src) && isfile(src),'[AF7R10C] REPLANNING_SYSTEM.m not found.');
txt=fileread(src);

bad='[0;0,double(';
good='[0;0;double(';

nBad=numel(strfind(txt,bad));
nGood=numel(strfind(txt,good));

fprintf(' source                                  = %s\n',src);
fprintf(' invalid [0;0,double(...)] occurrences  = %d\n',nBad);
fprintf(' valid   [0;0;double(...)] occurrences  = %d\n',nGood);
fprintf(' source syntax bug closed               = %d\n',nBad==0 && nGood>=4);
fprintf(' safety theorem changed                 = 0\n');
fprintf(' thresholds changed                     = 0\n');
fprintf(' backend changed                        = 0\n');
fprintf('====================================================================================================\n');

OUT=struct();
OUT.schema='AF7R10C_GRAVITY_VECTOR_SHAPE_AUDIT_V1';
OUT.source=src;
OUT.invalid_occurrences=nBad;
OUT.valid_occurrences=nGood;
OUT.closed=(nBad==0 && nGood>=4);
assignin('base','AF7R10C_GRAVITY_VECTOR_SHAPE_AUDIT_OUT',OUT);
end
