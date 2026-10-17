function OUT = AF7R53_INSTALL_RUN_SWITCH(varargin)
enabled=true;
if nargin>=1, enabled=logical(varargin{1}); end

clc;
fprintf('\n====================================================================================================\n');
fprintf(' AF7R53A | RUN-SCRIPT PREPARED-COMMIT SWITCH\n');
fprintf('====================================================================================================\n');

src=which('run_flight_headless');
assert(~isempty(src)&&isfile(src),'[AF7R53A] run_flight_headless unresolved.');
raw=fileread(src);

topAnchor='ENABLE_AUTO_REPLAN_COMMIT = true;';
optAnchor='replan_opts.ENABLE_AUTO_REPLAN_COMMIT = ENABLE_AUTO_REPLAN_COMMIT;';
assert(numel(strfind(raw,topAnchor))==1,'[AF7R53A] top switch anchor mismatch.');
assert(numel(strfind(raw,optAnchor))==1,'[AF7R53A] opts anchor mismatch.');

raw=regexprep(raw, ...
    '(?m)^[ \t]*ENABLE_AF7R53_PREPARED_COMMIT[ \t]*=[^\r\n]*[\r\n]+','');
raw=regexprep(raw, ...
    '(?m)^[ \t]*replan_opts\.ENABLE_AF7R53_PREPARED_COMMIT[ \t]*=[^\r\n]*[\r\n]+','');

val='false';
if enabled, val='true'; end

raw=strrep(raw,topAnchor, ...
    sprintf('%s\nENABLE_AF7R53_PREPARED_COMMIT = %s; %% AF7R53 explicit A/B switch', ...
        topAnchor,val));
raw=strrep(raw,optAnchor, ...
    sprintf('%s\nreplan_opts.ENABLE_AF7R53_PREPARED_COMMIT = ENABLE_AF7R53_PREPARED_COMMIT;', ...
        optAnchor));

assert(numel(strfind(raw,'ENABLE_AF7R53_PREPARED_COMMIT ='))==2, ...
    '[AF7R53A] generated switch count invalid.');

stamp=datestr(now,'yyyymmdd_HHMMSS');
bak=fullfile(fileparts(src),['run_flight_headless_pre_AF7R53A_' stamp '.m']);
[ok,msg]=copyfile(src,bak,'f');assert(ok,'[AF7R53A] backup failed: %s',msg);
fid=fopen(src,'w');assert(fid>=0);c=onCleanup(@()fclose(fid));
fwrite(fid,raw,'char');clear c;
rehash;

fprintf(' run script                              = %s\n',src);
fprintf(' backup                                  = %s\n',bak);
fprintf(' prepared commit switch                 = %d\n',enabled);
fprintf(' simulation loop modified               = 0\n');
fprintf(' post-run dashboard modified            = 0\n');
fprintf('====================================================================================================\n');

OUT=struct('schema','AF7R53A_RUN_SWITCH_V1','closed',true,'enabled',enabled, ...
    'source',src,'backup',bak);
assignin('base','AF7R53A_RUN_SWITCH_OUT',OUT);
end
