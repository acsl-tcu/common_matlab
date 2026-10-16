function OUT = AF7R38_REPAIR_V2()
clc;
fprintf('\n====================================================================================================\n');
fprintf(' AF7R38 V2 | RESTORE CLEAN R14D + REINSTALL R38 SAFELY\n');
fprintf('====================================================================================================\n');

src=which('REPLANNING_SYSTEM');
if isempty(src)
    c=dir(fullfile(pwd,'REPLANNING_SYSTEM.m'));
    assert(~isempty(c),'[AF7R38V2] REPLANNING_SYSTEM.m unresolved. cd to the reference folder first.');
    src=fullfile(c(1).folder,c(1).name);
end
root=fileparts(src);

D=dir(fullfile(root,'REPLANNING_SYSTEM_pre_R38_*.m'));
assert(~isempty(D),'[AF7R38V2] REPLANNING_SYSTEM_pre_R38_*.m backup missing.');
[~,ix]=max([D.datenum]);
cleanBak=fullfile(D(ix).folder,D(ix).name);
clean=fileread(cleanBak);
assert(contains(clean,'AF7R37-R14D-PRODUCTION'), ...
    '[AF7R38V2] newest pre-R38 backup is not R14D authority.');
assert(~contains(clean,'AF7R38-PREFLIGHT-POLICY-HOTPATH'), ...
    '[AF7R38V2] newest pre-R38 backup is not clean.');

stamp=datestr(now,'yyyymmdd_HHMMSS');
brokenCopy=fullfile(root,['REPLANNING_SYSTEM_broken_R38_' stamp '.m']);
[ok,msg]=copyfile(src,brokenCopy,'f');
assert(ok,'[AF7R38V2] could not preserve broken source: %s',msg);

[ok,msg]=copyfile(cleanBak,src,'f');
assert(ok,'[AF7R38V2] could not restore clean R14D source: %s',msg);
rehash;

I=AF7R38_INSTALL_PREFLIGHT_AND_POLICY_HOTPATH_V2();
A=AF7R38_INSTALL_10S_ATTRIBUTION_V2();

OUT=struct('schema','AF7R38_REPAIR_V2', ...
    'repaired',true,'clean_backup',cleanBak, ...
    'broken_preserved',brokenCopy,'installer',I,'attribution',A);
assignin('base','AF7R38_REPAIR_V2_OUT',OUT);

fprintf(' clean R14D restored from                = %s\n',cleanBak);
fprintf(' broken R38 preserved as                 = %s\n',brokenCopy);
fprintf(' corrected R38 installed                 = 1\n');
fprintf(' 10s commit attribution installed        = 1\n');
fprintf('====================================================================================================\n');
end
