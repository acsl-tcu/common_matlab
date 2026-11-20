function result = generate_CBF_REAL_XY_TORQUE()
% Regenerate dedicated XY-order-4 physical-torque jet (no Symbolic Toolbox).
% Run from any folder; writes GENERATED under this generator folder only.
root=fileparts(mfilename('fullpath'));
src=fullfile(root,'CBF_REAL_XY_TORQUE_JET_SOURCE.m');
destDir=fullfile(root,'GENERATED');
dest=fullfile(destDir,'CBF_REAL_XY_TORQUE_JET.m');
assert(isfile(src),'CBFXY:MissingSource','XY torque source missing.');
txt=fileread(src);
old='CBF_REAL_XY_TORQUE_JET_SOURCE';new='CBF_REAL_XY_TORQUE_JET';
assert(startsWith(strtrim(txt),['function data = ' old '(']), ...
    'CBFXY:SourceFormat','Expected source function signature.');
txt=strrep(txt,old,new);
if ~isfolder(destDir),mkdir(destDir);end
fid=fopen(dest,'w');
if fid<0,error('CBFXY:Write','Cannot write %s',dest);end
cleanup=onCleanup(@()fclose(fid));
fprintf(fid,'%s',txt);
clear cleanup
rehash
result=struct('source',src,'generated',dest,'order',4, ...
    'controlVariables',{{'tauX','tauY'}},'thrustFixed',true, ...
    'yawTorqueFixed',true,'groundConstraints',false);
fprintf('[CBF XY GENERATE] Wrote %s\n',dest);
end
