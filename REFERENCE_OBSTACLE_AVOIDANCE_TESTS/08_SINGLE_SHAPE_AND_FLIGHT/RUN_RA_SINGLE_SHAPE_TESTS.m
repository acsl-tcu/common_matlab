function R=RUN_RA_SINGLE_SHAPE_TESTS()
%RUN_RA_SINGLE_SHAPE_TESTS No unzip. Call after SETUP...
t=tic;R=struct('status','NOT_RUN','errorText','', ...
    'result',struct(),'resultsZip','');
try
    R.result=TEST_RA_SINGLE_SHAPE_HANDOFF();
    R.stream=TEST_RA_SINGLE_SENSOR_STREAM();
    R.status='PASS_SINGLE_SHAPE_AND_SENSOR_STREAM';
catch ME
    R.status='FAIL';R.errorText=[ME.identifier ' ' ME.message];
    fprintf(2,'[RA] %s\n',getReport(ME,'extended','hyperlinks','off'));
end
R.elapsedMs=1000*toc(t);
outdir=fullfile(fileparts(fileparts(mfilename('fullpath'))),'RESULTS');
if ~isfolder(outdir),mkdir(outdir);end
stamp=datestr(now,'yyyymmdd_HHMMSS_FFF');
matfile=fullfile(outdir,['RA_SINGLE_' stamp '.mat']);
zipfile=fullfile(outdir,['RA_SINGLE_' stamp '.zip']);
R.resultsZip=zipfile;save(matfile,'R');zip(zipfile,matfile);
end
