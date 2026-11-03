function R=CHECK_TRIGGER_STABLE_ID_EVENTS()
% Independent oracle against MATLAB stable set semantics; no Flight data.
% Tests every set orientation and duplicates; no source/HLC/MEX mutations.
old_rng=rng;
restore=onCleanup(@()rng(old_rng)); %#ok<NASGU>
rng(271828,'twister');
case_count=0;
for attempt=1:6000
    h=randi([1 16],1,randi([0 12]));
    a=randi([1 16],1,randi([0 12]));
    ids=randi([1 16],1,randi([0 14]));
    if mod(attempt,2)==0,h=h.';end
    if mod(attempt,3)==0,a=a.';end
    if mod(attempt,5)==0,ids=ids.';end
    [n,d,z]=TRIGGER_STABLE_EVENT_IDS(h,a,ids);
    nr=setdiff(h,a,'stable');
    dr=any(ismember(h,a));
    zr=intersect(union(a,nr,'stable'),ids,'stable');
    nr=reshape(double(nr),1,[]);
    zr=reshape(double(zr),1,[]);
    assert(isequal(n,nr)&&isequal(d,logical(dr))&&isequal(z,zr), ...
        'CHECK_TRIGGER_STABLE_ID_EVENTS:Mismatch', ...
        'Stable-set semantics differ in case %d.',attempt);
    case_count=case_count+1;
end
R=struct('status','PASS_STABLE_ID_EVENTS_6000_ORACLE_CASES__NOT_FLIGHT_SAFETY', ...
    'tested_cases',case_count,'physical_model_modified',false, ...
    'continuous_time_safe',false,'flight_tested',false);
root=fileparts(mfilename('fullpath'));
save(fullfile(root,'TRIGGER_STABLE_ID_EVENTS_RESULT.mat'),'R','-v7');
fid=fopen(fullfile(root,'TRIGGER_STABLE_ID_EVENTS_RESULT.txt'),'wt');
if fid>=0
    c=onCleanup(@()fclose(fid)); %#ok<NASGU>
    fprintf(fid,'%s\ntested_cases=%d\n',R.status,R.tested_cases);
end
end
