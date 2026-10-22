function R = AF7R58R27_FAILURE_REPORT()
% Print failure-only roll-lift certificate inputs, preserving the abort.
key='AF7R58R27_ROLL_FAILURE';
assert(isappdata(0,key), ...
    '[R27] No roll-lift failure captured this MATLAB session.');
R=getappdata(0,key);
assert(isstruct(R)&&strcmp(char(R.schema),'AF7R58R27_ROLL_LIFT_FAILURE_V1'));
if R.kmin>R.kmax
    R.classification='NO_PERIODIC_LIFT_INTERSECTS_REACHABLE_SET';
elseif R.kmin<R.kmax
    R.classification='MULTIPLE_PERIODIC_LIFTS_REMAIN_POSSIBLE';
else
    R.classification='NUMERICAL_INCONSISTENCY_CHECK_SOURCE';
end
outdir=fileparts(which(mfilename));
save(fullfile(outdir,'AF7R58R27_ROLL_FAILURE.mat'),'R');
fid=fopen(fullfile(outdir,'AF7R58R27_ROLL_FAILURE.txt'),'w');
assert(fid>=0,'[R27] cannot write diagnostic report.');
cc=onCleanup(@()fclose(fid));
fprintf(fid,'R27 | OBSERVER ROLL 2PI CERTIFICATE FAILURE | DIAGNOSTIC ONLY\n');
fprintf(fid,'classification=%s\n',R.classification);
fields=fieldnames(R);
for k=1:numel(fields)
    f=fields{k}; v=R.(f);
    if islogical(v) || (isnumeric(v)&&isscalar(v))
        fprintf(fid,'%s=%.17g\n',f,double(v));
    elseif ischar(v)
        fprintf(fid,'%s=%s\n',f,v);
    end
end
fprintf('[R27] %s at t=%.9f s (kmin=%g, kmax=%g).\n', ...
    R.classification,R.t_s,R.kmin,R.kmax);
end
