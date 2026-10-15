function OUT = AF7R31A_FLAT_BATCH_EQUIVALENCE_AUDIT(replanner)
clc;
fprintf('\n====================================================================================================\n');
fprintf(' AF7R31A | SOURCE-BOUND FLAT BATCH EQUIVALENCE AUDIT\n');
fprintf('====================================================================================================\n');
fprintf('[AUTHORITY] implementationEquivalence=1 | sampledSafetyAcceptance=0 | productionModification=0\n');
assert(isa(replanner,'REPLANNING_SYSTEM'));
assert(exist('af7r31_nominal_flat_batch_mex','file')==3,'[AF7R31A] flat batch MEX missing.');
src=which('REPLANNING_SYSTEM'); root=fileparts(src);
metaPath=fullfile(root,'AF7R31_FLAT_BATCH_META.mat');
assert(isfile(metaPath),'[AF7R31A] metadata missing.');
Q=load(metaPath,'meta'); meta=Q.meta;
F=replanner.nominal_flat_derivative_fn(1:7);
S=cellfun(@func2str,F,'UniformOutput',false);
signature=sha256_text(strjoin(S,char(10)));
assert(strcmp(signature,meta.signature), ...
    '[AF7R31A] current nominal flat source differs from compiled MEX source.');
tf0=double(replanner.flight_reference_time_origin);
assert(isfinite(tf0),'[AF7R31A] flight reference time origin unavailable.');
g0=double(replanner.nominal_table_t0); g1=double(replanner.nominal_table_tend);
assert(isfinite(g0)&&isfinite(g1)&&g1>g0,'[AF7R31A] nominal table domain unavailable.');
tg=unique([g0,g1,tf0,linspace(g0,g1,257)]);
tau=max(0,double(tg)-tf0);
Ym=af7r31_nominal_flat_batch_mex(tau);
Yr=zeros(21,numel(tau));
for k=1:numel(tau)
    for r=1:7
        z=double(F{r}(tau(k))); z=z(:);
        assert(numel(z)==3&&all(isfinite(z)));
        Yr(3*r-2:3*r,k)=z;
    end
end
D=abs(double(Ym)-Yr);
maxAbs=max(D,[],'all');
scale=max(1,max(abs(Yr),[],'all'));
tol=2048*eps(scale);
closed=isequal(size(Ym),size(Yr))&&all(isfinite(Ym),'all')&&maxAbs<=tol;
tbench=linspace(max(0,g0-tf0),max(0,g1-tf0),2048);
for k=1:5, af7r31_nominal_flat_batch_mex(tbench); end
R=zeros(1,31);
for k=1:numel(R)
    q=tic; af7r31_nominal_flat_batch_mex(tbench); R(k)=1e3*toc(q);
end
fprintf(' deterministic comparison times          = %d\n',numel(tau));
fprintf(' max |MEX-MATLAB|                        = %.17g\n',maxAbs);
fprintf(' equivalence tolerance                   = %.17g\n',tol);
fprintf(' 2048-time batch min/med/max [ms]        = %.6g / %.6g / %.6g\n',min(R),median(R),max(R));
fprintf(' equivalence closed                      = %d\n',closed);
fprintf(' production source modified              = 0\n');
fprintf('====================================================================================================\n');
if closed
    token=struct('schema','AF7R31_FLAT_BATCH_EQUIVALENCE_TOKEN_V1', ...
        'closed',true,'signature',signature,'max_abs_diff',maxAbs,'tol',tol);
    save(fullfile(root,'AF7R31_FLAT_BATCH_EQUIVALENCE_TOKEN.mat'),'token');
end
OUT=struct('schema','AF7R31A_FLAT_BATCH_EQUIVALENCE_AUDIT_V1', ...
    'closed',closed,'signature',signature,'max_abs_diff',maxAbs, ...
    'tolerance',tol,'batch_ms',R);
assignin('base','AF7R31A_FLAT_BATCH_EQUIVALENCE_AUDIT_OUT',OUT);
end

function h=sha256_text(s)
md=java.security.MessageDigest.getInstance('SHA-256');
md.update(uint8(unicode2native(char(s),'UTF-8')));
d=typecast(md.digest(),'uint8');
h=lower(reshape(dec2hex(d,2).',1,[]));
end
