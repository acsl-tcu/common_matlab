function OUT = AF7R31A_FLAT_BATCH_EQUIVALENCE_AUDIT_V3(replanner)
% No MATLAB Coder and no ccode required.

clc;
fprintf('\n====================================================================================================\n');
fprintf(' AF7R31A V3 | VECTOR/SYMBOLIC-COMPONENT FLAT BATCH EQUIVALENCE AUDIT\n');
fprintf('====================================================================================================\n');
fprintf('[AUTHORITY] implementationEquivalence=1 | sampledSafetyAcceptance=0 | productionModification=0\n');

assert(isa(replanner,'REPLANNING_SYSTEM'));
F=replanner.nominal_flat_derivative_fn(1:7);
assert(all(cellfun(@(f)isa(f,'function_handle'),F)));

tf0=double(replanner.flight_reference_time_origin);
g0=double(replanner.nominal_table_t0);
g1=double(replanner.nominal_table_tend);
assert(isfinite(tf0)&&isfinite(g0)&&isfinite(g1)&&g1>g0);

tg=unique([g0,g1,tf0,linspace(g0,g1,257)]);
[Yb,vb,meta]=af7r31_flat_jet_batch_v3(replanner,tg);
assert(all(vb),'[AF7R31A-V3] batch evaluator produced nonfinite values.');

tau=max(0,tg-tf0);
Yr=zeros(21,numel(tau));
for k=1:numel(tau)
    for r=1:7
        z=double(F{r}(tau(k))); z=z(:);
        assert(numel(z)==3&&all(isfinite(z)));
        Yr(3*r-2:3*r,k)=z;
    end
end

D=abs(double(Yb)-Yr);
maxAbs=max(D,[],'all');
scale=max(1,max(abs(Yr),[],'all'));
tol=2048*eps(scale);
closed=isequal(size(Yb),size(Yr))&&maxAbs<=tol;

% Timing comparison is implementation benchmarking only, never safety
% acceptance and never a Windows hard-WCET claim.
tbench=linspace(g0,g1,2048);
for k=1:5, af7r31_flat_jet_batch_v3(replanner,tbench); end
tb=zeros(1,31);
for k=1:numel(tb)
    q=tic; af7r31_flat_jet_batch_v3(replanner,tbench); tb(k)=1e3*toc(q);
end

% Legacy scalar-dispatch cost over the same 2048 times.
ts=zeros(1,5);
for rep=1:numel(ts)
    q=tic;
    tauB=max(0,tbench-tf0);
    dummy=0;
    for k=1:numel(tauB)
        for r=1:7
            z=double(F{r}(tauB(k)));
            dummy=dummy+sum(z(:)); %#ok<NASGU>
        end
    end
    ts(rep)=1e3*toc(q);
end

fprintf(' exact derivative-handle signature       = %s\n',meta.signature);
fprintf(' direct-vector derivative handles        = %d / 7\n',nnz(meta.direct_vector_ok));
fprintf(' symbolic scalar fallback derivatives    = %d / 7\n',meta.scalar_fallback_count);
fprintf(' deterministic comparison times          = %d\n',numel(tg));
fprintf(' max |batch-scalar MATLAB|               = %.17g\n',maxAbs);
fprintf(' equivalence tolerance                   = %.17g\n',tol);
fprintf(' batch 2048 min/med/max [ms]             = %.6g / %.6g / %.6g\n',min(tb),median(tb),max(tb));
fprintf(' legacy scalar 2048 min/med/max [ms]     = %.6g / %.6g / %.6g\n',min(ts),median(ts),max(ts));
fprintf(' equivalence closed                      = %d\n',closed);
fprintf(' MATLAB Coder required                   = 0\n');
fprintf(' ccode required                          = 0\n');
fprintf(' production source modified              = 0\n');
fprintf('====================================================================================================\n');

OUT=struct('schema','AF7R31A_FLAT_BATCH_EQUIVALENCE_AUDIT_V3', ...
    'closed',closed,'signature',meta.signature,'max_abs_diff',maxAbs, ...
    'tolerance',tol,'batch_ms',tb,'legacy_scalar_ms',ts, ...
    'direct_vector_ok',meta.direct_vector_ok);
assignin('base','AF7R31A_FLAT_BATCH_EQUIVALENCE_AUDIT_V3_OUT',OUT);
end
