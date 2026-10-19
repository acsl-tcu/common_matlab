function [ok,Dmin,nextAllowed,reason] = af7r58i_commit_spacing_guard(replanner,t_commit)
% AF7R58I explicit positive successive-commit spacing.
assert(isa(replanner,'REPLANNING_SYSTEM'));
ok=false; Dmin=NaN; nextAllowed=NaN; reason='';
t_commit=double(t_commit);
dt=double(replanner.prediction_dt);
tf0=double(replanner.flight_reference_time_origin);
tend=double(replanner.nominal_table_tend);

if ~(all(isfinite([t_commit dt tf0 tend]))&&dt>0&&tend>tf0)
    reason='AF7R58I_COMMIT_TIME_AUTHORITY_OPEN';
    return;
end

M=max([1 abs(tf0) abs(tend) abs(dt)]);
tol=64*eps(M);
loss=tol+4*eps(M);
d0=dt-loss;
if ~(isfinite(d0)&&d0>0)
    reason='AF7R58I_DCOMMIT_LOWER_BOUND_OPEN';
    return;
end
Dmin=af7r58i_nextdown_positive(d0);
if ~(isfinite(Dmin)&&Dmin>0)
    reason='AF7R58I_DCOMMIT_LOWER_BOUND_OPEN';
    return;
end

last=double(replanner.last_replan_commit_time);
if isnan(last)
    ok=true;
    reason='FIRST_COMMIT';
    return;
end
if ~isfinite(last)
    reason='AF7R58I_LAST_COMMIT_TIME_INVALID';
    return;
end

nextAllowed=(last+dt)-tol;
if ~isfinite(nextAllowed)
    reason='AF7R58I_NEXT_COMMIT_TIME_OVERFLOW';
    return;
end
if t_commit<nextAllowed
    reason='AF7R58I_COMMIT_SPACING_GUARD';
    return;
end

ok=true;
reason='COMMIT_SPACING_CLOSED';
end