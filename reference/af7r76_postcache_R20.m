function [P0,A2,valid,fastUsed] = af7r76_postcache_R20(replanner,tvec,model)
% AF7R58ZZY-R20 contiguous-grid Stage2 postcache evaluator.
% Exact nominal base comes from existing retained dt-grid tables. Correction
% groups are accumulated directly into that base by one strict-FP MEX call.
% Falls back to the R19 authority whenever the exact contiguous-table gate is open.

assert(isa(replanner,'REPLANNING_SYSTEM'), '[R20] invalid replanner authority.');
assert(isstruct(model) && isfield(model,'certified') && logical(model.certified) && ...
    isfield(model,'schema') && strcmp(char(model.schema),'AF7R33_BATCH_FLAT_MODEL_V1') && ...
    isfield(model,'flight_reference_time_origin') && ...
    isfield(model,'corrections') && isstruct(model.corrections), ...
    '[R20] invalid canonical batch model.');
assert(exist('af7r76_apply_correction_mex','file')==3, ...
    '[R20] direct correction MEX unavailable.');

tvec=double(tvec(:).');
N=numel(tvec);
assert(N>=1 && all(isfinite(tvec)), '[R20] invalid query times.');
fastUsed=false;

if ~(isprop(replanner,'r19_nominal_accel_table_valid') && ...
        logical(replanner.r19_nominal_accel_table_valid) && ...
        replanner.nominal_table_valid && ...
        ~isempty(replanner.nominal_table_pL) && ...
        ~isempty(replanner.r19_nominal_accel_table))
    [P0,A2,valid,~]=af7r75_posaccel_batch_R19(replanner,tvec,model);
    return;
end

dt=double(replanner.nominal_table_dt);
t0=double(replanner.nominal_table_t0);
if ~(isfinite(dt)&&dt>0&&isfinite(t0) && ...
        abs(dt-double(replanner.prediction_dt))<=1e-12 && ...
        isequal(double(model.flight_reference_time_origin),t0))
    [P0,A2,valid,~]=af7r75_posaccel_batch_R19(replanner,tvec,model);
    return;
end

k0=round((tvec(1)-t0)/dt)+1;
k1=k0+N-1;
if k0<1 || k1>size(replanner.nominal_table_pL,2) || ...
        k1>size(replanner.r19_nominal_accel_table,2)
    [P0,A2,valid,~]=af7r75_posaccel_batch_R19(replanner,tvec,model);
    return;
end

% Contiguous-grid proof: only first/last points need checking because both
% sequences use the identical dt and the Stage2 query is contiguous.
tg0=t0+(k0-1)*dt;
tg1=t0+(k1-1)*dt;
tol=1e-10*max(1,max(abs(tvec)));
if abs(tg0-tvec(1))>tol || abs(tg1-tvec(end))>tol
    [P0,A2,valid,~]=af7r75_posaccel_batch_R19(replanner,tvec,model);
    return;
end

idx=k0:k1;
Pbase=double(replanner.nominal_table_pL(:,idx));
Abase=double(replanner.r19_nominal_accel_table(:,idx));
if ~(isequal(size(Pbase),[3 N]) && isequal(size(Abase),[3 N]) && ...
        all(isfinite(Pbase),'all') && all(isfinite(Abase),'all'))
    [P0,A2,valid,~]=af7r75_posaccel_batch_R19(replanner,tvec,model);
    return;
end

[P0,A2,valid]=af7r76_apply_correction_mex(Pbase,Abase,tvec,model.corrections);
assert(isequal(size(P0),[3 N]) && isequal(size(A2),[3 N]) && ...
    numel(valid)==N && all(logical(valid(:))) && ...
    all(isfinite(P0),'all') && all(isfinite(A2),'all'), ...
    '[R20] direct correction evaluator produced invalid output.');
fastUsed=true;
end
