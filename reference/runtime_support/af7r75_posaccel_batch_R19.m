function [P0,A2,valid,tableUsed] = af7r75_posaccel_batch_R19(replanner,tvec,model)
% AF7R58ZZY-R19 exact r={0,2} reference evaluator.
% Nominal r=0/r=2 comes from the existing dt-grid tables. Only correction
% evaluation is native. If the exact table gate is open, the R16 scalar
% authority is used unchanged as fail-safe fallback.

assert(isa(replanner,'REPLANNING_SYSTEM'), '[R19] invalid replanner authority.');
assert(isstruct(model) && isfield(model,'certified') && logical(model.certified) && ...
    isfield(model,'schema') && strcmp(char(model.schema),'AF7R33_BATCH_FLAT_MODEL_V1') && ...
    isfield(model,'flight_reference_time_origin') && ...
    isfield(model,'corrections') && isstruct(model.corrections), ...
    '[R19] invalid canonical batch model.');
assert(exist('af7r75_correction_posaccel_mex','file')==3, ...
    '[R19] native correction evaluator unavailable.');

tvec=double(tvec(:).');
N=numel(tvec);
assert(N>=1 && all(isfinite(tvec)), '[R19] invalid query times.');

[P0,A2,tableUsed]=nominal_base_r19(replanner,tvec,N);
if tableUsed
    assert(isequal(double(model.flight_reference_time_origin), ...
        double(replanner.nominal_table_t0)), ...
        '[R19] canonical model/table Flight origin mismatch.');
else
    assert(iscell(replanner.nominal_flat_derivative_fn) && ...
        numel(replanner.nominal_flat_derivative_fn)>=3 && ...
        isa(replanner.nominal_flat_derivative_fn{1},'function_handle') && ...
        isa(replanner.nominal_flat_derivative_fn{3},'function_handle'), ...
        '[R19] nominal flat position/acceleration authority unavailable.');
    [P0,A2,valid]=af7r72_posaccel_core_R16( ...
        replanner.nominal_flat_derivative_fn{1}, ...
        replanner.nominal_flat_derivative_fn{3}, ...
        double(model.flight_reference_time_origin),tvec,model);
    return;
end

[P0,A2,valid]=af7r75_posaccel_core_R19(P0,A2,tvec,model);
end

function [P0,A2,ok]=nominal_base_r19(replanner,tvec,N)
ok=false; P0=[]; A2=[];
if ~(isprop(replanner,'r19_nominal_accel_table_valid') && ...
        logical(replanner.r19_nominal_accel_table_valid) && ...
        replanner.nominal_table_valid && ...
        ~isempty(replanner.nominal_table_pL) && ...
        ~isempty(replanner.r19_nominal_accel_table))
    return;
end

dt=double(replanner.nominal_table_dt);
t0=double(replanner.nominal_table_t0);
assert(isfinite(dt)&&dt>0&&isfinite(t0), '[R19] nominal table timing invalid.');
assert(abs(dt-double(replanner.prediction_dt))<=1e-12, '[R19] nominal table dt is stale.');
idx=round((tvec-t0)/dt)+1;
if any(idx<1) || any(idx>size(replanner.nominal_table_pL,2)) || ...
        any(idx>size(replanner.r19_nominal_accel_table,2))
    return;
end

tgrid=t0+(idx-1)*dt;
tol=1e-10*max(1,max(abs(tvec)));
if any(abs(tgrid-tvec)>tol), return; end

P0=double(replanner.nominal_table_pL(:,idx));
A2=double(replanner.r19_nominal_accel_table(:,idx));
if ~(isequal(size(P0),[3 N]) && isequal(size(A2),[3 N]) && ...
        all(isfinite(P0),'all') && all(isfinite(A2),'all'))
    P0=[]; A2=[]; return;
end
ok=true;
end
