function [P0,A2,valid,tableUsed,T] = af7r74_posaccel_batch_R18(replanner,tvec,model)
% AF7R58ZZY-R18 diagnostic-only exact r={0,2} batch evaluator.
% Same numerical path as R17, with lightweight substage timing.

T=struct('total_ms',0,'input_ms',0,'table_gate_ms',0,'table_index_ms',0, ...
    'table_gridcheck_ms',0,'table_slice_ms',0,'table_validate_ms',0, ...
    'fallback_ms',0,'origin_check_ms',0,'correction_total_ms',0, ...
    'correction',struct(),'residual_ms',0);
totalTic=tic;

tx=tic;
assert(isa(replanner,'REPLANNING_SYSTEM'), '[R18] invalid replanner authority.');
assert(isstruct(model) && isfield(model,'certified') && logical(model.certified) && ...
    isfield(model,'schema') && strcmp(char(model.schema),'AF7R33_BATCH_FLAT_MODEL_V1') && ...
    isfield(model,'flight_reference_time_origin') && ...
    isfield(model,'corrections') && isstruct(model.corrections), ...
    '[R18] invalid canonical batch model.');
assert(exist('af7r71_decasteljau_batch_mex','file')==3, ...
    '[R18] exact de Casteljau MEX unavailable.');
tvec=double(tvec(:).');
N=numel(tvec);
assert(N>=1 && all(isfinite(tvec)), '[R18] invalid query times.');
T.input_ms=1e3*toc(tx);

[P0,A2,tableUsed,TB]=nominal_base_r18(replanner,tvec,N);
T.table_gate_ms=TB.gate_ms;
T.table_index_ms=TB.index_ms;
T.table_gridcheck_ms=TB.gridcheck_ms;
T.table_slice_ms=TB.slice_ms;
T.table_validate_ms=TB.validate_ms;

if tableUsed
    tx=tic;
    assert(isequal(double(model.flight_reference_time_origin), ...
        double(replanner.nominal_table_t0)), ...
        '[R18] canonical model/table Flight origin mismatch.');
    T.origin_check_ms=1e3*toc(tx);
else
    tx=tic;
    assert(iscell(replanner.nominal_flat_derivative_fn) && ...
        numel(replanner.nominal_flat_derivative_fn)>=3 && ...
        isa(replanner.nominal_flat_derivative_fn{1},'function_handle') && ...
        isa(replanner.nominal_flat_derivative_fn{3},'function_handle'), ...
        '[R18] nominal flat position/acceleration authority unavailable.');
    [P0,A2,valid]=af7r72_posaccel_core_R16( ...
        replanner.nominal_flat_derivative_fn{1}, ...
        replanner.nominal_flat_derivative_fn{3}, ...
        double(model.flight_reference_time_origin),tvec,model);
    T.fallback_ms=1e3*toc(tx);
    T.total_ms=1e3*toc(totalTic);
    T.residual_ms=T.total_ms-(T.input_ms+T.table_gate_ms+T.table_index_ms+ ...
        T.table_gridcheck_ms+T.table_slice_ms+T.table_validate_ms+ ...
        T.fallback_ms+T.origin_check_ms);
    return;
end

tx=tic;
[P0,A2,valid,TC]=af7r74_posaccel_core_R18(P0,A2,tvec,model);
T.correction_total_ms=1e3*toc(tx);
T.correction=TC;
T.total_ms=1e3*toc(totalTic);
T.residual_ms=T.total_ms-(T.input_ms+T.table_gate_ms+T.table_index_ms+ ...
    T.table_gridcheck_ms+T.table_slice_ms+T.table_validate_ms+ ...
    T.fallback_ms+T.origin_check_ms+T.correction_total_ms);
end

function [P0,A2,ok,T]=nominal_base_r18(replanner,tvec,N)
T=struct('gate_ms',0,'index_ms',0,'gridcheck_ms',0,'slice_ms',0,'validate_ms',0);
ok=false; P0=[]; A2=[];

tx=tic;
if ~(isprop(replanner,'r17_nominal_accel_table_valid') && ...
        logical(replanner.r17_nominal_accel_table_valid) && ...
        replanner.nominal_table_valid && ...
        ~isempty(replanner.nominal_table_pL) && ...
        ~isempty(replanner.r17_nominal_accel_table))
    T.gate_ms=1e3*toc(tx);
    return;
end
T.gate_ms=1e3*toc(tx);

tx=tic;
dt=double(replanner.nominal_table_dt);
t0=double(replanner.nominal_table_t0);
assert(isfinite(dt)&&dt>0&&isfinite(t0), '[R18] nominal table timing invalid.');
assert(abs(dt-double(replanner.prediction_dt))<=1e-12, ...
    '[R18] nominal table dt is stale.');
idx=round((tvec-t0)/dt)+1;
if any(idx<1) || any(idx>size(replanner.nominal_table_pL,2)) || ...
        any(idx>size(replanner.r17_nominal_accel_table,2))
    T.index_ms=1e3*toc(tx);
    return;
end
T.index_ms=1e3*toc(tx);

tx=tic;
tgrid=t0+(idx-1)*dt;
tol=1e-10*max(1,max(abs(tvec)));
if any(abs(tgrid-tvec)>tol)
    T.gridcheck_ms=1e3*toc(tx);
    return;
end
T.gridcheck_ms=1e3*toc(tx);

tx=tic;
P0=double(replanner.nominal_table_pL(:,idx));
A2=double(replanner.r17_nominal_accel_table(:,idx));
T.slice_ms=1e3*toc(tx);

tx=tic;
if ~(isequal(size(P0),[3 N]) && isequal(size(A2),[3 N]) && ...
        all(isfinite(P0),'all') && all(isfinite(A2),'all'))
    P0=[]; A2=[];
    T.validate_ms=1e3*toc(tx);
    return;
end
T.validate_ms=1e3*toc(tx);
ok=true;
end
