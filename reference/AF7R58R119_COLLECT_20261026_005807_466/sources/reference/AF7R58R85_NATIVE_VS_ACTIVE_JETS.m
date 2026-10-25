function R = AF7R58R85_NATIVE_VS_ACTIVE_JETS(root,times)
% R85 SOURCE-BOUND READ-ONLY audit of the CURRENT committed reference.
% Compare actual flat_eval_fn 0..6 with the SAME native correction packet's
% full 0..6, canonical MATLAB reconstruction, and native correction jet7.
% Does not modify production, HLC, EKF, plant, trigger, candidates or commit.
% Call ONLY AFTER SimHL; native MEX must be initialized by the actual run.
if nargin<2 || isempty(times)
    error('R85:ExplicitTimesRequired', ...
        'Provide actual postflight query times from the committed version.');
end
times=double(times(:).');
assert(~isempty(times)&&all(isfinite(times)), 'R85:InvalidTimes');
obj=find_replanner_r85(root,5);
assert(~isempty(obj),'R85:ReplannerNotFound');
assert(exist('af7r36_single_core_mex','file')==3,'R85:NativeMEXMissing');
assert(strcmpi(char(obj.active_reference_mode),'REPAIRED'), ...
    'R85:CommittedRepairedReferenceRequired');
Aref=obj.active_repaired_reference;
assert(isstruct(Aref)&&isfield(Aref,'batch_flat_model')&& ...
    isfield(Aref,'flat_eval_fn')&&isa(Aref.flat_eval_fn,'function_handle'), ...
    'R85:ActualActiveSourceMissing');
M=Aref.batch_flat_model;
assert(isstruct(M)&&isfield(M,'corrections')&&isstruct(M.corrections), ...
    'R85:ActualBatchModelMissing');
kind='';
if isfield(Aref,'path_parameterization') && ...
        isstruct(Aref.path_parameterization) && ...
        isfield(Aref.path_parameterization,'kind')
    kind=char(Aref.path_parameterization.kind);
end
identityHold=strcmp(kind,'EXACT_C7_FREE_TERMINAL_OFFSET_HOLD');
if isfield(M,'phase_layers') && ~isempty(M.phase_layers) && ~identityHold
    error('R85:RetimingNotSupported', ...
        'Native op25 identity-time jet cannot certify a retimed reference.');
end
if identityHold,M.phase_layers=struct([]);end
% Crucially, both op2 and op25 consume ONE source-derived packet, never an
% independently fabricated polynomial or a finite-difference jet.
Q=obj.af7r48_native_eval_packet_from_batch_model(M);
assert(isstruct(Q),'R85:NativePacketMissing');
Native06=double(af7r36_single_core_mex(2,Q,times));
NativeCorrection7=double(af7r36_single_core_mex(25,Q,times));
assert(isequal(size(Native06),[21,numel(times)]) && ...
    isequal(size(NativeCorrection7),[3,numel(times)]) && ...
    all(isfinite(Native06(:))) && all(isfinite(NativeCorrection7(:))), ...
    'R85:InvalidNativeJetOutput');
n=numel(times);
rows=repmat(struct('time_s',NaN,'reference_version',NaN, ...
    'r63_valid',false,'r63_error',NaN, ...
    'flat_vs_native_0to6_max_abs',NaN, ...
    'r63_vs_native_0to6_max_abs',NaN, ...
    'flat_y_velocity',NaN,'native_y_velocity',NaN, ...
    'r63_y_velocity',NaN,'nominal_y_velocity',NaN, ...
    'native_full_jet7',nan(3,1), ...
    'status','INCONCLUSIVE'),1,n);
for j=1:n
    t=times(j);
    rows(j).time_s=t;
    rows(j).reference_version=double(obj.active_reference_version);
    X=double(Aref.flat_eval_fn(t));X=X(:);
    assert(numel(X)>=28 && all(isfinite(X(1:28))), ...
        'R85:ActiveFlatEvaluatorInvalid');
    Flat=zeros(3,7);
    N=zeros(3,7);
    for r=0:6
        Flat(:,r+1)=X(4*r+(1:3));
        N(:,r+1)=Native06(3*r+(1:3),j);
    end
    rows(j).flat_vs_native_0to6_max_abs=max(abs(Flat(:)-N(:)));
    rows(j).flat_y_velocity=Flat(2,2);
    rows(j).native_y_velocity=N(2,2);
    Nom=obj.r63_nominal_position_jet_global(t);
    rows(j).nominal_y_velocity=Nom(2,2);
    rows(j).native_full_jet7=Nom(:,8)+NativeCorrection7(:,j);
    try
        A=obj.r63_live_c7_authority(t);
        rows(j).r63_valid=logical(A.valid);
        rows(j).r63_error=double(A.flat_active_jet_consistency_error);
        Z=double(A.active_jets_0_to_7);
        rows(j).r63_y_velocity=Z(2,2);
        rows(j).r63_vs_native_0to6_max_abs=max( ...
            abs(Z(:,1:7)-N),[],'all');
    catch ME
        rows(j).status=['R63_READ_FAILED__' char(ME.identifier)];
        continue;
    end
    % Only diagnose: this does NOT promote a candidate to a safety proof.
    if rows(j).flat_vs_native_0to6_max_abs > ...
            4096*eps(max(1,max(abs([Flat(:);N(:)]))))
        rows(j).status='ACTIVE_FLAT_VS_NATIVE_MISMATCH__STOP_INTEGRATION';
    elseif ~rows(j).r63_valid
        rows(j).status='R63_CANONICAL_RECONSTRUCTION_MISMATCH__NO_R81';
    else
        rows(j).status='JET_SOURCE_AGREES__NOT_EXECUTION_SAFETY';
    end
end
R=struct('status','R85_NATIVE_VS_ACTIVE_SOURCE_AUDIT_ONLY', ...
    'reference_version',double(obj.active_reference_version), ...
    'current_mode',char(obj.active_reference_mode), ...
    'phase_identity_hold',identityHold, ...
    'rows',rows,'production_modified',false, ...
    'replan_trigger_modified',false, ...
    'c7_full_interval_proved',false, ...
    'closed_loop_safety_proved',false);
rootDir=fileparts(mfilename('fullpath'));
R.report_file=fullfile(rootDir,'AF7R58R85_NATIVE_JETS_REPORT.txt');
R.mat_file=fullfile(rootDir,'AF7R58R85_NATIVE_JETS_OUT.mat');
fid=fopen(R.report_file,'wt');assert(fid>0,'R85:CannotWriteReport');
c=onCleanup(@()fclose(fid)); %#ok<NASGU>
fprintf(fid,'R85 SOURCE-BOUND CURRENT ACTIVE NATIVE JET AUDIT (NO COMMIT)\n');
fprintf(fid,'version=%g phase_identity_hold=%d samples=%d\n', ...
    R.reference_version,identityHold,n);
for j=1:n
    a=rows(j);
    fprintf(fid,['t=%.9f ver=%g r63_valid=%d r63_err=%.9g ' ...
        'flat_native_err=%.9g r63_native_err=%.9g ' ...
        'yvel_flat=%.9g yvel_native=%.9g yvel_r63=%.9g yvel_nom=%.9g ' ...
        'status=%s\n'],a.time_s,a.reference_version,a.r63_valid, ...
        a.r63_error,a.flat_vs_native_0to6_max_abs, ...
        a.r63_vs_native_0to6_max_abs, ...
        a.flat_y_velocity,a.native_y_velocity, ...
        a.r63_y_velocity,a.nominal_y_velocity,a.status);
end
fprintf(fid,'NONE OF THESE RESULTS PROVE CONTINUOUS CLOSED-LOOP SAFETY.\n');
save(R.mat_file,'R');
end

function obj=find_replanner_r85(x,depth)
obj=[];
if isa(x,'REPLANNING_SYSTEM') && isscalar(x),obj=x;return;end
if depth<=0||isempty(x)||numel(x)~=1,return;end
if isstruct(x), names=fieldnames(x);
elseif isobject(x)
    try,names=properties(x);catch,return;end
else,return;end
skip={'self','result','parameter','plant','sensor','estimator', ...
    'controller','logger','time','state','parent'};
for k=1:min(numel(names),100)
    if any(strcmpi(names{k},skip)),continue;end
    try
        obj=find_replanner_r85(x.(names{k}),depth-1);
        if ~isempty(obj),return;end
    catch
    end
end
end
