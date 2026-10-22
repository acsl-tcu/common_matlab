function OUT = AF7R58R13_GATE_REPORT(S)
% AF7R58R13_GATE_REPORT
% Read-only explanation of the *new* R58 R13 speed-provenance matrix.
% Run R58 diagnostic shadow once with ENABLE_AF7R58_SHADOW_DIAGNOSTIC=1.
% Then pass either a native PlanResult struct or the resulting planner snapshot:
%   R13 = AF7R58R13_GATE_REPORT(S);
% The diagnostic rows cannot grant acceptance. No sampled acceptance is used.
%
% 38 columns (source order):
%  1 branch_index, 2 last_side, 3 side_depth, 4 segment_count,
%  5 qp_cost, 6 finite_sensing_speed_limit,
%  7 scaled_jet_quant_error, 8 physical_jet_quant_error,
%  9 correction_velocity_Bernstein_UB, 10 accel_UB, 11 jerk_UB,
% 12 A_certified, 13 A_impossible_bound, 14 A_repr_fail,
% 15 A_splits, 16..18 A_derivative_upper, 19..21 A_lower,
% 22 B_attempted, 23 B_result (1=cert,2=finite witness,
%  3=representation,4=final failure),
% 24 B_leaves, 25 B_splits, 26 B_worst_segment, 27 B_worst_time,
% 28 B_worst_VQ_bound, 29 B_finite_speed_witness,
% 30 B_sigma_bound,31 B_qdot_bound,
% 32 A_estimated_VQ_bound,33 A_sigma_bound,
% 34 B_finite_witness_flag,35 B_representation_flag,
% 36..38 pre-lattice raw QP Bernstein speed/acceleration/jerk UBs.
% Any 'impossible' verdict in Stage A pertains to THAT certificate
% bounding expression, not global physical infeasibility.

if nargin ~= 1 || ~isstruct(S) || ~isscalar(S)
    error('AF7R58R13:Input','Pass one scalar struct from a native result or planner snapshot.');
end
if isfield(S,'r58_speed_gate_rows')
    T=double(S.r58_speed_gate_rows);
elseif isfield(S,'staged_replan_predeadline_r58_speed_gate_rows')
    T=double(S.staged_replan_predeadline_r58_speed_gate_rows);
else
    error('AF7R58R13:Missing', ...
        ['R13 diagnostic rows not found. Rebuild all 3 R13 sources, ', ...
        'enable the R58 diagnostic shadow, and rerun a REAL trigger.']);
end
if ~isnumeric(T) || size(T,2)~=38
    error('AF7R58R13:Shape','Expected an N-by-38 R13 native speed diagnostic matrix.');
end
OUT=struct();
OUT.schema='AF7R58R13_READ_ONLY_GATE_REPORT_V1';
OUT.row_count=size(T,1);
OUT.rows=T;
OUT.production_modified=false;
OUT.acceptance_modified=false;
OUT.certificate_modified=false;
OUT.shadow_ran=NaN;
if isfield(S,'r58_shadow_ran'),OUT.shadow_ran=double(S.r58_shadow_ran);end
OUT.finite_speed_witness_count=sum(T(:,34)==1 & isfinite(T(:,29)) & T(:,29)>=T(:,6));
OUT.stageA_impossible_bound_count=sum(T(:,13)==1);
OUT.stageA_representation_count=sum(T(:,14)==1);
OUT.stageB_representation_count=sum(T(:,23)==3);
OUT.stageB_finite_witness_count=sum(T(:,23)==2);
OUT.stageB_unresolved_count=sum(T(:,22)==1 & T(:,23)~=1 & T(:,23)~=2 & T(:,23)~=3);
if isempty(T)
    OUT.classification='NO_R58_GATE_ROWS';
elseif OUT.finite_speed_witness_count>0
    OUT.classification='AT_LEAST_ONE_CERTIFIED_FINITE_SPEED_VIOLATION_OF_THE_SAME_CANDIDATE';
elseif OUT.stageB_representation_count>0
    OUT.classification='STAGE_B_REPRESENTATION_LIMIT_REQUIRES_INSPECTION';
else
    OUT.classification='NO_FINITE_SPEED_WITNESS_CAPTURED__CHECK_STAGE_A_BOUND_AND_B_TRACE';
end
OUT.mean_quantized_physical_error=NaN;
if ~isempty(T),OUT.mean_quantized_physical_error=mean(T(:,8));end
R={};
R{end+1}=sprintf('AF7R58R13 | R58 SPEED-GATE PROVENANCE');
R{end+1}=sprintf('diagnostic_only=1 | acceptance_changed=0 | hard_cert_changed=0');
R{end+1}=sprintf('classification = %s',OUT.classification);
R{end+1}=sprintf('shadow_ran = %g | branch rows = %d',OUT.shadow_ran,OUT.row_count);
R{end+1}=sprintf('finite violation witness count = %d',OUT.finite_speed_witness_count);
R{end+1}=sprintf('Stage A bounding-expression impossible count = %d',OUT.stageA_impossible_bound_count);
R{end+1}=sprintf('Stage A/B representation = %d / %d',OUT.stageA_representation_count,OUT.stageB_representation_count);
for i=1:size(T,1)
    z=T(i,:);
    R{end+1}=sprintf(['branch=%g side=%+g qpCost=%.9g speedLimit=%.9g ', ...
        'quantScaled=%.9g quantPhysical=%.9g'], ...
        z(1),z(2),z(5),z(6),z(7),z(8));
    R{end+1}=sprintf(['   before-lattice rawQP [v,a,j] UBs= [%.9g %.9g %.9g] ', ...
        '| after-lattice UBs=[%.9g %.9g %.9g]'], ...
        z(36),z(37),z(38),z(9),z(10),z(11));
    R{end+1}=sprintf(['   StageA cert/impossibleBound/representation=%g/%g/%g splits=%g'], ...
        z(12),z(13),z(14),z(15));
    R{end+1}=sprintf(['   StageA VQbound=%.9g sigma=%.9g ', ...
        'StageB attempted/code/leaves/splits=%g/%g/%g/%g'], ...
        z(32),z(33),z(22),z(23),z(24),z(25));
    R{end+1}=sprintf(['   StageB worst seg/time/VQ/sigma/qdot = ', ...
        '%g / %.9g / %.9g / %.9g / %.9g | witness=%.9g'], ...
        z(26),z(27),z(28),z(30),z(31),z(29));
end
R{end+1}='WARNING: Bernstein UBs are upper bounds, not measured actual speed.';
R{end+1}='WARNING: finite witness must satisfy the actual candidate speed formula and limit.';
R{end+1}='WARNING: R58 is NON-COMMITTABLE REFERENCE-ONLY SHADOW; full execution safety remains open.';
R{end+1}='WARNING: zero rows with shadow ON may mean early pre-speed failure.';
report=strjoin(R,newline);
fprintf('%s\n',report);
root=fileparts(mfilename('fullpath'));
p=fullfile(root,'AF7R58R13_GATE_REPORT.txt');
fid=fopen(p,'w');
if fid<0,error('AF7R58R13:IO','Cannot write %s',p);end
cleanup=onCleanup(@()fclose(fid));
fprintf(fid,'%s\n',report);
save(fullfile(root,'AF7R58R13_GATE_CAPTURE.mat'),'OUT','-v7.3');
end
