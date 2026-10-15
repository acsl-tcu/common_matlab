function OUT = AF7R21A_R9_LOCAL_VECTOR_SOURCE_AUDIT()
% AF7R21A_R9_LOCAL_VECTOR_SOURCE_AUDIT
% Read-only audit of R9 source integration.

clc;
fprintf('\n');
fprintf('====================================================================================================\n');
fprintf(' AF7R21A | R9 LOCAL VECTOR SPEED SOURCE AUDIT\n');
fprintf('====================================================================================================\n');
fprintf('[AUTHORITY] readOnly=1 | sampledAcceptance=0 | caseIndependent=1\n');

src=which('REPLANNING_SYSTEM');
assert(~isempty(src)&&isfile(src),'[AF7R21A] REPLANNING_SYSTEM unresolved.');
raw=fileread(src);
c=compact21(raw);

A=struct();
A.r9_header=contains(raw,'AF7R21-R9');
A.r8_piecewise_retained=contains(raw,'PIECEWISE_DEGREE15_C7_WITH_AFFINE_UNSAFE_COMPONENTS');
A.stage_a_retained=contains(raw,'tighten_piecewise_derivative_bounds_for_speed');
A.stage_b_present=contains(raw,'build_piecewise_local_vector_speed_certificate');
A.vector_coupled_velocity=contains(raw,'Vctrl=C1+v0');
A.base_acceleration_local_enclosure=contains(raw,'double(A0)*h');
A.specific_force_local_lower_bound=contains(raw,'lf.sigma_lb=norm(sMid)-lf.jl_ub*h');
A.suspended_qdot_bound=contains(raw,'lf.qdot_ub=lf.jl_ub/lf.sigma_lb');
A.uav_speed_bound=contains(raw,'lf.vq_ub=lf.vl_ub+double(L)*lf.qdot_ub');
A.midpoint_reject_only=contains(raw,'''midpoint_rejection_witness_only'',true');
A.no_fixed_depth=contains(raw,'''fixed_depth_cap'',false');
A.representability_fail_closed=contains(raw,'BINARY64_TIME_SUBDIVISION_REPRESENTABILITY_OPEN');
A.sampled_acceptance_true=contains(c,'''sampled_acceptance'',true');
A.outward_false_explicit=contains(raw,'''ieee754_outward_closed'',false');
A.local_cert_in_schema=contains(raw,'''local_vector_speed_certificate'',struct()');
A.stage_b_source=contains(raw,'R9_LOCAL_VECTOR_COUPLED_ADAPTIVE_BERNSTEIN_FLATNESS_SPEED');

i0=strfind(raw,'function [SC,CERT] = build_piecewise_local_vector_speed_certificate');
i1=strfind(raw,'function leaves = initialize_piecewise_derivative_bernstein_leaves');
caseConstants=false;
if numel(i0)==1&&numel(i1)==1&&i1>i0
    block=raw(i0:i1-1);
    forbidden={'3.105590062','13.963699','2.932184','2.3235', ...
        'sensor_range=15','horizon=4','1.5000'};
    caseConstants=any(cellfun(@(s)contains(block,s),forbidden));
end
A.case_constants_hardcoded=caseConstants;

closed=A.r9_header&&A.r8_piecewise_retained&&A.stage_a_retained&& ...
    A.stage_b_present&&A.vector_coupled_velocity&& ...
    A.base_acceleration_local_enclosure&&A.specific_force_local_lower_bound&& ...
    A.suspended_qdot_bound&&A.uav_speed_bound&&A.midpoint_reject_only&& ...
    A.no_fixed_depth&&A.representability_fail_closed&& ...
    ~A.sampled_acceptance_true&&A.outward_false_explicit&& ...
    A.local_cert_in_schema&&A.stage_b_source&&~A.case_constants_hardcoded;

fprintf(' source                                  = %s\n',src);
fprintf(' R9 header                               = %d\n',A.r9_header);
fprintf(' R8 piecewise towing retained            = %d\n',A.r8_piecewise_retained);
fprintf(' Stage-A global theorem retained         = %d\n',A.stage_a_retained);
fprintf(' Stage-B local vector theorem present    = %d\n',A.stage_b_present);
fprintf(' vector-coupled payload velocity         = %d\n',A.vector_coupled_velocity);
fprintf(' base acceleration local enclosure       = %d\n',A.base_acceleration_local_enclosure);
fprintf(' local specific-force lower bound        = %d\n',A.specific_force_local_lower_bound);
fprintf(' suspended-load qdot bound               = %d\n',A.suspended_qdot_bound);
fprintf(' UAV speed bound                         = %d\n',A.uav_speed_bound);
fprintf(' midpoint rejection-only policy          = %d\n',A.midpoint_reject_only);
fprintf(' no fixed subdivision depth              = %d\n',A.no_fixed_depth);
fprintf(' representability fail-closed            = %d\n',A.representability_fail_closed);
fprintf(' sampled acceptance introduced           = %d\n',A.sampled_acceptance_true);
fprintf(' IEEE754 outward remains explicitly open = %d\n',A.outward_false_explicit);
fprintf(' R9 certificate retained in proposal     = %d\n',A.local_cert_in_schema);
fprintf(' current-case constants hardcoded        = %d\n',A.case_constants_hardcoded);
fprintf(' R9 source closed                        = %d\n',closed);
fprintf('====================================================================================================\n');

OUT=struct('schema','AF7R21A_R9_LOCAL_VECTOR_SOURCE_AUDIT_V1', ...
    'source',src,'checks',A,'closed',closed);
assignin('base','AF7R21A_R9_LOCAL_VECTOR_SOURCE_AUDIT_OUT',OUT);
end


function s=compact21(raw)
raw=strrep(raw,sprintf('\r\n'),sprintf('\n'));
raw=strrep(raw,sprintf('\r'),sprintf('\n'));
L=splitlines(string(raw));
K=strings(0,1);
for i=1:numel(L)
    z=char(L(i));
    p=find(z=='%',1,'first');
    if ~isempty(p), z=z(1:p-1); end
    z=strrep(z,'...','');
    z=regexprep(z,'\s+','');
    if ~isempty(z), K(end+1,1)=string(z); end %#ok<AGROW>
end
s=char(strjoin(K,''));
end
