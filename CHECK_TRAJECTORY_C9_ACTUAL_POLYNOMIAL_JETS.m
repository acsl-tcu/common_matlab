function R = CHECK_TRAJECTORY_C9_ACTUAL_POLYNOMIAL_JETS()
% Verify actual C9 control-point jet consistency, continuation C7/yaw C6,
% and the ideal-taut continuous-time tension certificate.
% This is a SYNTHETIC NUMERICAL REGRESSION, NOT a real Flight or HLC proof.
% Always exports MAT/TXT/ZIP under common_matlab; never changes the runner.
root=fileparts(mfilename('fullpath'));
R=struct('status','FAIL_ACTUAL_C9_POLYNOMIAL_JETS', ...
    'stage','INITIALIZE','error_identifier','','error_message','', ...
    'actual_endpoint_evaluated',false, ...
    'claimed_endpoint_tampering_ignored',false, ...
    'cache_tampering_heading_rejected',false, ...
    'cache_tampering_tension_rejected',false, ...
    'polynomial_tampering_rejected',false, ...
    'true_C9_C7_C6_join_pass',false, ...
    'broken_actual_join_rejected',false, ...
    'unmodified_tension_all_time_pass',false, ...
    'normal_heading_status','NOT_EVALUATED', ...
    'true_join_status','NOT_EVALUATED', ...
    'discontinuous_join_status','NOT_EVALUATED', ...
    'actual_metadata_mismatch_reported',false, ...
    'elapsed_ms',NaN,'evaluate_median_ms',NaN, ...
    'evaluate_p95_ms',NaN,'evaluate_max_ms',NaN, ...
    'heading_audit_ms',NaN, ...
    'actual_HLC_tracking_proven',false, ...
    'safe_to_commit',false,'fixture_only',true);
timer=tic;
try
    R.stage='PATH_AND_MODEL';
    assert(isfile(fullfile(root,'run_flight_headless.m')), ...
        'CHECK_C9_JETS:Root', 'Put this CHECK under common_matlab.');
    addpath(fullfile(root,'reference','REPLANNING_SYSTEM', ...
        'TRAJECTORY_GENERATION'),'-begin');

    % Construct exactly representable linear control points: the test time
    % happens to equal the degree 23, so each +0.5 m control step is an
    % exact binary value. This is a test fixture, NOT a production time.
    T=23;
    [P1,P2]=make_test_pieces(T);
    A=struct('cable_length_m',2,'uav_mass_kg',0.762, ...
        'payload_mass_kg',0.14,'gravity_world_mps2',[0;0;-9.81], ...
        'thrust_limits_N',[0;20],'minimum_tension_N',0, ...
        'acceleration_guard_mps2',0);
    O=struct('max_subdivision_depth',4,'max_node_count',31, ...
        'horizontal_velocity_guard_mps',0,'w_guard_mps2',0, ...
        'payload_jet_tolerance',zeros(3,10), ...
        'uav_jet_tolerance',zeros(3,8), ...
        'yaw_jet_tolerance',zeros(1,7));

    R.stage='RECONSTRUCT_REAL_BERNSTEIN_ENDPOINTS';
    actual=TRAJECTORY_C9_FLAT_BERNSTEIN.actual_derivative_controls(P1);
    left=zeros(3,10);right=zeros(3,10);
    for k=1:10
        left(:,k)=actual{k}(:,1);right(:,k)=actual{k}(:,end);
    end
    e0=TRAJECTORY_C9_FLAT_BERNSTEIN.evaluate(P1,P1.t0_s);
    e1=TRAJECTORY_C9_FLAT_BERNSTEIN.evaluate(P1,P1.t0_s+P1.duration_s);
    R.actual_endpoint_evaluated=isequal(e0,left)&&isequal(e1,right)&& ...
        isequal(left,P1.start_jets)&&isequal(right,P1.end_jets);

    R.stage='STOP_TRUSTING_ENDPOINT_METADATA';
    lied=P1;lied.start_jets(1,1)=lied.start_jets(1,1)+100;
    lied.end_jets(1,1)=lied.end_jets(1,1)-100;
    R.claimed_endpoint_tampering_ignored= ...
        isequal(TRAJECTORY_C9_FLAT_BERNSTEIN.evaluate(lied,lied.t0_s),e0) && ...
        isequal(TRAJECTORY_C9_FLAT_BERNSTEIN.evaluate( ...
            lied,lied.t0_s+lied.duration_s),e1);
    caught=TRAJECTORY_C9_UAV_C7_YAW_C6_AUDIT(lied,A,O);
    R.actual_metadata_mismatch_reported= ...
        caught.actual_start_metadata_disagreement_by_order(1)>50 && ...
        caught.actual_end_metadata_disagreement_by_order(1)>50;

    R.stage='HEADING_AND_UNMODIFIED_TENSION';
    H=TRAJECTORY_C9_UAV_C7_YAW_C6_AUDIT(P1,A,O);
    R.normal_heading_status=H.status;
    R.heading_audit_ms=H.elapsed_ms;
    P=TRAJECTORY_TAUT_FLATNESS.certify_positive_tension( ...
        P1,A,struct('max_subdivision_depth',4));
    R.unmodified_tension_all_time_pass=P.certified;

    R.stage='REJECT_FALSE_DERIVATIVE_CACHE';
    bad=P1;bad.derivative_controls{3}(1,7)= ...
        bad.derivative_controls{3}(1,7)+1;
    X=TRAJECTORY_C9_UAV_C7_YAW_C6_AUDIT(bad,A,O);
    R.cache_tampering_heading_rejected=~X.certified && ...
        strcmp(X.status,'UNKNOWN_C9_DERIVATIVE_CACHE_NOT_BOUND');
    try
        TRAJECTORY_TAUT_FLATNESS.certify_positive_tension( ...
            bad,A,struct('max_subdivision_depth',4));
    catch ME
        R.cache_tampering_tension_rejected= ...
            strcmp(ME.identifier,'TRAJECTORY_TAUT_FLATNESS:UnboundJets');
    end
    altered=P1;altered.control_points_m(1,5)= ...
        altered.control_points_m(1,5)+1;
    Y=TRAJECTORY_C9_UAV_C7_YAW_C6_AUDIT(altered,A,O);
    R.polynomial_tampering_rejected=~Y.certified && ...
        strcmp(Y.status,'UNKNOWN_C9_DERIVATIVE_CACHE_NOT_BOUND');

    R.stage='CERTIFY_ACTUAL_REPLAN_JOIN';
    Oj=O;Oj.previous_piece=P1;
    join=TRAJECTORY_C9_UAV_C7_YAW_C6_AUDIT(P2,A,Oj);
    R.true_join_status=join.status;
    R.true_C9_C7_C6_join_pass=H.certified&&join.certified && ...
        join.strict_C9_load_join&&join.strict_C7_uav_join&& ...
        join.strict_C6_yaw_join;

    R.stage='REJECT_TRULY_BROKEN_JOIN';
    shift=P2;shift.control_points_m(1,:)=shift.control_points_m(1,:)+1;
    shift.derivative_controls= ...
        TRAJECTORY_C9_FLAT_BERNSTEIN.actual_derivative_controls(shift);
    shift.start_jets(1,1)=shift.start_jets(1,1)+1;
    shift.end_jets(1,1)=shift.end_jets(1,1)+1;
    broken=TRAJECTORY_C9_UAV_C7_YAW_C6_AUDIT(shift,A,Oj);
    R.discontinuous_join_status=broken.status;
    R.broken_actual_join_rejected=~broken.certified && ...
        strcmp(broken.status,'UNKNOWN_REPLAN_C7_OR_YAW_C6_JOIN');

    R.stage='MEASURE_ACTUAL_POLYNOMIAL_EVALUATION';
    k=25;dt_ms=zeros(1,k);
    for bi=1:k
        clk=tic;
        TRAJECTORY_C9_FLAT_BERNSTEIN.evaluate(P1,P1.t0_s+T/2);
        dt_ms(bi)=1000*toc(clk);
    end
    R.evaluate_median_ms=median(dt_ms);
    sorted=sort(dt_ms);
    R.evaluate_p95_ms=sorted(ceil(0.95*k));
    R.evaluate_max_ms=max(dt_ms);

    checks=[R.actual_endpoint_evaluated, ...
        R.claimed_endpoint_tampering_ignored, ...
        R.cache_tampering_heading_rejected, ...
        R.cache_tampering_tension_rejected, ...
        R.polynomial_tampering_rejected, ...
        R.true_C9_C7_C6_join_pass, ...
        R.broken_actual_join_rejected, ...
        R.unmodified_tension_all_time_pass, ...
        R.actual_metadata_mismatch_reported];
    if all(checks)
        R.status='PASS_ACTUAL_C9_POLYNOMIAL_C7_C6_JOINS__HLC_PENDING';
    end
catch ME
    R.error_identifier=ME.identifier;
    R.error_message=ME.message;
end
R.elapsed_ms=1000*toc(timer);
output_mat=fullfile(root,'TRAJECTORY_C9_ACTUAL_POLYNOMIAL_JETS_RESULT.mat');
output_txt=fullfile(root,'TRAJECTORY_C9_ACTUAL_POLYNOMIAL_JETS_RESULT.txt');
output_zip=fullfile(root,'TRAJECTORY_C9_ACTUAL_POLYNOMIAL_JETS_RESULT_FOR_CHATGPT.zip');
save(output_mat,'R');
fid=fopen(output_txt,'w');
if fid>=0
    cleanup=onCleanup(@()fclose(fid));
    fields=fieldnames(R);
    for k=1:numel(fields)
        value=R.(fields{k});
        if ischar(value)||isstring(value)
            fprintf(fid,'%s=%s\n',fields{k},char(value));
        elseif isnumeric(value)||islogical(value)
            fprintf(fid,'%s=%s\n',fields{k},mat2str(value,16));
        end
    end
    clear cleanup
end
if isfile(output_zip),delete(output_zip);end
zip(output_zip,{output_mat,output_txt});
fprintf('[C9 ACTUAL JETS] %s | %s\n',R.status,output_zip);
end

function [A,B]=make_test_pieces(T)
V=[0.5;0;0];
Ja=zeros(3,10);Jb=zeros(3,10);Jc=zeros(3,10);
Ja(:,1)=[0;0;3];Ja(:,2)=V;
Jb(:,1)=[0.5*T;0;3];Jb(:,2)=V;
Jc(:,1)=[T;0;3];Jc(:,2)=V;
F1=zeros(3,4);F2=zeros(3,4);
for i=1:4
    F1(:,i)=[0.5*(9+i);0;3];
    F2(:,i)=[0.5*(23+9+i);0;3];
end
A=TRAJECTORY_C9_FLAT_BERNSTEIN.build(0,T,Ja,Jb,F1);
B=TRAJECTORY_C9_FLAT_BERNSTEIN.build(T,T,Jb,Jc,F2);
end
