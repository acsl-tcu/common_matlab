function R = CHECK_TRAJECTORY_C9_ACTUAL_FLIGHT_STATE(logger_object)
%CHECK_TRAJECTORY_C9_ACTUAL_FLIGHT_STATE  Actual in-memory LOGGER diagnosis.
% No run_flight_headless, HLC, EKF, plant or REPLANNING_SYSTEM edits.
% Usage after the ORIGINAL run_flight_headless script (without clearvars):
%   R = CHECK_TRAJECTORY_C9_ACTUAL_FLIGHT_STATE();
% Can also pass the current in-memory LOGGER explicitly.
% Produces MAT/TXT/ZIP in common_matlab. Never reads an archived Flight MAT.
root=fileparts(mfilename('fullpath'));
assert(isfile(fullfile(root,'run_flight_headless.m')), ...
    'C9_STATE_CHECK:Root','Place CHECK in common_matlab, not the module folder.');
module=fullfile(root,'reference','REPLANNING_SYSTEM','TRAJECTORY_GENERATION');
assert(isfile(fullfile(module,'TRAJECTORY_C9_ACTUAL_FLIGHT_STATE_AUDIT.m')), ...
    'C9_STATE_CHECK:Module','Missing new source file in TRAJECTORY_GENERATION.');
if exist('TRAJECTORY_C9_ACTUAL_FLIGHT_STATE_AUDIT','file')~=2
    addpath(module,'-begin');
end
assert(exist('TRAJECTORY_C9_RECORDED_FLIGHT_HLC_AUDIT','file')==2, ...
    'C9_STATE_CHECK:Trace','Existing Flight HLC audit not on MATLAB path.');
R=struct('status','NOT_RUN','fixture_pass',false, ...
    'runner_changed',false,'saved_Flight_mat_input',false, ...
    'generated_C9_reference_sent',false,'tracking_all_time_proven',false, ...
    'safe_to_commit',false);

%% Synthetic fixture: correct x/xd indices, signed errors, prior-plant alignment.
S=make_fixture();
Q=TRAJECTORY_C9_ACTUAL_FLIGHT_STATE_AUDIT(S);
assert(Q.sample_count==3 && Q.valid_controller_reference_count==3, ...
    'C9_STATE_CHECK:Fixture','Fixture HLC/reference samples missing.');
assert(Q.valid_EKF_count==3 && Q.preceding_postplant_count==3, ...
    'C9_STATE_CHECK:Fixture','EKF/preceding plant alignment missing.');
assert(max(abs(Q.signed_payload_position_error_m(2,:)+0.2))<1e-12 && ...
    max(abs(Q.signed_payload_velocity_error_mps(2,:)+0.15))<1e-12, ...
    'C9_STATE_CHECK:Fixture','Signed tracking axis/index convention incorrect.');
assert(Q.max_HLC_EKF_position_disagreement_m<1e-12, ...
    'C9_STATE_CHECK:Fixture','HLC-consumed state vs EKF identity broken.');
assert(abs(Q.max_EKF_preceding_postplant_position_disagreement_m-0.01)<1e-12, ...
    'C9_STATE_CHECK:Fixture','One-record plant shift is incorrect.');
S.agent(1).estimator.result{4}.state.pL(2)=99;
Qbad=TRAJECTORY_C9_ACTUAL_FLIGHT_STATE_AUDIT(S);
assert(Qbad.max_HLC_EKF_position_disagreement_m>1 && ...
    ~Qbad.safe_to_commit, ...
    'C9_STATE_CHECK:Fixture','Intentional stale EKF trace not detected.');
D=make_fixture();
D.agent=rmfield(D.agent,'plant');
Qmissing=TRAJECTORY_C9_ACTUAL_FLIGHT_STATE_AUDIT(D);
assert(Qmissing.preceding_postplant_count==0 && ...
    strcmp(Qmissing.status, ...
    'CHECKED_HLC_AND_EKF_SAMPLES__PREVIOUS_PLANT_INCOMPLETE'), ...
    'C9_STATE_CHECK:Fixture','Missing plant is not distinguished from PASS.');
% Adversarial: old 18-state audit was wrong by one index and must fail.
wrong_layout=make_fixture();
wrong_layout.agent.controller.result{3}.x(1)=[];
Qwrong=TRAJECTORY_C9_ACTUAL_FLIGHT_STATE_AUDIT(wrong_layout);
assert(Qwrong.valid_controller_reference_count==2 && ...
    Qwrong.invalid_controller_reference_count==1 && ...
    strcmp(Qwrong.status,'UNKNOWN_INCOMPLETE_ACTUAL_FLIGHT_REFERENCE_OR_HLC_STATE'), ...
    'C9_STATE_CHECK:WrongLayout','Outdated 18-state layout was not rejected.');
R.fixture_pass=true;

%% Actual data from the current MATLAB workspace only.
if nargin<1 || isempty(logger_object)
    if evalin('base','exist(''logger'',''var'')')~=1
        error('C9_STATE_CHECK:Logger', ...
            ['No current in-memory logger exists. Run the UNCHANGED ' ...
             'run_flight_headless script, then this CHECK without clearvars.']);
    end
    logger_object=evalin('base','logger');
    R.data_source='CURRENT_BASE_WORKSPACE_LOGGER';
else
    R.data_source='CURRENT_EXPLICIT_IN_MEMORY_LOGGER';
end
if isobject(logger_object)
    D=logger_object.Data;
elseif isstruct(logger_object)&&isscalar(logger_object)&&isfield(logger_object,'Data')
    D=logger_object.Data;
else
    error('C9_STATE_CHECK:Logger','Pass an in-memory Sim LOGGER.');
end
R.audit=TRAJECTORY_C9_ACTUAL_FLIGHT_STATE_AUDIT(D);
R.trace=TRAJECTORY_C9_RECORDED_FLIGHT_HLC_AUDIT(D);
R.flight_records=R.audit.sample_count;
R.trace_records=R.trace.flight_records;
R.preclamp_saturation_count=R.trace.preclamp_saturation_count;
R.max_payload_position_error_m=R.audit.max_position_error_m;
R.max_payload_velocity_error_mps=R.audit.max_velocity_error_mps;
R.max_signed_position_error_xyz_m=R.audit.max_abs_signed_position_error_xyz_m;
R.time_of_max_payload_error_s=R.audit.time_of_max_position_error_s;
R.max_EKF_preceding_postplant_disagreement_m= ...
    R.audit.max_EKF_preceding_postplant_position_disagreement_m;
R.valid_preceding_postplant_count=R.audit.preceding_postplant_count;
R.max_HLC_EKF_disagreement_m=R.audit.max_HLC_EKF_position_disagreement_m;
assert(R.flight_records==R.trace_records, ...
    'C9_STATE_CHECK:Timeline','Independent Flight trace length mismatch.');
if R.audit.valid_controller_reference_count<R.flight_records || ...
        R.trace.missing_or_invalid_record_count>0
    R.status='UNKNOWN_INCOMPLETE_NOMINAL_FLIGHT_DATA__NO_COMMIT';
elseif R.preclamp_saturation_count>0 || R.trace.failed_flight_records>0
    R.status='OBSERVED_NOMINAL_FLIGHT_ACTUATOR_SATURATION__NO_COMMIT';
else
    R.status='CHECKED_NOMINAL_FLIGHT_SAMPLES__HLC_TUBE_UNCERTIFIED';
end
% This historical Flight used timevarying nominal reference, NOT C9.
R.generated_C9_reference_sent=false;
R.tracking_all_time_proven=false;
R.safe_to_commit=false;

base='TRAJECTORY_C9_ACTUAL_FLIGHT_STATE_RESULT';
matname=[base '.mat'];txtname=[base '.txt'];zipname=[base '_FOR_CHATGPT.zip'];
matpath=fullfile(root,matname);
txtpath=fullfile(root,txtname);
zippath=fullfile(root,zipname);
save(matpath,'R','-v7');
fid=fopen(txtpath,'w');
assert(fid>=0,'C9_STATE_CHECK:Write','Cannot write common_matlab result TXT.');
closer=onCleanup(@()fclose(fid));
fprintf(fid,'status=%s\nsource=%s\nfixture_pass=%d\n', ...
    R.status,R.data_source,R.fixture_pass);
fprintf(fid,'runner_changed=0\nsaved_flight_mat_input=0\n');
fprintf(fid,'flight_records=%d\npreclamp_saturation_count=%d\n', ...
    R.flight_records,R.preclamp_saturation_count);
fprintf(fid,'valid_EKF_count=%d\npreceding_postplant_count=%d\n', ...
    R.audit.valid_EKF_count,R.valid_preceding_postplant_count);
fprintf(fid,'max_payload_position_error_m=%.12g\nmax_payload_velocity_error_mps=%.12g\n', ...
    R.max_payload_position_error_m,R.max_payload_velocity_error_mps);
fprintf(fid,'time_of_max_payload_error_s=%.12g\n',R.time_of_max_payload_error_s);
fprintf(fid,'abs_signed_position_error_xyz_m=[%.12g %.12g %.12g]\n', ...
    R.max_signed_position_error_xyz_m);
fprintf(fid,'max_HLC_EKF_position_disagreement_m=%.12g\n', ...
    R.max_HLC_EKF_disagreement_m);
fprintf(fid,'max_EKF_preceding_postplant_disagreement_m=%.12g\n', ...
    R.max_EKF_preceding_postplant_disagreement_m);
fprintf(fid,'max_position_velocity_trapezoid_residual_m=%.12g\n', ...
    R.audit.max_position_velocity_trapezoid_residual_m);
fprintf(fid,'reference_start_end_displacement_m=%.12g\n', ...
    R.audit.actual_reference_start_end_displacement_m);
fprintf(fid,'plant_time_alignment=%s\n',R.audit.plant_time_alignment);
fprintf(fid,'NOTE: prior logged post-plant is an alignment hypothesis; not full-time truth.\n');
fprintf(fid,'generated_C9_reference_sent=0\ncontinuous_time_tube_proven=0\nsafe_to_commit=0\n');
clear closer
if isfile(zippath),delete(zippath);end
zip(zippath,{matname,txtname},root);
zinfo=dir(zippath);
assert(isfile(zippath) && ~isempty(zinfo) && zinfo.bytes>0, ...
    'C9_STATE_CHECK:ZIP','The result ZIP is missing or empty.');
R.export_zip_path=zippath;
fprintf('[C9 ACTUAL FLIGHT] %s | samples=%d | ZIP=%s\n', ...
    R.status,R.flight_records,zippath);
end

function D=make_fixture()
D.t=[0;0.975;1;1.025;1.05];
D.phase=double(['t';'t';'f';'f';'f']);
A=struct();A.controller.result=cell(1,5);
A.reference.result=cell(1,5);
A.estimator.result=cell(1,5);
A.plant.result=cell(1,5);
for k=1:5
    ref=zeros(28,1);ref(1:3)=[0;k;1];ref(5:7)=[0;1.5;0];
    p=ref(1:3)+[0;-0.2;0.1];
    v=ref(5:7)+[0;-0.15;0];
    x=[1;0;0;0;0;0;900+k;p;v;0;0;-1;0;0;0]; % q(4),w(3),pL(3),vL(3),pT(3),wL(3)
    assert(numel(x)==19,'C9_STATE_CHECK:FixtureLayout', ...
        'HLC fixture must model actual 19-entry state.');
    A.controller.result{k}=struct('x',x);
    A.reference.result{k}=struct('state',struct('xd',ref));
    A.estimator.result{k}=struct('state',struct('pL',p,'vL',v,'mL',0.12));
end
for k=2:5
    p=A.estimator.result{k}.state.pL;
    v=A.estimator.result{k}.state.vL;
    A.plant.result{k-1}=struct('state',struct('pL',p+[0.01;0;0],'vL',v));
end
D.agent=A;
end
