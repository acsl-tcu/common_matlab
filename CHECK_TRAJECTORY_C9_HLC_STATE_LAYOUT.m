function R = CHECK_TRAJECTORY_C9_HLC_STATE_LAYOUT(logger_data)
%CHECK_TRAJECTORY_C9_HLC_STATE_LAYOUT  Standalone source-layout and flight audit.
% No archived MAT inputs and no modifications to runner/HLC/EKF/plant.
% Call after the original run_flight_headless while base logger exists:
%   R = CHECK_TRAJECTORY_C9_HLC_STATE_LAYOUT();
% Or explicitly: R = CHECK_TRAJECTORY_C9_HLC_STATE_LAYOUT(logger);
% No in-memory logger: fixture-only UNKNOWN (not a Flight PASS).
root=fileparts(mfilename('fullpath'));
module=fullfile(root,'reference','REPLANNING_SYSTEM','TRAJECTORY_GENERATION');
assert(isfile(fullfile(module,'TRAJECTORY_C9_HLC_PAYLOAD_STATE.m'))&& ...
       isfile(fullfile(module,'TRAJECTORY_C9_RECORDED_TRACKING_WITNESS.m'))&& ...
       isfile(fullfile(module,'TRAJECTORY_C9_ACTUAL_FLIGHT_STATE_AUDIT.m')), ...
       'C9_LAYOUT_CHECK:Sources','Expected patched source files not found.');
addpath(module,'-begin');
R=struct('status','NOT_RUN','fixture_pass',false,'flight_data_supplied',false, ...
         'flight_records',0,'flight_valid_samples',0, ...
         'max_observed_payload_position_error_m',NaN, ...
         'max_observed_payload_velocity_error_mps',NaN, ...
         'max_HLC_EKF_disagreement_m',NaN, ...
         'safe_to_commit',false,'continuous_time_tracking_proven',false);
% The seventh state is body yaw angular rate, NOT payload X.
x=[1;0;0;0;0;0;900; 0.2;-0.1;1.15; 0.4;1.5;0.07; 0;0;-1; 0;0;0];
[p,v,ok]=TRAJECTORY_C9_HLC_PAYLOAD_STATE(x);
assert(ok&&isequal(p,[0.2;-0.1;1.15])&&isequal(v,[0.4;1.5;0.07]), ...
    'C9_LAYOUT_CHECK:Decode','The 19-entry HLC state was decoded incorrectly.');
[~,~,old_ok]=TRAJECTORY_C9_HLC_PAYLOAD_STATE(x(2:end));
assert(~old_ok,'C9_LAYOUT_CHECK:RejectOld', ...
       'The erroneous 18-entry layout must be rejected.');
[~,~,inf_ok]=TRAJECTORY_C9_HLC_PAYLOAD_STATE([x(1:18);Inf]);
assert(~inf_ok,'C9_LAYOUT_CHECK:RejectNonfinite', ...
       'Nonfinite data must be rejected.');
% Synthetic HLC+EKF+reference agreement and deliberate ~0.2m mismatch.
D.t=[0;1;1.025];D.phase=double(['t';'f';'f']);
ref=zeros(28,1);ref(1:3)=[0;0;1];ref(5:7)=[0;1.5;0];
D.agent(1).controller.result={struct('x',x),struct('x',x),struct('x',x)};
D.agent(1).reference.result={struct('state',struct('xd',ref)), ...
    struct('state',struct('xd',ref)),struct('state',struct('xd',ref))};
D.agent(1).estimator.result={struct('state',struct('pL',p,'vL',v,'mL',0.1)), ...
    struct('state',struct('pL',p,'vL',v,'mL',0.1)), ...
    struct('state',struct('pL',p,'vL',v,'mL',0.1))};
A=TRAJECTORY_C9_ACTUAL_FLIGHT_STATE_AUDIT(D);
W=TRAJECTORY_C9_RECORDED_TRACKING_WITNESS(D);
assert(A.valid_controller_reference_count==2 && ...
    A.valid_EKF_count==2 && abs(A.max_HLC_EKF_position_disagreement_m)<1e-12 && ...
    abs(A.max_position_error_m-norm(p-ref(1:3)))<1e-12 && ...
    abs(W.max_observed_payload_position_error_m-A.max_position_error_m)<1e-12, ...
    'C9_LAYOUT_CHECK:TwoAuditors','Auditors disagree with 19-entry fixture.');
D.agent(1).controller.result{3}.x(1)=[];
B=TRAJECTORY_C9_ACTUAL_FLIGHT_STATE_AUDIT(D);
WB=TRAJECTORY_C9_RECORDED_TRACKING_WITNESS(D);
assert(B.invalid_controller_reference_count==1 && WB.invalid_sample_count==1 && ...
    startsWith(B.status,'UNKNOWN') && startsWith(WB.status,'UNKNOWN'), ...
    'C9_LAYOUT_CHECK:OldLayoutAccepted','Old 18-entry layout was accepted.');
R.fixture_pass=true;
if nargin<1 || isempty(logger_data)
    if evalin('base','exist(''logger'',''var'')')==1
        logger_data=evalin('base','logger');
    else
        logger_data=[];
    end
end
if isempty(logger_data)
    R.status='PASS_HLC_19_LAYOUT_FIXTURE__NO_FLIGHT_LOGGER';
else
    A=TRAJECTORY_C9_ACTUAL_FLIGHT_STATE_AUDIT(logger_data);
    W=TRAJECTORY_C9_RECORDED_TRACKING_WITNESS(logger_data);
    R.flight_data_supplied=true;
    R.flight_records=A.sample_count;
    R.flight_valid_samples=A.valid_controller_reference_count;
    R.max_observed_payload_position_error_m=A.max_position_error_m;
    R.max_observed_payload_velocity_error_mps=A.max_velocity_error_mps;
    R.max_HLC_EKF_disagreement_m=A.max_HLC_EKF_position_disagreement_m;
    assert(A.sample_count==W.sample_count && ...
        A.valid_controller_reference_count==W.valid_sample_count, ...
        'C9_LAYOUT_CHECK:Mismatch','Different tracking sample counts.');
    if A.invalid_controller_reference_count>0 || W.invalid_sample_count>0
        R.status='UNKNOWN_INVALID_FLIGHT_TRACE';
    elseif abs(A.max_position_error_m-W.max_observed_payload_position_error_m)>1e-10 || ...
           abs(A.max_velocity_error_mps-W.max_observed_payload_velocity_error_mps)>1e-10
        R.status='UNKNOWN_TRACKING_AUDIT_DISAGREEMENT';
    else
        R.status='CHECKED_CORRECT_19_HLC_STATE_AND_SAMPLED_FLIGHT__NO_TUBE_PROOF';
    end
end
% After-flight only. Output under common_matlab, regardless of pwd.
base='TRAJECTORY_C9_HLC_STATE_LAYOUT_RESULT';
matname=[base '.mat'];txtname=[base '.txt'];zipname=[base '_FOR_CHATGPT.zip'];
save(fullfile(root,matname),'R');
fid=fopen(fullfile(root,txtname),'wt');
assert(fid>0,'C9_LAYOUT_CHECK:Write','Could not create results TXT.');
closer=onCleanup(@()fclose(fid));
fprintf(fid,'status=%s\nfixture_pass=%d\nflight_data_supplied=%d\n', ...
 R.status,R.fixture_pass,R.flight_data_supplied);
fprintf(fid,'flight_records=%d\nflight_valid_samples=%d\n', ...
 R.flight_records,R.flight_valid_samples);
fprintf(fid,'max_payload_position_error_m=%.12g\n',R.max_observed_payload_position_error_m);
fprintf(fid,'max_payload_velocity_error_mps=%.12g\n',R.max_observed_payload_velocity_error_mps);
fprintf(fid,'max_HLC_EKF_position_gap_m=%.12g\n',R.max_HLC_EKF_disagreement_m);
fprintf(fid,'tracking_continuous_time_certified=0\nsafe_to_commit=0\n');
clear closer
zippath=fullfile(root,zipname);
if isfile(zippath),delete(zippath);end
zip(zippath,{matname,txtname},root);
assert(isfile(zippath),'C9_LAYOUT_CHECK:ZIP','Results ZIP was not created.');
end
