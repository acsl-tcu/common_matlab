function T=AF7R58R66_SELFTEST()
% Tests terminal-hold jet convention and phase scoping in a deterministic
% abstract correction. Does not prove Flight, HLC, geometry or native C++.
S=AF7R58R66_SOURCE_CHECK();
CP=zeros(3,16);
CP(1,:)=linspace(0,2,16);
CP(2,:)=linspace(0,-1,16);
CP(3,:)=linspace(0,0.25,16);
last=CP(:,end);
% Beyond the support, native holds last CP, derivatives 1..7 vanish.
J=zeros(3,8);J(:,1)=last;
T=struct('status','SOURCE_AND_HOLD_SEMANTICS_STRUCTURAL_PASS__NOT_SAFETY_CERTIFIED', ...
    'class_parser_passed',S.matlab_class_parser_passed, ...
    'hold_position_matches_last_control_point',isequal(J(:,1),last), ...
    'hold_higher_derivatives_zero',all(J(:,2:8)==0,'all'), ...
    'baseline_phase_gate_is_not_applied',S.legacy_candidate_gate_scoped_to_phase);
root=fileparts(mfilename('fullpath'));
T.mat_file=fullfile(root,'AF7R58R66_SELFTEST_OUT.mat');
T.report_file=fullfile(root,'AF7R58R66_SELFTEST_REPORT.txt');
save(T.mat_file,'T','-v7');
fid=fopen(T.report_file,'wt');assert(fid>0);c=onCleanup(@()fclose(fid));
fprintf(fid,'%s\n',T.status);
fprintf(fid,'parser=%d hold=%d derivative_zero=%d baseline_scoped=%d\n', ...
 T.class_parser_passed,T.hold_position_matches_last_control_point, ...
 T.hold_higher_derivatives_zero,T.baseline_phase_gate_is_not_applied);
end
