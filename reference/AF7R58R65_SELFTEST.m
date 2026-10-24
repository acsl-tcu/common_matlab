function T=AF7R58R65_SELFTEST()
% Test the screen on actual R61 committed-source correction and a corrupted
% return.  The test does not change REPLANNING_SYSTEM, HLC, plant or EKF.
root=fileparts(mfilename('fullpath'));
source=fullfile(root,'AF7R58R61_OUT.mat');
assert(isfile(source), '[R65] required real-source R61 MAT is absent.');
S=load(source,'R61');
C=double(S.R61.real_correction.candidate.control_points);
assert(isequal(size(C),[3 16]), '[R65] expected actual degree-15 candidate.');
P=AF7R58R65_RETURN_BERNSTEIN_SCREEN(C);
assert(P.valid, '[R65] actual R61 offset return did not meet screen.');
% Build an actual convex-hull violation; not a made-up time/velocity cap.
Bad=C;
Bad(:,9)=-2*C(:,1);
Q=AF7R58R65_RETURN_BERNSTEIN_SCREEN(Bad);
assert(~Q.valid,'[R65] known overshoot/reverse polygon not rejected.');
% Check invariance to a rotation in SO(3): test is geometric, not tied to
% global x or y, so it works with non-straight nominal curves too.
a=0.42;b=-0.79;c=0.31;
Rx=[1 0 0;0 cos(a) -sin(a);0 sin(a) cos(a)];
Ry=[cos(b) 0 sin(b);0 1 0;-sin(b) 0 cos(b)];
Rz=[cos(c) -sin(c) 0;sin(c) cos(c) 0;0 0 1];
Z=AF7R58R65_RETURN_BERNSTEIN_SCREEN((Rz*Ry*Rx)*C);
assert(Z.valid,'[R65] SO(3) rotation invariance regression.');
T=struct('status','PASS_OFFSET_RETURN_STRUCTURAL_ONLY', ...
    'actual_real_polygon_pass',logical(P.valid), ...
    'bad_overshoot_rejected',logical(~Q.valid), ...
    'rotation_invariant_pass',logical(Z.valid), ...
    'max_radial_bernstein_upper',double(P.max_radial_bernstein_upper), ...
    'natural_rejoin_committed',false, ...
    'wholebody_safety_proved',false, ...
    'actual_hlc_future_proved',false);
p=fullfile(root,'AF7R58R65_SELFTEST_OUT.mat');
save(p,'T');
fp=fullfile(root,'AF7R58R65_SELFTEST_REPORT.txt');
fid=fopen(fp,'wt');
assert(fid>0,'[R65] cannot open report file.');
cleanup=onCleanup(@()fclose(fid));
fprintf(fid,'R65 | REAL R61 OFFSET RETURN SCREEN | NOT A SAFETY CERTIFICATE\n');
fprintf(fid,'status=%s\n',T.status);
fprintf(fid,'actual_polygon_pass=%d negative_rejected=%d rotation_pass=%d\n',P.valid,~Q.valid,Z.valid);
fprintf(fid,'Bernstein radial upper=%.16g\n',P.max_radial_bernstein_upper);
fprintf(fid,'natural_rejoin_committed=0 wholebody_proved=0 HLC_future_proved=0\n');
end
