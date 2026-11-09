function R=CHECK_TRAJECTORY_GENERATION_FOUNDATION()
% Standalone new-module test, not a certified avoidance/Flight test.
root=fileparts(mfilename('fullpath'));
folder=fullfile(root,'reference','REPLANNING_SYSTEM','TRAJECTORY_GENERATION');
assert(isfolder(folder),'CHECK_TRAJECTORY:Folder','Extract ZIP at common_matlab root.');
addpath(folder);
% Arbitrary nonzero planar velocity: two segments sharing the exact 0..7 jet.
a=zeros(3,8);a(:,1)=[1;2;3];a(:,2)=[0.9;0.4;-0.1];
b=a;b(:,1)=a(:,1)+2*a(:,2);c=b;c(:,1)=b(:,1)+3*b(:,2);
p=TRAJECTORY_C7_BERNSTEIN.build(4,2,a,b);
q=TRAJECTORY_C7_BERNSTEIN.build(6,3,b,c);
assert(p.degree==15&&isequal(size(p.control_points_m),[3 16]));
assert(TRAJECTORY_C7_BERNSTEIN.join_jet_check(p,q,0));
for t=[4,4.5,5,5.5,6]
    J=TRAJECTORY_C7_BERNSTEIN.evaluate(p,t);
    expect=a(:,1)+a(:,2)*(t-4);
    assert(norm(J(:,1)-expect,inf)<1e-9 && ...
        norm(J(:,2)-a(:,2),inf)<1e-9 && norm(J(:,3:8),inf)<1e-8);
    [r,y]=TRAJECTORY_C7_BERNSTEIN.reference28(p,t,atan2(0.4,0.9));
    assert(isequal(size(r),[28 1])&&all(isfinite(r))&& ...
        abs(y(1)-atan2(0.4,0.9))<1e-12 && max(abs(y(2:7)))<1e-8);
end
assert(norm(TRAJECTORY_C7_BERNSTEIN.evaluate(p,6)- ...
    TRAJECTORY_C7_BERNSTEIN.evaluate(q,6),inf)<1e-10);
% Time and transverse heading derivatives of a genuine curved polynomial.
s=a;e=b;e(:,3)=[0.4;-0.25;0.15];
curve=TRAJECTORY_C7_BERNSTEIN.build(0,2,s,e);
[r,y,J]=TRAJECTORY_C7_BERNSTEIN.reference28(curve,1,atan2(0.4,0.9));
assert(abs(y(1)-atan2(J(2,2),J(1,2)))<1e-10);
assert(norm(r(1:3)-J(:,1),inf)<1e-10);
% A horizontal stationary segment has no heading; never invent a yaw.
z=zeros(3,8);z(:,1)=[1;2;3];stationary=TRAJECTORY_C7_BERNSTEIN.build(0,1,z,z);
try
    TRAJECTORY_C7_BERNSTEIN.reference28(stationary,0,0);
    error('CHECK_TRAJECTORY:MissedSingularity','Zero horizontal speed was accepted.');
catch ME
    if strcmp(ME.identifier,'CHECK_TRAJECTORY:MissedSingularity'),rethrow(ME);end
    assert(strcmp(ME.identifier,'TRAJECTORY_C7_BERNSTEIN:HeadingSingular'));
end
G=TRAJECTORY_GENERATOR();
req=struct('time_s',0,'ekf_now',struct(), ...
    'observed_frame',struct('detected_ids',[]),'active_reference',[], ...
    'latest_payload_mass_kg',1,'physical_parameters',struct(), ...
    'collision_event',struct());
proposal=G.propose(req);
assert(~proposal.safe_to_commit&&~proposal.candidate_available);
try
    G.commit();
    error('CHECK_TRAJECTORY:UnsafeCommit','Uncertified proposal was committed.');
catch ME
    if strcmp(ME.identifier,'CHECK_TRAJECTORY:UnsafeCommit'),rethrow(ME);end
    assert(strcmp(ME.identifier,'TRAJECTORY_GENERATOR:CommitNotCertified'));
end
d=TRAJECTORY_OBSTACLE_RELEASE([1 2 3],1,struct('t_s',100));
assert(isempty(d.released_ids)&&isequal(d.retained_ids,[1 2 3]));
row=TRAJECTORY_REFERENCE_TRACE(5,uint64(0),r,r);
assert(row.exact_delivery_match&&isequal(row.actual_minus_generated_ref28,zeros(28,1)));
R=struct('status','PASS_C7_FOUNDATION__GENERATOR_NOT_YET_CONNECTED', ...
    'tests',8,'C7_endpoint_jets_verified',true, ...
    'heading_nonzero_only',true,'uncertified_commit_blocked',true, ...
    'observed_obstacles_retained',true,'real_flight_tested',false, ...
    'full_safety_certified',false);
fid=fopen(fullfile(root,'TRAJECTORY_FOUNDATION_RESULT.txt'),'wt');
if fid>=0
    closer=onCleanup(@()fclose(fid)); %#ok<NASGU>
    fprintf(fid,'%s\n',R.status);
end
end
