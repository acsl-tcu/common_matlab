function T=AF7R58R99_SELFTEST()
S=AF7R58R99_SOURCE_CHECK();
assert(strcmp(S.status,'PASS_R99_ALL_RUNTIME_DEPENDENCIES_IN_REFERENCE'), ...
 'R99:SourceBinding','Missing or shadowed required MATLAB function.');
% Quadratic arch, elevated to degree 15; its whole endpoint chord passes
% through a known sphere, but the actual arc is safe. A global single cell
% must fail; multiple local overlapping corridors must pass.
N=15;P=zeros(3,16);
p0=[-2;0;0];p1=[0;1.8;0];p2=[2;0;0];
for j=0:N
 a=(N-j)*(N-j-1)/(N*(N-1));
 b=2*j*(N-j)/(N*(N-1));
 c=j*(j-1)/(N*(N-1));
 P(:,j+1)=a*p0+b*p1+c*p2;
end
ob=struct('center',[0;0;0],'axes',[.5;.5;.5],'R',eye(3));
G=AF7R58R97_TIGHT_SUPPORT_CORRIDOR(P(:,[1 end]),ob,.01);
assert(~G.feasible_corridor_family,'R99:CoarseFalseNegativeTest', ...
 'A single chord corridor should not contain this curved arc.');
route=struct('segments',struct('payload_cp',P),'num_segments',1);
opts=struct('max_nodes_per_check',512);
C=AF7R58R99_OVERLAP_LOCAL_CORRIDOR_CHAIN(route,ob,.01,opts);
assert(C.payload_corridor_certified&&C.leaves>1&& ...
 C.overlap_count==C.leaves-1&&C.minimum_overlap_margin_m>0, ...
 'R99:LocalCorridorOverlap','Local convex chain should be connected and safe.');
for i=1:numel(C.corridor_chain)-1
 a=C.corridor_chain(i);b=C.corridor_chain(i+1);
 m=(a.end_p+b.start_p)/2;
 assert(all([a.A;b.A]*m < [a.b;b.b]+1e-12), ...
 'R99:OverlapWitness','Shared knot must belong to both convex polytopes.');
end
Q=zeros(3,16);for j=0:N,Q(:,j+1)=[-2+4*j/N;0;0];end
bad=struct('segments',struct('payload_cp',Q),'num_segments',1);
opts.max_nodes_per_check=48;
D=AF7R58R99_OVERLAP_LOCAL_CORRIDOR_CHAIN(bad,ob,.01,opts);
assert(~D.payload_corridor_certified && ~D.production_commit_authorized, ...
 'R99:UnsafeStraightAccepted','Unsafe crossing must not pass.');
W=[0 10 21;0 0 0;0 0 0];
X=AF7R58R98_ALLOCATE_KNOT_TIMES(W,0,15,2,2);
assert(X.feasible_necessary_time&&all(diff(X.times_s)>0), ...
 'R99:KnotAllocator','Required nonuniform knot allocator failed.');
% Whole-body multi-size knot corridor uses UAV and payload positions
% reconstructed from the actual taut specific-force direction. Test with
% unequal radii and multiple known obstacles; do not synthesize authority
% values in the production runner.
J=zeros(3,8,3);J(:,1,1)=[0;0;0];J(:,1,2)=[1;0;0];
J(:,1,3)=[2;0;0];
routeJ=struct('knot_jets',J,'num_segments',2);
obFar1=struct('center',[-10;0;0],'axes',[.5;.5;.5],'R',eye(3));
ob2=struct('center',[1;10;0],'axes',[.5;.7;1],'R',eye(3));
external=struct('gravity_m_s2',9.81,'cable_length_m',2, ...
 'rQ_m',.25,'rL_m',.15,'rC_m',.03,'d_margin_m',.01);
MC=AF7R58R99_MULTI_SIZE_KNOT_CORRIDOR(routeJ,[obFar1 ob2],external);
assert(MC.whole_body_knot_corridor_found && ...
 MC.adjacent_overlap_count==1 && MC.minimum_support_margin_m>0, ...
 'R99:MultisizeOverlap','UAV+payload knot prism corridor must overlap.');
T=struct('status','PASS_R99_OVERLAPPING_LOCAL_BERNSTEIN_CORRIDOR_CHAIN', ...
 'coarse_chord_fails',~G.feasible_corridor_family, ...
 'safe_curved_arc_chain_passes',C.payload_corridor_certified, ...
 'leaf_count',C.leaves,'adjacent_overlap_witnesses',C.overlap_count, ...
 'minimum_overlap_margin_m',C.minimum_overlap_margin_m, ...
 'unsafe_curve_not_accepted',~D.payload_corridor_certified, ...
 'nonuniform_necessary_time_pass',X.feasible_necessary_time, ...
 'multi_size_knot_corridor_overlap_passes',MC.whole_body_knot_corridor_found, ...
 'source_dependencies_complete',true,'real_snapshot_exercised',false, ...
 'whole_body_certified',false,'closed_loop_proved',false, ...
 'production_modified',false,'flight_trigger_switched',false);
end
