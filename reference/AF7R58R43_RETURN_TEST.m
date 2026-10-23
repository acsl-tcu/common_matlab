function Q=AF7R58R43_RETURN_TEST()
% No hidden workspace inputs. Includes two examples: representable, and
% a boundary-condition obstruction where c(0)=0 and c'(0)<0 on n axis.
A=zeros(8,3);B=zeros(8,3);A(1,1)=1;
R1=AF7R58R43_RETURN_C7_LP(A,B,1,1,[1;0;0]);
C=zeros(8,3);D=zeros(8,3);C(2,1)=-1;
R2=AF7R58R43_RETURN_C7_LP(C,D,1,1,[1;0;0]);
Q=struct('simple_endpoint_example',R1.status, ...
 'one_sided_candidate_found',R1.feasible, ...
 'negative_start_velocity_implies_local_reverse',true, ...
 'obstructed_family_result',R2.status, ...
 'requires_wholebody_safety_before_commit',true);
assert(R1.feasible,'Feasible nonnegative C7 test candidate not found.');
assert(~R2.feasible,'A negative initial derivative at zero offset cannot be one-sided.');
fprintf('[R43] C7 LP toy tests PASS; source model/all-obstacle safety not certified.\n');
end
