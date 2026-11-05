function R=CHECK_TRIGGER_SEGMENT_CLEARANCE_PROOF()
% One deterministic geometry theorem verification; NO obstacle-specific tuning.
% For any q nearest to line segment V and every convex hull point x:
% (q-c)'*(x-q)>=0. An enclosing ball B(c,b) gives the shortcut below.
% This validates the shortcut's algebra and a range of orientations,
% independently of the real Flight predictor or source files.
% Deterministic samples, WITHOUT altering MATLAB's global RNG state.
worst=inf;checks=0;
for j=1:1000
    c=10*sin([j;2*j;3*j]*0.137);
    a=10*cos([j;3*j;4*j]*0.319);
    d=10*sin([2*j;5*j;7*j]*0.223);
    u=d-a; den=dot(u,u);
    if den>0,alpha=max(0,min(1,dot(c-a,u)/den));
    else,alpha=0;end
    q=a+alpha*u;
    nn=norm(q-c);
    b=2*(1+sin(j*0.41))/2; radius=(1+cos(j*0.29))/2;
    tol=1024*eps(max([1;abs(c);abs(a);abs(d);b;radius]));
    if nn>b+radius+tol
        n=(q-c)/nn;
        % A stricter inequality than support_obstacle(n)<=b implies the
        % original support-plane certificate necessarily succeeds.
        margin=min(n.'*[a-c d-c])-b-radius-tol;
        assert(margin>0,'CHECK_TRIGGER_SEGMENT_CLEARANCE_PROOF:Plane', ...
            'Exact segment-ball exclusion did not imply separation.');
        worst=min(worst,margin);checks=checks+1;
    end
end
assert(checks>0,'CHECK_TRIGGER_SEGMENT_CLEARANCE_PROOF:Coverage');
R=struct('status','PASS_SEGMENT_ENCLOSING_BALL_SHORTCUT', ...
    'trials',1000,'successful_shortcuts',checks,'minimum_margin',worst);
end
