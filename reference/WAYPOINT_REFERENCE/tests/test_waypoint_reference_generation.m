function test_waypoint_reference_generation()
%TEST_WAYPOINT_REFERENCE_GENERATION  No plant, HLC or toolbox required.
% Run after adding the project root and all subfolders to MATLAB path.
check_open(gen_wp_straight());
check_open(gen_wp_gentle_turn('speed',0.16));
check_open(gen_wp_custom_points());
check_open(gen_wp_formula_open());
check_periodic(gen_wp_circle('speed',0.33));
check_periodic(gen_wp_saddle('speed',0.22));
check_periodic(gen_wp_custom_formula());
fprintf('PASS: 7 generators, 28x1, non-stopping, C6 joints and endless loops.\n');
end

function check_open(ref)
assert(ref.order==6 && ~ref.isLoop && isfinite(ref.duration));
W=ref.waypoints;
assert(size(W,1)==3 && size(W,2)>=2);
for t = linspace(0,ref.duration+3,121)
    xd=ref.evaluate(t);
    assert(isequal(size(xd),[28,1]) && all(isfinite(xd)));
end
x0=ref.evaluate(0); xT=ref.evaluate(ref.duration);
assert(norm(x0(1:3)-W(:,1)) < 1e-7);
assert(norm(xT(1:3)-W(:,end)) < 1e-7);
assert(norm(xT(5:end))<1e-7);
for j=2:size(W,2)-1
    target=ref.cumulativeS(j);
    lo=0;hi=ref.duration;
    for k=1:50
        mid=(lo+hi)/2;
        sd=ref.timeLaw(mid);
        if sd(1)<target;lo=mid;else;hi=mid;end
    end
    xd=ref.evaluate((lo+hi)/2);
    assert(norm(xd(1:3)-W(:,j))<1e-6,'Intermediate waypoint mismatch.');
    assert(norm(xd(5:7))>1e-6,'Unexpected stop at intermediate waypoint.');
end
% Piecewise open path must match first through sixth spatial derivatives
% across the boundaries of every smooth corner transition.
for j=2:size(W,2)-1
    b=ref.blendRadiusEffective(j);
    for signn=[-1,1]
        q=ref.cumulativeS(j)+signn*b;
        epss=1e-11;
        a=ref.evaluatePath(q-epss); c=ref.evaluatePath(q+epss);
        for k=0:6
            assert(norm(a(:,k+1)-c(:,k+1))<1e-2, ...
                'C6 corner continuity failed.');
        end
    end
end
end

function check_periodic(ref)
assert(ref.order==6 && ref.isLoop && isinf(ref.duration));
N=size(ref.waypoints,2);
assert(N>=16);
D=ref.periodS;
for j=1:N
    s=(j-1)*D/N;
    deriv=ref.evaluatePath(s);
    assert(norm(deriv(:,1)-ref.waypoints(:,j))<1e-7, ...
        'Fourier curve must interpolate all sampled waypoints.');
end
seamA=ref.evaluatePath(0);
seamB=ref.evaluatePath(D);
assert(max(abs(seamA(:)-seamB(:)))<1e-7);
Ta=ref.rampTimeEffective; % Auto-computed acceleration time.
t1=Ta+2;
t2=t1+D/ref.speedPeak;
a=ref.evaluate(t1);b=ref.evaluate(t2);
assert(norm(a-b)<1e-5,'The reference must repeat without stopping.');
assert(norm(a(5:7))>1e-5,'Periodic route should continue moving.');
for t=linspace(0,t2+20,101)
    xd=ref.evaluate(t);
    assert(isequal(size(xd),[28,1]) && all(isfinite(xd)));
end
end
