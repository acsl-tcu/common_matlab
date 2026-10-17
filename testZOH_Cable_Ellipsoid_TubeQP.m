%% testZOH_Cable_Ellipsoid_TubeQP.m
% Empirical cross-check of the analytic (not numerically fitted) tube bound.
% Requires FLxyDst, GLxyDst, ENVIRONMENT_OBSTACLE_ELLIPSOID_STATIC.
% Not a substitute for the conditional mathematical proof or a full Sim run.
rng(20261017,'twister');
P=[0.75 0.02 0.025 0.035 9.81 0.14 2 0 0];
P22=zeros(1,22);P22([1 6 7 8 9 19 20 21 22])=P;
obs=ENVIRONMENT_OBSTACLE_ELLIPSOID_STATIC();
dt=0.025; rhoCable=0.04;
N=50;certified=0;minMeasured=Inf;minAlgebraic=Inf;
for k=1:N
    q=[1;0.05*randn(3,1)];q=q/norm(q);
    pT=[0.1*randn(2,1);-1];pT=pT/norm(pT);
    ol=0.12*randn(3,1);ol=ol-pT*(pT.'*ol);
    % Start in diverse states and heights; rejected trials are NOT counted.
    pL=[-5+10*rand;-5+10*rand;5+27*rand];
    x=[q;0.2*randn(3,1);pL;1.0*randn(3,1);pT;ol];
    u=[20*rand; -1+2*rand; -1+2*rand; -1+2*rand];
    [A,b,info]=ZOH_Cable_Ellipsoid_TubeQP(x,P,dt,obs,rhoCable,0);
    if any(b-A*u(1:3)<1e-8),continue;end
    certified=certified+1;
    fun=@(t,y) FLxyDst(y,P22)+GLxyDst(y,P22)*u;
    opts=odeset('RelTol',1e-11,'AbsTol',1e-12);
    [~,xx]=ode45(fun,linspace(0,dt,17),x,opts);
    for j=1:numel(obs)
        n=info.plane(:,j);
        S=obs(j).R_obs*diag(obs(j).ellipsoid_radii(:).^2)*obs(j).R_obs.';
        lim=n.'*obs(j).p_obs(:)+sqrt(n.'*S*n)+rhoCable+obs(j).d_margin;
        for idx=1:size(xx,1)
            y=xx(idx,:).';
            gap=min(n.'*y(8:10), n.'*(y(8:10)-P(7)*y(14:16)))-lim;
            minMeasured=min(minMeasured,gap);
            assert(gap>=-1e-6,'Certified tube failed numerical rollout!');
        end
    end
    minAlgebraic=min(minAlgebraic,min(b-A*u(1:3)));
end
fprintf('[ZOH tube test] %d/%d randomly chosen periods certified.\n',certified,N);
fprintf('[ZOH tube test] min simulated support gap = %.6e m.\n',minMeasured);
fprintf('[ZOH tube test] min algebraic slack = %.6e m.\n',minAlgebraic);
assert(certified>=5,'Too few certified periods to evaluate numerical test.');
fprintf('[PASS] Accepted commands stayed outside obstacles throughout all sampled substeps.\n');
fprintf('[NOTE] ode45 comparison is empirical; theorem depends on model exactness and ongoing feasibility.\n');
