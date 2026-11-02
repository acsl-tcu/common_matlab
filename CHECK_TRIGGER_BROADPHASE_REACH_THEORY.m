function R=CHECK_TRIGGER_BROADPHASE_REACH_THEORY()
% Synthetic-only test; no Flight/EKF, obstacle information or sensor scans.
% Tests that shape-independent broad phase NEVER discards a sample that
% intersects even the obstacle ENCLOSING ball; and checks the residual
% comparison tube against closed-form scalar comparison ODE solutions.
types={'sphere','ellipsoid','box','cylinder','capsule','cone', ...
    'frustum','pyramid','regular_prism','torus','polygon_prism','polyhedron'};
N=161; t=linspace(0,4,N);
U=[4*cos(2*t);2*sin(3*t);.5*sin(7*t)];
L=U+[1.5*sin(4*t);.7*cos(2*t);-2+0*t];
shapes=repmat(struct('id',0,'center0',zeros(3,1), ...
    'bounding_radius_m',0,'is_stationary',true,'type',''),1,numel(types));
for j=1:numel(types)
    shapes(j).id=j;
    shapes(j).center0=[2*cos(j);2*sin(j);sin(j)];
    shapes(j).bounding_radius_m=.2+.1*j;
    shapes(j).type=types{j};
end
ru=.5;rl=.5;rc=0;
G=TRIGGER_OBSERVED_BROADPHASE.evaluate(U,L,shapes,ru,rl,rc);
pass=true;checked=0;
for j=1:numel(shapes)
    if ~G.sampled_excluded(j),continue;end
    c=shapes(j).center0;b=shapes(j).bounding_radius_m;
    for k=1:N
        u=U(:,k);l=L(:,k);d=l-u;
        s=dot(d,d);
        if s>0,alpha=max(0,min(1,dot(c-u,d)/s));else,alpha=0;end
        closest=u+alpha*d;
        if norm(u-c)<=b+ru || norm(l-c)<=b+rl || norm(closest-c)<=b+rc
            pass=false;
        end
        checked=checked+1;
    end
end
% Tangencies must NOT be excluded.
shapes(1).center0=[1;0;0];shapes(1).bounding_radius_m=1;
T=TRIGGER_OBSERVED_BROADPHASE.evaluate([0;0;0],[0;0;-2],shapes(1),0,0,0);
assert(~T.sampled_excluded(1),'Exact boundary contact was incorrectly culled.');
% A moving/invalid enclosure is never excluded.
shapes(1).is_stationary=false;
M=TRIGGER_OBSERVED_BROADPHASE.evaluate(U,L,shapes(1),ru,rl,rc);
assert(~M.sampled_excluded(1),'Moving shape was wrongly culled.');
% Exact scalar comparison equation for three different logarithmic rates.
mu=[-2 0 1.25];rho=[.3 .5 .7];grid=[0 .05 .15 .4];e0=.12;
V=TRIGGER_RESIDUAL_REACH_ENCLOSURE.evaluate( ...
    grid,e0,rho,mu,[.02 .04 .08],[.03 .06 .09],1,1);
exact=e0;
for k=1:numel(mu)
    h=grid(k+1)-grid(k);
    if mu(k)==0
        exact=exact+rho(k)*h;
    else
        exact=exp(mu(k)*h)*exact+rho(k)*expm1(mu(k)*h)/mu(k);
    end
    assert(abs(exact-V.state_error_at_knots(k+1))<= ...
        1e-12*max(1,abs(exact)), ...
        'Residual bound does not match scalar comparison equation.');
end
assert(all(V.interval_uav_excursion_m>=0) && ...
    ~V.continuous_closed_loop_safe_certified, ...
    'An unverified conditional tube must not claim safety.');
rejected=false;
try
    TRIGGER_RESIDUAL_REACH_ENCLOSURE.evaluate(grid,e0,rho(1:2),mu, ...
        [.02 .04 .08],[.03 .06 .09],1,1);
catch
    rejected=true;
end
assert(rejected,'Missing bound must fail closed.');
assert(pass,'A valid enclosing ball was incorrectly discarded.');
R=struct('status','PASS_SYNTHETIC_BROADPHASE_AND_CONDITIONAL_REACH__NO_FLIGHT', ...
    'sample_count',N,'shape_count',numel(types), ...
    'shape_ids_sampled_excluded',G.sampled_excluded_ids, ...
    'oracle_evaluations',checked,'no_missed_enclosing_ball_contacts',pass, ...
    'comparison_ode_exact_scalar_test_pass',true, ...
    'missing_bound_rejected',rejected, ...
    'continuous_closed_loop_safety_proven',false);
root=fileparts(mfilename('fullpath'));
save(fullfile(root,'TRIGGER_BROADPHASE_REACH_THEORY_TEST.mat'),'R','-v7');
fid=fopen(fullfile(root,'TRIGGER_BROADPHASE_REACH_THEORY_TEST.txt'),'wt');
if fid>=0
    C=onCleanup(@()fclose(fid)); %#ok<NASGU>
    fprintf(fid,'status=%s\nshapes=%d\nsamples=%d\n',R.status,numel(types),N);
    fprintf(fid,'contact_oracle_checks=%d\ncontinuous_safe=%d\n', ...
        checked,R.continuous_closed_loop_safety_proven);
end
end
