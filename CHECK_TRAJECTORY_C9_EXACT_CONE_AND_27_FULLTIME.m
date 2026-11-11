function R=CHECK_TRAJECTORY_C9_EXACT_CONE_AND_27_FULLTIME()
% Development ONLY, placed at common_matlab root.
% Tests mathematical directional cone support and a REAL full-time
% geometry/tension/thrust proof for degree-23 and elevated degree-27 C9.
% Does not establish runtime 5-7ms, HLC B, torque or a flight COMMIT.
root=fileparts(mfilename('fullpath'));
R=struct('status','FAIL_CONE_AUDIT','error_message','', ...
    'cone_random_cases',0,'cone_containment_violations',NaN, ...
    'cone_strictly_tighter_cases',0, ...
    'cone_max_chord_excess',NaN,'degree_elevation_max_error',NaN, ...
    'degree23_old_pass',false,'degree23_cone_pass',false, ...
    'degree27_cone_pass',false,'certificate_statuses',{{}}, ...
    'oracle_ms',NaN(1,3),'matlab_full_time_hlc_proven',false, ...
    'safe_to_commit',false);
try
    % Fixed RNG affects this CHECK alone, never the online planner.
    oldrng=rng; clean=onCleanup(@()rng(oldrng)); %#ok<NASGU>
    rng(490261,'twister');
    K=4000; breaches=0;tighter=0;excess=0;
    for k=1:K
        c=randn(3,1);c=c/norm(c)*(0.1+20*rand());
        rho=norm(c)*rand()*0.999;
        n=randn(3,1);n=n/norm(n);
        [qmax,qmin,ok]=TRAJECTORY_C9_DIRECTION_CONE_SUPPORT(c,rho,n);
        assert(ok,'Invalid exact cap bound');
        ratio=rho/norm(c);
        ca=sqrt(max(0,1-ratio^2));
        chord=ratio*sqrt(2/(1+ca));
        a=(c.'*n)/norm(c);
        deltaHi=qmax-(a+chord);
        deltaLo=(a-chord)-qmin;
        candidateExcess=max([0,deltaHi,deltaLo]);
        excess=max(excess,candidateExcess);
        if candidateExcess>3e-12,breaches=breaches+1;end
        if qmax < min(1,a+chord)-1e-12,tighter=tighter+1;end
        v=randn(3,1);v=v/norm(v);
        w=c+rho*rand()^(1/3)*v;
        q=w/norm(w);
        if n.'*q>qmax+3e-12 || n.'*q<qmin-3e-12
            breaches=breaches+1;
        end
    end
    R.cone_random_cases=K;
    R.cone_containment_violations=breaches;
    R.cone_strictly_tighter_cases=tighter;
    R.cone_max_chord_excess=excess;
    assert(breaches==0 && tighter>0, ...
        'Directional cone mathematically violates the old conservative hull');
    [badmax,badmin,ok]=TRAJECTORY_C9_DIRECTION_CONE_SUPPORT([0;0;1],1,[1;0;0]);
    assert(~ok&&isnan(badmax)&&isnan(badmin), ...
        'Degenerate slack direction must be UNKNOWN');
    % Elevate a degree-23 C9 curve exactly four times, no new segment.
    t=linspace(0,1,24);
    P23=[0.06*sin(pi*t);0.04*sin(2*pi*t);3+0.04*sin(pi*t)];
    P27=P23;
    for k=1:4
        n=size(P27,2)-1;
        Pnew=zeros(3,n+2);
        Pnew(:,1)=P27(:,1);Pnew(:,end)=P27(:,end);
        for i=1:n
            Pnew(:,i+1)=(i/(n+1))*P27(:,i)+ ...
                (1-i/(n+1))*P27(:,i+1);
        end
        P27=Pnew;
    end
    worst=0;
    for u=linspace(0,1,81)
        worst=max(worst,max(abs(decasteljau(P23,u)-decasteljau(P27,u))));
    end
    R.degree_elevation_max_error=worst;
    assert(worst<2e-12,'Degree elevation altered physical C9 curve');
    % Synthetically fixed TEST authority only; not used as Flight fallback.
    authority=struct('cable_length_m',2, ...
        'payload_mass_kg',0.14,'uav_mass_kg',1.1, ...
        'gravity_world_mps2',[0;0;9.80665], ...
        'thrust_limits_N',[0;20], ...
        'minimum_tension_N',0,'acceleration_guard_mps2',0);
    rad=struct('uav_m',0.3,'cable_m',0.03,'payload_m',0.25);
    observed=[OBSTACLE_INFORMATION.define(31,'sphere',[4 0 3],0.5), ...
        OBSTACLE_INFORMATION.define(32,'ellipsoid',[-4 1 3],[0.6 1 2], ...
        'euler_deg',[17 29 41])];
    opts=struct('max_subdivision_depth',6,'max_node_count',300, ...
        'numerical_guard_m',1e-9,'robust_tube_enabled',false, ...
        'trajectory_phase','f','use_native_support_separator',false, ...
        'use_ancestral_obstacle_certificate_reuse',true, ...
        'use_exact_direction_cone_support',false);
    force=struct('max_subdivision_depth',6,'max_node_count',300, ...
        'force_guard_N',1e-9);
    A=struct('control_points_m',P23,'t0_s',0,'duration_s',5);
    B=A;B.control_points_m=P27;
    tic1=tic;
    S0=TRAJECTORY_C9_JOINT_FAST_CERTIFICATE(A,observed,rad,authority,opts,force);
    R.oracle_ms(1)=1000*toc(tic1);
    opts.use_exact_direction_cone_support=true;
    tic1=tic;
    S1=TRAJECTORY_C9_JOINT_FAST_CERTIFICATE(A,observed,rad,authority,opts,force);
    R.oracle_ms(2)=1000*toc(tic1);
    tic1=tic;
    S2=TRAJECTORY_C9_JOINT_FAST_CERTIFICATE(B,observed,rad,authority,opts,force);
    R.oracle_ms(3)=1000*toc(tic1);
    R.degree23_old_pass=S0.certified;
    R.degree23_cone_pass=S1.certified;
    R.degree27_cone_pass=S2.certified;
    R.certificate_statuses={S0.status,S1.status,S2.status};
    assert(S0.certified&&S1.certified&&S2.certified, ...
        'Synthetic degree23/27 geometry+force certificate not obtained.');
    assert(isequal(S0.considered_obstacle_ids,S1.considered_obstacle_ids) && ...
        isequal(S1.considered_obstacle_ids,S2.considered_obstacle_ids), ...
        'Observed IDs changed between certificates');
    assert(~S0.safe_to_commit&&~S1.safe_to_commit&&~S2.safe_to_commit, ...
        'A nominal geometry proof must never authorize Flight commit.');
    R.status='PASS_CONE_REAL_ARITHMETIC_AND_DEGREE27_FULLTIME__HLC_PENDING';
catch ME
    R.error_message=getReport(ME,'extended','hyperlinks','off');
end
% Results are anchored at common_matlab/ regardless of MATLAB pwd.
matfile=fullfile(root,'TRAJECTORY_C9_EXACT_CONE_AND_27_RESULT.mat');
txtfile=fullfile(root,'TRAJECTORY_C9_EXACT_CONE_AND_27_RESULT.txt');
zipfile=fullfile(root,'TRAJECTORY_C9_EXACT_CONE_AND_27_RESULT_FOR_CHATGPT.zip');
save(matfile,'R');
fid=fopen(txtfile,'w');
if fid>=0
    fprintf(fid,'status=%s\n',R.status);
    fprintf(fid,'error_message=%s\n',R.error_message);
    fprintf(fid,'cone_random_cases=%d\n',R.cone_random_cases);
    fprintf(fid,'cone_containment_violations=%g\n',R.cone_containment_violations);
    fprintf(fid,'cone_strictly_tighter_cases=%d\n',R.cone_strictly_tighter_cases);
    fprintf(fid,'degree_elevation_max_error=%.16g\n',R.degree_elevation_max_error);
    fprintf(fid,'certified_23_old=%d certified_23_cone=%d certified_27_cone=%d\n', ...
        R.degree23_old_pass,R.degree23_cone_pass,R.degree27_cone_pass);
    fprintf(fid,'oracle_ms=%.6g %.6g %.6g\n',R.oracle_ms);
    fclose(fid);
end
try
    if exist(zipfile,'file'),delete(zipfile);end
    zip(zipfile,{'TRAJECTORY_C9_EXACT_CONE_AND_27_RESULT.mat', ...
        'TRAJECTORY_C9_EXACT_CONE_AND_27_RESULT.txt'},root);
catch
    % Raw .mat/.txt remain for inspection even if ZIP API is unavailable.
end
end

function v=decasteljau(P,u)
Q=P;
for k=1:size(P,2)-1
    Q=(1-u)*Q(:,1:end-1)+u*Q(:,2:end);
end
v=Q(:,1);
end
