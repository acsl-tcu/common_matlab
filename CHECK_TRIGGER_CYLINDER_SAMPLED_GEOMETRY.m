function A=CHECK_TRIGGER_CYLINDER_SAMPLED_GEOMETRY()
% Independent OFFLINE geometry fixtures. No Flight / warmup / sensor update.
% Exercises finite cylinders with exact 3-D line segments and SO(3) poses.
% Does not claim continuous-time or uncertain closed-loop safety.
c=struct('id',1,'type','cylinder','dimensions',[1;2], ...
    'center0',[0;0;0],'rotation',eye(3), ...
    'is_stationary',true);
% q, l, expected [UAV PAYLOAD CABLE] at the nominal sample.
cases={ ...
    [-2;0;0], [ 2;0;0], [false,false,true]; ...
    [-2;0;3], [ 2;0;3], [false,false,false]; ...
    [-2;0;2], [ 2;0;-2], [false,false,true]; ...
    [-2;0;3], [ 2;0;2], [false,false,false]; ...
    [1.4;0;0],[3;0;0], [true,false,false]; ...
    [3;0;0], [0;0;0], [false,true,true]; ...
    [0;0;1.3],[0;0;3], [true,false,false]; ...
    [0;0;-3],[0;0;3], [false,false,true]; ...
    [-2;1.1;0],[2;1.1;0], [false,false,false]; ...
    [-2;1;0], [2;1;0], [false,false,true] ...
    };
A=struct('status','FAIL_CLOSED','case_count',size(cases,1), ...
    'failed_cases',zeros(1,0),'rotated_case_pass',false, ...
    'combined_vector_pass',false,'point_oracle_pass',false,'no_flight',true, ...
    'continuous_time_certified',false,'error','');
try
    q=cell2mat(cases(:,1)'); l=cell2mat(cases(:,2)');
    [U,L,C]=TRIGGER_CYLINDER_SAMPLED_GEOMETRY.evaluate(q,l,c,.5,.5);
    ref=cell2mat(cases(:,3));
    got=[U(:),L(:),C(:)];
    A.failed_cases=find(any(got~=ref,2)).';
    A.combined_vector_pass=isempty(A.failed_cases);
    angle=.713; axis=[1;2;-1]; axis=axis/norm(axis);
    K=[0 -axis(3) axis(2);axis(3) 0 -axis(1);-axis(2) axis(1) 0];
    R=eye(3)+sin(angle)*K+(1-cos(angle))*(K*K);
    c.rotation=R;c.center0=[5;-3;2];
    % Exclude the exactly tangent floating-point fixture from the strict
    % rotated equality check (roundoff can change a tangency by ~1 ulp).
    [UR,LR,CR]=TRIGGER_CYLINDER_SAMPLED_GEOMETRY.evaluate( ...
        R*q(:,1:end-1)+c.center0,R*l(:,1:end-1)+c.center0,c,.5,.5);
    A.rotated_case_pass=isequal(UR,U(1:end-1)) && ...
        isequal(LR,L(1:end-1)) && isequal(CR,C(1:end-1));
    % Independent reference: the original exact point-to-cylinder distance.
    % Check the non-tangent cases in the rotated frame to avoid a 1-ulp
    % ambiguity at a mathematical tangency.
    oracle_ok=true;
    for k=1:size(q,2)-1
        qu=R*q(:,k)+c.center0;
        ql=R*l(:,k)+c.center0;
        du=OBSTACLE_GEOMETRY_QUERY.distance(qu,c,0,c.center0);
        dl=OBSTACLE_GEOMETRY_QUERY.distance(ql,c,0,c.center0);
        oracle_ok=oracle_ok && (du<=.5)==UR(k) && (dl<=.5)==LR(k);
    end
    A.point_oracle_pass=oracle_ok;
    if A.combined_vector_pass && A.rotated_case_pass && A.point_oracle_pass
        A.status='PASS_SYNTHETIC_FINITE_CYLINDER_GEOMETRY__NOT_FLIGHT_PROOF';
    end
catch ME
    A.error=[ME.identifier ' | ' ME.message];
end
home=fileparts(mfilename('fullpath'));
save(fullfile(home,'TRIGGER_CYLINDER_GEOMETRY_AUDIT.mat'),'A','-v7');
fid=fopen(fullfile(home,'TRIGGER_CYLINDER_GEOMETRY_AUDIT.txt'),'wt');
if fid>=0
    cleaner=onCleanup(@()fclose(fid));
    fprintf(fid,'status=%s\ncases=%d\nfailed_cases=%s\nrotated_pass=%d\ncombined_pass=%d\npoint_oracle_pass=%d\nerror=%s\n', ...
        A.status,A.case_count,mat2str(A.failed_cases), ...
        A.rotated_case_pass,A.combined_vector_pass,A.point_oracle_pass,A.error);
end
fprintf('[CYLINDER GEOMETRY] %s | failed=%s\n',A.status,mat2str(A.failed_cases));
end
