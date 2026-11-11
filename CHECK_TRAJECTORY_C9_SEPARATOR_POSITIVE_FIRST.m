function R=CHECK_TRAJECTORY_C9_SEPARATOR_POSITIVE_FIRST()
%CHECK_TRAJECTORY_C9_SEPARATOR_POSITIVE_FIRST
% Development-only low-cost independent smoke test of the positive-first
% separator. No physical Flight, no precomputed obstacle-specific path.
% Does NOT purport to certify HLC or total 25-ms cycle.
root=fileparts(mfilename('fullpath'));
R=struct('status','FAIL_NOT_STARTED','error_message','', ...
    'tests_completed',0,'certified_cases',0, ...
    'native_calls_avoided',0,'native_calls_required',0, ...
    'matlab_positive_parity',false,'elapsed_ms',NaN, ...
    'safe_to_commit',false);
clock=tic;
try
    % The obstacle and hull move through 3D arbitrary directions; all
    % normals are derived from data. No left/right/up bypass rule.
    for k=1:300
        a=2*pi*k/300;b=pi*(mod(19*k,300)+0.5)/301;
        d=[cos(a)*sin(b);sin(a)*sin(b);cos(b)];
        c=[0.3*sin(2*a);0.4*cos(3*a);0.2*sin(b)];
        H=c+6*d+[0.04 -0.04 0.02;0.03 0.01 -0.04;0.01 0.02 -0.03];
        S=struct('id',k,'kind','sphere','dim',0.5,'center',c, ...
            'R',eye(3),'local_vertices',zeros(0,3));
        ref=mean(H,2);
        [aOK,~,~,~,~,aNative]=TRAJECTORY_C9_SUPPORT_SEPARATOR( ...
            H,S,0.01,ref,32,false);
        [bOK,n,~,~,~,bNative]=TRAJECTORY_C9_SUPPORT_SEPARATOR( ...
            H,S,0.01,ref,32,true);
        assert(aOK&&bOK&&~aNative&&~bNative, ...
            'C9_SEPARATOR_POSITIVE_FIRST:MissedDirectProof', ...
            'Directly provable separation must not call MEX.');
        assert(min(n.'*H)>TRAJECTORY_OBSTACLE_SUPPORT.value(S,n)+0.01, ...
            'C9_SEPARATOR_POSITIVE_FIRST:InvalidProof', ...
            'A positive result must satisfy the canonical support condition.');
        R.tests_completed=R.tests_completed+1;
        R.certified_cases=R.certified_cases+1;
        R.native_calls_avoided=R.native_calls_avoided+1;
    end
    R.matlab_positive_parity=true;
    R.status='PASS_POSITIVE_SUPPORT_FIRST__HLC_AND_FULL_FLIGHT_PENDING';
catch ME
    R.status='FAIL_SEPARATOR_POSITIVE_FIRST';
    R.error_message=[ME.identifier ' ' ME.message];
end
R.elapsed_ms=1000*toc(clock);
try
    save(fullfile(root,'TRAJECTORY_C9_SEPARATOR_POSITIVE_FIRST_RESULT.mat'),'R');
    f=fopen(fullfile(root,'TRAJECTORY_C9_SEPARATOR_POSITIVE_FIRST_RESULT.txt'),'wt');
    fprintf(f,'status=%s\nerror=%s\ntests=%d\nnative_avoided=%d\n', ...
        R.status,R.error_message,R.tests_completed,R.native_calls_avoided);
    fclose(f);
    zip(fullfile(root,'TRAJECTORY_C9_SEPARATOR_POSITIVE_FIRST_RESULT_FOR_CHATGPT.zip'), ...
        {'TRAJECTORY_C9_SEPARATOR_POSITIVE_FIRST_RESULT.mat', ...
         'TRAJECTORY_C9_SEPARATOR_POSITIVE_FIRST_RESULT.txt'},root);
catch ME
    R.status=['FAIL_SAVE_RESULTS_' ME.identifier];
    R.error_message=ME.message;
end
end
