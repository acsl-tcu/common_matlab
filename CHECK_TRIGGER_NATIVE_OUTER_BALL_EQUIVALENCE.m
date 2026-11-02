function R=CHECK_TRIGGER_NATIVE_OUTER_BALL_EQUIVALENCE()
% Independent synthetic sampled-only equivalence test. NO live Flight reads.
% It does NOT establish continuous-time physical safety, nor benchmark 4s.
root=fileparts(mfilename('fullpath'));
N=0; mismatch=0; worst=[];
rngstate=rng;
c=onCleanup(@()rng(rngstate)); %#ok<NASGU>
rng(15);
for trial=1:180
    K=1+mod(trial,17); M=1+mod(trial,11);
    U=4*randn(3,K);L=U+3*randn(3,K);
    if mod(trial,9)==0, L=U; end % degenerate finite segment
    radius=[0.5;0.5;0];
    if mod(trial,5)==0,radius(3)=0.03;end
    balls=zeros(5,M);
    shapes=repmat(struct('id',0,'center0',[0;0;0], ...
        'bounding_radius_m',0,'is_stationary',true),1,M);
    for j=1:M
        center=4*randn(3,1);extent=0.02+rand*2;
        stationary=mod(trial+7*j,13)~=0;
        balls(:,j)=[center;extent;double(stationary)];
        shapes(j)=struct('id',j,'center0',center, ...
            'bounding_radius_m',extent,'is_stationary',logical(stationary));
    end
    old=TRIGGER_OBSERVED_BROADPHASE.evaluate( ...
        U,L,shapes,radius(1),radius(2),radius(3));
    now=TRIGGER_SYMBOLIC_NATIVE_CORE( ...
        'sampled_outer_ball_test',U,L,balls,radius);
    N=N+M;
    if ~isequal(logical(old.sampled_excluded),logical(now))
        mismatch=mismatch+1;
        worst=[worst,trial]; %#ok<AGROW>
    end
end
R=struct('status','', 'cases',180, 'objects_compared',N, ...
    'mismatching_cases',mismatch,'mismatching_case_ids',worst, ...
    'native_flow_tested',false,'real_flight_tested',false, ...
    'continuous_safety_proved',false);
if mismatch==0
    R.status='PASS_SAMPLED_OUTER_BALL_ORACLE__NOT_FLIGHT_OR_SAFETY';
else
    R.status='FAIL_SAMPLED_OUTER_BALL_ORACLE';
end
save(fullfile(root,'TRIGGER_NATIVE_OUTER_BALL_EQUIVALENCE.mat'),'R');
fid=fopen(fullfile(root,'TRIGGER_NATIVE_OUTER_BALL_EQUIVALENCE.txt'),'wt');
if fid>=0
 fprintf(fid,'status=%s\ncases=%d objects=%d mismatches=%d\n', ...
     R.status,180,N,mismatch);
 fclose(fid);
end
end
