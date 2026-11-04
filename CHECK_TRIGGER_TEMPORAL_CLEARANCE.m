function R=CHECK_TRIGGER_TEMPORAL_CLEARANCE()
% Disposable COMMON ROOT audit. Synthetic checks only; no Flight authority.
root=fileparts(mfilename('fullpath'));
R=struct('status','FAIL_TEMPORAL_CLEARANCE','cases',0, ...
    'failures',0,'reason','','true_clear_cases',0, ...
    'multiple_ids',0,'safe_to_commit',false);
old_rng=rng;on_cleanup=onCleanup(@()rng(old_rng)); %#ok<NASGU>
try
    rng(771390,'twister');
    % 1. Strict zero is not clearance; negative/singular inputs fail closed.
    B=struct('time_grid_s',[0 1], 'observed_ids',5, ...
        'sample_clearance_lower_m',[0 0], 'speed_upper_mps',0);
    X=TRIGGER_TAUT_TEMPORAL_CLEARANCE_WITNESS(B);
    assert(~X.all_intervals_separated && ~X.safe_to_commit, ...
        'CHECK_TIME:Zero','Tangent material body cannot be certified clear.');
    R.cases=R.cases+1;
    % 2. Simultaneous risk for unsorted observed IDs, no fixed mask.
    B.time_grid_s=[0 .2 .4];B.observed_ids=[19 5 27];
    B.sample_clearance_lower_m=[1 1 1; .1 .1 .1; 0 0 0];
    B.speed_upper_mps=[0 0];
    X=TRIGGER_TAUT_TEMPORAL_CLEARANCE_WITNESS(B);
    assert(isequal(X.separated_intervals,[true true;true true;false false]) && ...
        isequal(X.possible_risk_ids,27) && ~X.safe_to_commit, ...
        'CHECK_TIME:IDs','Incorrect observed ID ordering/risk mapping.');
    R.cases=R.cases+1;
    B.sample_clearance_lower_m(2,:)=[0 0 0];
    X=TRIGGER_TAUT_TEMPORAL_CLEARANCE_WITNESS(B);
    assert(isequal(X.possible_risk_ids,[5 27]) && ...
        ~X.all_intervals_separated && ~X.safe_to_commit, ...
        'CHECK_TIME:Multiple','Two simultaneous unproved IDs must be returned.');
    R.multiple_ids=numel(X.possible_risk_ids);R.cases=R.cases+1;
    % 3. Invalid physical speed, invalid distance or time -> UNKNOWN.
    for j=1:4
        C=B;
        switch j
            case 1,C.speed_upper_mps(1)=-1;
            case 2,C.speed_upper_mps(1)=NaN;
            case 3,C.sample_clearance_lower_m(1,1)=Inf;
            case 4,C.time_grid_s(2)=C.time_grid_s(1);
        end
        X=TRIGGER_TAUT_TEMPORAL_CLEARANCE_WITNESS(C);
        assert(~X.all_intervals_separated && ~X.safe_to_commit, ...
            'CHECK_TIME:FailClosed','Bad input was incorrectly accepted.');
        R.cases=R.cases+1;
    end
    % 4. Shape-neutral physical oracle cross-check: a sphere obstacle and
    % linearly moving point as a degenerate taut segment, with exact speed.
    % Check no certified case collides at 41 intermediate real time values.
    for j=1:400
        center=randn(3,1);p0=randn(3,1)*3;
        vel=randn(3,1)*2;h=0.01+rand*0.8;
        r=0.5+rand;
        d0=norm(p0-center)-r;
        d1=norm(p0+vel*h-center)-r;
        epsilon=2048*eps(max([1;abs(p0);abs(center);r;abs(vel)]));
        B=struct('time_grid_s',[0 h],'observed_ids',j, ...
            'sample_clearance_lower_m',[d0-epsilon d1-epsilon], ...
            'speed_upper_mps',norm(vel)+epsilon);
        X=TRIGGER_TAUT_TEMPORAL_CLEARANCE_WITNESS(B);
        if X.conditional_clearance
            R.true_clear_cases=R.true_clear_cases+1;
            for i=0:40
                tt=h*i/40;
                assert(norm(p0+vel*tt-center)-r>0, ...
                    'CHECK_TIME:SampleCounterexample', ...
                    'A certified case intersected the sphere.');
            end
        end
        assert(~X.safe_to_commit, ...
            'CHECK_TIME:FlightSafety','Synthetic theorem authorized Flight.');
        R.cases=R.cases+1;
    end
    R.status='PASS_TEMPORAL_SHAPE_NEUTRAL_THEOREM__CONDITIONAL_ONLY';
catch ME
    R.failures=R.failures+1;
    R.reason=[ME.identifier ' | ' ME.message];
end
save(fullfile(root,'TRIGGER_TEMPORAL_CLEARANCE_RESULT.mat'),'R');
f=fopen(fullfile(root,'TRIGGER_TEMPORAL_CLEARANCE_RESULT.txt'),'w');
if f>=0
    fprintf(f,'status=%s\ncases=%d\nfailures=%d\n', ...
        R.status,R.cases,R.failures);
    fprintf(f,'true_clear_cases=%d\nmultiple_ids=%d\n', ...
        R.true_clear_cases,R.multiple_ids);
    fprintf(f,'safe_to_commit=%d\nreason=%s\n',R.safe_to_commit,R.reason);
    fclose(f);
end
end
