function R=CHECK_TRIGGER_RISK_MULTI_ID()
% One-flight-independent audit of the conditional earliest-interval theorem.
% Check both all simultaneous IDs and strictly earlier interval precedence.
% Random cases are regression tests, NOT proof of externally supplied bounds.
R=struct('status','FAIL_RISK_MULTI_ID','cases',0, ...
    'failures',0,'reason','','safe_to_commit',false);
try
    % Audits must not perturb MATLAB's global random stream before Flight.
    prior_rng=rng;
    restore_rng=onCleanup(@()rng(prior_rng)); %#ok<NASGU>
    rng(49371,'twister');
    for z=1:2400
        K=randi([1 35]);n=randi([0 18]);
        ids=randperm(3000,n);
        dt=0.001+rand(1,K);
        t=[0 cumsum(dt)];
        v=5*rand(1,K);
        e=rand(1,K+1);
        d=7*rand(n,K+1);
        C=struct('sample_times_s',t,'observed_ids',ids, ...
            'nominal_clearance_lower_m',d, ...
            'full_body_endpoint_error_upper_m',e, ...
            'whole_body_speed_upper_mps',v);
        P=TRIGGER_EKF_INTERVAL_RISK_DECISION(C);
        assert(~P.safe_to_commit);
        if n==0
            assert(~P.conditional_replan_required && ...
                strcmp(P.status, ...
                'NO_OBSERVED_OR_RETAINED_OBSTACLES__UNKNOWN_WORLD'));
        else
            [expected_k,expected_mask]=reference_oracle(t,d,e,v);
            if isnan(expected_k)
                assert(~P.conditional_replan_required && ...
                    isempty(P.earliest_possible_ids) && ...
                    P.certified_clear_intervals==K);
            else
                expected=ids(expected_mask);
                assert(P.conditional_replan_required && ...
                    P.first_interval_index==expected_k && ...
                    isequal(P.earliest_possible_ids,expected) && ...
                    P.obstacle_id==expected(1) && ...
                    P.first_possible_ttc_left_s==t(expected_k) && ...
                    P.first_possible_ttc_right_s==t(expected_k+1) && ...
                    P.certified_clear_intervals==expected_k-1);
            end
        end
        R.cases=R.cases+1;
    end
    % Every exit path must have a stable, schema-complete result.
    clearCase=struct('sample_times_s',[0 0.5 1], ...
        'observed_ids',[100 300], ...
        'nominal_clearance_lower_m',[10 10 10;15 15 15], ...
        'full_body_endpoint_error_upper_m',[0 0 0], ...
        'whole_body_speed_upper_mps',[1 1]);
    clearResult=TRIGGER_EKF_INTERVAL_RISK_DECISION(clearCase);
    assert(~clearResult.conditional_replan_required && ...
        strcmp(clearResult.status,'CONDITIONAL_ALL_OBSERVED_INTERVALS_SEPARATED') && ...
        isempty(clearResult.earliest_possible_ids) && ...
        isnan(clearResult.first_interval_index) && ...
        clearResult.certified_clear_intervals==2 && ...
        ~clearResult.safe_to_commit);
    R.cases=R.cases+1;
    emptyCase=clearCase;
    emptyCase.observed_ids=zeros(1,0);
    emptyCase.nominal_clearance_lower_m=zeros(0,3);
    emptyResult=TRIGGER_EKF_INTERVAL_RISK_DECISION(emptyCase);
    assert(~emptyResult.conditional_replan_required && ...
        strcmp(emptyResult.status,'NO_OBSERVED_OR_RETAINED_OBSTACLES__UNKNOWN_WORLD') && ...
        isempty(emptyResult.earliest_possible_ids) && ...
        isnan(emptyResult.first_interval_index) && ...
        ~emptyResult.safe_to_commit);
    R.cases=R.cases+1;
    % Zero-width and tangency are UNKNOWN. No false strict clearance.
    C=struct('sample_times_s',[0 1], ...
        'observed_ids',[41 7 95], ...
        'nominal_clearance_lower_m',[1 1;2 2;10 10], ...
        'full_body_endpoint_error_upper_m',[0 0], ...
        'whole_body_speed_upper_mps',4);
    P=TRIGGER_EKF_INTERVAL_RISK_DECISION(C);
    assert(isequal(P.earliest_possible_ids,[41 7]) && ...
        P.obstacle_id==41 && P.first_interval_index==1 && ...
        ~P.safe_to_commit);
    R.cases=R.cases+1;
    % Arbitrary IDs do not imply any mask enumeration or fixed count.
    C.observed_ids=[11 12 13];
    C.sample_times_s=[0 0.1 0.2];
    C.nominal_clearance_lower_m=zeros(3,3);
    C.full_body_endpoint_error_upper_m=[0 0 0];
    C.whole_body_speed_upper_mps=[0 0];
    P=TRIGGER_EKF_INTERVAL_RISK_DECISION(C);
    assert(isequal(P.earliest_possible_ids,C.observed_ids));
    R.cases=R.cases+1;
    C.full_body_endpoint_error_upper_m=[0 -1 0];
    P=TRIGGER_EKF_INTERVAL_RISK_DECISION(C);
    assert(P.conditional_replan_required && ...
        strcmp(P.status,'UNKNOWN_INVALID_INTERVAL_RISK_INPUT') && ...
        isempty(P.earliest_possible_ids) && ...
        isnan(P.first_interval_index) && ...
        ~P.safe_to_commit);
    R.cases=R.cases+1;
    R.status='PASS_EARLIEST_SIMULTANEOUS_IDS__CONDITIONAL_ONLY';
catch ME
    R.failures=1;
    R.reason=[ME.identifier ' | ' ME.message];
end
try
    p=fileparts(mfilename('fullpath'));
    fid=fopen(fullfile(p,'TRIGGER_RISK_MULTI_ID_RESULT.txt'),'wt');
    if fid<0,error('CHECK_TRIGGER_RISK_MULTI_ID:Write','TXT unavailable');end
    cl=onCleanup(@()fclose(fid));
    fprintf(fid,'status=%s\ncases=%d\nfailures=%d\nreason=%s\n', ...
        R.status,R.cases,R.failures,R.reason);
    clear cl
    save(fullfile(p,'TRIGGER_RISK_MULTI_ID_RESULT.mat'),'R');
catch ME
    warning('CHECK_TRIGGER_RISK_MULTI_ID:Save','%s',ME.message);
end
end

function [first_k,mask]=reference_oracle(t,d,e,v)
% Independent vectorized mathematical oracle with extra local padding.
% The tested code expands each primitive result by exactly one adjacent
% float; use the same directed operations here without calling production
% control-flow code, so all tie/near-tangent semantics are tested.
K=numel(v);n=size(d,1);
first_k=NaN;mask=false(1,n);
for k=1:K
    h=TRIGGER_FLOAT_OUTWARD_ONE_ULP(t(k+1)-t(k),+1);
    travel=TRIGGER_FLOAT_OUTWARD_ONE_ULP(v(k)*h,+1);
    d0=max(0,TRIGGER_FLOAT_OUTWARD_ONE_ULP(d(:,k)-e(k),-1));
    d1=max(0,TRIGGER_FLOAT_OUTWARD_ONE_ULP(d(:,k+1)-e(k+1),-1));
    sep=TRIGGER_FLOAT_OUTWARD_ONE_ULP(d0+d1,-1)>travel;
    if any(~sep)
        first_k=k;
        mask=reshape(~sep,1,[]);
        return
    end
end
end
