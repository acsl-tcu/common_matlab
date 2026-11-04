function R=CHECK_TRIGGER_INTERVAL_BRIDGE()
% Offline mathematical regression only; does not depend on a Flight/Sim.
R=struct('status','FAIL_INTERVAL_BRIDGE','cases',0,'failures',0, ...
    'speed_cases',0,'risk_cases',0,'lookup_cases',0, ...
    'legacy_lookup_median_ms',NaN,'linear_lookup_median_ms',NaN, ...
    'reason','');
try
    rng(46317,'twister');
    for k=1:360
        n=randi([1 12]);K=randi([1 20]);
        ids=randperm(999,n);
        grid=cumsum([0,0.01+rand(1,K)]);
        mat=rand(n,K)>0.23;
        E=TRIGGER_FIRST_UNPROVED_INTERVAL(grid,ids,mat);
        assert(~E.safe_to_commit);
        bad=find(any(~mat,1),1,'first');
        if isempty(bad)
            assert(~E.conditional_replan_required && ...
                E.conditional_clear_intervals==K);
        else
            correct=ids(~mat(:,bad).');
            assert(E.conditional_replan_required && ...
                isequal(E.earliest_possible_ids,correct) && ...
                E.obstacle_id==correct(1) && ...
                E.first_interval_index==bad && ...
                E.conditional_clear_intervals==bad-1 && ...
                E.first_possible_ttc_left_s==grid(bad)-grid(1) && ...
                E.first_possible_ttc_right_s==grid(bad+1)-grid(1));
        end
        R.risk_cases=R.risk_cases+1;
    end
    for k=1:300
        K=randi([1 15]);x=randn(24,K);
        w=0.02+rand(24,K);
        lo=x-w;hi=x+w;
        S=TRIGGER_TAUT_TRUE_BODY_SPEED_FROM_TUBE(lo,hi);
        assert(S.conditional_bound && ~S.safe_to_commit && ...
            all(S.whole_body_speed_upper_mps>=S.uav_speed_upper_mps) && ...
            all(S.whole_body_speed_upper_mps>=S.payload_speed_upper_mps));
        for j=1:K
            for b=1:2
                if b==1,v=7:9; else,v=16:18;end
                for m=0:7
                    corner=lo(v,j);
                    bit=[bitget(uint8(m),1),bitget(uint8(m),2), ...
                        bitget(uint8(m),3)]>0;
                    corner(bit)=hi(v(bit),j);
                    if b==1,B=S.uav_speed_upper_mps(j);
                    else,B=S.payload_speed_upper_mps(j);end
                    assert(norm(corner)<=B);
                end
            end
        end
        R.speed_cases=R.speed_cases+1;
    end
    % Fail-closed tests for missing evidence and unobserved world.
    empty=TRIGGER_FIRST_UNPROVED_INTERVAL([0 1],[],false(0,1));
    assert(~empty.safe_to_commit && ~empty.conditional_replan_required);
    malformed=TRIGGER_FIRST_UNPROVED_INTERVAL([0 1],[7],true(1,2));
    assert(~malformed.safe_to_commit && malformed.conditional_replan_required);
    for k=1:280
        a=sort(randi(200,1,randi([0 240])));
        b=sort(randi(200,1,randi([0 240])));
        [tf,idx]=TRIGGER_EXACT_SORTED_TIME_LOOKUP(double(a),double(b));
        [expect_t,expect_i]=ismember(double(a),double(b));
        assert(isequal(tf,expect_t) && isequal(idx,expect_i));
        R.lookup_cases=R.lookup_cases+1;
    end
    % Alternating A/B measurement for the exact-time reference fallback.
    % This is wall-clock benchmarking, NOT worst-case execution proof.
    query=double(1:160);known=double(2:161);
    t_old=nan(1,1000);t_new=nan(1,1000);
    for k=1:1000
        if mod(k,2)==1
            z=tic;[aa,bb]=ismember(query,known);t_old(k)=1000*toc(z);
            z=tic;[cc,dd]=TRIGGER_EXACT_SORTED_TIME_LOOKUP( ...
                query,known);t_new(k)=1000*toc(z);
        else
            z=tic;[cc,dd]=TRIGGER_EXACT_SORTED_TIME_LOOKUP( ...
                query,known);t_new(k)=1000*toc(z);
            z=tic;[aa,bb]=ismember(query,known);t_old(k)=1000*toc(z);
        end
        assert(isequal(aa,cc)&&isequal(bb,dd));
    end
    R.legacy_lookup_median_ms=median(t_old);
    R.linear_lookup_median_ms=median(t_new);
    R.cases=R.risk_cases+R.speed_cases+R.lookup_cases;
    R.status='PASS_CONDITIONAL_INTERVAL_BRIDGE__EXTERNAL_PROOF_UNFINISHED';
catch ME
    R.failures=R.failures+1;
    R.reason=[ME.identifier ' | ' ME.message];
end
% Only audit artifacts are written; never to reference/ or the trigger folder.
% The audit resides at common_matlab/ and output follows its physical path,
% independent of MATLAB's current folder.
try
    audit_root=fileparts(mfilename('fullpath'));
    txt_path=fullfile(audit_root,'TRIGGER_INTERVAL_BRIDGE_RESULT.txt');
    mat_path=fullfile(audit_root,'TRIGGER_INTERVAL_BRIDGE_RESULT.mat');
    fid=fopen(txt_path,'wt');
    if fid<0
        error('CHECK_TRIGGER_INTERVAL_BRIDGE:Output', ...
            'Cannot create audit TXT in common_matlab root.');
    end
    close_file=onCleanup(@()fclose(fid));
    fprintf(fid,'status=%s\n',R.status);
    fprintf(fid,'cases=%d\n',R.cases);
    fprintf(fid,'failures=%d\n',R.failures);
    fprintf(fid,'risk_cases=%d\n',R.risk_cases);
    fprintf(fid,'speed_cases=%d\n',R.speed_cases);
    fprintf(fid,'lookup_cases=%d\n',R.lookup_cases);
    fprintf(fid,'legacy_lookup_median_ms=%.9g\n',R.legacy_lookup_median_ms);
    fprintf(fid,'linear_lookup_median_ms=%.9g\n',R.linear_lookup_median_ms);
    fprintf(fid,'reason=%s\n',R.reason);
    clear close_file
    save(mat_path,'R');
catch ME_output
    warning('CHECK_TRIGGER_INTERVAL_BRIDGE:ArtifactWrite', ...
        'Audit result could not be saved to common_matlab: %s',ME_output.message);
end
end
