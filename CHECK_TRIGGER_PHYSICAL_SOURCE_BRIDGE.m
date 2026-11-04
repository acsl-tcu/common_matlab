function R=CHECK_TRIGGER_PHYSICAL_SOURCE_BRIDGE()
% Temporary common-root audit; no Flight, no fake physical safety.
% Checks that the ORIGINAL-source HLC box and physically supplied mass
% change-rate hypothesis are mapped to the original 24-state chain.
root=fileparts(mfilename('fullpath'));
R=struct('status','FAIL_PHYSICAL_SOURCE_BRIDGE','reason','', ...
    'cases',0,'failures',0,'mass_witnesses',0, ...
    'source_hlc_pinned',false,'missing_physics_rejected',false, ...
    'safe_to_commit',false);
try
    saved_rng=rng;cleanup=onCleanup(@()rng(saved_rng)); %#ok<NASGU>
    rng(312891,'twister');
    for z=1:1200
        n=randi([1 6]);
        d=0.002+0.22*rand(1,n);
        t=13+cumsum([0 d]);
        lo=0.3+3*rand();hi=lo+rand();
        beta=0.1*rand();
        T=t(end)-t(1);
        if lo-beta*T<=0.02,beta=0;end
        A=TRIGGER_FUTURE_MASS_RATE_ENCLOSURE(struct( ...
            'time_grid_s',t,'true_mass_now_lower_kg',lo, ...
            'true_mass_now_upper_kg',hi, ...
            'true_mass_rate_bound_kg_s',beta));
        assert(A.conditional_enclosure && ~A.safe_to_commit && ...
            numel(A.true_mass_lower_kg)==n);
        % This includes both worst-case +beta and -beta histories; check
        % points within EVERY actual hold, not only its end timestamp.
        for k=1:n
            for j=1:5
                s=t(k)+(t(k+1)-t(k))*(j-1)/4;
                exactlow=lo-beta*(s-t(1));
                exacthigh=hi+beta*(s-t(1));
                assert(A.true_mass_lower_kg(k)<=exactlow+1e-13 && ...
                    A.true_mass_upper_kg(k)>=exacthigh-1e-13);
                R.mass_witnesses=R.mass_witnesses+1;
            end
        end
        R.cases=R.cases+1;
    end
    invalid=struct('time_grid_s',[0 0.5 1], ...
        'true_mass_now_lower_kg',1,'true_mass_now_upper_kg',1.3, ...
        'true_mass_rate_bound_kg_s',4);
    A=TRIGGER_FUTURE_MASS_RATE_ENCLOSURE(invalid);
    assert(~A.conditional_enclosure&&~A.safe_to_commit);
    R.cases=R.cases+1;
    invalid=rmfield(invalid,'true_mass_rate_bound_kg_s');
    A=TRIGGER_FUTURE_MASS_RATE_ENCLOSURE(invalid);
    assert(~A.conditional_enclosure);
    R.cases=R.cases+1;
    H=TRIGGER_ORIGINAL_HLC_HOLD_ENVELOPE([2 2.01 2.04 2.27]);
    R.source_hlc_pinned=H.conditional_saturated_hold;
    assert(H.conditional_saturated_hold, ...
        'CHECK_TRIGGER_PHYSICAL_SOURCE_BRIDGE:HLCSourceNotPinned', ...
        'Original HLC / MODEL_CLASS source pin failed: %s', H.reason);
    assert(isequal(size(H.lower),[4 3]) && ...
        isequal(H.lower,repmat([0;-1;-1;-1],1,3)) && ...
        isequal(H.upper,repmat([20;1;1;1],1,3)) && ~H.safe_to_commit);
    R.cases=R.cases+1;
    bad=TRIGGER_CLOSED_LOOP_FROM_SOURCE_LIMITS(struct());
    assert(~bad.safe_to_commit && ...
        ~bad.full_physical_4s_safety_proven);
    R.missing_physics_rejected=true;R.cases=R.cases+1;
    R.status='PASS_SOURCE_AND_FUTURE_MASS_ALGEBRA__PHYSICAL_BOUND_STILL_REQUIRED';
catch ME
    R.failures=R.failures+1;
    R.reason=[ME.identifier ' | ' ME.message];
end
save(fullfile(root,'TRIGGER_PHYSICAL_SOURCE_BRIDGE_RESULT.mat'),'R');
f=fopen(fullfile(root,'TRIGGER_PHYSICAL_SOURCE_BRIDGE_RESULT.txt'),'w');
if f>=0
    fprintf(f,'status=%s\ncases=%d\nfailures=%d\nmass_witnesses=%d\nsource_hlc_pinned=%d\nreason=%s\n', ...
        R.status,R.cases,R.failures,R.mass_witnesses, ...
        R.source_hlc_pinned,R.reason);
    fclose(f);
end
end
