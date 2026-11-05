function A=CHECK_TRIGGER_SIM_INITIAL_TAUT_SOURCE()
% Verifies that SOURCE-derived Sim t=0 true-set exists and preserves all
% exact taut equalities. No Flight and no altered EKF/plant.
A=struct('status','FAIL_SIM_T0_INITIAL_SOURCE','reason','', ...
    'cases',0,'failures',0,'sim_seed_proven',false, ...
    'source',struct());
try
    S=TRIGGER_SIM_INITIAL_TAUT_SOURCE();A.source=S;
    A.cases=A.cases+1;
    assert(S.simulation_initial_set_proven, ...
        'CHECK_TRIGGER_SIM_INITIAL_TAUT_SOURCE:Source',S.reason);
    A.sim_seed_proven=true;
    x=S.initial_lower;
    A.cases=A.cases+1;
    assert(isequal(size(x),[24 1]) && ...
        isequal(S.initial_upper,x)&&isequal(S.initial_time_s,0), ...
        'CHECK_TRIGGER_SIM_INITIAL_TAUT_SOURCE:Vector');
    A.cases=A.cases+1;
    assert(isequal(x(19:21),[0;0;-1]) && ...
        isequal(x(13:15)-x(1:3),S.cable_length_m*x(19:21))&& ...
        isequal(x(16:18)-x(7:9), ...
            S.cable_length_m*cross(x(22:24),x(19:21))), ...
        'CHECK_TRIGGER_SIM_INITIAL_TAUT_SOURCE:Manifold');
    A.cases=A.cases+1;
    assert(~S.flight_current_ekf_true_set_proven && ...
        ~S.full_4s_physical_safety_proven && ~S.safe_to_commit, ...
        'CHECK_TRIGGER_SIM_INITIAL_TAUT_SOURCE:FalsePhysicalClaim');
    % Connect the source-derived true initial set to the EXISTING
    % 24-state original-EKF posterior tracker (without modifying Flight).
    tracker=TRIGGER_EKF_TRUE_SET_TRACKER();
    E=tracker.seed(0,x,[x x],'SIM_T0_SOURCE_DERIVED_NO_TRUE_PEEK');
    A.cases=A.cases+1;
    assert(E.initialized && isequal(E.time_s,0) && ...
        all(E.error_lower<=0) && all(E.error_upper>=0) && ...
        ~E.actual_closed_loop_proven && ~E.safe_to_commit, ...
        'CHECK_TRIGGER_SIM_INITIAL_TAUT_SOURCE:TrackerSeed');
    A.status='PASS_SIM_T0_INITIAL_24_STATE_AND_TAUT_MANIFOLD__NO_FLIGHT_CLEARANCE';
    A.reason=S.reason;
catch ME
    A.failures=1;A.reason=[ME.identifier ' | ' ME.message];
end
try
    save('TRIGGER_SIM_INITIAL_TAUT_SOURCE_RESULT.mat','A');
    f=fopen('TRIGGER_SIM_INITIAL_TAUT_SOURCE_RESULT.txt','wt');
    if f>=0
        fprintf(f,'%s\ncases=%d failures=%d\n%s\n', ...
            A.status,A.cases,A.failures,A.reason);fclose(f);
    end
catch
end
end
