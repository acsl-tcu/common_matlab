function R=CHECK_TRIGGER_UNIVERSAL_TENSION_OBSTRUCTION()
% Mathematical NO-GO theorem for a strict-positive-taut certificate from
% ONLY the original universal saturated input interval.
% N=(mL/(m+mL))*(L*m*||omegaL x pT||^2-u1*D).
% The admissible example u1=0, omegaL=0, pT=[0;0;-1]
% has N=0 for all m>0, mL>0, L>0 and all orientations.
% This is a proof-by-counterexample, NOT a model/sensor failure or a
% numerical Picard convergence issue.
R=struct('status','FAIL_TENSION_COUNTEREXAMPLE','cases',0,'failures',0, ...
    'source_hlc_pinned',false,'zero_thrust_in_universal_box',false, ...
    'strict_positive_tension_from_universal_box_impossible',false, ...
    'reason','');
try
    H=TRIGGER_ORIGINAL_HLC_HOLD_ENVELOPE([0 4]);
    check(H.conditional_saturated_hold && ...
        H.universal_input_valid_on_arbitrary_partition, ...
        'Original source HLC bounds are not confirmed');
    R.source_hlc_pinned=true;
    check(isequal(size(H.lower),[4 1]) && ...
        H.lower(1)<=0 && H.upper(1)>=0, ...
        'Thrust zero not represented by universal HLC box');
    R.zero_thrust_in_universal_box=true;
    masses=[0.2 0.7;0.5 0.14;2.0 10.0;1.5 0.1];
    for k=1:size(masses,1)
        m=masses(k,1);mL=masses(k,2);L=2.0;
        omegaL=zeros(3,1);pT=[0;0;-1];u1=0;
        D=1;
        N=mL/(m+mL)*(L*m*norm(cross(omegaL,pT))^2-u1*D);
        check(N==0 && m>0 && mL>0 && L>0, ...
            'Free-fall taut source model has zero tension');
    end
    R.strict_positive_tension_from_universal_box_impossible=true;
    R.status='PASS_UNIVERSAL_HLC_BOX_CANNOT_PROVE_STRICT_TENSION';
    R.reason=['Counterexample u1=0, omegaL=0 gives N=0 exactly. ' ...
        'A future 4-second taut certificate requires stronger closed-loop ' ...
        'bounds or a model including slack. Retuning Picard cannot fix this.'];
catch ME
    R.failures=R.failures+1;
    R.reason=[ME.identifier ' | ' ME.message];
end
    try
        save('TRIGGER_UNIVERSAL_TENSION_OBSTRUCTION_RESULT.mat','R');
        f=fopen('TRIGGER_UNIVERSAL_TENSION_OBSTRUCTION_RESULT.txt','w');
        if f>=0
            fprintf(f,'%s\ncases=%d failures=%d\n%s\n', ...
                R.status,R.cases,R.failures,R.reason);
            fclose(f);
        end
    catch
    end
    function check(ok,description)
        R.cases=R.cases+1;
        assert(ok,'CHECK_TRIGGER_UNIVERSAL_TENSION_OBSTRUCTION:Failure',description);
    end
end
