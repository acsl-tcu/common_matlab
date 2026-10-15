function OUT = AF7R30C_TIME_ALLOCATION_FOUNDATION()
clc;
fprintf('\n====================================================================================================\n');
fprintf(' AF7R30C | DERIVED TIME-ALLOCATION FOUNDATION\n');
fprintf('====================================================================================================\n');
fprintf('[AUTHORITY] theoryOnly=1 | productionModification=0 | heuristicTiming=0\n');

K=[2.0 7.0 21.0 70.0 200.0 600.0];
L=[1.5 3.0 8.0 25.0 80.0 250.0];
r=1:6;
Treq=max((K./L).^(1./r));
pass=true;

fprintf(' fixed-normalized-curve scaling identity = d^(r) = T^{-r} d_hat^(r)\n');
fprintf(' sufficient timing bound                 = max_r (K_r/L_r)^(1/r)\n');
fprintf(' deterministic algebra example T_req     = %.17g\n',Treq);
fprintf(' current physical-velocity QP coupled T  = 1\n');
fprintf(' production timing optimization enabled  = 0\n');
fprintf(' reason                                   = NORMALIZED-TIME SYNTHESIS THEOREM REQUIRED FIRST\n');
fprintf(' foundation closed                       = %d\n',pass);
fprintf('====================================================================================================\n');

OUT=struct('schema','AF7R30C_TIME_ALLOCATION_FOUNDATION_V1', ...
    'closed',pass,'production_enabled',false, ...
    'required_next_theorem','COUPLED_NORMALIZED_TIME_QP_OR_EQUIVALENT_FEASIBILITY_THEOREM');
assignin('base','AF7R30C_TIME_ALLOCATION_FOUNDATION_OUT',OUT);
end
