function R=AF7R58R45R1_RUN()
% Real MAT source-bound, non-mutating, two-candidate comparison.
root=fileparts(mfilename('fullpath'));
S=fullfile(root,'AF7R58R44R2_ACTIVE_SNAPSHOT.mat');
N=fullfile(root,'AF7R58R45_NUMERICAL_RETURN_CANDIDATE.mat');
B=fullfile(root,'AF7R58R45_REBUILT_RETURN.mat');
assert(isfile(S),'[R45R1] Upload or retain ACTUAL snapshot next to m files.');
R=struct();
R.terminal=AF7R58R45R1_TERMINAL_AUDIT(S);
R.numerical_candidate_available=isfile(N);
R.rebuilt_candidate_available=isfile(B);
if isfile(N)
    R.numerical=AF7R58R45_VERIFY_ACTUAL_RETURN(S,N);
    fprintf('[R45R1] NUMERICAL candidate C7 start/internal/end max = %.6g / %.6g / %.6g\n', ...
        max(R.numerical.C7_start_residual_by_order), ...
        max(R.numerical.C7_internal_residual_by_order), ...
        max(R.numerical.C7_terminal_combined_residual_by_order));
end
if isfile(B)
    R.rebuilt=AF7R58R45_VERIFY_ACTUAL_RETURN(S,B);
    fprintf('[R45R1] REBUILT candidate C7 start/internal/end max = %.6g / %.6g / %.6g\n', ...
        max(R.rebuilt.C7_start_residual_by_order), ...
        max(R.rebuilt.C7_internal_residual_by_order), ...
        max(R.rebuilt.C7_terminal_combined_residual_by_order));
end
R.production_modified=false;
R.wholebody_collision_certified=false;
end
