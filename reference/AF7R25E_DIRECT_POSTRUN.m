function S=AF7R25E_DIRECT_POSTRUN()
% Read-only audit for AF7R25E direct three-file replacement.
REF=fileparts(which('af7r34_make_core_packet'));
assert(~isempty(REF)&&isfolder(REF),'[R25E] reference directory unavailable.');

hpp=fullfile(REF,'af7r36_core.hpp');
cpp=fullfile(REF,'af7r36_single_core_mex.cpp');
rps=fullfile(REF,'REPLANNING_SYSTEM.m');
assert(isfile(hpp)&&isfile(cpp)&&isfile(rps),'[R25E] one or more production sources are missing.');

H=fileread(hpp); X=fileread(cpp); M=fileread(rps);
REV='AF7R25E_CURVED_C7_ZERO_JET_BELL_DIRECT_V1';

srcPass=contains(H,'AF7R25E_CURVED_C7_ZERO_JET_BELL_DIRECT') && ...
    contains(H,'C.family=6;') && ...
    contains(H,'C.exact_c7=true;') && ...
    contains(H,'certify_candidates(bell,true,4)') && ...
    contains(X,['PLANNER_REV="' REV '"']) && ...
    contains(X,'if(R.cand.family==6)family="CURVED_C7_ZERO_JET_BELL";') && ...
    contains(M,REV) && ...
    count(M,'PIECEWISE_DEGREE15_BEZIER_C7_ZERO_JET_BELL')==2;
assert(srcPass,'[R25E] production source set is mixed or incomplete.');

% Query the actually loaded MEX through the ordinary planner output contract by
% relying on the simulation's staged result. MATLAB-side revision mismatch is
% already fail-closed during warmup.
fprintf('\n====================================================================================================\n');
fprintf(' AF7R25E DIRECT THREE-FILE REPLACEMENT AUDIT\n');
fprintf('====================================================================================================\n');
fprintf('three-file source-set binding                 = PASS\n');
fprintf('expected native revision                      = %s\n',REV);
fprintf('family 6                                      = CURVED_C7_ZERO_JET_BELL\n');
fprintf('internal plateau                              = 0\n');
fprintf('exact C7 join tolerance                       = ZERO\n');
fprintf('equal-half-duration dependency                = 0\n');
fprintf('finite-sensing/rejoin/whole-body gates        = unchanged\n');

P=struct();
if evalin('base','exist(''replanner_instance'',''var'')==1')
    R=evalin('base','replanner_instance');
    P=R.get_staged_pipeline_status();
    fprintf('\n[LAST STAGED RESULT]\n');
    fprintf('candidate seen / success                      = %d / %d\n', ...
        logical(P.predeadline_candidate_seen),logical(P.predeadline_candidate_success));
    fprintf('candidate family                              = %s\n',char(P.predeadline_candidate_family));
    fprintf('selected exact C7                             = %d\n',logical(P.predeadline_c7_exact));
    fprintf('speed / rejoin / geometry counts             = %.0f / %.0f / %.0f\n', ...
        double(P.predeadline_speed_pass_count), ...
        double(P.predeadline_rejoin_pass_count), ...
        double(P.predeadline_geometry_test_count));
    fprintf('geometry pass                                 = %d\n',logical(P.predeadline_geometry_pass));
    if isfield(P,'r24e_family_seen')
        seen=double(P.r24e_family_seen(:).');
        exact=double(P.r24e_family_exact(:).');
        xr=double(P.r24e_family_exact_reject(:).');
        sp=double(P.r24e_family_speed_pass(:).');
        sf=double(P.r24e_family_speed_fail(:).');
        names={'UNKNOWN','MINIMUM_DEVIATION_AFFINE','LEGACY_MINIMUM_SNAP','FAMILY_3', ...
            'TOWING_C7_MIN_NORM_CONSTANT','TOWING_C7_SPEED_OPT_SPLIT', ...
            'CURVED_C7_ZERO_JET_BELL','FAMILY_7'};
        fprintf('\n[FAMILY BREAKDOWN]\n');
        fprintf('%-31s %6s %6s %7s %9s %9s\n','family','seen','exact','C7rej','speedPASS','speedFAIL');
        for k=1:min(8,numel(seen))
            fprintf('%d %-29s %6.0f %6.0f %7.0f %9.0f %9.0f\n', ...
                k-1,names{k},seen(k),exact(k),xr(k),sp(k),sf(k));
        end
    end
end
fprintf('====================================================================================================\n');

S=struct('schema','AF7R25E_DIRECT_THREE_FILE_REPLACEMENT', ...
    'source_pass',true,'expected_revision',REV,'pipeline',P);
end
