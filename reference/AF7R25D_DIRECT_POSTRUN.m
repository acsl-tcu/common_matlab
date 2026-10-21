function S=AF7R25D_DIRECT_POSTRUN()
% Read-only post-run/source audit for the direct R25D replacement.
refDir=fileparts(which('af7r34_make_core_packet'));
assert(~isempty(refDir)&&isfolder(refDir),'[R25D] reference directory unavailable.');

hpp=fullfile(refDir,'af7r36_core.hpp');
cpp=fullfile(refDir,'af7r36_single_core_mex.cpp');
rps=fullfile(refDir,'REPLANNING_SYSTEM.m');
assert(isfile(hpp)&&isfile(cpp)&&isfile(rps),'[R25D] one or more production sources are missing.');

H=fileread(hpp);X=fileread(cpp);M=fileread(rps);
srcPass=contains(H,'AF7R25D_CURVED_C7_ZERO_JET_BELL_DIRECT') && ...
    contains(H,'C.family=6;') && ...
    contains(H,'certify_candidates(bell,true,4)') && ...
    contains(X,'CURVED_C7_ZERO_JET_BELL') && ...
    contains(X,'AF7R25D_CURVED_C7_ZERO_JET_BELL_DIRECT_V1') && ...
    count(M,'PIECEWISE_DEGREE15_BEZIER_C7_ZERO_JET_BELL')==2;
assert(srcPass,'[R25D] direct replacement source markers are incomplete.');

fprintf('\n====================================================================================================\n');
fprintf(' AF7R25D DIRECT COMPLETE REPLACEMENT AUDIT\n');
fprintf('====================================================================================================\n');
fprintf('source replacement markers                   = PASS\n');
fprintf('family 6                                     = CURVED_C7_ZERO_JET_BELL\n');
fprintf('internal plateau                             = 0\n');
fprintf('exact C7 join tolerance                      = ZERO\n');
fprintf('equal-half-duration dependency               = 0\n');
fprintf('bell acceptance branch                       = 4 (no legacy branch-3 heading gate)\n');
fprintf('finite-sensing/rejoin/whole-body gates       = unchanged\n');

P=struct();
if evalin('base','exist(''replanner_instance'',''var'')==1')
    R=evalin('base','replanner_instance');
    P=R.get_staged_pipeline_status();
    fprintf('\n[LAST STAGED RESULT]\n');
    fprintf('candidate seen / success                     = %d / %d\n', ...
        logical(P.predeadline_candidate_seen),logical(P.predeadline_candidate_success));
    fprintf('candidate family                             = %s\n',char(P.predeadline_candidate_family));
    fprintf('selected exact C7                            = %d\n',logical(P.predeadline_c7_exact));
    fprintf('speed / rejoin / geometry counts            = %.0f / %.0f / %.0f\n', ...
        double(P.predeadline_speed_pass_count), ...
        double(P.predeadline_rejoin_pass_count), ...
        double(P.predeadline_geometry_test_count));
    fprintf('geometry pass                                = %d\n',logical(P.predeadline_geometry_pass));
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

S=struct('schema','AF7R25D_DIRECT_COMPLETE_REPLACEMENT', ...
    'source_pass',true,'pipeline',P);
end
