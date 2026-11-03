function D=CHECK_TRIGGER_EXACT_CLOCK_SEMANTICS()
% SOFTWARE-ONLY examination of exact double time-grid semantics.
% No real Flight epoch, obstacle, EKF, nominal function or model is queried.
% This is NOT a green light to switch Flight's reference-time arithmetic.
D=struct('status','FAIL_CLOCK_SEMANTICS_AUDIT','pass',false, ...
    'legacy_shift_hits',0,'recursive_shift_hits',0, ...
    'different_reference_arguments',0,'maximum_time_difference_s',NaN, ...
    'bitwise_original_reference_preserved',false, ...
    'safe_to_modify_flight_clock',false,'reason','');
try
    dt=0.025; t0=5.0; horizon=4.0;
    offsets=(0:floor(horizon/dt))*dt;
    offsets=offsets(offsets<horizon);
    n=numel(offsets);
    assert(n>2 && abs(offsets(end)+dt-horizon)<1e-12);
    t_next=t0+dt; % the same floating operation as iterative time.t
    old0=t0+offsets;
    old1=t_next+offsets;
    rec0=zeros(1,n);rec0(1)=t0;
    rec1=zeros(1,n);rec1(1)=t_next;
    for j=2:n
        rec0(j)=rec0(j-1)+dt;
        rec1(j)=rec1(j-1)+dt;
    end
    % Examine exactly the arguments actually passed to nominal.func:
    % the subtraction of the true Flight epoch is part of the contract.
    oldarg0=old0-t0;oldarg1=old1-t0;
    recarg0=rec0-t0;recarg1=rec1-t0;
    D.legacy_shift_hits=nnz(typecast(oldarg0(2:end),'uint64')== ...
        typecast(oldarg1(1:end-1),'uint64'));
    D.recursive_shift_hits=nnz(typecast(recarg0(2:end),'uint64')== ...
        typecast(recarg1(1:end-1),'uint64'));
    D.different_reference_arguments=nnz(typecast(oldarg0,'uint64')~= ...
        typecast(recarg0,'uint64'));
    D.maximum_time_difference_s=max(abs(old0-rec0));
    D.bitwise_original_reference_preserved=(D.different_reference_arguments==0);
    assert(D.recursive_shift_hits==n-1, ...
        'CHECK_TRIGGER_EXACT_CLOCK_SEMANTICS:RecursiveClock', ...
        'Recursive time values should be exactly shifted for this fixture.');
    assert(D.different_reference_arguments>0, ...
        'CHECK_TRIGGER_EXACT_CLOCK_SEMANTICS:Counterexample', ...
        'Fixture did not expose the original numerical difference.');
    D.pass=true;
    D.status='PASS_CLOCK_REUSE_POSSIBLE__ORIGINAL_TIME_ARGUMENTS_DIFFER__NO_ADOPTION';
catch ME
    D.reason=[ME.identifier ' | ' ME.message];
end
save('TRIGGER_CLOCK_SEMANTICS_RESULT.mat','D');
fid=fopen('TRIGGER_CLOCK_SEMANTICS_RESULT.txt','w');
if fid>=0
    fprintf(fid,'status=%s\nlegacy_shift_hits=%d\nrecursive_shift_hits=%d\n', ...
        D.status,D.legacy_shift_hits,D.recursive_shift_hits);
    fprintf(fid,'different_reference_arguments=%d\nmax_time_delta_s=%.17g\n', ...
        D.different_reference_arguments,D.maximum_time_difference_s);
    fprintf(fid,'bitwise_original_reference_preserved=%d\n', ...
        D.bitwise_original_reference_preserved);
    fprintf(fid,'reason=%s\n',D.reason);fclose(fid);
end
end
