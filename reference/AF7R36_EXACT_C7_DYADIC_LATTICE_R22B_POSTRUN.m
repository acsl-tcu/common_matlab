function S=AF7R36_EXACT_C7_DYADIC_LATTICE_R22B_POSTRUN()
% Read-only postrun authority audit for R22.
assert(evalin('base','exist(''replanner_instance'',''var'')==1'));
R=evalin('base','replanner_instance');
P=R.get_staged_pipeline_status();

fprintf('\n====================================================================================================\n');
fprintf(' AF7R36 EXACT C7 DYADIC-LATTICE R22B RESULT\n');
fprintf('====================================================================================================\n');

fprintf('\n[A. LAST PRE-DEADLINE NATIVE CANDIDATE]\n');
fprintf('seen / success                              = %d / %d\n', ...
    logical(P.predeadline_candidate_seen),logical(P.predeadline_candidate_success));
fprintf('family                                      = %s\n',char(P.predeadline_candidate_family));
if isfield(P,'predeadline_c7_exact')
    fprintf('native stored exact C7                      = %d\n',logical(P.predeadline_c7_exact));
else
    fprintf('native stored exact C7                      = NOT EXPOSED\n');
end
fprintf('reason code                                 = %.0f\n',double(P.predeadline_reason_code));
fprintf('speed/rejoin/geometry counts                = %.0f / %.0f / %.0f\n', ...
    double(P.predeadline_speed_pass_count),double(P.predeadline_rejoin_pass_count), ...
    double(P.predeadline_geometry_test_count));
fprintf('geometry pass                               = %d\n',logical(P.predeadline_geometry_pass));
fprintf('Stage1 wall time (record only)              = %.9g ms\n',double(P.last_stage1_ms));

mode=char(R.active_reference_mode);ver=double(R.active_reference_version);
fprintf('\n[B. ACTIVE REFERENCE]\n');
fprintf('mode / version                              = %s / %.0f\n',mode,ver);

exactOrder=NaN;bind='NO_ACTIVE_REPAIRED_REFERENCE';
if strcmpi(mode,'REPAIRED')
    assert(exist('sym','file')==2,'[R22B-POST] Symbolic Math Toolbox required.');
    AR=R.active_repaired_reference;
    AC=R.active_repaired_certificates;
    M=AR.batch_flat_model;

    exactInternal=true(1,8);exactOuter0=true(1,8);exactOuter1=true(1,8);
    G=numel(M.corrections);
    for g=1:G
        C=M.corrections(g);
        s0=double(C.segment_t0(:).');s1=double(C.segment_t1(:).');CP=C.control_points;
        for rr=0:7
            exactOuter0(rr+1)=exactOuter0(rr+1)&&allzero(ep(CP{1},s1(1)-s0(1),rr,true));
            exactOuter1(rr+1)=exactOuter1(rr+1)&&allzero(ep(CP{end},s1(end)-s0(end),rr,false));
        end
        for k=1:numel(CP)-1
            assert(s1(k)==s0(k+1),'[R22B-POST] join times not bitwise identical.');
            for rr=0:7
                d=simplify(ep(CP{k},s1(k)-s0(k),rr,false)-ep(CP{k+1},s1(k+1)-s0(k+1),rr,true));
                exactInternal(rr+1)=exactInternal(rr+1)&&allzero(d);
            end
        end
    end
    allord=exactInternal&exactOuter0&exactOuter1;
    exactOrder=-1;
    for rr=0:7
        if all(allord(1:rr+1)),exactOrder=rr;else,break;end
    end

    cert=false;ord=NaN;
    if isfield(AC,'smoothness')
        Q=AC.smoothness;
        cert=isfield(Q,'certified')&&logical(Q.certified);
        if isfield(Q,'continuity_order'),ord=double(Q.continuity_order);end
    end
    fprintf('certificate C-order                        = %.0f\n',ord);
    fprintf('exact stored continuity order               = C%d\n',exactOrder);
    fprintf('exact internal d0..d7                       = %s\n',bvec(exactInternal));
    fprintf('exact outer-start d0..d7                    = %s\n',bvec(exactOuter0));
    fprintf('exact outer-end d0..d7                      = %s\n',bvec(exactOuter1));

    if cert&&ord>=7&&exactOrder>=7
        bind='CERTIFICATE_AND_EXECUTED_CANONICAL_REPRESENTATION_BOTH_EXACT_C7';
    elseif exactOrder<7
        bind='EXECUTED_CANONICAL_REPRESENTATION_NOT_C7';
    else
        bind='C7_AUTHORITY_INCOMPLETE';
    end

    yawPolicy='';
    if isfield(AR,'yaw_policy'),yawPolicy=char(AR.yaw_policy);end
    yawC6=false;
    if isfield(AC,'yaw_c6')&&isstruct(AC.yaw_c6)&& ...
            isfield(AC.yaw_c6,'certified')
        yawC6=logical(AC.yaw_c6.certified);
    end
    fprintf('yaw policy                                  = %s\n',yawPolicy);
    fprintf('yaw C6 exported                             = %d\n',yawC6);
end

fprintf('\n[C. DECISION]\n');
fprintf('binding classification                      = %s\n',bind);
if isfield(P,'predeadline_c7_exact') && logical(P.predeadline_candidate_success)
    if logical(P.predeadline_c7_exact)
        fprintf('candidate-side exact C7 gate                = PASS\n');
    else
        fprintf('candidate-side exact C7 gate                = FAIL\n');
    end
end
fprintf('NOTE: runtime is recorded only; R22B does not relax timing gates.\n');
fprintf('====================================================================================================\n');

S=struct('schema','AF7_EXACT_C7_DYADIC_LATTICE_R22B', ...
    'pipeline',P,'active_mode',mode,'active_version',ver, ...
    'exact_stored_order',exactOrder,'classification',bind);
end

function y=ep(P,T,r,startSide)
P=sym(double(P),'f');T=sym(double(T),'f');n=15;c=sym(1);
for j=0:r-1,c=c*sym(n-j);end
v=sym(zeros(3,1));
if startSide
    for k=0:r,v=v+sym((-1)^(r-k)*nchoosek(r,k))*P(:,k+1);end
else
    for k=0:r,v=v+sym((-1)^k*nchoosek(r,k))*P(:,16-k);end
end
y=simplify(c*v/(T^r));
end
function tf=allzero(v)
tf=true;for i=1:numel(v),tf=tf&&isequal(simplify(v(i)),sym(0));end
end
function s=bvec(v)
s=['[' strjoin(arrayfun(@(x)sprintf('%d',logical(x)),v,'UniformOutput',false),' ') ']'];
end
