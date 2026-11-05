function R=TRIGGER_TAUT_SOURCE_INVARIANTS()
% Exact POLYNOMIAL identity for the original symbolic 24-state taut ODE.
% This audit is not a proof of initial tautness, positive tension, real
% model fidelity, or EKF error. No Flight side effects, no toolbox needed.
%
% State order [q,Euler,v,wq,l,vl,pT,wL]; original source aliases:
% t33 = w1*p2 - w2*p1; t34 = w1*p3 - w3*p1;
% t35 = w2*p3 - w3*p2;  pTdot=[t35;-t34;t33];
% a=t81/(L*m), b=t82/(L*m), c=(t71-t78)/(L*m);
% wLdot=[-b;-c;a].  The original RHS has
%   (vl_dot-v_dot)/L = wLdot CROSS pT + wL CROSS pTdot.
% The source expressions are checked as exact 9-var polynomials below.
% Consequently, at CONSTANT physical cable length L,
% c0 = l-q-L*pT, c1=vl-v-L*(wL CROSS pT):
%   c0dot=c1, c1dot=0, d/dt(pT'*pT)=0
% for every differentiable ODE solution (also a.e. under measurable inputs).
% If c0=c1=0 and pT'pT=1 INITIALLY, these identities preserve them.
% The source-pinned claim is false if any source file changed.
R=struct('status','UNKNOWN_TAUT_SOURCE_INVARIANTS','reason','', ...
    'algebra_exact',false,'unit_invariant_exact',false, ...
    'position_velocity_invariant_exact',false, ...
    'original_rhs_source_pinned',false,'interval_wrapper_source_pinned',false, ...
    'source_bound_invariance',false,'constant_length_required',true, ...
    'initial_on_manifold_proven',false,'positive_tension_proven',false, ...
    'hybrid_physical_mode_proven',false,'safe_to_commit',false);
try
    v=cell(1,9);
    for j=1:9,v{j}=variable(j);end
    p=v(1:3);w=v(4:6);a=v{7};b=v{8};c=v{9};
    pd=crosspoly(w,p);
    wdot={minuspoly(zero(),b),minuspoly(zero(),c),a};
    t33=pd{3};t34=minuspoly(zero(),pd{2});t35=pd{1};
    % Independently transcribed EXACT source acceleration difference / L.
    source={minuspoly(minuspoly(pluspoly(timespoly(w{2},t33), ...
               timespoly(w{3},t34)),timespoly(p{2},a)),timespoly(p{3},c)), ...
        pluspoly(pluspoly(minuspoly(timespoly(w{3},t35), ...
               timespoly(w{1},t33)),timespoly(p{1},a)),timespoly(p{3},b)), ...
        pluspoly(minuspoly(minuspoly(zero(),timespoly(w{1},t34)), ...
               timespoly(w{2},t35)),minuspoly(timespoly(p{1},c),timespoly(p{2},b)))};
    predicted=addvector(crosspoly(wdot,p),crosspoly(w,pd));
    for j=1:3
        assert(iszero(minuspoly(source{j},predicted{j})), ...
            'TRIGGER_TAUT_SOURCE_INVARIANTS:AccelerationIdentity', ...
            'The original acceleration-difference identity is not exact.');
    end
    unit=zero();
    for j=1:3,unit=pluspoly(unit,timespoly(p{j},pd{j}));end
    assert(iszero(unit), ...
        'TRIGGER_TAUT_SOURCE_INVARIANTS:UnitIdentity', ...
        'The source pT derivative does not preserve unit length.');
    R.algebra_exact=true;R.unit_invariant_exact=true;
    R.position_velocity_invariant_exact=true;
    % The verified interval mirror uses exact real algebraic factoring of
    % 3 inertia RHS coordinates and unit-rotation coefficient intersection.
    % Both transformations are enclosure-preserving. This source pin
    % invalidates this invariant proof if the mirror changes again.
    % This is a cryptographic SOURCE PIN, not a claim that a code hash
    % alone proves the physical plant exactly obeys this model.
    rhs=which('with_load_model_euler_for_HL');
    wrapper=which('TRIGGER_ORIGINAL_PLANT_INTERVAL_RHS');
    R.original_rhs_source_pinned=sha_matches(rhs, ...
        '183837c484f6ec8953998aacb20b93120ce0d6fde647c0e54bcc27f5ae02164a');
    R.interval_wrapper_source_pinned=sha_matches(wrapper, ...
        'bb56978faa7d7386ed91007d4d69e3ed4c4a8a79af538749bfdc789c953d5542');
    R.source_bound_invariance=R.algebra_exact && ...
        R.original_rhs_source_pinned && R.interval_wrapper_source_pinned;
    if R.source_bound_invariance
        R.status='PASS_SOURCE_PINNED_EXACT_TAUT_ODE_INVARIANCE__INITIAL_MODE_OPEN';
        R.reason=['Exact 9-variable polynomial identities prove the original pinned ' ...
            'taut ODE preserves initial manifold constraints for constant L. ' ...
            'True initial constraints, physical mode, tension and numeric/physical ' ...
            'source validity are not proven.'];
    else
        R.status='UNKNOWN_TAUT_SOURCE_HASH_NOT_MATCHED';
        R.reason=['Algebraic identity holds, but actual physical RHS source and ' ...
            'interval mirror must BOTH match byte-pinned versions.'];
    end
catch ME
    R.status='FAIL_TAUT_SYMBOLIC_IDENTITY';
    R.reason=[ME.identifier ' | ' ME.message];
end
end
function yes=sha_matches(path,expected)
yes=false;
if isempty(path)||~isfile(path),return;end
fid=fopen(path,'rb');
if fid<0,return;end
clean=onCleanup(@()fclose(fid)); %#ok<NASGU>
bytes=fread(fid,inf,'*uint8');
md=java.security.MessageDigest.getInstance('SHA-256');
md.update(typecast(bytes,'int8'));
raw=typecast(int8(md.digest()),'uint8');
got=lower(reshape(dec2hex(raw,2).',1,[]));
yes=strcmp(got,expected);
end
% Formal multivariate polynomial ring over integer coefficients.
% A key encodes exponents of independent p(1:3), w(1:3), a,b,c.
% All operations here are exact integer arithmetic, not sampled tests.
function A=zero()
A=containers.Map('KeyType','char','ValueType','double');
end
function A=variable(i)
A=zero();e=zeros(1,9);e(i)=1;A(char(48+e))=1;
end
function A=pluspoly(X,Y)
A=zero();xx=keys(X);yy=keys(Y);
for j=1:numel(xx),key=xx{j};A(key)=X(key);end
for j=1:numel(yy)
    key=yy{j};coeff=Y(key);
    if isKey(A,key),coeff=coeff+A(key);end
    assert(coeff==fix(coeff) && abs(coeff)<2^40);
    if coeff==0
        if isKey(A,key),remove(A,key);end
    else
        A(key)=coeff;
    end
end
end
function A=minuspoly(X,Y)
A=pluspoly(X,scalepoly(Y,-1));
end
function A=scalepoly(X,a)
A=zero();kk=keys(X);
for j=1:numel(kk)
    val=a*X(kk{j});
    if val~=0,A(kk{j})=val;end
end
end
function A=timespoly(X,Y)
A=zero();xx=keys(X);yy=keys(Y);
for i=1:numel(xx)
    for j=1:numel(yy)
        key=char(48+double(xx{i}-'0')+double(yy{j}-'0'));
        assert(all(key>='0' & key<='9'), ...
            'TRIGGER_TAUT_SOURCE_INVARIANTS:PolynomialDegree');
        c=X(xx{i})*Y(yy{j});
        if isKey(A,key),c=c+A(key);end
        if c==0
            if isKey(A,key),remove(A,key);end
        else
            assert(c==fix(c) && abs(c)<2^40);
            A(key)=c;
        end
    end
end
end
function A=crosspoly(x,y)
A={minuspoly(timespoly(x{2},y{3}),timespoly(x{3},y{2})), ...
   minuspoly(timespoly(x{3},y{1}),timespoly(x{1},y{3})), ...
   minuspoly(timespoly(x{1},y{2}),timespoly(x{2},y{1}))};
end
function A=addvector(x,y)
A=cell(1,3);
for k=1:3,A{k}=pluspoly(x{k},y{k});end
end
function yes=iszero(A)
yes=(A.Count==0);
end
