function R=TRIGGER_ORIGINAL_ANGULAR_STRICT_TUBE(D)
% Verified triangular reduction of the SOURCE-PINNED original plant.
% Original states 10:12 form a closed 3D Euler rigid-body subsystem:
% w1'=(M1+(Jy-Jz)*w2*w3)/Jx
% w2'=(M2-(Jx-Jz)*w1*w3)/Jy
% w3'=(M3+(Jx-Jy)*w1*w2)/Jz.
% NO nominal/EKF/truth sample is read. U and J must enclose the actual
% values over the WHOLE time interval, including any switches.
%
% Each angular proof slab checks the strict Picard inclusion
% W0 + [0,h] F(W,U,J) subset int(W), using outward interval operations.
% Dyadic subdivision is only a proof search strategy (not a model change).
% A certified all-time angular hull is later used to bound the OTHER 21
% original state coordinates via their original 24D interval RHS.
% Endpoint time uncertainty [h_lower_s,h_s] is enclosed by backward
% derivative displacement from the rigorously bounded upper-time endpoint.
R=struct('status','UNKNOWN_ANGULAR_NOT_PROVEN','reason','', ...
    'conditional_strict_inclusion',false, ...
    'tube_lower',nan(3,1),'tube_upper',nan(3,1), ...
    'endpoint_lower',nan(3,1),'endpoint_upper',nan(3,1), ...
    'subslabs',0,'attempts',0,'minimum_margin',NaN, ...
    'source_pinned',false,'safe_to_commit',false);
try
    need={'x0_lower','x0_upper','u_lower','u_upper', ...
        'param_lower','param_upper','h_s'};
    assert(isstruct(D)&&isscalar(D)&&all(isfield(D,need)), ...
        'TRIGGER_ORIGINAL_ANGULAR_STRICT_TUBE:Fields', ...
        'Original initial state, input, parameter and duration boxes required.');
    xlo=double(D.x0_lower(:));xhi=double(D.x0_upper(:));
    ulo=double(D.u_lower(:));uhi=double(D.u_upper(:));
    plo=double(D.param_lower(:));phi=double(D.param_upper(:));
    assert(isequal(size(xlo),[24 1])&&isequal(size(xhi),[24 1]) && ...
        isequal(size(ulo),[4 1])&&isequal(size(uhi),[4 1]) && ...
        isequal(size(plo),[21 1])&&isequal(size(phi),[21 1]) && ...
        all(isfinite([xlo;xhi;ulo;uhi;plo;phi])) && ...
        all(xlo<=xhi)&&all(ulo<=uhi)&&all(plo<=phi) && ...
        all(plo(6:8)>0), ...
        'TRIGGER_ORIGINAL_ANGULAR_STRICT_TUBE:InputBox', ...
        'Finite ordered state, torque and strictly positive inertia boxes required.');
    h=double(D.h_s);
    if isfield(D,'h_lower_s'),hl=double(D.h_lower_s);else,hl=h;end
    assert(isscalar(h)&&isscalar(hl)&&isreal(h)&&isreal(hl) && ...
        isfinite(h)&&isfinite(hl)&&h>0&&hl>=0&&hl<=h, ...
        'TRIGGER_ORIGINAL_ANGULAR_STRICT_TUBE:Time', ...
        'The actual elapsed time must be contained in [lower,upper].');
    % The coefficient/variable separation is valid ONLY for this precise
    % original 24D plant and its currently pinned interval mirror.
    S=TRIGGER_TAUT_SOURCE_INVARIANTS();
    assert(S.source_bound_invariance && S.algebra_exact, ...
        'TRIGGER_ORIGINAL_ANGULAR_STRICT_TUBE:Source', ...
        'Current original 24D source identity is not pinned.');
    R.source_pinned=true;
    initial_lo=xlo(10:12);initial_hi=xhi(10:12);
    % A bounded 3D-only search (at most 32 slabs); no full 24D
    % refinement is added by this module.
    for lev=0:5
        n=2^lev;
        hs=h/n; % exponent-only scaling by 2^-lev: exact in normal binary64
        if ~(isfinite(hs)&&hs>0&&n*hs==h),break;end
        wlo=initial_lo;whi=initial_hi;
        hull_lo=inf(3,1);hull_hi=-inf(3,1);
        minmargin=inf;
        success=true;
        R.attempts=R.attempts+1;
        for k=1:n
            [ok,Zlo,Zhi,Nlo,Nhi,margin]=one_slab( ...
                wlo,whi,ulo,uhi,plo,phi,hs);
            if ~ok,success=false;break;end
            hull_lo=min(hull_lo,Zlo);hull_hi=max(hull_hi,Zhi);
            wlo=Nlo;whi=Nhi;
            minmargin=min(minmargin,margin);
        end
        if ~success,continue;end
        % The true elapsed time can be slightly LESS than the upper-time
        % integration horizon. Undo the terminal part by a bounded backward
        % integral, not by treating the upper duration as exact.
        f=angular_rhs(hull_lo,hull_hi,ulo,uhi,plo,phi);
        delta=h-hl;
        assert(isfinite(delta)&&delta>=0, ...
            'TRIGGER_ORIGINAL_ANGULAR_STRICT_TUBE:Delta');
        if delta>0
            delta=TRIGGER_FLOAT_OUTWARD_ONE_ULP(delta,+1);
        end
        ELo=wlo;EHi=whi;
        for j=1:3
            term=TRIGGER_OUTWARD_INTERVAL(0,delta).*f{j};
            back=TRIGGER_OUTWARD_INTERVAL(wlo(j),whi(j))-term;
            ELo(j)=back.lo;EHi(j)=back.hi;
        end
        assert(all(isfinite(ELo))&&all(isfinite(EHi)) && ...
            all(ELo<=EHi),'TRIGGER_ORIGINAL_ANGULAR_STRICT_TUBE:Endpoint');
        R.conditional_strict_inclusion=true;
        R.tube_lower=hull_lo;R.tube_upper=hull_hi;
        R.endpoint_lower=ELo;R.endpoint_upper=EHi;
        R.subslabs=n;R.minimum_margin=minmargin;
        R.status='CONDITIONAL_SOURCE_PINNED_3D_ANGULAR_FLOWPIPE';
        R.reason=['All-time angular subsystem strictly enclosed on ' ...
            'every slab. The other 21 original states and physical model ' ...
            'still require independent proof.'];
        return
    end
    R.status='UNKNOWN_ANGULAR_ALL_REFINEMENTS_REJECTED';
    R.reason=['No source-pinned strict angular tube found within the ' ...
        'bounded 3D proof-search budget.'];
catch ME
    R.status='UNKNOWN_ANGULAR_INPUT_SOURCE_OR_ARITHMETIC';
    R.reason=[ME.identifier ' | ' ME.message];
end
end

function [ok,zlo,zhi,nlo,nhi,margin]=one_slab(lo,hi,ul,uu,pl,pu,h)
ok=false;
zlo=nan(3,1);zhi=nan(3,1);nlo=nan(3,1);nhi=nan(3,1);
margin=NaN;
f0=angular_rhs(lo,hi,ul,uu,pl,pu);
for j=1:3
    x=TRIGGER_OUTWARD_INTERVAL(lo(j),hi(j));
    image=x+TRIGGER_OUTWARD_INTERVAL(0,h).*f0{j};
    zlo(j)=min(lo(j),image.lo);
    zhi(j)=max(hi(j),image.hi);
end
[zlo,zhi]=inflate3(zlo,zhi);
for it=1:14
    f=angular_rhs(zlo,zhi,ul,uu,pl,pu);
    flo=zeros(3,1);fhi=zeros(3,1);
    tlo=zeros(3,1);thi=zeros(3,1);
    for j=1:3
        x=TRIGGER_OUTWARD_INTERVAL(lo(j),hi(j));
        flow=x+TRIGGER_OUTWARD_INTERVAL(0,h).*f{j};
        endpoint=x+TRIGGER_OUTWARD_INTERVAL(h,h).*f{j};
        flo(j)=flow.lo;fhi(j)=flow.hi;
        tlo(j)=endpoint.lo;thi(j)=endpoint.hi;
    end
    % Comparing already-outward representable bounds avoids ambiguous
    % cancellation in difference-of-bound margin arithmetic.
    if all(flo>zlo)&&all(fhi<zhi)
        nlo=tlo;nhi=thi;
        margin=min([flo-zlo;zhi-fhi]);
        ok=true;
        return
    end
    [zlo,zhi]=inflate3(min(zlo,flo),max(zhi,fhi));
end
end

function f=angular_rhs(lo,hi,ul,uu,pl,pu)
w=cell(3,1);J=cell(3,1);M=cell(3,1);
for j=1:3
    w{j}=TRIGGER_OUTWARD_INTERVAL(lo(j),hi(j));
    J{j}=TRIGGER_OUTWARD_INTERVAL(pl(j+5),pu(j+5));
    M{j}=TRIGGER_OUTWARD_INTERVAL(ul(j+1),uu(j+1));
end
f=cell(3,1);
f{1}=(M{1}+(J{2}-J{3}).*w{2}.*w{3})./J{1};
f{2}=(M{2}-(J{1}-J{3}).*w{1}.*w{3})./J{2};
f{3}=(M{3}+(J{1}-J{2}).*w{1}.*w{2})./J{3};
end

function [lo,hi]=inflate3(lo,hi)
width=hi-lo;
assert(all(isfinite([lo;hi;width]))&&all(width>=0), ...
    'TRIGGER_ORIGINAL_ANGULAR_STRICT_TUBE:Overflow');
pad=max(0.2*width,16*eps(max(max(abs(lo),abs(hi)),1)));
for j=1:3
    a=TRIGGER_OUTWARD_INTERVAL(lo(j),lo(j))- ...
        TRIGGER_OUTWARD_INTERVAL(pad(j),pad(j));
    b=TRIGGER_OUTWARD_INTERVAL(hi(j),hi(j))+ ...
        TRIGGER_OUTWARD_INTERVAL(pad(j),pad(j));
    lo(j)=a.lo;hi(j)=b.hi;
end
assert(all(isfinite([lo;hi]))&&all(lo<hi), ...
    'TRIGGER_ORIGINAL_ANGULAR_STRICT_TUBE:NoInterior');
end
