function E=TRIGGER_ORIGINAL_ANGULAR_ENERGY_TUBE(D)
%TRIGGER_ORIGINAL_ANGULAR_ENERGY_TUBE Globally bounded, ALL-TIME Euler gyro flow.
% The original source-pinned angular subsystem is
%   J*w' = M - w cross (J*w), J=diag(Jx,Jy,Jz)>0, constant on this slab.
% Hence (d/dt)(w'*J*w)/2 = w'*M EXACTLY: no interval gyro term.
% Writing V = sqrt(w'*J*w) gives D^+ V <= sqrt(M'*inv(J)*M).
% For ANY measurable torque in [u_lower(2:4),u_upper(2:4)],
%   V(t) <= V0_upper + t * G_upper, 0<=t<=h_upper.
% This proves the whole angular tube without an ODE Picard fixed point,
% even when the direct 3D interval Picard fails from wrapping.
%
% The optional interval-RHS intersection is SOUND because the energy tube
% already contains the ENTIRE true angular trajectory; it can only narrow
% that certified tube. Strict Picard inclusion is NOT falsely claimed.
% Real Sim source equality and fixed inertia (within each slab) remain
% necessary. This module NEVER authorizes safe_to_commit.
E=struct('status','UNKNOWN_ANGULAR_ENERGY_NOT_CERTIFIED', ...
    'reason','','energy_all_time_proven',false, ...
    'constant_inertia_source_pinned',false,'source_pinned',false, ...
    'conditional_strict_inclusion',false, ...
    'tube_lower',nan(3,1),'tube_upper',nan(3,1), ...
    'endpoint_lower',nan(3,1),'endpoint_upper',nan(3,1), ...
    'energy_norm_upper',NaN,'torque_energy_rate_upper',NaN, ...
    'contraction_steps',0,'subslabs',0,'minimum_margin',NaN, ...
    'safe_to_commit',false);
try
    need={'x0_lower','x0_upper','u_lower','u_upper', ...
        'param_lower','param_upper','h_s'};
    assert(isstruct(D)&&isscalar(D)&&all(isfield(D,need)), ...
        'TRIGGER_ORIGINAL_ANGULAR_ENERGY_TUBE:Fields', ...
        'Original 24D initial, 4D input, 21D parameters and time required.');
    xl=double(D.x0_lower(:));xh=double(D.x0_upper(:));
    ul=double(D.u_lower(:));uh=double(D.u_upper(:));
    pl=double(D.param_lower(:));ph=double(D.param_upper(:));
    h=double(D.h_s);
    if isfield(D,'h_lower_s'),hl=double(D.h_lower_s);else,hl=h;end
    assert(isequal(size(xl),[24 1])&&isequal(size(xh),[24 1])&& ...
        isequal(size(ul),[4 1])&&isequal(size(uh),[4 1])&& ...
        isequal(size(pl),[21 1])&&isequal(size(ph),[21 1])&& ...
        all(isfinite([xl;xh;ul;uh;pl;ph])) && ...
        all(xl<=xh)&&all(ul<=uh)&&all(pl<=ph) && ...
        isscalar(h)&&isfinite(h)&&h>0&&isscalar(hl)&& ...
        isfinite(hl)&&hl>=0&&hl<=h, ...
        'TRIGGER_ORIGINAL_ANGULAR_ENERGY_TUBE:Boxes');
    % A time-varying inertia gives an additional (1/2)*w'*(dJ/dt)*w
    % term, so it MUST NOT use the energy identity without a rate bound.
    assert(all(pl(6:8)>0) && all(pl(6:8)==ph(6:8)), ...
        'TRIGGER_ORIGINAL_ANGULAR_ENERGY_TUBE:ConstantInertia', ...
        'Strictly positive point-valued fixed inertias are necessary.');
    S=TRIGGER_TAUT_SOURCE_INVARIANTS();
    assert(S.source_bound_invariance && S.algebra_exact, ...
        'TRIGGER_ORIGINAL_ANGULAR_ENERGY_TUBE:Source', ...
        'Euler subsystem identity is not bound to the actual source.');
    E.source_pinned=true;
    E.constant_inertia_source_pinned=true;
    J=pl(6:8);
    w0lo=xl(10:12);w0hi=xh(10:12);
    torqueMag=max(abs(ul(2:4)),abs(uh(2:4)));
    initMag=max(abs(w0lo),abs(w0hi));
    init2=TRIGGER_OUTWARD_INTERVAL(0,0);
    rate2=TRIGGER_OUTWARD_INTERVAL(0,0);
    for j=1:3
        a=TRIGGER_OUTWARD_INTERVAL(initMag(j),initMag(j));
        b=TRIGGER_OUTWARD_INTERVAL(torqueMag(j),torqueMag(j));
        ij=TRIGGER_OUTWARD_INTERVAL(J(j),J(j));
        init2=init2+ij.*(a.^2);
        rate2=rate2+(b.^2)./ij;
    end
    % Do not infer rigorous square-root bounds from the library's
    % rounding promise alone. Each proposed bound is checked by an
    % outward enclosure of its square before use below.
    v0=upperSqrt(init2.hi);
    g=upperSqrt(rate2.hi);
    V=TRIGGER_OUTWARD_INTERVAL(v0,v0)+ ...
        TRIGGER_OUTWARD_INTERVAL(0,h).*TRIGGER_OUTWARD_INTERVAL(g,g);
    upperV=V.hi;
    E.energy_norm_upper=upperV;
    E.torque_energy_rate_upper=g;
    wlo=zeros(3,1);whi=zeros(3,1);
    for j=1:3
        rootJ=lowerSqrt(J(j));
        rad=TRIGGER_OUTWARD_INTERVAL(upperV,upperV)./ ...
            TRIGGER_OUTWARD_INTERVAL(rootJ,rootJ);
        r=rad.hi;
        wlo(j)=min(-r,w0lo(j));
        whi(j)=max(r,w0hi(j));
    end
    assert(all(isfinite([wlo;whi])) && all(wlo<=whi), ...
        'TRIGGER_ORIGINAL_ANGULAR_ENERGY_TUBE:FiniteEnergy');
    % Guaranteed enclosure is established ABOVE. Intersecting it with
    % the integral displacement image is sound without strict inclusion.
    % A few iterations avoid the wrapping of the original wide energy box.
    for it=1:5
        F=angularRhs(wlo,whi,ul,uh,J);
        oldlo=wlo;oldhi=whi;
        for j=1:3
            X=TRIGGER_OUTWARD_INTERVAL(w0lo(j),w0hi(j));
            delta=TRIGGER_OUTWARD_INTERVAL(0,h).*F{j};
            image=X+delta;
            wlo(j)=max(wlo(j),image.lo);
            whi(j)=min(whi(j),image.hi);
        end
        assert(all(isfinite([wlo;whi]))&&all(wlo<=whi), ...
            'TRIGGER_ORIGINAL_ANGULAR_ENERGY_TUBE:EmptyIntersection', ...
            'Unverified arithmetic led to contradictory enclosures.');
        E.contraction_steps=it;
        if isequal(wlo,oldlo) && isequal(whi,oldhi),break;end
    end
    F=angularRhs(wlo,whi,ul,uh,J);
    elo=zeros(3,1);ehi=zeros(3,1);
    for j=1:3
        X=TRIGGER_OUTWARD_INTERVAL(w0lo(j),w0hi(j));
        % The physical elapsed time is any h_actual in [hl,h].
        endpoint=X+TRIGGER_OUTWARD_INTERVAL(hl,h).*F{j};
        elo(j)=max(wlo(j),endpoint.lo);
        ehi(j)=min(whi(j),endpoint.hi);
    end
    assert(all(isfinite([elo;ehi]))&&all(elo<=ehi), ...
        'TRIGGER_ORIGINAL_ANGULAR_ENERGY_TUBE:Endpoint');
    E.tube_lower=wlo;E.tube_upper=whi;
    E.endpoint_lower=elo;E.endpoint_upper=ehi;
    E.energy_all_time_proven=true;
    E.subslabs=1;
    E.status='CONDITIONAL_SOURCE_PINNED_EULER_ENERGY_ALL_TIME';
    E.reason=['Constant-inertia Euler energy inequality bounds every ' ...
        'omega(t), including arbitrary bounded torque switching. ' ...
        'No Picard convergence was assumed. 21 other coordinates, ' ...
        'taut-mode tension and actual numerical plant still need proof.'];
catch ME
    E.status='UNKNOWN_ANGULAR_ENERGY_INPUT_SOURCE_OR_ARITHMETIC';
    E.reason=[ME.identifier ' | ' ME.message];
end
end

function F=angularRhs(lo,hi,ul,uh,J)
w=cell(3,1);M=cell(3,1);
for j=1:3
    w{j}=TRIGGER_OUTWARD_INTERVAL(lo(j),hi(j));
    M{j}=TRIGGER_OUTWARD_INTERVAL(ul(j+1),uh(j+1));
end
Jx=TRIGGER_OUTWARD_INTERVAL(J(1),J(1));
Jy=TRIGGER_OUTWARD_INTERVAL(J(2),J(2));
Jz=TRIGGER_OUTWARD_INTERVAL(J(3),J(3));
F=cell(3,1);
F{1}=(M{1}+(Jy-Jz).*w{2}.*w{3})./Jx;
F{2}=(M{2}-(Jx-Jz).*w{1}.*w{3})./Jy;
F{3}=(M{3}+(Jx-Jy).*w{1}.*w{2})./Jz;
end
function r=upperSqrt(x)
assert(isfinite(x)&&x>=0, ...
    'TRIGGER_ORIGINAL_ANGULAR_ENERGY_TUBE:SqrtInput');
r=sqrt(x);
for k=1:32
    r2=TRIGGER_OUTWARD_INTERVAL(r,r).^2;
    % r2.lo <= exact r^2; if even that lower bound exceeds x,
    % r is a mathematically proven upper bound for sqrt(x).
    if r2.lo>=x,return;end
    r=TRIGGER_FLOAT_OUTWARD_ONE_ULP(r,+1);
    assert(isfinite(r), ...
        'TRIGGER_ORIGINAL_ANGULAR_ENERGY_TUBE:SqrtOverflow');
end
error('TRIGGER_ORIGINAL_ANGULAR_ENERGY_TUBE:UnprovedSqrtUpper', ...
    'No outward square-checked upper sqrt within the fixed proof budget.');
end
function r=lowerSqrt(x)
assert(isfinite(x)&&x>0, ...
    'TRIGGER_ORIGINAL_ANGULAR_ENERGY_TUBE:SqrtInput');
r=sqrt(x);
for k=1:32
    r2=TRIGGER_OUTWARD_INTERVAL(r,r).^2;
    % r2.hi >= exact r^2; if its upper bound is below x,
    % r is a mathematically proven LOWER bound for sqrt(x).
    if r2.hi<=x && r>0,return;end
    r=TRIGGER_FLOAT_OUTWARD_ONE_ULP(r,-1);
    assert(isfinite(r)&&r>0, ...
        'TRIGGER_ORIGINAL_ANGULAR_ENERGY_TUBE:SqrtUnderflow');
end
error('TRIGGER_ORIGINAL_ANGULAR_ENERGY_TUBE:UnprovedSqrtLower', ...
    'No outward square-checked lower sqrt within the fixed proof budget.');
end
