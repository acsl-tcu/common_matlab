function [CANDS,SYN] = af7r36_build_canonical_piecewise_candidates(replanner,P,CTX,EPI,t1)
% MATLAB shadow specification for the AF7R36 canonical-separator synthesis.
% It reuses the current production Hessian, corridor rows, exact sign-QP and
% Bezier control-point construction. The ONLY mathematical synthesis change
% is the explicit canonical choice inside a degenerate principal eigenspace,
% plus a deterministic numerical-tie ordering of equal-cost sign branches.

CANDS=struct('z',{},'qp_cost',{},'corridor_signs',{}, ...
    'segment_t0',{},'segment_t1',{},'segment_kind',{}, ...
    'control_points',{},'component_start',{},'component_end',{}, ...
    'component_position',{},'component_velocity',{}, ...
    'derivative_ub',{},'max_control_norm_m',{}, ...
    'first_component_position',{},'towing_structure_certificate',{});

C=double(EPI.component_count);
E=numel(EPI.episodes);
nslot=2*C;
nvar=3*nslot;

SYN=struct('certified',false,'continuous_time',true, ...
    'sampled_acceptance',false, ...
    'representation','PIECEWISE_DEGREE15_C7_WITH_AFFINE_UNSAFE_COMPONENTS', ...
    'component_count',C,'episode_count',E, ...
    'sign_assignments_solved',0,'fixed_sign_qp_feasible_count',0, ...
    'qp_solver','CURRENT_PRODUCTION_FIXED_SIGN_QP_WITH_CANONICAL_BRANCH_ORDER', ...
    'separator_rule','AF7R36_CANONICAL_ANALYTIC_PRINCIPAL_EIGENSPACE', ...
    'objective','EXACT_INTEGRAL_SQUARED_CORRECTION_SNAP_ON_SAFE_TRANSITIONS');

if C<1 || E<1 || ~(isfinite(t1)&&t1>double(P.t0))
    return;
end

Hs=replanner.piecewise_minimum_snap_hessian( ...
    EPI.components,double(P.t0),double(t1));
if ~isequal(size(Hs),[nslot nslot]) || any(~isfinite(Hs(:)))
    return;
end
H=kron(Hs,eye(3));
H=0.5*(H+H.');

OR=struct('rows_plus',{},'b_plus',{}, ...
    'rows_minus',{},'b_minus',{},'normal',{}, ...
    'episode_index',{},'obstacle_index',{},'component_index',{});

for ie=1:E
    ep=EPI.episodes(ie);
    jj=double(ep.component_index);
    tca=double(ep.start_s);
    tcb=double(ep.end_s);

    xa=double(P.base_flat_eval_fn(tca)); xa=xa(:);
    xb=double(P.base_flat_eval_fn(tcb)); xb=xb(:);
    if numel(xa)<7 || numel(xb)<7
        return;
    end
    progress=xb(1:3)-xa(1:3);
    if norm(progress)<=128*eps(max(1,norm(xa(1:3))))
        xm=double(P.base_flat_eval_fn(0.5*(tca+tcb))); xm=xm(:);
        if numel(xm)>=7
            progress=xm(5:7);
        end
    end

    ii=double(ep.obstacle_index);
    R=reshape(double(CTX.obstacles.rotations_flat(:,ii)),3,3);
    a=double(CTX.obstacles.base_axes(:,ii));
    [n0,~]=af7r36_canonical_separator_normal(R,a,progress);
    if any(~isfinite(n0)) || norm(n0)<=0
        return;
    end
    n0=n0/norm(n0);

    [Ap,bp,okp]=replanner.bezier_piecewise_episode_corridor_rows( ...
        P,CTX,EPI,ie,n0,nvar);
    [Am,bm,okm]=replanner.bezier_piecewise_episode_corridor_rows( ...
        P,CTX,EPI,ie,-n0,nvar);
    if ~(okp&&okm)
        return;
    end

    OR(ie)=struct('rows_plus',Ap,'b_plus',bp, ...
        'rows_minus',Am,'b_minus',bm,'normal',n0, ...
        'episode_index',ie,'obstacle_index',ii, ...
        'component_index',jj); %#ok<AGROW>
end

BASE=struct('z',{},'qp_cost',{},'corridor_signs',{},'min_slack',{});
BASE=replanner.enumerate_piecewise_sign_qp_recursive( ...
    H,OR,1,zeros(E,1),BASE);
SYN.sign_assignments_solved=2^E;
SYN.fixed_sign_qp_feasible_count=numel(BASE);
if isempty(BASE)
    return;
end

BASE=canonical_sort_base(BASE);

for ib=1:numel(BASE)
    z=double(BASE(ib).z(:));
    [seg0,seg1,kind,CP,pPos,pVel,okCP]= ...
        replanner.build_piecewise_bezier_control_points( ...
            EPI.components,double(P.t0),double(t1),z);
    if ~okCP
        continue;
    end

    dub=replanner.piecewise_bezier_derivative_bounds(seg0,seg1,CP,6);
    if numel(dub)~=6 || any(~isfinite(dub)) || any(dub<0)
        continue;
    end

    mx=0;
    for is=1:numel(CP)
        cc=double(CP{is});
        mx=max(mx,max(sqrt(sum(cc.^2,1))));
    end

    TOW=struct('certified',true,'continuous_time',true, ...
        'sampled_acceptance',false,'continuity_order',7, ...
        'unsafe_component_affine_correction',true, ...
        'unsafe_component_correction_acceleration_zero',true, ...
        'q_equals_base_on_unsafe_components',true, ...
        'uav_payload_cable_common_translation_on_unsafe_components',true, ...
        'internal_component_velocity_free',true, ...
        'endpoint_zero_jet_order',7, ...
        'theorem',['D2_CORRECTION_ZERO_IMPLIES_IDENTICAL_SPECIFIC_FORCE__' ...
                   'IDENTICAL_Q__COMMON_WHOLE_BODY_TRANSLATION']);

    CANDS(end+1)=struct( ... %#ok<AGROW>
        'z',z,'qp_cost',double(BASE(ib).qp_cost), ...
        'corridor_signs',double(BASE(ib).corridor_signs(:)), ...
        'segment_t0',seg0,'segment_t1',seg1, ...
        'segment_kind',{kind},'control_points',{CP}, ...
        'component_start',[EPI.components.start_s], ...
        'component_end',[EPI.components.end_s], ...
        'component_position',pPos, ...
        'component_velocity',pVel, ...
        'derivative_ub',dub, ...
        'max_control_norm_m',mx, ...
        'first_component_position',pPos(:,1), ...
        'towing_structure_certificate',TOW);
end

SYN.certified=~isempty(CANDS);
end


function B=canonical_sort_base(B)
% Objective ordering with a binary64 ambiguity interval. This is NOT a
% safety threshold. It only fixes which already-feasible disjunctive branch
% is selected when floating arithmetic cannot distinguish their objective.
for i=2:numel(B)
    x=B(i);
    j=i-1;
    while j>=1 && comes_before(x,B(j))
        B(j+1)=B(j);
        j=j-1;
    end
    B(j+1)=x;
end
end


function tf=comes_before(a,b)
ca=double(a.qp_cost);
cb=double(b.qp_cost);
guard=65536*eps(max([1,abs(ca),abs(cb)]));
if ca<cb-guard
    tf=true;
    return;
elseif cb<ca-guard
    tf=false;
    return;
end

sa=double(a.corridor_signs(:));
sb=double(b.corridor_signs(:));
n=min(numel(sa),numel(sb));
for k=1:n
    if sa(k)~=sb(k)
        tf=sa(k)<sb(k); % -1 before +1
        return;
    end
end
tf=numel(sa)<numel(sb);
end
