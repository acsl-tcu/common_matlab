function R=TRAJECTORY_C9_HLC_FOUR_INPUT_INTERVAL_INTLAB(lower,upper,B,r,f)
%TRAJECTORY_C9_HLC_FOUR_INPUT_INTERVAL_INTLAB Directed 3x3 Cramer enclosure.
% Actual HLC law (preclamp) is tmp=[Uf(1); Beta2\rhs]. Here B, r, f
% MUST be externally justified INTLAB interval evaluations of the actual
% source formulas on an enclosing state/reference set at EACH tick.
% Interval determinant and division are evaluated by INTLAB throughout;
% no unrounded floating-point interval arithmetic is used for the solve.
% This proves a CONDITIONAL pointwise algebraic 4-input inclusion, not
% an intersample/plant/EKF reachability guarantee and never COMMIT.
R=struct('status','UNKNOWN_INTLAB_INTERVALS_REQUIRED', ...
    'safe_to_commit',false, ...
    'conditional_4input_bound_proven',false, ...
    'directed_rounding_used',false, ...
    'beta2_invertible_on_box',false, ...
    'input_enclosure_4x2',NaN(4,2), ...
    'determinant_enclosure_1x2',NaN(1,2), ...
    'headroom_N_Nm',NaN, ...
    'source_state_reachability_proven',false);
if exist('infsup','file')~=2 || exist('intval','file')~=2
    R.status='UNKNOWN_INTLAB_ENGINE_NOT_INSTALLED';return
end
if ~(isa(B,'intval')&&isa(r,'intval')&&isa(f,'intval')&& ...
        isequal(size(B),[3,3])&&isequal(size(r),[3,1])&& ...
        isscalar(f)&&isnumeric(lower)&&isnumeric(upper)&& ...
        isequal(size(lower),[4,1])&&isequal(size(upper),[4,1])&& ...
        isreal(lower)&&isreal(upper)&& ...
        all(isfinite([lower(:);upper(:)]))&& ...
        all(lower(:)<upper(:)))
    R.status='UNKNOWN_INTLAB_INPUT_FORMAT_OR_LIVE_LIMITS';return
end
try
    db=det3(B);
    dlo=double(inf(db));dhi=double(sup(db));
    R.determinant_enclosure_1x2=[dlo,dhi];
    if ~(isfinite(dlo)&&isfinite(dhi)&& ...
            ((dlo>0)||(dhi<0)))
        R.status='UNKNOWN_HLC_INTERVAL_BETA2_SINGULARITY';return
    end
    R.beta2_invertible_on_box=true;
    u=cell(1,3);
    for k=1:3
        M=B;
        M(:,k)=r;
        u{k}=det3(M)/db;
    end
    box=NaN(4,2);
    box(1,:)=[double(inf(f)),double(sup(f))];
    for k=1:3
        box(k+1,:)=[double(inf(u{k})),double(sup(u{k}))];
    end
    if any(~isfinite(box(:))) || any(box(:,1)>box(:,2))
        R.status='UNKNOWN_HLC_INTERVAL_INPUT_NOT_FINITE';return
    end
    R.directed_rounding_used=true;
    R.input_enclosure_4x2=box;
    R.headroom_N_Nm=min([box(:,1)-lower(:);upper(:)-box(:,2)]);
    if all(box(:,1)>=lower(:) & box(:,2)<=upper(:))
        R.conditional_4input_bound_proven=true;
        R.status='CONDITIONAL_HLC_INTLAB_FOUR_PRECLAMP_INPUTS_IN_LIMITS__REACHABILITY_PENDING';
    else
        R.status='UNKNOWN_HLC_FOUR_INPUT_LIMITS_NOT_ENCLOSED';
    end
catch ME
    R.status=['UNKNOWN_INTLAB_SOLVE_' ME.identifier];
end
end

function d=det3(M)
% Exact algebraic 3x3 determinant expansion; INTLAB owns all arithmetic.
d=M(1,1)*(M(2,2)*M(3,3)-M(2,3)*M(3,2)) ...
  -M(1,2)*(M(2,1)*M(3,3)-M(2,3)*M(3,1)) ...
  +M(1,3)*(M(2,1)*M(3,2)-M(2,2)*M(3,1));
end
