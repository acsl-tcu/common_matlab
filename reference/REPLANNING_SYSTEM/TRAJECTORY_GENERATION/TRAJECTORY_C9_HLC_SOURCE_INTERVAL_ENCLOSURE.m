function R=TRAJECTORY_C9_HLC_SOURCE_INTERVAL_ENCLOSURE(binding,agent,time,boxes)
%TRAJECTORY_C9_HLC_SOURCE_INTERVAL_ENCLOSURE  The real HLC algebra, interval-evaluated.
% Development/reference implementation: not enabled in the real-time loop.
% Uses the EXACT ACTIVE HLC_SUSPENDED_LOAD methods (NOT a nominal flatness
% surrogate) and a directed-rounding interval package (INTLAB infsup).
%
% Inputs: externally certified full-time enclosures of the 19-state x and
% 28-state reference xd which are actually passed to this HLC. In particular,
% xd(4) is the HLC yaw-branch-adjusted target, not an arbitrarily unwrapped
% yaw reference. The caller must separately prove the enclosures cover ALL
% intersample states and references; this function CANNOT do that by merely
% receiving finite bounds. It derives all-time beta2/rhs/Uf enclosures
% algebraically, conditional on that premise, from the actual source.
%
% The active source uses P=[mass,jx,jy,jz,gravity,loadmass,cableL,0,0].
% Those per-tick values are read from the agent, not fixed here.
% No saturations are used during interval evaluation: tmp=[Uf(1);beta2\rhs]
% is the PRE-CLAMP input. A separate direct-interval lemma checks invertibility
% and actuator limits; its interval determinant is directed-rounded by
% INTLAB in TRAJECTORY_C9_HLC_FOUR_INPUT_INTERVAL_INTLAB.
%
% This is NOT yet a MODEL_TUBE-B/actual flight guarantee. Missing ingredients:
% mechanically certified full-time x/xd boxes from exact HLC+plant+EKF,
% residual auto derivation. The 3x3 solve now uses directed INTLAB.
R=struct('status','UNKNOWN_HLC_SOURCE_INTERVAL_NOT_DERIVED', ...
    'safe_to_commit',false,'full_time_state_box_proven',false, ...
    'actual_HLC_execution_proven',false,'rounding_engine','NOT_AVAILABLE', ...
    'hlc_source_expression_evaluated',false,'source_bound',false, ...
    'mass_kg',NaN,'beta2_interval_3x3x2',NaN(3,3,2), ...
    'rhs_interval_3x2',NaN(3,2),'thrust_interval_1x2',NaN(1,2), ...
    'input_lemma_status','NOT_EVALUATED', ...
    'directed_4input_lemma_used',false, ...
    'conditional_source_4input_proven',false, ...
    'input_enclosure_4x2',NaN(4,2),'headroom_N_Nm',NaN, ...
    'scope','CONDITIONAL_SOURCE_FORMULA_INTERVAL_NOT_STATE_REACHABILITY');
if ~isstruct(boxes)||~isscalar(boxes)|| ...
        ~all(isfield(boxes,{'x_19x2','xd_28x2','interval_s'}))
    R.status='UNKNOWN_FULLTIME_HLC_STATE_BOXES_MISSING';return
end
xbox=boxes.x_19x2;dbox=boxes.xd_28x2;interval=boxes.interval_s;
if ~is_valid_box(xbox,19) || ~is_valid_box(dbox,28)|| ...
        ~(isnumeric(interval)&&isreal(interval)&&numel(interval)==2&& ...
          all(isfinite(interval))&&interval(2)>interval(1))
    R.status='UNKNOWN_FULLTIME_HLC_STATE_BOXES_INVALID';return
end
% pT3 is a literal divisor in real Uf and beta2 source. A box crossing
% zero cannot certify a finite uniform full-time raw-input enclosure.
if xbox(16,1)<=0 && xbox(16,2)>=0
    R.status='UNKNOWN_HLC_CABLE_VERTICAL_COMPONENT_CROSSES_ZERO';return
end
if exist('infsup','file')~=2 || exist('intval','file')~=2
    R.status='UNKNOWN_DIRECTED_ROUNDING_INTERVAL_ENGINE_UNAVAILABLE';return
end
try
    live=TRAJECTORY_C9_HLC_FORCE_AUTHORITY.current(binding,agent,time);
catch ME
    R.status=['UNKNOWN_HLC_AUTHORITY_' ME.identifier];return
end
if ~live.valid
    R.status=live.status;return
end
% Do not accept an interval claiming to cover a time domain that does not
% include the current HLC tick.
if interval(1)>live.tick_time_s || interval(2)<live.tick_time_s
    R.status='UNKNOWN_HLC_BOXES_DO_NOT_CONTAIN_CURRENT_TICK';return
end
try
    ctrl=agent.controller.hlc_suspended;
    if ~isa(ctrl,'HLC_SUSPENDED_LOAD') || ~isequal(ctrl,binding.hlc_handle)
        R.status='UNKNOWN_HLC_INSTANCE_NOT_SOURCE_BOUND';return
    end
    % This is precisely the parameter read in the active HLC_SUSPENDED_LOAD.do.
    P=double([agent.parameter.get(["mass","jx","jy","jz", ...
        "gravity","loadmass","cableL"]),0,0]);
    if ~isequal(size(P),[1 9]) || any(~isfinite(P)) || ...
            ~(P(1)>0&&P(2)>0&&P(3)>0&&P(4)>0&&P(7)>0) || ...
            ~isequal(P(6),live.estimated_payload_mass_kg)
        R.status='UNKNOWN_HLC_LIVE_PARAMETERS_NOT_BOUND';return
    end
    gain=ctrl.param;
    for key={'F1','F2','F3','F4'}
        name=key{1};
        if ~isfield(gain,name)||~isfield(binding.controller_gains,name)
            R.status='UNKNOWN_LIVE_HLC_GAINS_CHANGED';return
        end
        gvalue=gain.(name);
        bvalue=binding.controller_gains.(name);
        gactual=double(gvalue(:));
        gbound=double(bvalue(:));
        if ~isequal(gactual,gbound)
            R.status='UNKNOWN_LIVE_HLC_GAINS_CHANGED';return
        end
    end
    % Source methods must be called in the same order and with the SAME
    % arguments as HLC_SUSPENDED_LOAD.do; no separately reimplemented formula.
    x=infsup(double(xbox(:,1)),double(xbox(:,2)));
    xd=infsup(double(dbox(:,1)),double(dbox(:,2)));
    vf=ctrl.Vfd_SuspendedLoadxyDst(gain.dt,x,xd.',gain.F1);
    vs=ctrl.Vs_SuspendedLoadxyDst(x,xd.',vf,P, ...
        gain.F2,gain.F3,gain.F4);
    beta=ctrl.Beta2_SuspendedLoadxyDst(x,xd.',vf,P);
    rhs=ctrl.V2_alpha2_SuspendedLoadxyDst(x,xd.',vf,vs',P);
    uf=ctrl.Uf_SuspendedLoadxyDst(x,xd.',vf,P);
    if ~isequal(size(beta),[3,3]) || ~isequal(size(rhs),[3,1]) || ...
            ~isequal(size(uf),[4,1])
        R.status='UNKNOWN_HLC_INTERVAL_EXPRESSION_OUTPUT_SHAPE';return
    end
    lowB=double(inf(beta)); highB=double(sup(beta));
    lowr=double(inf(rhs)); highr=double(sup(rhs));
    lowf=double(inf(uf(1))); highf=double(sup(uf(1)));
    if ~all(isfinite([lowB(:);highB(:);lowr(:);highr(:);lowf;highf])) || ...
            any(lowB>highB,'all') || any(lowr>highr) || lowf>highf
        R.status='UNKNOWN_HLC_INTERVAL_EXPRESSION_SINGULAR_OR_UNBOUNDED';return
    end
    R.beta2_interval_3x3x2=cat(3,lowB,highB);
    R.rhs_interval_3x2=[lowr,highr];
    R.thrust_interval_1x2=[lowf,highf];
    R.mass_kg=P(6);
    R.source_bound=true;
    R.hlc_source_expression_evaluated=true;
    R.rounding_engine='INTLAB_INTERVAL_SOURCE_FORMULAS';
    % A complete independent state-reachability certificate must be
    % checked by the caller. The caller cannot obtain full-time proof by
    % merely setting a field called certified=true.
    R.status='CONDITIONAL_HLC_SOURCE_BETA2_RHS_UF_INTERVALS__STATE_REACHABILITY_PENDING';
    if isfield(boxes,'state_reachability_certificate')
        R.status='CONDITIONAL_HLC_SOURCE_INTERVALS__EXTERNAL_REACHABILITY_UNVERIFIED';
    end
    % This lemma has not been promoted to a true execution guarantee.
    % The previous double-valued Cramer/Neumann lemma was a
    % REAL-ARITHMETIC analysis without outward rounding. Do NOT use it
    % to assert a numerical bound. Keep every determinant, division and
    % source term as directed INTLAB intervals to the final comparison.
    I=TRAJECTORY_C9_HLC_FOUR_INPUT_INTERVAL_INTLAB( ...
        live.input_limits_lower,live.input_limits_upper,beta,rhs,uf(1));
    R.input_lemma_status=I.status;
    R.input_enclosure_4x2=I.input_enclosure_4x2;
    R.headroom_N_Nm=I.headroom_N_Nm;
    R.directed_4input_lemma_used=I.directed_rounding_used;
    R.conditional_source_4input_proven=I.conditional_4input_bound_proven;
    % No assumption about future EKF/plant state enclosures is inferred.
    % Even a true source-algebra proof does not justify a Flight COMMIT.
catch ME
    R.status=['UNKNOWN_HLC_SOURCE_INTERVAL_EVALUATION_' ME.identifier];
end
end

function ok=is_valid_box(x,n)
ok=isnumeric(x)&&isreal(x)&&isequal(size(x),[n,2])&& ...
    all(isfinite(x(:)))&&all(x(:,1)<=x(:,2));
end
