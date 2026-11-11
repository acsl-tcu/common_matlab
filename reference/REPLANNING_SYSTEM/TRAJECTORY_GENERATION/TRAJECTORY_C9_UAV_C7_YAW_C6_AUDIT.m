function R = TRAJECTORY_C9_UAV_C7_YAW_C6_AUDIT(piece, authority, options)
% TRAJECTORY_C9_UAV_C7_YAW_C6_AUDIT
% One ideal-taut C9 load piece -> derived UAV C7 and heading-exact yaw C6.
% Uses all-time Bernstein sufficient inequalities, NOT pointwise samples:
%   w = g - pL'', q = w/||w||, pU' = pL' - L*q'.
%   ||q'|| <= max_i||w'_i|| / min_i(n'*w_i),
%   e'*pU'_xy >= min_i(e'*pL'_xy) - L*max_i||w'_i|| / min_i(n'*w_i).
% Here n and e are directions of the current control polygons (not fixed
% world axes). A strictly positive bound on every subdivided leaf proves
% horizontal velocity never vanishes over the whole piece. Together with
% the analytic nonzero w bound and C9 load regularity this implies C7 UAV
% and a continuous lifted C6 yaw. No arbitrary speed limit or motion margin.
%
% Optional options.previous_piece: previously executed C9 load piece.
% The previous piece is evaluated at the new piece's t0_s (replan event),
% NOT at an original nominal catch-up time. Its full 0..9 payload jets,
% 0..7 UAV jets and 0..6 yaw jets are compared at the true join instant.
%
% This is a REAL-ARITHMETIC sufficient result with caller-owned numerical
% guard; no directed rounding / formal floating point enclosure is claimed.
% It proves neither physical HLC tracking nor torque inputs or COMMIT.
start_clock=tic;
required={'max_subdivision_depth','max_node_count', ...
    'horizontal_velocity_guard_mps','w_guard_mps2', ...
    'payload_jet_tolerance','uav_jet_tolerance','yaw_jet_tolerance'};
assert(isstruct(options)&&isscalar(options)&&all(isfield(options,required)), ...
    'TRAJECTORY_C9_HEADING:Options','Explicit all-time proof and join tolerances required.');
assert(isstruct(piece)&&isscalar(piece)&& ...
    all(isfield(piece,{'t0_s','duration_s','derivative_controls'}))&& ...
    piece.duration_s>0&&isfinite(piece.t0_s), ...
    'TRAJECTORY_C9_HEADING:Piece','Require complete C9 load piece.');
for field={'max_subdivision_depth','max_node_count'}
    value=options.(field{1});
    assert(isnumeric(value)&&isscalar(value)&&isfinite(value)&& ...
        value>=0&&value==fix(value), ...
        'TRAJECTORY_C9_HEADING:Budget','Finite integer budgets required.');
end
assert(options.max_node_count>=1,'TRAJECTORY_C9_HEADING:Budget', ...
    'At least one leaf node is required.');
for field={'horizontal_velocity_guard_mps','w_guard_mps2'}
    value=options.(field{1});
    assert(isnumeric(value)&&isscalar(value)&&isfinite(value)&&value>=0, ...
        'TRAJECTORY_C9_HEADING:Guard','Nonnegative supplied numerical guard required.');
end
assert(isequal(size(options.payload_jet_tolerance),[3,10])&& ...
    isequal(size(options.uav_jet_tolerance),[3,8])&& ...
    isequal(size(options.yaw_jet_tolerance),[1,7])&& ...
    all(isfinite(options.payload_jet_tolerance(:)))&& ...
    all(isfinite(options.uav_jet_tolerance(:)))&& ...
    all(isfinite(options.yaw_jet_tolerance(:)))&& ...
    all(options.payload_jet_tolerance(:)>=0)&& ...
    all(options.uav_jet_tolerance(:)>=0)&& ...
    all(options.yaw_jet_tolerance(:)>=0), ...
    'TRAJECTORY_C9_HEADING:Join','Caller must supply derivative-order tolerances.');
assert(isstruct(authority)&&isscalar(authority)&& ...
    isfield(authority,'gravity_world_mps2')&& ...
    isfield(authority,'cable_length_m')&& ...
    isequal(size(authority.gravity_world_mps2),[3,1])&& ...
    all(isfinite(authority.gravity_world_mps2(:)))&& ...
    isscalar(authority.cable_length_m)&& ...
    isfinite(authority.cable_length_m)&&authority.cable_length_m>0, ...
    'TRAJECTORY_C9_HEADING:Authority','External gravity and cable length required.');
R=struct('status','UNKNOWN_NOT_PROVEN','certified',false, ...
    'continuous_horizontal_heading_proven',false, ...
    'certified_stationary_yaw_hold',false, ...
    'all_time_yaw_C6_defined_proven',false, ...
    'yaw_mode','UNPROVEN', ...
    'derived_uav_C7_regular',false,'derived_yaw_C6_regular',false, ...
    'join_requested',isfield(options,'previous_piece'), ...
    'strict_C9_load_join',false,'strict_C7_uav_join',false, ...
    'strict_C6_yaw_join',false,'max_payload_join_ratio',NaN, ...
    'max_uav_join_ratio',NaN,'max_yaw_join_ratio',NaN, ...
    'start_uav_jets',NaN(3,8),'end_uav_jets',NaN(3,8), ...
    'start_yaw_jets',NaN(1,7),'end_yaw_jets',NaN(1,7), ...
    'minimum_horizontal_speed_lower_bound_mps',NaN, ...
    'minimum_w_lower_bound_mps2',NaN,'checked_nodes',0, ...
    'checked_leaf_count',0,'unresolved_interval_s',[NaN NaN], ...
    'safe_to_commit',false,'physical_HLC_certified',false, ...
    'elapsed_ms',NaN, ...
    'scope','IDEAL_TAUT_C9_DERIVED_UAV_C7_YAW_C6__HLC_PENDING');
% Never certify the heading/C7 join for an unbound derivative cache.
% The endpoint jets stored by the constructor are only requested values;
% the actual polynomial can differ due to roundoff or tampering.
try
    Dactual=TRAJECTORY_C9_FLAT_BERNSTEIN.actual_derivative_controls(piece);
    D=piece.derivative_controls;
    if ~(iscell(D)&&numel(D)==10)
        R.status='UNKNOWN_C9_DERIVATIVE_CACHE_NOT_BOUND';
        R.elapsed_ms=1000*toc(start_clock);return
    end
    for order=1:10
        if ~(isnumeric(D{order})&&isreal(D{order})&& ...
                isequal(size(D{order}),size(Dactual{order}))&& ...
                all(isfinite(D{order}(:)))&& ...
                isequal(double(D{order}),Dactual{order}))
            R.status='UNKNOWN_C9_DERIVATIVE_CACHE_NOT_BOUND';
            R.elapsed_ms=1000*toc(start_clock);return
        end
    end
    % Endpoint metadata is NOT used as the polynomial's truth.  Its
    % discrepancy is reported, not silently substituted for actual jets.
    for boundary={'start','end'}
        name=[boundary{1} '_jets'];
        if isfield(piece,name)
            claimed=piece.(name);
            actual=zeros(3,10);
            for order=1:10
                if strcmp(boundary{1},'start')
                    actual(:,order)=Dactual{order}(:,1);
                else
                    actual(:,order)=Dactual{order}(:,end);
                end
            end
            if ~(isnumeric(claimed)&&isreal(claimed)&& ...
                    isequal(size(claimed),[3 10])&& ...
                    all(isfinite(claimed(:))))
                R.status='UNKNOWN_C9_CLAIMED_ENDPOINT_JETS_MALFORMED';
                R.elapsed_ms=1000*toc(start_clock);return
            end
            deviation=abs(double(claimed)-actual);
            if strcmp(boundary{1},'start')
                R.actual_start_metadata_disagreement_by_order=max(deviation,[],1);
            else
                R.actual_end_metadata_disagreement_by_order=max(deviation,[],1);
            end
        end
    end
catch ME
    R.status=['UNKNOWN_C9_POLYNOMIAL_BINDING_' ME.identifier];
    R.elapsed_ms=1000*toc(start_clock);return
end
V=double(D{2}(1:2,:));
W=repmat(double(authority.gravity_world_mps2),1,size(D{3},2))-double(D{3});
W1=-double(D{4});
if ~(all(isfinite([V(:);W(:);W1(:)])))
    R.status='UNKNOWN_NONFINITE_CONTROLS';
    R.elapsed_ms=1000*toc(start_clock);return
end
% Genuine whole-piece horizontal rest is a separate analytic yaw mode.
% A degree-23 payload polynomial with every XY control point identical,
% and zero XY gravity, has w_xy=q_xy=0 for the WHOLE interval. If w_z
% keeps one strict sign, the reconstructed UAV XY coordinate is constant.
% Thus the external yaw anchor may be held with zero derivatives to C6.
% Merely having v_xy(t*)=0 at ONE instant is not sufficient.
stationary_xy=all(all(double(Dactual{1}(1:2,:))== ...
    double(Dactual{1}(1:2,1)))) && ...
    all(double(authority.gravity_world_mps2(1:2))==0);
stationary_mode=false;
if stationary_xy
    R.checked_nodes=1;
    z=double(W(3,:));
    if min(z)>options.w_guard_mps2
        w_lower=min(z);
        stationary_mode=true;
    elseif max(z)<-options.w_guard_mps2
        w_lower=-max(z);
        stationary_mode=true;
    else
        R.status='UNKNOWN_STATIONARY_CABLE_DIRECTION_DENOMINATOR';
        R.elapsed_ms=1000*toc(start_clock);return
    end
    if ~(isfield(options,'stationary_yaw_anchor_rad')&& ...
            isnumeric(options.stationary_yaw_anchor_rad)&& ...
            isreal(options.stationary_yaw_anchor_rad)&& ...
            isscalar(options.stationary_yaw_anchor_rad)&& ...
            isfinite(options.stationary_yaw_anchor_rad))
        R.status='UNKNOWN_STATIONARY_YAW_ANCHOR_REQUIRED';
        R.elapsed_ms=1000*toc(start_clock);return
    end
    yes=true;v_lower=0;
    R.checked_leaf_count=1;
    R.certified_stationary_yaw_hold=true;
    R.yaw_mode='EXACT_STATIONARY_HOLD';
else
    [yes,v_lower,w_lower]=test_interval(V,W,W1,0,1,0);
    R.continuous_horizontal_heading_proven=yes;
    if yes,R.yaw_mode='EXACT_NONZERO_TANGENT';end
end
R.all_time_yaw_C6_defined_proven=yes;
R.minimum_horizontal_speed_lower_bound_mps=v_lower;
R.minimum_w_lower_bound_mps2=w_lower;
if ~yes
    R.all_time_yaw_C6_defined_proven=false;
    % Preserve endpoint/join diagnostics even if the all-time heading
    % sufficiency proof is inconclusive. Never certify the whole piece.
    R.status='UNKNOWN_ALL_TIME_HORIZONTAL_SPEED_OR_W';
end
try
    ts=piece.t0_s;te=ts+piece.duration_s;
    % Dactual was computed and checked against the ACTUAL degree-23
    % payload polynomial above. Its endpoint columns therefore ARE the
    % polynomial's 0..9 jets; do not rebuild ten derivatives twice.
    Sj=zeros(3,10);Ej=zeros(3,10);
    for order=1:10
        Sj(:,order)=Dactual{order}(:,1);
        Ej(:,order)=Dactual{order}(:,end);
    end
    s=TRAJECTORY_TAUT_FLATNESS.evaluate_jet_matrix(Sj,authority);
    e=TRAJECTORY_TAUT_FLATNESS.evaluate_jet_matrix(Ej,authority);
    if ~(s.valid&&e.valid)
        R.status='UNKNOWN_FLATNESS_AT_ENDPOINT';
        R.elapsed_ms=1000*toc(start_clock);return
    end
    % Reuse the verified endpoint flatness jets. Before this change,
    % reference28 recomputed the very same 0..9 Bernstein derivatives
    % twice, which added latency without strengthening certification.
    if stationary_mode
        anchor=double(options.stationary_yaw_anchor_rad);
    else
        anchor=atan2(s.uav_jets(2,2),s.uav_jets(1,2));
    end
    ys=TRAJECTORY_TAUT_FLATNESS.yaw_from_uav_jets(s.uav_jets,anchor);
    ye=TRAJECTORY_TAUT_FLATNESS.yaw_from_uav_jets(e.uav_jets,ys(1));
    R.start_uav_jets=s.uav_jets;R.end_uav_jets=e.uav_jets;
    R.start_yaw_jets=ys;R.end_yaw_jets=ye;
catch ME
    R.status=['UNKNOWN_ENDPOINT_JETS_' ME.identifier];
    R.elapsed_ms=1000*toc(start_clock);return
end
R.derived_uav_C7_regular=yes;
R.derived_yaw_C6_regular=yes;
if R.join_requested
    prev=options.previous_piece;
    if ~(isstruct(prev)&&isscalar(prev)&& ...
            isfield(prev,'t0_s')&&isfield(prev,'duration_s')&& ...
            ts>=prev.t0_s&&ts<=prev.t0_s+prev.duration_s)
        R.status='UNKNOWN_JOIN_TIME_NOT_IN_ACTIVE_PIECE';
        R.elapsed_ms=1000*toc(start_clock);return
    end
    try
        prevE=TRAJECTORY_TAUT_FLATNESS.evaluate(prev,ts,authority);
        if ~prevE.valid
            R.status='UNKNOWN_PREVIOUS_MODEL_SINGULAR';
            R.elapsed_ms=1000*toc(start_clock);return
        end
        prevL=prevE.load_jets;
        nextL=s.load_jets;
        % An external yaw anchor alone cannot prove a C6 join to a
        % previously moving piece at a zero-speed join. For an exact
        % stationary predecessor, verify its WHOLE polynomial and w_z
        % sign as well; otherwise leave the join UNKNOWN (no fake PASS).
        if stationary_mode
            pD=TRAJECTORY_C9_FLAT_BERNSTEIN.actual_derivative_controls(prev);
            pxy=all(all(pD{1}(1:2,:)==pD{1}(1:2,1)));
            pW=double(authority.gravity_world_mps2(3))-pD{3}(3,:);
            pwok=(min(pW)>options.w_guard_mps2)|| ...
                (max(pW)<-options.w_guard_mps2);
            if ~(pxy&&pwok)
                R.status='UNKNOWN_STATIONARY_JOIN_PREVIOUS_YAW_NOT_PROVEN';
                R.elapsed_ms=1000*toc(start_clock);return
            end
        end
        prev_yaw=TRAJECTORY_TAUT_FLATNESS.yaw_from_uav_jets( ...
            prevE.uav_jets,R.start_yaw_jets(1));
        ya=R.start_yaw_jets;yb=prev_yaw;
        ya(1)=ya(1)+2*pi*round((yb(1)-ya(1))/(2*pi));
        R.max_payload_join_ratio=maximum_ratio(nextL-prevL,options.payload_jet_tolerance);
        R.max_uav_join_ratio=maximum_ratio( ...
            R.start_uav_jets-prevE.uav_jets,options.uav_jet_tolerance);
        R.max_yaw_join_ratio=maximum_ratio(ya-yb,options.yaw_jet_tolerance);
        R.strict_C9_load_join=R.max_payload_join_ratio<=1;
        R.strict_C7_uav_join=R.max_uav_join_ratio<=1&&R.strict_C9_load_join;
        R.strict_C6_yaw_join=R.max_yaw_join_ratio<=1&&R.strict_C7_uav_join;
    catch ME
        R.status=['UNKNOWN_PREVIOUS_PIECE_JOIN_' ME.identifier];
        R.elapsed_ms=1000*toc(start_clock);return
    end
    if ~(R.strict_C7_uav_join&&R.strict_C6_yaw_join)
        R.status='UNKNOWN_REPLAN_C7_OR_YAW_C6_JOIN';
        R.elapsed_ms=1000*toc(start_clock);return
    end
end
if yes
    R.certified=true;
    if stationary_mode
        R.status='CERTIFIED_IDEAL_TAUT_EXACT_STATIONARY_YAW_C6__HLC_PENDING';
    else
        R.status='CERTIFIED_IDEAL_TAUT_C7_YAW_C6_ALL_TIME_AND_REQUESTED_JOIN__HLC_PENDING';
    end
else
    R.status='UNKNOWN_ALL_TIME_HORIZONTAL_SPEED_OR_W';
end
R.elapsed_ms=1000*toc(start_clock);

    function [ok,lowv,loww]=test_interval(Vc,Wc,W1c,lo,hi,depth)
        R.checked_nodes=R.checked_nodes+1;
        ok=false;lowv=-Inf;loww=-Inf;
        if R.checked_nodes>options.max_node_count
            R.unresolved_interval_s=piece.t0_s+piece.duration_s*[lo hi];
            return
        end
        a=mean(Wc,2);na=norm(a);
        b=mean(Vc,2);nb=norm(b);
        if na>0&&nb>0&&isfinite(na)&&isfinite(nb)
            n=a/na;h=b/nb;
            loww=min(n.'*Wc);
            upperwprime=max(sqrt(sum(W1c.^2,1)));
            if loww>options.w_guard_mps2
                lowv=min(h.'*Vc)-authority.cable_length_m*upperwprime/loww;
                if isfinite(lowv)&&lowv>options.horizontal_velocity_guard_mps
                    ok=true;R.checked_leaf_count=R.checked_leaf_count+1;
                    return
                end
            end
        end
        if depth>=options.max_subdivision_depth
            R.unresolved_interval_s=piece.t0_s+piece.duration_s*[lo hi];
            return
        end
        [vl,vr]=split_bernstein(Vc);[wl,wr]=split_bernstein(Wc);
        [wl1,wr1]=split_bernstein(W1c);
        mid=(lo+hi)/2;
        [pass1,l1,s1]=test_interval(vl,wl,wl1,lo,mid,depth+1);
        if ~pass1,lowv=l1;loww=s1;return;end
        [pass2,l2,s2]=test_interval(vr,wr,wr1,mid,hi,depth+1);
        ok=pass2;lowv=min(l1,l2);loww=min(s1,s2);
    end
end

function ratio=maximum_ratio(error,tol)
% For zero tolerances, exact equality is required; no hidden epsilon.
x=abs(double(error(:)));y=double(tol(:));
ratio=max(x./max(y,realmin));
if any(x(y==0)>0),ratio=Inf;end
end

function [A,B]=split_bernstein(P)
% One shared exact Bernstein subdivision primitive; same all-time hull.
[A,B]=TRAJECTORY_C9_BERNSTEIN_MIDPOINT_SPLIT(P);
end
