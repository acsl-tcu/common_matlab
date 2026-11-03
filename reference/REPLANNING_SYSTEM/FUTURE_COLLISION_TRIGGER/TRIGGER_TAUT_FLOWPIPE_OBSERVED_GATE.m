function R=TRIGGER_TAUT_FLOWPIPE_OBSERVED_GATE(C)
% Connect ORIGINAL 24-state all-time Picard tubes to full UAV/cable/load
% directional separation of ONLY explicitly supplied, observed shapes.
%
% REQUIRED: C.flowpipe_inputs: explicit uncertainty inputs for
% TRIGGER_ORIGINAL_PLANT_AUTO_CHAIN (no future EKF, hidden world or reference).
% C.observed_prepared_shapes: shapes ALREADY acquired by real sensor memory.
% C.body: actual r_uav_m, r_cable_m, r_payload_m, from live geometry.
%
% One STRICT separating plane is sufficient for each (obstacle,interval),
% but failing to find one is UNKNOWN, never an assertion of collision.
% For known normal n, h_UAV/cable/payload(n) < min_{o in O} n'*o
% separates the ENTIRE interval, provided the upstream tube, taut physical
% manifold, physical radii, and support arithmetic are sound.
%
% CONDITIONAL ENGINEERING ANALYSIS ONLY. Independent certification of
% continuous EKF initial set, actual HLC hold, mass evolution, taut hybrid
% mode and outward-rounded operations is NOT present. safe_to_commit FALSE.
R=struct('status','UNKNOWN_TAUT_FLOWPIPE_OBSERVED_GATE', ...
    'reason','','safe_to_commit',false,'conditional_pass',false, ...
    'all_intervals_converged',false,'all_tensions_positive',false, ...
    'all_manifold_boxes_compatible',false, ...
    'all_observed_shapes_separated',false, ...
    'observed_ids',zeros(1,0),'time_grid_s',zeros(1,0), ...
    'separated_intervals',false(0,0), ...
    'separation_witness_normals',zeros(3,0,0), ...
    'smallest_conditional_clearance_m',NaN, ...
    'tension_lower_N',zeros(1,0),'reach',struct(), ...
    'required_evidence',struct('EKF_error_set',false, ...
    'actual_HLC_applied_input',false,'future_mass_bound',false, ...
    'exact_taut_initial_manifold',false,'taut_mode_and_tension',false, ...
    'interval_source_and_rounding',false, ...
    'observed_only_shape_provenance',false), ...
    'note','Cannot authorize Flight safety.');
try
    assert(isstruct(C)&&isscalar(C)&& ...
        all(isfield(C,{'flowpipe_inputs','observed_prepared_shapes','body'})), ...
        'TRIGGER_TAUT_FLOWPIPE:Inputs', ...
        'Provide explicit flowpipe inputs, observed shapes and body radii.');
    S=C.observed_prepared_shapes;
    assert(isstruct(S)&&~isempty(S)&& ...
        all(isfield(S,{'id','center0','rotation','dimensions','type','is_stationary'})), ...
        'TRIGGER_TAUT_FLOWPIPE:Observed', ...
        'A nonempty array of prepared, observed static shapes is required.');
    ids=reshape(double([S.id]),1,[]);
    assert(all(isfinite(ids))&&all(ids==fix(ids))&&all(ids>=1) && ...
        numel(unique(ids))==numel(ids)&&all([S.is_stationary]), ...
        'TRIGGER_TAUT_FLOWPIPE:Observed', ...
        'Unique positive observed IDs and static shapes are required.');
    for k=1:numel(S)
        assert(isequal(size(S(k).center0),[3 1]) && ...
            isequal(size(S(k).rotation),[3 3]) && ...
            all(isfinite([S(k).center0(:);S(k).rotation(:)])), ...
            'TRIGGER_TAUT_FLOWPIPE:Geometry','Invalid prepared geometry.');
    end
    body=C.body;
    assert(isstruct(body)&&isscalar(body)&& ...
        all(isfield(body,{'r_uav_m','r_payload_m','r_cable_m'})), ...
        'TRIGGER_TAUT_FLOWPIPE:Body','Live physical radii required.');
    radii=struct('uav',body.r_uav_m,'cable',body.r_cable_m, ...
        'payload',body.r_payload_m);
    rr=[radii.uav,radii.cable,radii.payload];
    assert(isnumeric(rr)&&isreal(rr)&&numel(rr)==3 && ...
        all(isfinite(rr))&&all(rr>=0), ...
        'TRIGGER_TAUT_FLOWPIPE:Body','Invalid actual radii.');
    P=C.flowpipe_inputs;
    F=TRIGGER_ORIGINAL_PLANT_AUTO_CHAIN(P);
    R.reach=F;
    if ~F.conditional_chain_pass
        R.status='UNKNOWN_PICARD_CHAIN_NOT_ENCLOSED';
        R.reason=F.reason;
        return
    end
    K=F.completed_intervals;
    R.all_intervals_converged=K==numel(P.time_grid_s)-1;
    R.all_tensions_positive=F.conditional_tension_positive_all_intervals;
    R.all_manifold_boxes_compatible=all(F.manifold_compatible_intervals);
    R.tension_lower_N=F.tension_lower_N;
    R.observed_ids=ids;
    R.time_grid_s=double(P.time_grid_s(:).');
    R.separated_intervals=false(numel(S),K);
    R.separation_witness_normals=nan(3,K,numel(S));
    all_clearance=[];
    for i=1:K
        X=struct('q',xbox(F,1:3,i), 'v',xbox(F,7:9,i), ...
            'd',xbox(F,19:21,i),'w',xbox(F,22:24,i));
        L=struct('lower',double(P.param_lower(21,i)), ...
                 'upper',double(P.param_upper(21,i)));
        q=(X.q.lower+X.q.upper)/2;
        % Candidate center for normal SEARCH only, never used as a bound.
        d=(X.d.lower+X.d.upper)/2;
        pm=q+(L.lower+L.upper)*0.5*d;
        mid=0.5*(q+pm);
        for j=1:numel(S)
            sh=S(j);
            c=double(sh.center0(:));
            ns=[c-q,c-pm,c-mid,eye(3),-eye(3), ...
                double(sh.rotation),-double(sh.rotation)];
            [a,b,z]=ndgrid(-1:1,-1:1,-1:1); %#ok<ASGLU>
            ns=[ns,[a(:).';b(:).';z(:).']]; %#ok<AGROW>
            norms=sqrt(sum(ns.^2,1));
            ns=ns(:,isfinite(norms)&norms>0);
            D=TRIGGER_TAUT_REDUCED_DIRECTIONAL_ENCLOSURE( ...
                X,L,radii,ns,sh);
            if ~D.conditional_enclosure
                R.status='UNKNOWN_TAUT_SUPPORT_NOT_ENCLOSED';
                R.reason=sprintf('Interval %d, observed ID %g: %s', ...
                    i,ids(j),D.reason);
                return
            end
            clearances=D.obstacle_support_lower-D.union_support_upper;
            [m,ix]=max(clearances);
            all_clearance(end+1)=m; %#ok<AGROW>
            if isfinite(m) && m>0 && D.conditional_separated(ix)
                R.separated_intervals(j,i)=true;
                nn=ns(:,ix);
                R.separation_witness_normals(:,i,j)=nn/norm(nn);
            end
        end
    end
    if ~isempty(all_clearance)
        R.smallest_conditional_clearance_m=min(all_clearance);
    end
    R.all_observed_shapes_separated=all(R.separated_intervals(:));
    R.conditional_pass=R.all_intervals_converged && ...
        R.all_manifold_boxes_compatible && ...
        R.all_tensions_positive && R.all_observed_shapes_separated;
    if R.conditional_pass
        R.status='CONDITIONAL_TAUT_ALL_INTERVALS_SEPARATED__NOT_FLIGHT_SAFETY';
        R.reason=['All supplied Picard intervals had positive conditional tension and ' ...
            'separating directions for all supplied observed shapes. ' ...
            'Actual EKF error, controller input, mass, taut mode, source and rounding ' ...
            'remain unverified; this cannot authorize Flight.'];
    else
        R.status='UNKNOWN_TAUT_FLOWPIPE_OR_SEPARATION';
        R.reason=['At least one original-flowpipe tension, manifold compatibility, ' ...
            'or full-time separation condition was not established.'];
    end
catch ME
    R.status='UNKNOWN_TAUT_FLOWPIPE_GATE_ERROR';
    R.reason=[ME.identifier ' | ' ME.message];
end
end
function B=xbox(F,idx,k)
B=struct('lower',F.tube_lower(idx,k),'upper',F.tube_upper(idx,k));
end
