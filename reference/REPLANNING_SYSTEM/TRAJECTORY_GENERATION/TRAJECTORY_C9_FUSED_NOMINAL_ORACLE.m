function [R,prepared]=TRAJECTORY_C9_FUSED_NOMINAL_ORACLE(piece,observed,radii,authority,geo,force)
% One-call native sufficient real-arithmetic nominal joint proof.
% This replaces repeated MATLAB recursion/support for a POSITIVE candidate.
% Unproved candidate -> UNKNOWN, never COLLISION. All current Flight
% physics, radii, phase and proof budgets enter EVERY call. Optional tube
% inflation is applied only from the caller's certified external bounds.
% Numerical directed rounding and actual HLC B are NOT proved.
clock=tic;
R=struct('certified',false,'status','UNKNOWN_FUSED_NATIVE_NOT_RUN', ...
    'certified_continuous_geometry',false, ...
    'certified_continuous_tension',false, ...
    'certified_continuous_thrust',false, ...
    'safe_to_commit',false,'numerically_directed_rounding_proven',false, ...
    'actual_HLC_tracking_proven',false, ...
    'checked_node_count',0,'checked_leaf_count',0,'shape_support_calls',0, ...
    'near_obstacle_leaf_checks',0,'outer_rejections',0, ...
    'authority_snapshot',authority,'ground_checked',false, ...
    'trajectory_phase','', 'considered_obstacle_ids',[], ...
    'unresolved_obstacle_ids',[], 'unresolved_body_indices',[], ...
    'unresolved_time_interval_s',[NaN NaN], ...
    'geometry_snapshot_cache_hit',false, ...
    'geometry_snapshot_prepare_ms',NaN, ...
    'elapsed_ms',NaN,'native_elapsed_ms',NaN, ...
    'generic_separator_calls',0,'generic_separator_positives',0, ...
    'generic_separator_iterations',0, ...
    'gjk_steps',0,'gjk_fallbacks',0,'gjk_convex_hull_nonseparation_witnesses',0, ...
    'native_status','NOT_RUN', ...
    'witness_bank_count',0,'unresolved_witness_bank',struct([]), ...
    'inherited_shape_prunes',0,'banked_shape_skips',0, ...
    'scope','EXPERIMENTAL_REAL_ARITHMETIC_TAUT_NOMINAL_ONLY__NO_DIRECTED_ROUNDING_OR_HLC');
prepared=struct([]);
try
 [ground,phase]=TRAJECTORY_GROUND_SCOPE(geo);
 R.ground_checked=ground;R.trajectory_phase=phase;
 if ~strcmp(phase,'f'),R.status='UNKNOWN_FUSED_NATIVE_NONFLIGHT';return;end
 if ~isfield(geo,'robust_tube_enabled')||...
    ~all(isfield(geo,{'numerical_guard_m','max_subdivision_depth','max_node_count'}))||...
    ~all(isfield(force,{'max_subdivision_depth','max_node_count','force_guard_N'}))
     R.status='UNKNOWN_FUSED_NATIVE_MISSING_PROOF_OPTIONS';return
 end
 if exist('TRAJECTORY_C9_FUSED_ALLTIME_CORE','file')~=3
     R.status='UNKNOWN_FUSED_NATIVE_MEX_MISSING';return
 end
 if ~strcmp(TRAJECTORY_C9_FUSED_ALLTIME_CORE(), ...
    'C9_FUSED_V6_ANCESTOR_PROOF_REUSE_20261112')
     R.status='UNKNOWN_FUSED_NATIVE_MEX_SIGNATURE';return
 end
 if ~isstruct(piece)||~isequal(size(piece.control_points_m,1),3)||...
    ~any(size(piece.control_points_m,2)==[24 28])
    R.status='UNKNOWN_FUSED_NATIVE_PIECE_INVALID';return
 end
 keys={'uav_m','cable_m','payload_m'};
 rr=zeros(1,3);
 for j=1:3
    rr(j)=double(radii.(keys{j}));
    if geo.robust_tube_enabled
       rr(j)=rr(j)+double(geo.uncertainty_bounds_m.(keys{j}));
    end
 end
 if any(~isfinite(rr))||any(rr<0)
    R.status='UNKNOWN_FUSED_NATIVE_RADII_INVALID';return
 end
 % Validated observed snapshot is the only persistent cache; never cache
 % physical parameters, tube, Flight phase, proof budgets or candidate.
 persistent oldObserved oldPrepared
 tpre=tic;
 if ~isempty(oldObserved)&&isequaln(observed,oldObserved)
    prepared=oldPrepared;R.geometry_snapshot_cache_hit=true;
 else
    prepared=TRAJECTORY_OBSTACLE_SUPPORT.prepare(observed);
    oldObserved=observed;oldPrepared=prepared;
 end
 R.geometry_snapshot_prepare_ms=1000*toc(tpre);
 R.considered_obstacle_ids=[prepared.id];
 if ground
    groundDef=GROUND_INFORMATION.definition();
    assert(strcmp(groundDef.type,'ground_halfspace') && ...
       isequal(groundDef.normal_world,[0;0;1]) && ...
       isequal(groundDef.height_m,0) && ...
       strcmp(groundDef.occupied_side,'negative') && ...
       isequal(groundDef.always_known,true), ...
       'C9_FUSED:Ground','Ground definition changed.');
 end
 p=[double(authority.uav_mass_kg),double(authority.payload_mass_kg), ...
    double(authority.cable_length_m), ...
    double(authority.minimum_tension_N)/double(authority.payload_mass_kg)+ ...
      double(authority.acceleration_guard_mps2), ...
    double(authority.thrust_limits_N(:)).', ...
    double(force.force_guard_N),rr, ...
    double(geo.numerical_guard_m), ...
    min(double(geo.max_subdivision_depth),double(force.max_subdivision_depth)), ...
    min(double(geo.max_node_count),double(force.max_node_count)), ...
    double(ground)];
 assert(numel(p)==14&&all(isfinite(p))&&all(p>=0));
 % This is a flight-dependent algorithm switch, not a physical constant.
 % Both modes require exactly the same final positive support-plane test.
 useGjk=0;
 if isfield(geo,'enable_fused_gjk_corrective')
     sw=geo.enable_fused_gjk_corrective;
     if ~(isscalar(sw)&&(isnumeric(sw)||islogical(sw)) && ...
             isfinite(double(sw)) && any(double(sw)==[0 1]))
         R.status='UNKNOWN_FUSED_GJK_FLAG_INVALID';return
     end
     useGjk=double(sw);
 end
 % Optional exact oracle witness collection for distinct obstacle IDs.
 % Every returned interval is an unproved SEARCH location, never a PASS.
 collectBank=0;
 if isfield(geo,'enable_fused_witness_bank')
     sw=geo.enable_fused_witness_bank;
     if ~(isscalar(sw)&&(isnumeric(sw)||islogical(sw)) && ...
             isfinite(double(sw)) && any(double(sw)==[0 1]))
         R.status='UNKNOWN_FUSED_WITNESS_BANK_FLAG_INVALID';return
     end
     collectBank=double(sw);
 end
 p=[p,useGjk,collectBank];
 if isfield(geo,'use_native_support_separator') && ...
    isequal(geo.use_native_support_separator,1)
    % Native fused oracle is independent of the old per-node proposal MEX.
 end
 Z=TRAJECTORY_C9_FUSED_ALLTIME_CORE( ...
   double(piece.control_points_m),double(piece.duration_s), ...
   double(authority.gravity_world_mps2(:)),p,prepared, ...
   'C9_FUSED_V6_ANCESTOR_PROOF_REUSE_20261112');
 R.native_status=Z.status;R.native_elapsed_ms=Z.elapsed_ms;
 if isfield(Z,'witness_bank_ids') && ~isempty(Z.witness_bank_ids)
     ids=double(Z.witness_bank_ids(:));
     iv=double(Z.witness_bank_phase_intervals);
     bm=double(Z.witness_bank_body_mask);
     dp=double(Z.witness_bank_depths(:));
     if isequal(size(iv),[numel(ids),2]) && ...
             isequal(size(bm),[numel(ids),3]) && numel(dp)==numel(ids) && ...
             all(isfinite([ids;iv(:);bm(:);dp])) && ...
             all(iv(:,1)>=0 & iv(:,2)<=1 & iv(:,2)>iv(:,1)) && ...
             all(bm(:)==0 | bm(:)==1) && all(any(bm~=0,2)) && ...
             numel(unique(ids))==numel(ids) && ...
             all(ismember(ids,[prepared.id]))
         bank=repmat(struct('id',NaN,'time_interval_s',[NaN NaN], ...
             'body_mask',false(1,3),'depth',NaN),numel(ids),1);
         for kk=1:numel(ids)
             bank(kk).id=ids(kk);
             bank(kk).time_interval_s= ...
                 double(piece.t0_s)+double(piece.duration_s)*iv(kk,:);
             bank(kk).body_mask=logical(bm(kk,:));
             bank(kk).depth=dp(kk);
         end
         R.unresolved_witness_bank=bank;
         R.witness_bank_count=numel(bank);
     else
         R.status='UNKNOWN_FUSED_MALFORMED_WITNESS_BANK';return
     end
 end
 if isfield(Z,'inherited_shape_prunes')
     R.inherited_shape_prunes=double(Z.inherited_shape_prunes);
 end
 if isfield(Z,'banked_shape_skips')
     R.banked_shape_skips=double(Z.banked_shape_skips);
 end
 R.gjk_steps=double(Z.gjk_steps);
 R.gjk_fallbacks=double(Z.gjk_fallbacks);
 R.gjk_convex_hull_nonseparation_witnesses=double(Z.gjk_convex_hull_nonseparation_witnesses);
 R.checked_node_count=Z.checked_node_count;
 R.checked_leaf_count=Z.checked_leaf_count;
 R.shape_support_calls=Z.shape_support_calls;
 R.near_obstacle_leaf_checks=Z.near_obstacle_leaf_checks;
 R.outer_rejections=Z.outer_rejections;
 if isfield(Z,'generic_separator_calls')
    R.generic_separator_calls=double(Z.generic_separator_calls);
    R.generic_separator_positives=double(Z.generic_separator_positives);
    R.generic_separator_iterations=double(Z.generic_separator_iterations);
 end
 if Z.certified
    R.certified=true;
    R.certified_continuous_geometry=true;
    R.certified_continuous_tension=true;
    R.certified_continuous_thrust=true;
    R.minimum_tension_lower_bound_N=Z.minimum_tension_lower_bound_N;
    R.thrust_lower_bound_N=Z.thrust_lower_bound_N;
    R.thrust_upper_bound_N=Z.thrust_upper_bound_N;
    R.ground_minimum_clearance_lower_bound_m=Z.ground_minimum_clearance_lower_bound_m;
    R.status='CONDITIONAL_FUSED_NATIVE_REAL_ARITHMETIC_ALLTIME_NOMINAL__NO_DIRECTED_ROUNDING_HLC';
 else
    R.status=['UNKNOWN_FUSED_NATIVE_' Z.status];
    % The NATIVE proof tree itself supplies a deterministic unresolved
    % time cell and the exact prepared obstacle ID.  This is a SEARCH
    % WITNESS only; failed separation is NOT a collision certificate.
    if isfield(Z,'unresolved_phase_interval') && ...
            isnumeric(Z.unresolved_phase_interval) && ...
            numel(Z.unresolved_phase_interval)==2
        ph=double(Z.unresolved_phase_interval(:)).';
        if all(isfinite(ph)) && ph(1)>=0 && ph(2)<=1 && ph(2)>ph(1)
            R.unresolved_time_interval_s= ...
                double(piece.t0_s)+double(piece.duration_s)*ph;
            if isfield(Z,'unresolved_obstacle_ids')
                R.unresolved_obstacle_ids=double(Z.unresolved_obstacle_ids(:)).';
            end
            if isfield(Z,'unresolved_body_indices')
                R.unresolved_body_indices=double(Z.unresolved_body_indices(:)).';
            end
            R.native_witness_depth=double(Z.witness_depth);
        end
    end
 end
catch ME
 R.status=['UNKNOWN_FUSED_NATIVE_EXCEPTION_' ME.identifier];
end
R.elapsed_ms=1000*toc(clock);
end
