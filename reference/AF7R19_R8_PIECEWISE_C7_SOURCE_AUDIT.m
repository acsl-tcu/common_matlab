function OUT = AF7R19_R8_PIECEWISE_C7_SOURCE_AUDIT()
% AF7R19_R8_PIECEWISE_C7_SOURCE_AUDIT
% Read-only source audit for backend-1 R8.

src=which('REPLANNING_SYSTEM');
assert(~isempty(src)&&isfile(src),'[AF7R19] REPLANNING_SYSTEM.m not found.');
txt=fileread(src);

s1=strfind(txt,'function OUT = builtin_bezier_d15_c7_proposal(obj,CTX)');
s2=strfind(txt,'function [EPI,ok] = build_bezier_nominal_unsafe_episode_components');
assert(numel(s1)==1 && numel(s2)==1 && s2>s1,'[AF7R19] unique R8 generator block not found.');
gen=txt(s1:s2-1);

hasEpisodes=contains(gen,'build_bezier_nominal_unsafe_episode_components');
hasPiecewise=contains(gen,'build_bezier_piecewise_towing_candidates');
hasRep2=contains(gen,'representation_code=2');
hasTight=contains(gen,'tighten_piecewise_derivative_bounds_for_speed');
hasIndependent=contains(gen,'build_static_continuous_wholebody_shadow_certificate');

hasAffine=contains(txt,'UNSAFE_AFFINE_TOWING_CORRIDOR') && ...
    contains(txt,'unsafe_component_correction_acceleration_zero') && ...
    contains(txt,'q_equals_base_on_unsafe_components');
hasC7=contains(txt,'bezier15_control_from_endpoint_jets_scalar') && ...
    contains(txt,'endpoint_zero_jet_order');
hasMinSnap=contains(txt,'piecewise_minimum_snap_hessian') && ...
    contains(txt,'EXACT_INTEGRAL_SQUARED_CORRECTION_SNAP_ON_SAFE_TRANSITIONS');
hasExactSigns=contains(txt,'enumerate_piecewise_sign_qp_recursive');
hasBernstein=contains(txt,'ADAPTIVE_BERNSTEIN_CONVEX_HULL_SUP_BOUND_NO_FIXED_DEPTH') && ...
    contains(txt,'representability_open');
hasQuadprog=contains(txt,'MATLAB_QUADPROG_SYNTHESIS_ONLY') && contains(txt,'quadprog(');

oldSingleHoldInGenerator=contains(gen,'build_bezier_hold_corridor_qp');
oldStoppedRiseInGenerator=contains(gen,'d2Rise=nw*K(2)/Tup^2');
oldFixedDepth=contains(gen,'maxDepth=12');
sampledAcceptanceTrue=contains(txt,'''sampled_acceptance'',true');

closed=hasEpisodes&&hasPiecewise&&hasRep2&&hasTight&&hasIndependent&& ...
    hasAffine&&hasC7&&hasMinSnap&&hasExactSigns&&hasBernstein&&hasQuadprog&& ...
    ~oldSingleHoldInGenerator&&~oldStoppedRiseInGenerator&&~oldFixedDepth&& ...
    ~sampledAcceptanceTrue;

fprintf('\n====================================================================================================\n');
fprintf(' AF7R19 | R8 PIECEWISE-C7 SUSPENDED-LOAD SOURCE AUDIT\n');
fprintf('====================================================================================================\n');
fprintf(' source                                  = %s\n',src);
fprintf(' continuous unsafe episodes             = %d\n',hasEpisodes);
fprintf(' piecewise towing candidate synthesis    = %d\n',hasPiecewise);
fprintf(' representation code 2 active           = %d\n',hasRep2);
fprintf(' unsafe affine d2=0 structure            = %d\n',hasAffine);
fprintf(' q=base on unsafe component theorem      = %d\n',contains(txt,'q_equals_base_on_unsafe_components'));
fprintf(' degree-15 endpoint-jet C7 map           = %d\n',hasC7);
fprintf(' minimum-snap transition QP              = %d\n',hasMinSnap);
fprintf(' exact episode-side enumeration          = %d\n',hasExactSigns);
fprintf(' adaptive Bernstein derivative proof     = %d\n',hasBernstein);
fprintf(' QP explicitly synthesis-only            = %d\n',hasQuadprog);
fprintf(' independent whole-body certificate      = %d\n',hasIndependent);
fprintf(' old single-hold generator call          = %d\n',oldSingleHoldInGenerator);
fprintf(' old stopped-rise flatness gate          = %d\n',oldStoppedRiseInGenerator);
fprintf(' old fixed maxDepth=12 authority         = %d\n',oldFixedDepth);
fprintf(' sampled acceptance introduced           = %d\n',sampledAcceptanceTrue);
fprintf(' backend-1 R8 source closed              = %d\n',closed);
fprintf('====================================================================================================\n');

OUT=struct('schema','AF7R19_R8_PIECEWISE_C7_SOURCE_AUDIT_V1', ...
    'source',src,'closed',closed, ...
    'episodes',hasEpisodes,'piecewise',hasPiecewise,'affine_towing',hasAffine, ...
    'c7_map',hasC7,'minimum_snap',hasMinSnap,'exact_signs',hasExactSigns, ...
    'adaptive_bernstein',hasBernstein,'independent_certificate',hasIndependent, ...
    'old_single_hold',oldSingleHoldInGenerator,'old_stopped_rise',oldStoppedRiseInGenerator, ...
    'old_fixed_depth',oldFixedDepth,'sampled_acceptance',sampledAcceptanceTrue);
end
