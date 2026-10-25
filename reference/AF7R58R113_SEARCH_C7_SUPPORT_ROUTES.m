function A=AF7R58R113_SEARCH_C7_SUPPORT_ROUTES(spec,obs,live,opts)
% Genuine R113 waypoint / physical-time candidate generation:
%   1) R92 ellipsoid-support homotopy waypoints (GEOMETRIC SEEDS ONLY)
%   2) allocate knot times by actual waypoint distances
%   3) R92 coupled C7 minimum eighth derivative segment polynomial
%   4) R99 knot-size check + R113 separate-body full sweep + R80/R91 gates.
% No global whole-body ball, fixed 4s maneuver cap or catch-up to NOMINAL
% SAME-CLOCK goal. end_nominal_phase_s is authoritative and fixed while
% duration grows, phase_delay_s = actual_end_time - nominal_phase.
% max_* counts are NUMERICAL WORK BUDGET only, not trajectory time limits.
assert(isstruct(spec)&&isscalar(spec)&&isstruct(live)&&isscalar(live)&& ...
 isstruct(opts)&&isscalar(opts)&&isstruct(obs), 'R113 failure - SearchInputs');
required={'start_time_s','minimum_initial_duration_s', ...
 'start_jets','end_jets','end_nominal_phase_s'};
for j=1:numel(required)
 assert(isfield(spec,required{j}),'R113:MissingLiveGoalPhase', ...
  'Missing %s',required{j});
end
need={'max_side_assignments','max_duration_trials','duration_growth_factor', ...
 'max_nodes_per_check'};
for j=1:numel(need)
 assert(isfield(opts,need{j}),'R113:MissingCpuBudget','Missing %s',need{j});
end
assert(isequal(size(spec.start_jets),[3 8])&& ...
 isequal(size(spec.end_jets),[3 8])&& ...
 all(isfinite([spec.start_jets(:);spec.end_jets(:)]))&& ...
 isscalar(spec.start_time_s)&&isfinite(spec.start_time_s)&& ...
 isscalar(spec.end_nominal_phase_s)&&isfinite(spec.end_nominal_phase_s)&& ...
 isscalar(spec.minimum_initial_duration_s)&& ...
 isfinite(spec.minimum_initial_duration_s)&&spec.minimum_initial_duration_s>0, ...
 'R113 failure - ExplicitEndpointAuthority');
for k={'max_side_assignments','max_duration_trials','max_nodes_per_check'}
 n=opts.(k{1});
 assert(isnumeric(n)&&isscalar(n)&&isfinite(n)&&n>=1&&fix(n)==n, ...
  'R113 failure - PositiveCpuBudget');
end
factor=opts.duration_growth_factor;
assert(isnumeric(factor)&&isscalar(factor)&&isfinite(factor)&&factor>1, ...
 'R113 failure - NumericalSearchGrowthFactor');
start=double(spec.start_jets(:,1));goal=double(spec.end_jets(:,1));
assert(norm(start-goal)>0,'R113 failure - SeparateGoalChartRequired');
infl=live.d_margin_m+max([live.rQ_m,live.rL_m,live.rC_m]);
if live.robust_tracking_margin_enabled
 assert(isfield(live,'tracking_tube_m')&& ...
  isnumeric(live.tracking_tube_m)&&isscalar(live.tracking_tube_m)&& ...
  isfinite(live.tracking_tube_m)&&live.tracking_tube_m>=0, ...
  'R113 failure - MissingCertifiedTube');
 infl=infl+live.tracking_tube_m;
end
seeds=AF7R58R92_SUPPORT_SEED_ROUTES(start,goal,obs, ...
 infl,opts.max_side_assignments);
A=struct('status','NO_CERTIFIED_REFERENCE_FROM_SEARCH__INCONCLUSIVE', ...
 'reference_found',false,'route',struct([]), ...
 'selected_assigned_side',NaN,'selected_duration_s',NaN, ...
 'selected_phase_delay_s',NaN,'candidate',struct([]), ...
 'attempts',struct([]),'seed_count',numel(seeds.routes), ...
 'no_fixed_maneuver_time_cap',true,'no_same_clock_nominal_catchup',true, ...
 'production_commit_authorized',false);
attempt=struct('duration_s',NaN,'seed',NaN,'status','', ...
 'polyline_length_m',NaN,'reference_pass',false);
logs=repmat(attempt,1,0);
for i=0:opts.max_duration_trials-1
 T=spec.minimum_initial_duration_s*factor^i;
 if ~(isfinite(T)&&T>0),break;end
 t1=double(spec.start_time_s)+T;
 phaseDelay=t1-double(spec.end_nominal_phase_s);
 if phaseDelay<0,continue;end
 for k=1:numel(seeds.routes)
  via=double(seeds.routes{k});
  way=[start,via,goal];
  legs=vecnorm(diff(way,1,2),2,1);
  if any(legs<=0)||~all(isfinite(legs)),continue;end
  % Knot-time placement proportional to geometric path length.
  % Every segment remains positive and actual T is unconstrained above.
  times=double(spec.start_time_s)+T*[0,cumsum(legs)/sum(legs)];
  entry=struct('times_s',times,'waypoints',way, ...
   'start_jets',spec.start_jets,'end_jets',spec.end_jets, ...
   'phase_delay_s',phaseDelay);
  G=AF7R58R113_GENERATE_C7_SEPARATE_ROUTE(entry,obs,live,opts);
  row=attempt;row.duration_s=T;row.seed=k;row.status=G.status;
  row.polyline_length_m=sum(legs);
  row.reference_pass=G.reference_geometry_speed_heading_pass;
  logs(end+1)=row; %#ok<AGROW>
  if G.reference_geometry_speed_heading_pass
   A.status='C7_SEPARATE_GEOMETRY_KINEMATICS_YAW_FOUND__NOT_FULL_EXECUTION';
   A.reference_found=true;A.route=G.route;
   A.selected_assigned_side=k;A.selected_duration_s=T;
   A.selected_phase_delay_s=phaseDelay;
   A.candidate=G;A.attempts=logs;
   return;
  end
 end
end
A.attempts=logs;
end
