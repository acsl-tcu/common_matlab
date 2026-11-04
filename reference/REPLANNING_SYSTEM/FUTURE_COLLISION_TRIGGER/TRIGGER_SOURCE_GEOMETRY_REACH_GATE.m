function R=TRIGGER_SOURCE_GEOMETRY_REACH_GATE(C)
% Connect a previously VERIFIED all-time 24-state plant flowpipe to a
% geometry verifier without trusting user-written rigour flags / radii.
% All shape definitions must be from sensor-observed/retained history.
R=struct('status','UNKNOWN_SOURCE_GEOMETRY_REACH','reason','', ...
 'source_balls',{{}},'conditional_geometry',struct(), ...
 'conditional_separation',false,'possible_risk_ids',zeros(1,0), ...
 'safe_to_commit',false,'full_physical_certificate',false);
try
 assert(isstruct(C)&&isscalar(C) && ...
  all(isfield(C,{'flowpipe','time_grid_s','observed_ids', ...
    'observed_prepared_shapes','body','observation_provenance'})) && ...
  strcmp(C.observation_provenance,'ACTUAL_SENSOR_HISTORY_ONLY'), ...
  'TRIGGER_SOURCE_GEOMETRY_REACH_GATE:Provenance', ...
  'Only the real observed/retained sensor snapshot can be queried.');
 ids=reshape(double(C.observed_ids),1,[]);
 shapes=C.observed_prepared_shapes;
 assert(isstruct(shapes) && numel(shapes)==numel(ids) && ...
   numel(unique(ids))==numel(ids) && ...
   isequal(reshape(double([shapes.id]),1,[]),ids), ...
   'TRIGGER_SOURCE_GEOMETRY_REACH_GATE:IDs','Observed and shape IDs must match.');
 R.possible_risk_ids=ids;
 if isempty(ids)
  R.status='UNKNOWN_NO_OBSERVED_OBSTACLES';
  R.reason='Empty sensor history is not evidence the world is clear.';return
 end
 source_balls=cell(1,numel(ids));
 for k=1:numel(ids)
    B=TRIGGER_SHAPE_SOURCE_OUTER_BALL(shapes(k));
    source_balls{k}=B;
    if ~B.source_geometry_enclosed
      R.source_balls=source_balls;
      R.status='UNKNOWN_UNPROVED_OBSTACLE_SOURCE_GEOMETRY';
      R.reason=sprintf('Observed ID %.0f: %s',ids(k),B.reason);
      return
    end
    % Substitution is ONLY a certified conservative exclusion envelope.
    % The original true geometry remains in C.observed_prepared_shapes.
    shapes(k).bounding_radius_m=B.radius_upper_m;
    shapes(k).rigorous_outer_ball_certified=true;
 end
 R.source_balls=source_balls;
 A=C;
 A.observed_prepared_shapes=shapes;
 A=rmfield(A,'observation_provenance');
 G=TRIGGER_BATCH_SHAPE_NEUTRAL_REACH(A);
 R.conditional_geometry=G;
 R.possible_risk_ids=G.possible_risk_ids;
 R.conditional_separation=G.conditional_all_intervals_separated;
 R.status=['SOURCE_GEOMETRY__' G.status];
 R.reason=['Shape outer bounds were derived from current immutable observed ' ...
  'geometry rather than supplied booleans; all-time plant enclosure, ' ...
  'initial EKF truth, actual future HLC, true mass/rate, taut mode and ' ...
  'numeric source chain are separately required. ' G.reason];
catch ME
 R.status='UNKNOWN_SOURCE_GEOMETRY_REACH_EXCEPTION';
 R.reason=[ME.identifier ' | ' ME.message];
end
end
