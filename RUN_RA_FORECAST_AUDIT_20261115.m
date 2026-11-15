function OUT = RUN_RA_FORECAST_AUDIT_20261115(root, runFlight, mode)
% Independent prediction/causality audit. TESTS ONLY; original run is untouched.
% Usage: OUT=RUN_RA_FORECAST_AUDIT_20261115(pwd,false); % saved HLC traces
%        OUT=RUN_RA_FORECAST_AUDIT_20261115(pwd,true);  % + real EKF flights
%        OUT=RUN_RA_FORECAST_AUDIT_20261115(pwd,true,'smoke'); % only 2
if nargin<1 || isempty(root), root=pwd; end
if nargin<2, runFlight=false; end
if nargin<3,mode='all';end
if ~ismember(char(mode),{'smoke','all'})
 error('RAAudit:Mode','Use smoke or all');
end
root=char(root);
this=fileparts(mfilename('fullpath'));
testDir=fullfile(root,'REFERENCE_OBSTACLE_AVOIDANCE_TESTS','10_FORECAST_AUDIT');
if ~isfolder(testDir), testDir=fullfile(this,'REFERENCE_OBSTACLE_AVOIDANCE_TESTS','10_FORECAST_AUDIT'); end
addpath(testDir,'-begin');
module=fullfile(root,'reference','WAYPOINT_REFERENCE','REFERENCE_OBSTACLE_AVOIDANCE');
if ~isfile(fullfile(module,'SETUP_REFERENCE_OBSTACLE_AVOIDANCE.m'))
    error('RAAudit:Install','Production module not found at %s',module);
end
addpath(module,'-begin');
SETUP_REFERENCE_OBSTACLE_AVOIDANCE();
% Other project classes and route generators need the usual project path.
addpath(genpath(fullfile(root,'reference')),'-end');
if isfolder(fullfile(root,'environment'))
    addpath(genpath(fullfile(root,'environment')),'-end');
end
SETUP_REFERENCE_OBSTACLE_AVOIDANCE(); % override obsolete TRIGGER_LIGHT copies
OUT=struct(); OUT.createdAt=char(datetime('now')); OUT.physicalSafetyCertified=false;
OUT.originalRunChanged=false;OUT.futurePlantUsedForForecast=false;
OUT.provenance=RA_AUDIT_SOURCE_PROVENANCE(module);
OUT.savedTrace=struct('status','NOT_RUN');
OUT.routeChange=struct('status','NOT_RUN');
OUT.routeInterpolation=struct('status','NOT_RUN');
OUT.flights=struct('name',{},'status',{},'message',{},'stats',{});
resultsDir=fullfile(root,'REFERENCE_OBSTACLE_AVOIDANCE_TESTS','RESULTS');
if ~isfolder(resultsDir),mkdir(resultsDir);end
try
    OUT.savedTrace=RA_AUDIT_SAVED_HLC_TRACES(root);
catch ME
    OUT.savedTrace=struct('status','FAIL','identifier',ME.identifier, 'message',ME.message);
end
try
 OUT.routeChange=RA_AUDIT_ROUTE_CHANGE();
catch ME
 OUT.routeChange=struct('status','FAIL','message',[ME.identifier ' ' ME.message]);
end
try
 OUT.routeInterpolation=RA_AUDIT_ROUTE_INTERPOLATION();
catch ME
 OUT.routeInterpolation=struct('status','FAIL','message',[ME.identifier ' ' ME.message]);
end
if runFlight
    scenarios={ ...
      struct('name','straight_1p5','route','gen_wp_straight','speed',1.5), ...
      struct('name','straight_0p6','route','gen_wp_straight','speed',0.6), ...
      struct('name','gentle_turn','route','gen_wp_gentle_turn','speed',0.6), ...
      struct('name','circle','route','gen_wp_circle','speed',0.55), ...
      struct('name','saddle','route','gen_wp_saddle','speed',0.55), ...
      struct('name','line_curve','route','gen_wp_ra_audit_line_curve','speed',0.65), ...
      struct('name','turn_fast','route','gen_wp_gentle_turn','speed',1.5)};
    if strcmp(char(mode),'smoke'), scenarios=scenarios([1,6]); end
    for j=1:numel(scenarios)
        S=scenarios{j};
        try
            trace=RA_AUDIT_RUN_FLIGHT_COPY(root,S,j);
            stat=RA_AUDIT_SCORE_FLIGHT(trace);
            OUT.flights(end+1)=struct('name',S.name,'status',stat.status,...
                'message','','stats',stat); %#ok<AGROW>
            % The saved numeric trace contains true data for SCORING only.
            % Drop generator function handles from saved results.
            if isfield(trace,'generator'),trace=rmfield(trace,'generator');end
            save(fullfile(resultsDir,['RA_FORECAST_TRACE_' S.name '.mat']),'trace','stat','-v7.3');
        catch ME
            OUT.flights(end+1)=struct('name',S.name,'status','FAIL',...
                'message',[ME.identifier ' ' ME.message],'stats',struct()); %#ok<AGROW>
        end
    end
end
OUT.allPassed=strcmp(OUT.provenance.status,'PASS_STATIC_SOURCE_CHECK') && ...
    strncmp(OUT.routeChange.status,'PASS',4) && ...
    strncmp(OUT.routeInterpolation.status,'PASS',4) && ...
    strncmp(OUT.savedTrace.status,'PASS',4) && ...
    all(strcmp({OUT.flights.status},'PASS_SCORED_NOT_CERTIFIED'));
OUT.status='INCOMPLETE_OR_FAILED';
if OUT.allPassed && runFlight
    OUT.status='PASS_AUDIT_AND_FLIGHT_SCENARIOS__NOT_SAFETY_CERTIFIED';
elseif OUT.allPassed
    OUT.status='PASS_SAVED_TRACE_AND_SOURCE_AUDIT__FLIGHTS_NOT_RUN';
end
stamp=datestr(now,'yyyymmdd_HHMMSS');
OUT.resultsMat=fullfile(resultsDir,['RA_FORECAST_AUDIT_' stamp '.mat']);
save(OUT.resultsMat,'OUT','-v7.3');
fprintf('[RA PREDICT] %s\n',OUT.status);
fprintf('[RA PREDICT] source: %s\n',OUT.provenance.status);
fprintf('[RA PREDICT] nominal saved traces: %s\n',OUT.savedTrace.status);
fprintf('[RA PREDICT] route switch: %s\n',OUT.routeChange.status);
fprintf('[RA PREDICT] route interpolation: %s\n',OUT.routeInterpolation.status);
for j=1:numel(OUT.flights)
 f=OUT.flights(j);fprintf('[RA PREDICT] flight %-16s %s %s\n',f.name,f.status,f.message);
 if isfield(f.stats,'maxCombinedError_m')
  fprintf('        4s-horizon empirical maximum (all tested times): %.4g m\n', ...
      f.stats.maxCombinedError_m);
 end
end
fprintf('[RA PREDICT] results: %s\n',OUT.resultsMat);
end
