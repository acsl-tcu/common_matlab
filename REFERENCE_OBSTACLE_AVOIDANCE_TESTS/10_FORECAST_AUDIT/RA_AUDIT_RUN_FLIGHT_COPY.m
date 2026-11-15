function trace = RA_AUDIT_RUN_FLIGHT_COPY(root,scenario,number)
% Generate and execute an INSTRUMENTED COPY under TESTS only.
% The user's original run_flight_*.m is read, never modified.
sourcePath=fullfile(root,'run_flight_SimSuspendedLoad_trj_obs_sns.m');
if ~isfile(sourcePath),error('RAAudit:MissingRun','Not found: %s',sourcePath);end
source=fileread(sourcePath);
folder=fullfile(root,'REFERENCE_OBSTACLE_AVOIDANCE_TESTS','10_FORECAST_AUDIT','_GENERATED');
if ~isfolder(folder),mkdir(folder);end
name=sprintf('RA_AUDIT_GENERATED_CASE_%02d',number);
target=fullfile(folder,[name '.m']);
source=requiredReplace(source,'clear all;','% AUDIT: do not clear caller state',1);
source=requiredReplace(source,...
    'script_dir = fileparts(mfilename(''fullpath''));',...
    ['script_dir = ''' strrep(root,'''','''''') ''';'],1);
source=requiredReplace(source,'userpath(''clear'');', ...
    sprintf('userpath(''clear'');\nSETUP_REFERENCE_OBSTACLE_AVOIDANCE();'),1);
source=requiredReplace(source,'logger.display_on = true;',...
    'logger.display_on = false;',1);
if isempty(regexp(source,'flightRouteGenerator\s*=\s*"[^"]+";','once'))
 error('RAAudit:RunTemplate','Cannot locate selected flightRouteGenerator');
end
source=regexprep(source,'flightRouteGenerator\s*=\s*"[^"]+";',...
    sprintf('flightRouteGenerator = "%s";',scenario.route),'once');
if isempty(regexp(source,'flightNominalSpeed\s*=\s*[^;\n]+;','once'))
 error('RAAudit:RunTemplate','Cannot locate flightNominalSpeed');
end
source=regexprep(source,'flightNominalSpeed\s*=\s*[^;\n]+;',...
    sprintf('flightNominalSpeed = %.17g;',scenario.speed),'once');
source=requiredReplace(source,'te = 50;','te = 16;',1);
source=requiredReplace(source,'close all hidden;', '% AUDIT: do not close user figures',1);
source=requiredReplace(source,'clc;','% AUDIT: do not clear command window',1);
source=requiredReplace(source,'obstacleDrawSceneViews = true;',...
    'obstacleDrawSceneViews = false;',1);
source=requiredReplace(source,'obstacleDrawFlightViews = true;',...
    'obstacleDrawFlightViews = false;',1);
source=requiredReplace(source,'obstacleDrawFlightMovie = true;',...
    'obstacleDrawFlightMovie = false;',1);
% Replace only test copy's HLC constructor with the identical object instance
% kept for its original F1/F2/F3. Controller computations are unchanged.
old=sprintf(['agent.controller.set_function_class("hlc_suspended", ...\n' ...
    '    HLC_SUSPENDED_LOAD(agent,Controller_HL_Suspended_Load(dt,agent)));']);
new=sprintf(['raAuditHlcCore = Controller_HL_Suspended_Load(dt,agent);\n' ...
    'agent.controller.set_function_class("hlc_suspended", ...\n' ...
    '    HLC_SUSPENDED_LOAD(agent,raAuditHlcCore));']);
source=requiredReplace(source,old,new,1);
source=requiredReplace(source,'%% Arming at t = 0 (one log entry)',...
    sprintf('raAuditLog = struct([]);\n%% Arming at t = 0 (one log entry)'),1);
hook='do_prop_local(time, cha, cha, logger, env, agent, "controller",      1, true);';
source=requiredReplace(source,hook,[hook sprintf('\nraAuditLog(end+1) = RA_AUDIT_CAPTURE_EKF_AND_ORACLE(agent,time);')],2);
start=strfind(source,'%% SimSuspendedLoad.m: postprocessing without app.UIAxes');
stop=strfind(source,'%% Local functions, moved to end as required for MATLAB scripts');
if numel(start)~=1 || numel(stop)~=1 || start>=stop
    error('RAAudit:RunTemplate','Postprocessing and local-function anchors not found');
end
handoff=sprintf(['trace = struct();\n' ...
 'trace.records=raAuditLog;\n' ...
 'trace.generator=agent.reference.waypoint.generator;\n' ...
 'trace.gains=struct(''F1'',raAuditHlcCore.F1,''F2'',raAuditHlcCore.F2,''F3'',raAuditHlcCore.F3);\n' ...
 'trace.routeName=char(flightRouteGenerator);\n' ...
 'trace.nominalSpeed=flightNominalSpeed;\n' ...
 'trace.truthColumnsReservedForScoringOnly=true;\n' ...
 'trace.originalRunWasNotModified=true;\n' ...
 'end\n\n']);
source=[source(1:start-1) handoff source(stop:end)];
% Make function-local script behavior independent of the test runner state.
source=[sprintf('function trace=%s()\n',name) source];
fid=fopen(target,'w');if fid<0,error('RAAudit:Write','Cannot write generated test file');end
clean=onCleanup(@()fclose(fid));fwrite(fid,source,'char');clear clean
addpath(folder,'-begin');rehash;
trace=feval(name);
end
function src=requiredReplace(src,from,to,expected)
n=length(strfind(src,from));
if n~=expected
 error('RAAudit:RunTemplate','Expected %d occurrences, found %d: %s',expected,n,from);
end
src=strrep(src,from,to);
end
