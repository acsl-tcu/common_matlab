function summary=test_CBF_V22_All(opts)
% TEST_CBF_V22_ALL -- Zheng paper audit + full-3D R1/R2_21/virtual audits.
%
% Run from common_matlab after installing V16-V19 test dependencies:
%   clear functions; test_CBF_SetupPaths();
%   o=struct('currentEstimatedLoadMassFcn',@()agent.estimator.result.state.mL, ...
%       'parameterSet','R193','full',false,'dt',.025);
%   summary=test_CBF_V22_All(o);
%
% If running offline, explicitly provide estimatedLoadMass. This is a
% STATIC SNAPSHOT diagnostic, NOT a functioning EKF:
%   o=struct('estimatedLoadMass',your_current_estimate,'parameterSet','R193');
%   summary=test_CBF_V22_All(o);
%
% Writes ONLY one ZIP in common_matlab/_CBF_TEST_OUTPUT/ on success.
% No CBF core, generator, HLC or full-flight code is modified.
if nargin<1,opts=struct();end
if ~isfield(opts,'full'),opts.full=false;end
if ~isfield(opts,'dt'),opts.dt=.025;end
if ~isfield(opts,'parameterSet'),opts.parameterSet='R193';end
if ~isfield(opts,'outputRoot')
 root=fileparts(mfilename('fullpath'));
 opts.outputRoot=fullfile(root,'_CBF_TEST_OUTPUT');
end
if isfield(opts,'currentEstimatedLoadMassFcn')
 assert(isa(opts.currentEstimatedLoadMassFcn,'function_handle'), ...
     'currentEstimatedLoadMassFcn must be a MATLAB function handle.');
 source='ESTIMATOR_CALLBACK_SNAPSHOT_PER_SCENARIO';
elseif isfield(opts,'estimatedLoadMass')
 assert(isscalar(opts.estimatedLoadMass) && isfinite(opts.estimatedLoadMass) ...
     && opts.estimatedLoadMass>0, 'Please supply a positive load estimate.');
 source='EXPLICIT_STATIC_ESTIMATE_NOT_LIVE_EKF';
else
 error('CBF:V22EstimateRequired', ...
 ['Current estimated load mass is required. Set opts.currentEstimatedLoadMassFcn ' ...
  '(live estimator) or opts.estimatedLoadMass (explicit offline snapshot). ' ...
  'We never substitute the true plant mass.']);
end
assert(opts.dt>0 && opts.dt<=.1);
if ~isfolder(opts.outputRoot),mkdir(opts.outputRoot);end
stamp=datestr(now,'yyyymmdd_HHMMSSFFF');
work=fullfile(opts.outputRoot,['.tmp_v22_' stamp]);
mkdir(work);
records={};failures={};
try
 % Pure formula checks from the provided Zheng paper.
 Tpaper=CBF_V22_ZhengDirectionAudit();
 writetable(Tpaper,fullfile(work,'zheng_direction_rank.csv'));
 writetable(CBF_V22_ZhengBoundaryAudit(),fullfile(work,'zheng_boundary_counterexample.csv'));
 results=[];
 if opts.full
  layouts={'single','rotated','large','multi'};
  stateIds=1:4;
 else
  layouts={'single','rotated'};
  stateIds=1:2;
 end
 for l=1:numel(layouts)
  sc=CBF_V19_Scenarios(['straight_' layouts{l}],opts.parameterSet);
  mhat=readMassEstimate(opts);
  % Controller/CBF mass always the ESTIMATE; plant truth stays separate.
  s=sc.settings;
  s.physical.massLoad=mhat;
  states=makeStates(sc.x0,stateIds);
  testObstacles=sc.obstacles;
  if strcmp(layouts{l},'single')
    % EXTRA DIAGNOSTIC PLACEMENTS near the two cable endpoints.
    % These are still obstacle ellipsoids against the same ONE cable ellipsoid.
    oLoad=sc.obstacles{1};oLoad.center=[1.1;0;sc.x0(10)];
    oDrone=sc.obstacles{1};oDrone.center=[1.1;0;...
        sc.x0(10)+sc.settings.physical.cableLength];
    testObstacles=[testObstacles,{oLoad,oDrone}];
  end
  for jj=1:numel(testObstacles)
   ob=testObstacles{jj};
   tag=sprintf('%s_obstacle%02d',layouts{l},jj);
   try
    Ta=CBF_V22_ModelAudit(s,states,ob);
    Tp=CBF_V22_EndpointAuthority(s,states);
    Tz=CBF_V22_OneCycleAudit(s,states,ob,struct('dt',opts.dt));
    Tv=CBF_V22_VirtualMapAudit(s,states,ob);
    Tpar=CBF_V22_ModelParity(s,states,ob);
    writetable(Ta,fullfile(work,['relative_order_' tag '.csv']));
    writetable(Tp,fullfile(work,['endpoint_authority_' tag '.csv']));
    writetable(Tz,fullfile(work,['sampled_cycle_' tag '.csv']));
    writetable(Tv,fullfile(work,['virtual_map_' tag '.csv']));
    writetable(Tpar,fullfile(work,['model_parity_' tag '.csv']));
    if ~all(Tpar.parityPass)
      error('CBF:V22Parity','New 21-state physical model fails 19-state Taylor parity.');
    end
    records(end+1,:)={tag,mhat,source,height(Ta),height(Tp),height(Tz),height(Tv),...
       sum(strcmp(Ta.status,'TAYLOR_FIXED_INPUT_CHECK_EACH_DIRECTION_LOCAL_NOT_GLOBAL_RELATIVE_DEGREE')), ...
       max(Tp.R1_d2Error),max(Tp.R2_21_d4Error),max(Tp.virtual_d4Error),...
       sum(Tz.sampledPass), 'TEST_EXECUTED_NOT_CERTIFIED'}; %#ok<AGROW>
   catch ME
    failures(end+1,:)={tag,ME.identifier,ME.message}; %#ok<AGROW>
   end
  end
 end
 if isempty(records)
  Tr=array2table(cell(0,13),'VariableNames', ...
    {'caseName','loadMassEstimate_kg','massSource','relativeRows','endpointRows', ...
    'oneStepRows','virtualMapRows','localCoefficientRows','R1maxError','R2maxError', ...
    'virtualMaxError','sampledPassCount','scope'});
 else
  Tr=cell2table(records,'VariableNames', ...
    {'caseName','loadMassEstimate_kg','massSource','relativeRows','endpointRows', ...
    'oneStepRows','virtualMapRows','localCoefficientRows','R1maxError','R2maxError', ...
    'virtualMaxError','sampledPassCount','scope'});
 end
 writetable(Tr,fullfile(work,'v22_execution_summary.csv'));
 if isempty(failures)
  Tf=cell2table(cell(0,3),'VariableNames',{'caseName','identifier','message'});
 else
  Tf=cell2table(failures,'VariableNames',{'caseName','identifier','message'});
 end
 writetable(Tf,fullfile(work,'v22_errors.csv'));
 file=fullfile(work,'README_RESULTS.txt');
 fid=fopen(file,'w');assert(fid>0);
 fprintf(fid,'V22 Zheng direction and fourth-order audit, %s\n',datestr(now,31));
 fprintf(fid,'SOURCE: Zheng et al., Control Engineering Practice 165 (2025) 106564, equations 5,6,18-30.\n');
 fprintf(fid,'DYNAMICS: original full-3D suspended-load 19-state model; R2 TWO-STAGE uses 21-state chain; ideal flat-input virtual 20-state.\n');
 fprintf(fid,'MASS SOURCE: %s (mass is sampled per scenario).\n',source);
 fprintf(fid,'Physical plant true mass is NEVER substituted into CBF estimate.\n');
 fprintf(fid,'Safety geometry: ONLY cable outer ellipsoid versus static obstacle ellipsoid.\n');
 fprintf(fid,'No drone sphere, load sphere, or ground CBF; endpoint point test is only a relative-degree diagnostic.\n');
 fprintf(fid,'IMPORTANT: This does NOT implement a CBF controller or demonstrate closed-loop avoidance.\n');
 fprintf(fid,'R1-1, R1-2, R1-3 / R2-1, R2-2, R2-3 / virtual remain independent DESIGN candidates.\n');
 fprintf(fid,'Sampled one cycle uses ZOH RK4 and checks a sampled residual; NOT sampled-data CBF certification.\n');
 fprintf(fid,'When massSource is EXPLICIT_STATIC_ESTIMATE_NOT_LIVE_EKF, EKF is NOT running.\n');
 fprintf(fid,'If errors exist, see v22_errors.csv; do NOT interpret as pass.\n');
 fclose(fid);
 zipPath=fullfile(opts.outputRoot,['CBF_V22_RESULTS_' stamp '.zip']);
 zip(zipPath,work);
 assert(isfile(zipPath) && dir(zipPath).bytes>0,'V22 ZIP output failed.');
 rmdir(work,'s');
 summary=struct('status','EXECUTED_NOT_CERTIFIED','resultsZip',zipPath, ...
   'nCases',height(Tr),'nFailedCases',height(Tf),'massSource',source, ...
   'scope','PAPER_THEORY_AND_3D_RELATIVE_ORDER_PLUS_SAMPLED_ZOH_DIAGNOSTICS');
 fprintf('[V22] Results ZIP: %s\n',zipPath);
 fprintf('[V22] Completed cases %d, failed cases %d. Not a closed-loop safety certification.\n',height(Tr),height(Tf));
catch ME
 % Never silently remove failed diagnostic work; leave recoverable files.
 fprintf(2,'[V22] ERROR: %s\n',ME.message);
 fprintf(2,'[V22] Diagnostic temporary folder kept: %s\n',work);
 rethrow(ME);
end
end
function m=readMassEstimate(opts)
if isfield(opts,'currentEstimatedLoadMassFcn')
 m=opts.currentEstimatedLoadMassFcn();
else
 m=opts.estimatedLoadMass;
end
assert(isnumeric(m)&&isscalar(m)&&isfinite(m)&&m>0, ...
  'Current estimated payload mass must be a positive, finite scalar.');
end
function xs=makeStates(base,ids)
xs=cell(numel(ids),1);
for j=1:numel(ids)
 x=base(:);
 switch ids(j)
  case 1
   % exact vertical hover configuration
  case 2
   th=.12;x(14:16)=[sin(th);0;-cos(th)];
   x(17:19)=[0;.03;0];
  case 3
   pitch=.20;x(1:4)=[cos(pitch/2);0;sin(pitch/2);0];
   x(5:7)=[.03;-.04;.01];
  case 4
   th=-.19;x(14:16)=[sin(th);0;-cos(th)];
   x(5:7)=[.10;-.08;.04];x(11:13)=[.20;-.10;-.15];
   x(17:19)=[.04;.01;0];
   x(17:19)=x(17:19)-x(14:16)*(x(14:16).'*x(17:19));
 end
 xs{j}=x;
end
end
