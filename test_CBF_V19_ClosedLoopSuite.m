function T=test_CBF_V19_ClosedLoopSuite(opts)
% V19 full 3D closed-loop candidates with REAL generated HLC functions.
% DEFAULT runs 3 trajectories x 2 layouts x (nominal,R1,R2-1,R2-3).
% No new barriers for drone/load/ground. R2-2/virtual remain UNIMPLEMENTED.
if nargin<1,opts=struct();end
if ~isfield(opts,'full'),opts.full=false;end
if ~isfield(opts,'parameterSet'),opts.parameterSet='R193';end
if ~isfield(opts,'plant'),opts.plant='V11_19';end
if ~isfield(opts,'dt'),opts.dt=.025;end
if ~isfield(opts,'outputDir'),opts.outputDir=fullfile(pwd,'cbf_v19_results');end
if ~isfield(opts,'makePlots'),opts.makePlots=false;end
if ~isfield(opts,'methods'),opts.methods={'NOMINAL','R1','R2-1','R2-3','R2-2','VIRTUAL'};end
% Call ONLY after installing the existing generate_cbf_suspended + HLC.
test_CBF_SetupPaths();
base=fileparts(mfilename('fullpath'));
root=fullfile(base,'controller');
if isfolder(fullfile(root,'@HLC_SUSPENDED_LOAD')),addpath(root,'-begin');end
if exist('HLC_SUSPENDED_LOAD','class')~=8
 error('CBF:V19HLCMissing','Put original HLC_SUSPENDED_LOAD under common_matlab/controller or add controller parent to path.');
end
if strcmpi(opts.plant,'R193_24')
 if exist('with_load_model_euler_for_HL','file')~=2
  error('CBF:V19PlantMissing','Add original model/substance path with with_load_model_euler_for_HL.m.');
 end
 parity=test_CBF_V19_OriginalPlantParity();
 if ~all(strcmp(parity.status,'MATCH_UNDER_TOLERANCE'))
  error('CBF:V19PlantParity','24-state R193 and V11 3D 19-state models differ: inspect v19_original_plant_parity.csv before simulation.');
 end
end
if opts.full
 trajectories={'straight','curve','climb'};
 layouts={'single','rotated','large','multi'};
else
 trajectories={'straight','curve','climb'};
 layouts={'single','multi'};
end
names={};
for k=1:numel(trajectories)
 for j=1:numel(layouts)
  names{end+1}=[trajectories{k} '_' layouts{j}]; %#ok<AGROW>
 end
end
rows=cell(0,15);
for i=1:numel(names)
 for j=1:numel(opts.methods)
  meth=opts.methods{j};
  try
   s=CBF_V19_ClosedLoop(meth,names{i},opts);
   rows(end+1,:)={meth,names{i},s.status,s.steps,s.completedSeconds,...
      s.nObstacles,s.minSampleBarrier,s.minSampleTension_N,...
      s.finalReferenceError_m,s.firstFailedStepH,s.maxFilterMilliseconds,s.maxRawNominalSaturation,s.errorText,...
      opts.parameterSet,opts.plant}; %#ok<AGROW>
   if opts.makePlots && s.steps>0
      label=sprintf('v19_%s_%s',regexprep(lower(meth),'[^a-z0-9]','_'),names{i});
      f=fullfile(opts.outputDir,[label '_trace.csv']);
      A=readtable(f);fig=figure('Visible','off','Position',[100 100 1100 720]);
      subplot(2,2,1);hold on;
      plot3(A.referenceX,A.referenceY,A.referenceZ,'k--','LineWidth',1.0);
      plot3(A.loadX,A.loadY,A.loadZ,'b-','LineWidth',1.4);
      plot3(A.cableCenterX,A.cableCenterY,A.cableCenterZ,'m-','LineWidth',1.0);
      sc=CBF_V19_Scenarios(names{i},opts.parameterSet);
      [xx,yy,zz]=sphere(14);
      for oi=1:numel(sc.obstacles)
       oo=sc.obstacles{oi};p=oo.R*diag(oo.axes)*[xx(:)';yy(:)';zz(:)'];
       X=reshape(p(1,:)+oo.center(1),size(xx));
       Y=reshape(p(2,:)+oo.center(2),size(yy));
       Z=reshape(p(3,:)+oo.center(3),size(zz));
       surf(X,Y,Z,'FaceAlpha',.17,'EdgeAlpha',.15);
      end
      grid on;axis equal;xlabel('X [m]');ylabel('Y [m]');zlabel('Z [m]');
      title([meth ' ' names{i}]);view(36,24);
      subplot(2,2,2);plot(A.time_s,A.minCableBarrier);yline(0,'--');
      xlabel('Time [s]');ylabel('Cable h');grid on;
      subplot(2,2,3);plot(A.time_s,A.appliedThrust_N);
      xlabel('Time [s]');ylabel('Thrust [N]');grid on;
      subplot(2,2,4);plot(A.time_s,A.filterMilliseconds);
      xlabel('Time [s]');ylabel('Filter milliseconds');grid on;
      exportgraphics(fig,fullfile(opts.outputDir,[label '.png']));close(fig);
   end
  catch ME
   rows(end+1,:)={meth,names{i},['EXECUTION_ERROR_' ME.identifier],0,...
      0,NaN,NaN,NaN,NaN,NaN,NaN,NaN,ME.message,opts.parameterSet,opts.plant}; %#ok<AGROW>
   fprintf(2,'[V19] %s %s ERROR: %s\n',meth,names{i},ME.message);
  end
 end
end
T=cell2table(rows,'VariableNames',{'method','scenario','status','steps',...
 'completedSeconds','nObstacles','minSampleBarrier','minSampleTension_N',...
 'finalReferenceError_m','firstFailedStepH','maxFilterMilliseconds','maxNominalSaturation','errorText','parameterSet','plant'});
if ~isfolder(opts.outputDir),mkdir(opts.outputDir);end
writetable(T,fullfile(opts.outputDir,'v19_closed_loop_summary.csv'));
fprintf('[V19] %d trials; outputs at %s\n',height(T),opts.outputDir);
end
