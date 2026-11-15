function T=test_CBF_V19_OriginalPlantParity()
% Compare ACTUAL R193 24-state plant against V11 full 3D 19-state plant.
% This MUST pass before treating 24 and 19 as interchangeable for V19.
test_CBF_SetupPaths();
if exist('with_load_model_euler_for_HL','file')~=2
 error('CBF:V19PlantMissing','R193 original with_load_model_euler_for_HL.m is not on path.');end
cases={'R193','V18'};rows=cell(0,7);
for i=1:numel(cases)
 sc=CBF_V19_Scenarios('straight_single',cases{i});s=sc.settings;
 for id=1:4
   [x,u]=CBF_V11_Examples(id,s,'ellipsoid',1);
   % 10 us derivative parity: should be close if coordinate conventions match.
   z=CBF_V11_RealTaylor('R1',x,u,s,1);
   a=CBF_V19_OriginalPlantStep(x,u,s,1e-5,1);
   d=(a-x)/1e-5-z(:,2);
   posVel=max(abs(d(8:13)));cab=max(abs(d(14:19)));
   orientation=max(abs(d(1:7)));
   status='MATCH_UNDER_TOLERANCE';
   if max(abs(d))>1e-3,status='MODEL_OR_COORDINATE_MISMATCH';end
   rows(end+1,:)={cases{i},id,posVel,cab,orientation,max(abs(d)),status}; %#ok<AGROW>
 end
end
T=cell2table(rows,'VariableNames',{'parameterSet','stateId',...
 'maxLoadStateDerivativeDifference','maxCableDerivativeDifference',...
 'maxBodyDerivativeDifference','maxDerivativeDifference','status'});
folder=fullfile(pwd,'cbf_v19_results');if ~isfolder(folder),mkdir(folder);end
writetable(T,fullfile(folder,'v19_original_plant_parity.csv'));
disp(T);
end
