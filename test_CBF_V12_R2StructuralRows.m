function report=test_CBF_V12_R2StructuralRows(opts)
%TEST_CBF_V12_R2STRUCTURALROWS Why a naive R2 r4/r6 is NOT a valid HOCBF.
% Reports per-row/per-control effects below selected order and nonaffinity.
% R2=[T''''; tau] remains a genuine investigation candidate, not a valid QP.
if nargin<1,opts=struct();end
test_CBF_SetupPaths();s=test_CBF_Fixture();
if isfield(opts,'full') && opts.full,ids=1:4;ds=[0.8 1.5 100];
else,ids=[1 2 3];ds=[1 100];end
shapes={'C','B','D'};kinds={'sphere','ellipsoid'};
rows=cell(0,13);n=0;
for id=ids
 for si=1:3
  for ti=1:2
   for dist=ds
    [x19,u,obs]=CBF_V11_Examples(id,s,kinds{ti},dist);
    x23=[x19;u(1);zeros(3,1)];du=zeros(4,1);
    H0=[];
    try
       [H0,labels]=CBF_V11_JetRows('R2',shapes{si},x23,du,s,obs,6);
       HP=zeros(size(H0,1),7,4);HM=HP;
       for ch=1:4
           de=zeros(4,1);de(ch)=1;
           HP(:,:,ch)=CBF_V11_JetRows('R2',shapes{si},x23,de,s,obs,6);
           HM(:,:,ch)=CBF_V11_JetRows('R2',shapes{si},x23,-de,s,obs,6);
       end
       for i=1:numel(labels)
           lab=labels{i};
           if strcmp(lab,'obstacle_load') || strcmp(lab,'ground_load') || ...
              strcmp(lab,'ground_cable_load_end'),r=6;
           else,r=4;end
           for ch=1:4
              deriv=(reshape(HP(i,:,ch),1,[])-reshape(HM(i,:,ch),1,[]))/2;
              curvature=HP(i,r+1,ch)+HM(i,r+1,ch)-2*H0(i,r+1);
              e=abs(deriv(1:r)).*factorial(0:r-1);
              A=factorial(r)*deriv(r+1);
              n=n+1;
              rows(n,:)={id,shapes{si},kinds{ti},dist,lab,r,ch,...
                 max(e),factorial(r)*abs(curvature),A, ...
                 double(any(e>1e-7)),double(abs(factorial(r)*curvature)>1e-6),''};
           end
       end
    catch ME
       n=n+1;
       rows(n,:)={id,shapes{si},kinds{ti},dist,'TEST_ERROR',NaN,NaN,...
         NaN,NaN,NaN,NaN,NaN,[ME.identifier ': ' ME.message]};
    end
   end
  end
 end
end
report=cell2table(rows,'VariableNames',...
 {'stateId','shape','obstacleType','distance','barrier','chosenOrder',...
 'channel','maxEarlierOrderEffect','chosenOrderCurvature',...
 'chosenOrderInputCoefficient','hasEarlierInput','hasNonaffine','errorText'});
writetable(report,fullfile(CBF_V11_OutputFolder(),'v12_r2_structural_rows.csv'));
fprintf('[V12] R2 early action/nonaffinity per geometry row: %d rows.\n',height(report));
end
