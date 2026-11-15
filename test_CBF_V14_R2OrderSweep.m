function [report,overview]=test_CBF_V14_R2OrderSweep(opts)
%TEST_CBF_V14_R2ORDERSWEEP For R2=[T'''';tau], locate FIRST input actions.
% This is not an implementation of a mixed-degree HOCBF.  It exposes
% lower-order torque effects and high-order tau nonaffinity before selecting
% a design order.  Numerical tests alone cannot prove a uniform degree.
if nargin<1,opts=struct();end
if ~isfield(opts,'full'),opts.full=false;end
if opts.full,ids=[1 2 3 4];ds=[0.8 1.5 100];
else,ids=[1 2 3];ds=[1 100];end
test_CBF_SetupPaths();s=test_CBF_Fixture();
shapes={'C','B','D'};kinds={'sphere','ellipsoid'};
rows=cell(0,17);n=0;
for id=ids
 for si=1:3
  for ki=1:2
   for distance=ds
    [x,u,ob]=CBF_V11_Examples(id,s,kinds{ki},distance);
    x23=[x;u(1);zeros(3,1)];nu0=zeros(4,1);
    try
        [H,labels]=CBF_V11_JetRows('R2',shapes{si},x23,nu0,s,ob,6);
        for ch=1:4
            e=zeros(4,1);e(ch)=1;
            Hp=CBF_V11_JetRows('R2',shapes{si},x23,e,s,ob,6);
            Hm=CBF_V11_JetRows('R2',shapes{si},x23,-e,s,ob,6);
            for r=1:numel(labels)
                lin=factorial(0:6).*(Hp(r,:)-Hm(r,:))/2;
                curv=factorial(0:6).*(Hp(r,:)+Hm(r,:)-2*H(r,:));
                first=NaN;firstCurv=NaN;
                v=find(abs(lin(2:end))>1e-7,1,'first');
                if ~isempty(v),first=v;end
                v=find(abs(curv(2:end))>1e-5,1,'first');
                if ~isempty(v),firstCurv=v;end
                maxEarly4=max(abs(lin(1:4)));
                maxEarly6=max(abs(lin(1:6)));
                n=n+1;
                rows(n,:)={id,shapes{si},kinds{ki},distance,labels{r},ch,...
                    first,firstCurv,lin(3),lin(5),lin(7),curv(5),curv(7),...
                    maxEarly4,maxEarly6,max(abs(curv)),''};
            end
        end
    catch ME
        n=n+1;rows(n,:)={id,shapes{si},kinds{ki},distance,'TEST_ERROR',...
            NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,...
            [ME.identifier ': ' ME.message]};
    end
   end
  end
 end
end
report=cell2table(rows,'VariableNames',...
 {'stateId','shape','obstacleType','distance','barrier','channel',...
 'firstDetectedInputOrder','firstDetectedCurvatureOrder','inputAtOrder2',...
 'inputAtOrder4','inputAtOrder6','curvatureAtOrder4','curvatureAtOrder6',...
 'maximumEarlierInputBefore4','maximumEarlierInputBefore6',...
 'maximumCurvatureThrough6','errorText'});
valid=~strcmp(report.barrier,'TEST_ERROR');X=report(valid,:);
keys=unique(X(:,{'shape','barrier','channel'}),'rows');
OV=cell(height(keys),9);
for j=1:height(keys)
    ind=strcmp(X.shape,keys.shape{j}) &...
        strcmp(X.barrier,keys.barrier{j}) & X.channel==keys.channel(j);
    S=X(ind,:);f=S.firstDetectedInputOrder;
    f=f(isfinite(f));
    if isempty(f),lo=NaN;hi=NaN;else,lo=min(f);hi=max(f);end
    OV(j,:)={keys.shape{j},keys.barrier{j},keys.channel(j),height(S),...
        lo,hi,sum(isfinite(S.firstDetectedCurvatureOrder)),...
        sum(S.maximumEarlierInputBefore6>1e-7),...
        sum(abs(S.curvatureAtOrder6)>1e-5)};
end
overview=cell2table(OV,'VariableNames',...
 {'shape','barrier','channel','sampleCount','minimumDetectedInputOrder',...
 'maximumDetectedInputOrder','nonlinearCasesThroughOrder6',...
 'earlyActionBefore6','nonaffineAtOrder6'});
out=CBF_V11_OutputFolder();
writetable(report,fullfile(out,'v14_r2_input_order_sweep.csv'));
writetable(overview,fullfile(out,'v14_r2_input_order_overview.csv'));
fprintf('[V14] R2 physical-channel per-order probes: %d rows / %d summaries.\n',...
 height(report),height(overview));
end
