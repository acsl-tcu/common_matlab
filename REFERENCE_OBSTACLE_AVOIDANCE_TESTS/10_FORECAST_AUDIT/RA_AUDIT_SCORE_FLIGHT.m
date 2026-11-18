function S=RA_AUDIT_SCORE_FLIGHT(trace)
% Replay flight *past/current EKF and known reference only* into V8 predictor.
% Saved future plant state is accessed ONLY AFTER a forecast has returned,
% for scoring. No future oracle is present in H, K, or routeCache.
r=trace.records;t=[r.time];n=numel(r);
if n<401,error('RAAudit:ShortFlight','Need > 10 s, real 25ms simulation');end
if max(abs(diff(t)-.025))>1e-7
 error('RAAudit:Time','Irregular 25ms flight sample grid');
end
maxTau=max([r.tau]);tauGrid=0:.025:(maxTau+6);
route=wp_six_trigger_route_cache_prepare_v8(trace.generator,tauGrid);
settings=r(find(t>=6,1,'first')).governorSettings;
K=wp_six_trigger_pv_cache_prepare(trace.gains,-1:.025:0,settings(4),.1);
% Empirical margins at exactly these future horizons. Not certified bounds.
horizons=[0 .5 1 2 3 4];idxh=round(horizons/.025);
selected=find(t>=6.0 & t<=t(end)-4.0 & t<47);
selected=selected(1:20:end);  % 0.5s between expensive offline forecasts
nh=numel(horizons);ne=numel(selected);
eLoad=nan(ne,nh);eDrone=eLoad;initialVel=nan(ne,1);
initialLoad=eLoad(:,1);initialDrone=initialLoad;
initialCableAngle=initialLoad;fitRms=nan(ne,3);
forecastMs=nan(ne,1);failText=cell(ne,1);forecastInputSource=cell(ne,1);
for row=1:ne
 i=selected(row);hist=i-40:i;
 cur=r(i);
 estLoad=[r(hist).estLoad];estVel=[r(hist).estLoadVelocity];
 ref=[r(hist).reference];
 if any(~isfinite([estLoad(:);estVel(:);ref(:);cur.estMass(:)])) || ...
         numel(cur.estMass)~=1 || cur.estMass<=0
   failText{row}='invalid_EKF_or_reference';continue
 end
 H=struct();H.pastTimes=K.pastRelativeTimes;
 H.pastLoad=estLoad;H.pastVelocity=estVel;
 H.pastRefLoad=ref(1:3,:);H.pastRefVelocity=ref(5:7,:);
 H.tauNow=cur.tau;H.rateNow=cur.rateState;
 H.heldCommandNow=cur.heldCommand;
 H.estimatedML=cur.estMass;
 % Original configured model only; NOT plant mass or oracle truth.
 H.cableLength=cur.cableLength;H.gravity=cur.gravity;
 H.slowdownStart=cur.governorSettings(1);
 H.slowdownFull=cur.governorSettings(2);
 H.minScale=cur.governorSettings(3);
 H.filterTime=cur.governorSettings(4);
 if numel(cur.estP)==3 && all(isfinite(cur.estP))
     H.currentEKFDronePosition=cur.estP(:);
 end
 if numel(cur.estCableDir)==3 && all(isfinite(cur.estCableDir))
     H.currentEKFCableDirection=cur.estCableDir(:);
 end
 try
   F=wp_six_trigger_pv_forecast_v8(H,K,route);
   forecastMs(row)=F.timeMs;
   forecastInputSource{row}='EKF_history_and_known_route_ONLY';
   initV=estVel(:,end);refV=H.pastRefVelocity(:,end);
   % Score the ACTUAL constrained forecast, not the old unconstrained W.
   fitRms(row,:)=F.initialAnchor.modelFitWeightedRMS;
   initialVel(row)=norm(F.vLoad(:,1)-initV);
   initialLoad(row)=norm(F.pLoad(:,1)-cur.estLoad);
   if all(isfinite(cur.estP))
       initialDrone(row)=norm(F.pDrone(:,1)-cur.estP);
   end
   if all(isfinite(cur.estCableDir)) && norm(cur.estCableDir)>0
       a=dot(F.predCableDirection(:,1),cur.estCableDir(:)/norm(cur.estCableDir));
       initialCableAngle(row)=acos(max(-1,min(1,a)))*180/pi;
   end
   for k=1:nh
      j=i+idxh(k);fp=idxh(k)+1;
      % ORACLE IS READ ONLY HERE, AFTER forecasting.
      eLoad(row,k)=norm(F.pLoad(:,fp)-r(j).oracleLoad);
      eDrone(row,k)=norm(F.pDrone(:,fp)-r(j).oracleDrone);
   end
 catch ME
   failText{row}=[ME.identifier ': ' ME.message];
 end
end
S=finish(eLoad,eDrone,initialLoad,initialVel,initialDrone,...
       initialCableAngle,fitRms,forecastMs,selected,t,horizons,K.condition,failText);
S.traceKind='ACTUAL_FLIGHT_REPLAY_EKF_ONLY';
S.fitVelocityWeight_s=K.velocityWeight;
S.futureOracleDataExcludedFromForecast=true;
S.sourceTruthStateUsedOnlyToScoreAfterForecast=true;
S.noNoiseAssumed=false;
S.safetyCertified=false;
S.horizonSpacing_s=horizons;
end
function B=historyObservationMatrices(gains,t,w)
A6=diag(ones(5,1),1);
Ax=A6;Ax(6,:)=-reshape(gains.F2,1,[]);
Ay=A6;Ay(6,:)=-reshape(gains.F3,1,[]);
Az=[0 1;-reshape(gains.F1,1,[])];
As={Ax,Ay,Az};B=cell(1,3);
for j=1:3
 n=size(As{j},1);M=zeros(numel(t),n);V=M;
 for k=1:numel(t)
   T=expm(As{j}*t(k));M(k,:)=T(1,:);V(k,:)=T(2,:);
 end
 B{j}=[M;w*V];
end
end
function S=finish(a,b,il,iv,id,ic,res,tm,inds,ts,hs,condNum,failText)
combined=max(a,b);
S=struct();S.status='FAIL_FORECASTS';S.horizons_s=hs;
S.startTimes_s=ts(inds);
S.errorsLoad_m=a;S.errorsDrone_m=b;S.errorsCombined_m=combined;
S.validForecastCount=nnz(all(isfinite(combined),2));
S.attemptedForecastCount=numel(inds);
S.failedForecastCount=S.attemptedForecastCount-S.validForecastCount;
S.errorMessages=failText;
S.initialEKFLoadMismatch_m=il;
S.initialEKFLoadVelocityMismatch_mps=iv;
S.initialEKFDroneMismatch_m=id;
S.initialEKFCableDirectionMismatch_deg=ic;
S.modelFitWeightedRMS=res;
S.modalMatrixCondition=condNum;
S.forecastRuntimeMs=tm;
S.p95Combined_m=nan(1,numel(hs));S.maxCombined_m=S.p95Combined_m;
for k=1:numel(hs)
 vals=combined(:,k);vals=vals(isfinite(vals));
 if ~isempty(vals),S.p95Combined_m(k)=quantileNoToolbox(vals,.95);
    S.maxCombined_m(k)=max(vals);end
end
S.maxCombinedError_m=max(combined(:),[],'omitnan');
S.maxAt4s_m=S.maxCombined_m(end);
S.initialMaxDroneMismatch_m=max(id,[],'omitnan');
S.initialMaxLoadMismatch_m=max(il,[],'omitnan');
S.neededMarginEmpiricalMax_m=S.maxCombined_m;
S.marginsToCheck_m=[.05 .1 .25 .5 1 3];
S.coverageAt4s=zeros(size(S.marginsToCheck_m));
for j=1:numel(S.marginsToCheck_m)
 v=combined(:,end);v=v(isfinite(v));
 if isempty(v),S.coverageAt4s(j)=NaN;
 else,S.coverageAt4s(j)=mean(v<=S.marginsToCheck_m(j));end
end
S.margin3mExceededAny=any(combined(:)>3);
S.geometryUsesZeroForecastErrorEnvelope=true;
S.empricalMarginsAreNotProvenUpperBounds=true;
S.initialEKFPositionForcedToMatch=true;
S.initialEKFDronePositionMatchedOnlyWhenCableModelAndEKFAgree=true;
S.physicalErrorUpperBoundCertified=false;
S.status='PASS_SCORED_NOT_CERTIFIED';
if S.validForecastCount==0 || S.failedForecastCount>0
 S.status='FAIL_OR_INCOMPLETE_FORECASTS';
end
end
function y=quantileNoToolbox(a,p)
a=sort(a(:));if isempty(a),y=NaN;return,end
pos=1+(numel(a)-1)*p;lo=floor(pos);hi=ceil(pos);
y=a(lo)+(pos-lo)*(a(hi)-a(lo));
end
