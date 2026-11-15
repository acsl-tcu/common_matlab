function S = TEST_TRIGGER_V5_TRACE_FACTS(R)
% Offline audit of USER-SUPPLIED original HLC 4-second trace.
% No future forecasting or physical error guarantee is implied.
% INPUT: R from TEST_TRIGGER_V4_HLC_STRESS() saved .mat.
if nargin<1 || isempty(R)
    error('V5:MissingR','Pass R loaded from your HLC_STRESS_*.mat');
end
assert(isfield(R,'results') && numel(R.results)==4);
S=repmat(struct('caseName','','status','','errorStart_m',NaN, ...
    'errorPeak_m',NaN,'errorPeakTime_s',NaN,'errorEnd_m',NaN, ...
    'maxCableAngle_deg',NaN,'governorTauAt4s_s',NaN, ...
    'maxDroneSpeed_m_s',NaN,'thrustMin_N',NaN, ...
    'thrustMax_N',NaN,'satObserved',false, ...
    'estimatedLoadMass_kg',NaN),1,4);
for i=1:4
    d=R.results(i); tr=d.nominalTrace;
    assert(strcmp(d.status,'NOMINAL_COMPLETED'));
    assert(size(tr.pLoad,2)==numel(tr.time));
    assert(size(tr.pT,1)==3 && size(tr.input,1)==4);
    tt=reshape(tr.time,1,[]);
    e=sqrt(sum((tr.pLoad-tr.refLoad).^2,1));
    [peak,j]=max(e);
    qCos=max(-1,min(1,-tr.pT(3,:)));
    sp=vecnorm(diff(tr.pDrone,1,2),2,1)./diff(tt);
    u=tr.input(1,:);
    S(i).caseName=char(d.caseName);
    S(i).status=char(d.status);
    S(i).errorStart_m=e(1);
    S(i).errorPeak_m=peak;
    S(i).errorPeakTime_s=tt(j);
    S(i).errorEnd_m=e(end);
    S(i).maxCableAngle_deg=max(acos(qCos))*180/pi;
    S(i).governorTauAt4s_s=tr.governorTau(end);
    S(i).maxDroneSpeed_m_s=max(sp);
    S(i).thrustMin_N=min(u);
    S(i).thrustMax_N=max(u);
    S(i).satObserved=logical(d.saturationSeen);
    S(i).estimatedLoadMass_kg=d.modelEstimatedMass;
end
T=struct2table(S);
disp(T(:,{'caseName','errorStart_m','errorPeak_m', ...
    'errorPeakTime_s','errorEnd_m','maxCableAngle_deg', ...
    'governorTauAt4s_s','maxDroneSpeed_m_s','satObserved'}));
end
