function S=AF7R58R104_PHASE_SCHEDULE(t0,baseT,maxAttempts,ratios)
% Deterministic finite-computation enumeration of independent physical
% maneuver duration T and NOMINAL PHASE advance. Neither the sensing horizon
% nor the nominal same-clock position is used as a maneuver-time deadline.
% Phase lag is a NONNEGATIVE global time shift of the verified nominal law.
assert(isnumeric(t0)&&isscalar(t0)&&isfinite(t0) && ...
    isnumeric(baseT)&&isscalar(baseT)&&isfinite(baseT)&&baseT>0 && ...
    isnumeric(maxAttempts)&&isscalar(maxAttempts)&&isfinite(maxAttempts)&& ...
    maxAttempts>=1&&maxAttempts==fix(maxAttempts), ...
    'R104:ScheduleArguments','Finite time, natural time unit and work budget required.');
if nargin<4||isempty(ratios)
    ratios=[1/4 1/2 1]; % Search sampling only; NOT a physical or time limit.
end
assert(isnumeric(ratios)&&isvector(ratios)&&~isempty(ratios)&& ...
    all(isfinite(ratios))&&all(ratios>0)&&all(ratios<=1), ...
    'R104:PhaseFractions','Search phase fractions must be in (0,1].');
ratios=unique(double(ratios(:).'),'stable');
E=struct('duration_s',NaN,'phase_advance_s',NaN, ...
    'phase_delay_s',NaN,'terminal_time_s',NaN,'nominal_phase_time_s',NaN);
S=repmat(E,1,0);
level=0;
while numel(S)<maxAttempts
    T=baseT*2^level;
    if ~(isfinite(T)&&isfinite(t0+T)&&t0+T>t0),break;end
    for r=1:numel(ratios)
        advance=T*ratios(r);
        delta=T-advance;
        if advance<=0||~isfinite(t0+advance)||t0+advance<=t0,continue;end
        X=E;X.duration_s=T;X.phase_advance_s=advance;
        X.phase_delay_s=delta;X.terminal_time_s=t0+T;
        X.nominal_phase_time_s=t0+advance;
        S(end+1)=X; %#ok<AGROW>
        if numel(S)>=maxAttempts,break;end
    end
    level=level+1;
end
end
