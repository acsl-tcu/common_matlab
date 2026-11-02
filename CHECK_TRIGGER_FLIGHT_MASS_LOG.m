function R=CHECK_TRIGGER_FLIGHT_MASS_LOG(logger)
% Read-only exact Flight LOGGER mass history, no runner/HLC/EKF changes.
% Distinguish landing mL=0 from invalid mass during actual Flight phase.
assert(nargin==1&&isa(logger,'LOGGER'), ...
 'CHECK_TRIGGER_FLIGHT_MASS_LOG:Logger','Pass the real Flight LOGGER.');
n=logger.k;
assert(n>=1,'CHECK_TRIGGER_FLIGHT_MASS_LOG:Empty','No recorded cycles.');
times=double(logger.Data.t(1:n));
phases=char(logger.Data.phase(1:n));
ml=nan(n,1);
missing=false(n,1);
for k=1:n
 try
  state=logger.Data.agent(1).estimator.result{k}.state;
  if isobject(state) && isprop(state,'mL')
   ml(k)=double(state.mL);
  elseif isstruct(state) && isfield(state,'mL')
   ml(k)=double(state.mL);
  else
   missing(k)=true;
  end
 catch
  missing(k)=true;
 end
end
flight=phases=='f';landing=phases=='l';
invalid_flight=flight & ~(isfinite(ml)&ml>0);
zero_landing=landing & ml==0;
if ~any(flight)
 status='UNKNOWN_NO_FLIGHT_PHASE';
elseif any(invalid_flight)
 status='FLIGHT_MASS_INVALID__TAUT_PREDICTION_NOT_BOUND';
else
 status='FLIGHT_MASS_POSITIVE__NOT_A_PREDICTION_CERTIFICATE';
end
R=struct('status',status,'n',n,'time_s',times, ...
 'phase',phases,'mL_kg',ml, ...
 'flight_count',sum(flight),'landing_count',sum(landing), ...
 'flight_invalid_count',sum(invalid_flight), ...
 'flight_invalid_times_s',times(invalid_flight), ...
 'landing_zero_count',sum(zero_landing), ...
 'first_zero_landing_time_s',NaN, ...
 'missing_count',sum(missing));
if any(zero_landing)
 R.first_zero_landing_time_s=times(find(zero_landing,1,'first'));
end
folder=fileparts(mfilename('fullpath'));
save(fullfile(folder,'TRIGGER_FLIGHT_MASS_LOG_RESULT.mat'),'R');
fid=fopen(fullfile(folder,'TRIGGER_FLIGHT_MASS_LOG_RESULT.txt'),'w');
if fid~=-1
 cleaner=onCleanup(@()fclose(fid));
 fprintf(fid,'status=%s\nflight_count=%d\nflight_invalid_count=%d\n', ...
    status,R.flight_count,R.flight_invalid_count);
 fprintf(fid,'landing_zero_count=%d\nfirst_zero_landing_time_s=%.17g\n', ...
    R.landing_zero_count,R.first_zero_landing_time_s);
 fprintf(fid,'missing_count=%d\n',R.missing_count);
 clear cleaner
end
disp(rmfield(R,{'time_s','phase','mL_kg'}));
end
