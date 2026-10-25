function R71=AF7R58R74_COLLECT(varargin)
% R72 postflight collector with locked registry. No Flight runner changes.
% Captured states/references are from the SAME Flight trigger instant.
% 4s HLC/plant numeric prediction is run HERE, not in Flight.
D=fileparts(mfilename('fullpath'));
R71=struct('status','NOT_RUN','tglobal',NaN,'frame',struct(), ...
    'prediction',struct(),'events',struct([]),'capture_available',false, ...
    'error_id','','error_message','', ...
    'mat_file',fullfile(D,'AF7R58R74_OUT.mat'), ...
    'report_file',fullfile(D,'AF7R58R74_REPORT.txt'));
try
    R71.source=AF7R58R74_SOURCE_CHECK();
    R71.registry_before=AF7R58R74_REGISTRY('status');
    if nargin==0
        % Independent of the base workspace AND root appdata lifetime.
        % `clear all` in run_flight_headless cannot clear mlocked registry.
        replanner=AF7R58R74_REGISTRY('get');
        assert(isa(replanner,'REPLANNING_SYSTEM') && isvalid(replanner), ...
            '[R71R2] replanner instance is unavailable.');
        C=replanner.r71r2_event_snapshots;
        assert(~isempty(C), ...
            '[R71R2] no Flight trigger snapshot captured (no valid trigger or not a Flight run).');
        R71.capture_available=true;
        R71.capture_error_count=double(replanner.r73_capture_error_count);
        R71.capture_error_id=char(replanner.r73_last_capture_error_id);
        R71.capture_error_message=char(replanner.r73_last_capture_error_message);
        for j=1:numel(C)
            c=C(j);
            E=struct('status','NOT_RUN','time_s',c.time_s, ...
                'reference_version',c.reference_version, ...
                'frame',struct(),'prediction',struct(), ...
                'error_id','','error_message','');
            try
                % The same-event frame check is available even when
                % controller/plant parameters later change during Flight.
                raw=double(c.raw_reference_fn(c.time_s));raw=raw(:);
                assert(numel(raw)>=3 && all(isfinite(raw(1:3))), ...
                    '[R71R2] invalid snapshot reference.');
                L=double(c.cable_length_m);
                pL=c.state_x24(13:15);
                E.frame=struct('status','SAME_EVENT_FROZEN_EKF_AND_REFERENCE', ...
                    'pre_adjust_position',raw(1:3), ...
                    'hlc_payload_position',raw(1:3)-[0;0;L], ...
                    'ekf_payload_position',pL, ...
                    'raw_to_estimate_gap_m',norm(raw(1:3)-pL), ...
                    'adjusted_to_estimate_gap_m',norm(raw(1:3)-[0;0;L]-pL), ...
                    'cable_length',L);
                % R74: replay ONLY with event-frozen parameters/plant and
                % captured HLC gains. End-of-Flight estimates may differ.
                % This does not authorize a production trigger/certificate.
                E.prediction=AF7R58R74_FROZEN_EVENT_PREVIEW(c);
                E.status=char(E.prediction.status);
            catch ME
                E.status='FAIL_CLOSED_EVENT_REPLAY';
                E.error_id=char(ME.identifier);
                E.error_message=char(ME.message);
            end
            if isempty(R71.events)
                R71.events=E; % first struct establishes the schema
            else
                R71.events(end+1)=E; %#ok<AGROW> postflight only
            end
        end
        R71.tglobal=R71.events(1).time_s;
        R71.frame=R71.events(1).frame;
        R71.prediction=R71.events(1).prediction;
        if all(arrayfun(@(a)isempty(a.error_message),R71.events))
            R71.status='EVENT_FROZEN_POSTFLIGHT_NUMERIC_PREVIEWS__NOT_CERTIFIED';
        else
            R71.status='PARTIAL_OR_FAILED_EVENT_BOUND_PREVIEW__SEE_EVENTS';
            R71.error_message=R71.events(find(arrayfun(@(a)~isempty(a.error_message),R71.events),1)).error_message;
        end
    else
        % Explicit caller-supplied live handle is supported, for compatibility.
        replanner=varargin{1};
        assert(isa(replanner,'REPLANNING_SYSTEM') && isvalid(replanner), ...
            '[R71R2] invalid caller-supplied replanner.');
        assert(nargin>=2 && isscalar(varargin{2}) && isfinite(varargin{2}), ...
            '[R71R2] explicit live call needs a matching runtime time.');
        R71.tglobal=double(varargin{2});
        R71.frame=replanner.r71_report_live_flight_frame(R71.tglobal);
        R71.prediction=replanner.r71_predict_live_hlc_future(R71.tglobal);
        R71.status=char(R71.prediction.status);
    end
catch ME
    R71.status='FAIL_CLOSED_MISSING_OR_INCONSISTENT_RUNTIME_AUTHORITY';
    R71.error_id=char(ME.identifier);
    R71.error_message=char(ME.message);
end
try
    R71.registry_after=AF7R58R74_REGISTRY('status');
catch
    R71.registry_after=struct();
end
save(R71.mat_file,'R71','-v7');
fid=fopen(R71.report_file,'w');
assert(fid>=0,'[R71R2] postflight report could not be opened.');
cl=onCleanup(@()fclose(fid)); %#ok<NASGU>
fprintf(fid,'R74 POSTFLIGHT EVENT-FROZEN HLC+PLANT PREVIEW | REPLANNER ONLY\n');
fprintf(fid,'status=%s | capture=%d | events=%d\n',R71.status, ...
    R71.capture_available,numel(R71.events));
fprintf(fid,'error_id=%s\nerror_message=%s\n',R71.error_id,R71.error_message);
if isfield(R71,'capture_error_count')
    fprintf(fid,'snapshot_capture_errors=%d | last_id=%s | last_message=%s\n', ...
        R71.capture_error_count,R71.capture_error_id,R71.capture_error_message);
end
if isfield(R71,'registry_before')
    B=R71.registry_before;
    fprintf(fid,'registry_before: registered=%d registrations=%d captures=%d event_count=%d\n', ...
        B.registered,B.registration_count,B.capture_count,B.snapshot_count);
end
for j=1:numel(R71.events)
    E=R71.events(j);
    fprintf(fid,'event=%d t=%.9f ver=%d status=%s\n',j,E.time_s,E.reference_version,E.status);
    if isfield(E.frame,'status')
        fprintf(fid,'  raw_EKF_gap_m=%.12g adjusted_EKF_gap_m=%.12g\n', ...
            E.frame.raw_to_estimate_gap_m,E.frame.adjusted_to_estimate_gap_m);
    end
    if isfield(E.prediction,'t')
        fprintf(fid,'  samples=%d mass_kg=%.12g cable_residual_m=%.12g saturation_samples=%d\n', ...
            numel(E.prediction.t),E.prediction.estimated_mass_kg, ...
            E.prediction.max_cable_closure_residual_m, ...
            E.prediction.saturated_sample_count);
    end
    fprintf(fid,'  error_id=%s error_message=%s\n',E.error_id,E.error_message);
end
fprintf(fid,'HLC/EKF/PLANT/runner modified=0 | mass authority=EKF AT EVENT\n');
fprintf(fid,'NOT a real-time trigger. NOT a continuous-time proof. No safety commit.\n');
end
