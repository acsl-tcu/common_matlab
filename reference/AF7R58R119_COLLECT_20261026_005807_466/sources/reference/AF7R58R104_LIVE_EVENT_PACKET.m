function [F,E]=AF7R58R104_LIVE_EVENT_PACKET(replanner,eventTime)
% Exactly bind current Flight R55 trigger event to the actual sensed packet.
% No substitution with future nominal samples or a later estimator state.
F=struct([]);
E=struct('status','EVENT_NOT_FOUND','event_time_s',eventTime, ...
    'reference_version',NaN,'r55_index',NaN, ...
    'source_reference_provenance','R55_SAME_EVENT_EKF_AND_KNOWN_OBSTACLES', ...
    'physical_forecast_certified',false);
assert(isa(replanner,'REPLANNING_SYSTEM')&&isscalar(replanner)&& ...
    isnumeric(eventTime)&&isscalar(eventTime)&&isfinite(eventTime), ...
    'R104:LiveEventSource','Provide native replanner and exact event time.');
needed={'r55_trigger_ring','r55_trigger_packets','r55_trigger_ring_count'};
assert(all(isprop(replanner,needed)), ...
    'R104:R55CaptureUnavailable','Native R55 event capture not installed.');
ring=double(replanner.r55_trigger_ring);
packets=replanner.r55_trigger_packets;
assert(isnumeric(ring)&&size(ring,2)>=2&&iscell(packets)&& ...
    numel(packets)==size(ring,1), ...
    'R104:R55CaptureFormat','Unexpected source R55 ring/packet layout.');
if replanner.r55_trigger_ring_count==0
    E.status='NO_R55_EVENT_CAPTURED';return;
end
idx=find(ring(:,1)==double(eventTime));
if isempty(idx)
    E.status='NO_EXACT_R55_EVENT_TIME';return;
end
% The ring is per-event and may have multiple identity versions.
% Never choose one by closest time or by a later version.
if numel(idx)~=1
    E.status='AMBIGUOUS_R55_SAME_TIME_EVENTS';return;
end
i=idx(1);P=packets{i};
if ~isstruct(P)||~isscalar(P)
    E.status='R55_PACKET_MISSING';return;
end
version=ring(i,2);
assert(isfinite(version)&&version==fix(version), ...
    'R104:R55InvalidVersion','Captured version must be an integer.');
F=AF7R58R103_FROM_R55_PACKET(P,eventTime,version);
E.status='R55_EVENT_FRAME_BOUND__PHYSICAL_PREVIEW_REQUIRED';
E.reference_version=version;E.r55_index=i;
end
