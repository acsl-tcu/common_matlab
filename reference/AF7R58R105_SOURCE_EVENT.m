function [F,D]=AF7R58R105_SOURCE_EVENT(O,eventTime,expectedVersion)
% Get EXACT time/version EKF+KNOWN geometry from pre-existing Flight traces.
% Try R55 packet, R71R2 full-EKF capture, then R63 EKF + exact R88.
% Never reconstruct EKF from a nominal reference or present reference-only
% future trajectories as HLC/plant physical forecasts.
F=struct([]);
D=struct('status','NO_AUTHENTIC_SAME_EVENT_EKF_FRAME', ...
    'source','NONE','time_s',double(eventTime), ...
    'reference_version',double(expectedVersion), ...
    'r55_status','NOT_CHECKED','r71r2_status','NOT_CHECKED', ...
    'r63_status','NOT_CHECKED','r55_ring_count',NaN, ...
    'r55_oldest_time_s',NaN,'r55_newest_time_s',NaN, ...
    'missing_r55_properties',{{}},'r88_exact_snapshot',false, ...
    'ekf_frame_authenticated',false,'hlc_physical_future_proved',false, ...
    'error_identifier','','error_message','');
if ~(isscalar(eventTime)&&isnumeric(eventTime)&&isfinite(eventTime)&& ...
       isscalar(expectedVersion)&&isnumeric(expectedVersion)&& ...
       isfinite(expectedVersion)&&expectedVersion==fix(expectedVersion))
    D.status='INVALID_EVENT_TIME_OR_VERSION';return;
end
if ~(isscalar(O)&&(isstruct(O)||isobject(O)))
    D.status='INVALID_CAPTURE_OWNER';return;
end
% R88 binds known-obstacle state, time, version, and mass, but has NO EKF.
% It can authenticate an R63 EKF frame only when both are from one clock tick.
Z=struct([]);
if local_has(O,'r88_snapshots')
    try
        snaps=O.r88_snapshots;
        if isstruct(snaps)&&~isempty(snaps)&&isfield(snaps,'time_s')
            ids=find([snaps.time_s]==double(eventTime));
            ids=ids([snaps(ids).reference_version]==double(expectedVersion));
            if numel(ids)==1
                Z=snaps(ids);
                D.r88_exact_snapshot=true;
            end
        end
    catch ME
        D.error_identifier=char(ME.identifier);
        D.error_message=char(ME.message);
    end
end
% 1) R55: only recorded on particular trigger/capture paths and held in a
% finite ring. Absence is not evidence that no obstacle was nearby.
req={'r55_trigger_ring','r55_trigger_packets','r55_trigger_ring_count'};
missing=req(~cellfun(@(p)local_has(O,p),req));
D.missing_r55_properties=missing;
if ~isempty(missing)
    D.r55_status='R55_CAPTURE_PROPERTIES_NOT_ON_LIVE_OBJECT';
else
    try
        ring=double(O.r55_trigger_ring);
        packet=O.r55_trigger_packets;
        D.r55_ring_count=double(O.r55_trigger_ring_count);
        if ~(isnumeric(ring)&&size(ring,2)>=2&& ...
             iscell(packet)&&numel(packet)==size(ring,1)&& ...
             isfinite(D.r55_ring_count)&&D.r55_ring_count>=0&& ...
             D.r55_ring_count<=size(ring,1))
            D.r55_status='R55_CAPTURE_ARRAY_SCHEMA_INVALID';
        elseif D.r55_ring_count==0
            D.r55_status='R55_RING_EMPTY';
        else
            valid=find(isfinite(ring(:,1))&isfinite(ring(:,2)));
            if ~isempty(valid)
                D.r55_oldest_time_s=min(ring(valid,1));
                D.r55_newest_time_s=max(ring(valid,1));
            end
            matches=find(ring(:,1)==double(eventTime)& ...
                         ring(:,2)==double(expectedVersion));
            if numel(matches)~=1
                if numel(matches)>1
                    D.r55_status='AMBIGUOUS_R55_EVENT_IDENTITY';
                elseif D.r55_ring_count==size(ring,1)&& ...
                       isfinite(D.r55_oldest_time_s)&& ...
                       eventTime<D.r55_oldest_time_s
                    D.r55_status='R55_OLD_EVENT_NOT_RETAINED_IN_RING';
                else
                    D.r55_status='NO_EXACT_R55_TIME_AND_VERSION';
                end
            else
                P=packet{matches(1)};
                if ~isstruct(P)||~isscalar(P)
                    D.r55_status='R55_PACKET_MISSING';
                else
                    candidate=AF7R58R103_FROM_R55_PACKET( ...
                        P,eventTime,expectedVersion);
                    [ok,reason]=local_validate(candidate,Z);
                    D.r55_status=reason;
                    if ok
                        F=candidate;D.source='R55_EXACT_EKF_PACKET';
                        D.status='EXACT_SAME_EVENT_EKF_AND_KNOWN_GEOMETRY';
                        D.ekf_frame_authenticated=true;return;
                    end
                end
            end
        end
    catch ME
        D.r55_status='R55_PACKET_REJECTED';
        D.error_identifier=char(ME.identifier);
        D.error_message=char(ME.message);
    end
end
% 2) Full source-bound R71R2 event: includes EKF 24-state and the actual
% per-event masked obstacle packet. Only exists for selected trigger edges.
if ~local_has(O,'r71r2_event_snapshots')
    D.r71r2_status='R71R2_EVENT_STORAGE_NOT_ON_OBJECT';
else
    try
        S=O.r71r2_event_snapshots;
        if ~isstruct(S)||isempty(S)
            D.r71r2_status='R71R2_NO_CAPTURED_EVENTS';
        else
            idx=find([S.time_s]==double(eventTime)& ...
                     [S.reference_version]==double(expectedVersion));
            if numel(idx)==1
                E=S(idx);
                x=double(E.state_x24(:));
                Q=E.r76_packet;
                if numel(x)~=24||~all(isfinite(x))|| ...
                   ~isstruct(Q)||~isscalar(Q)|| ...
                   ~isfield(E,'r76_known_mask')|| ...
                   ~isfield(E,'r76_radii_m')|| ...
                   ~isfield(E,'r76_d_margin_m')
                    D.r71r2_status='R71R2_INCOMPLETE_EKF_OR_PACKET';
                else
                    P=struct('pL_EKF',x(13:15),'pQ_EKF',x(1:3), ...
                        'active',E.r76_known_mask,'centers',Q.C, ...
                        'rotations',Q.R,'axes',Q.Araw,'ids',Q.ids, ...
                        'radii',[double(E.r76_radii_m(1)) + ...
                            double(E.r76_d_margin_m) + double(Q.delta_payload_m); ...
                            double(E.r76_radii_m(2)) + ...
                            double(E.r76_d_margin_m) + double(Q.delta_uav_m); ...
                            double(E.r76_radii_m(3)) + ...
                            double(E.r76_d_margin_m) + double(Q.delta_cable_m)], ...
                        'H',E.horizon_s,'dt',E.prediction_dt, ...
                        'L',E.cable_length_m);
                    candidate=AF7R58R103_FROM_R55_PACKET( ...
                        P,eventTime,expectedVersion);
                    [ok,reason]=local_validate(candidate,Z);
                    if ok
                        % Actual R71R2 capture order is
                        % [p(1:3); q(4:6); v(7:9); w(10:12);
                        %  pL(13:15); vL(16:18); pT(19:21); wL(22:24)].
                        % Physical cable direction is pT (19:21), not
                        % the UAV yaw/attitude q stored at 4:6.
                        cableDirection=x(19:21);
                        cableLength=double(E.cable_length_m);
                        cableConsistency=norm(x(13:15)-x(1:3) ...
                            -cableLength*cableDirection);
                        qNormError=abs(norm(cableDirection)-1);
                        ok=isfinite(cableConsistency)&& ...
                            qNormError<=1e-5 && ...
                            cableConsistency<=1e-5*max(1,cableLength);
                        if ~ok
                            reason='R71R2_EKF_CABLE_DIRECTION_INCONSISTENT';
                        end
                    end
                    D.r71r2_status=reason;
                    if ok
                        F=candidate;D.source='R71R2_FULL_EKF_SAVED_EVENT';
                        D.status='EXACT_SAME_EVENT_EKF_AND_KNOWN_GEOMETRY';
                        D.ekf_frame_authenticated=true;return;
                    end
                end
            elseif isempty(idx)
                D.r71r2_status='R71R2_NO_EXACT_TIME_VERSION';
            else
                D.r71r2_status='R71R2_AMBIGUOUS_EVENT';
            end
        end
    catch ME
        D.r71r2_status='R71R2_CAPTURE_REJECTED';
        D.error_identifier=char(ME.identifier);
        D.error_message=char(ME.message);
    end
end
% 3) R63 actual EKF endpoints + R88 exact same-tick obstacle authority.
% R88 by itself is never sufficient: it has no EKF position, so it MUST be
% paired with a source event captured on the same Flight cycle.
if ~D.r88_exact_snapshot
    D.r63_status='R63_NEEDS_EXACT_R88_OBSTACLE_SNAPSHOT';
elseif ~local_has(O,'r63_live_authority')
    D.r63_status='R63_EVENT_STORAGE_NOT_ON_OBJECT';
else
    try
        L=O.r63_live_authority;
        if ~isstruct(L)||isempty(L)
            D.r63_status='R63_NO_CAPTURED_EVENTS';
        else
            idx=find([L.time_s]==double(eventTime)& ...
                     [L.reference_version]==double(expectedVersion));
            if numel(idx)==1
                E=L(idx);Q=Z.obstacle_packet;
                if ~isfield(E,'valid')||~logical(E.valid)|| ...
                   ~isfield(E,'current_ekf_pL')|| ...
                   ~isfield(E,'current_ekf_pQ')|| ...
                   ~isfield(E,'current_ekf_q')|| ...
                   ~isfield(E,'sensor_known_count')|| ...
                   ~isequal(double(E.sensor_known_count),nnz(Z.known_mask))|| ...
                   ~isequal(double(E.estimated_mass_kg),double(Z.estimated_mass_kg))
                    D.r63_status='R63_EKF_EVENT_INCOMPLETE_OR_AUTHORITY_MISMATCH';
                else
                    P=struct('pL_EKF',E.current_ekf_pL, ...
                        'pQ_EKF',E.current_ekf_pQ, ...
                        'active',Z.known_mask,'centers',Q.C, ...
                        'rotations',Q.R,'axes',Q.Araw,'ids',Q.ids, ...
                        'radii',[double(Z.rL_m)+double(Z.d_margin_m)+ ...
                            double(Q.delta_payload_m); ...
                            double(Z.rQ_m)+double(Z.d_margin_m)+ ...
                            double(Q.delta_uav_m); ...
                            double(Z.rC_m)+double(Z.d_margin_m)+ ...
                            double(Q.delta_cable_m)], ...
                        'H',Z.horizon_s,'dt',Z.prediction_dt_s, ...
                        'L',Z.cable_length_m);
                    candidate=AF7R58R103_FROM_R55_PACKET( ...
                        P,eventTime,expectedVersion);
                    [ok,reason]=local_validate(candidate,Z);
                    if ok
                        d=double(E.current_ekf_q(:));
                        ok=numel(d)==3&&all(isfinite(d))&& ...
                            norm(candidate.pL_EKF-candidate.pQ_EKF- ...
                                double(Z.cable_length_m)*d)<= ...
                                1e-5*max(1,double(Z.cable_length_m));
                        if ~ok,reason='R63_EKF_CABLE_DIRECTION_INCONSISTENT';end
                    end
                    D.r63_status=reason;
                    if ok
                        F=candidate;D.source='R63_EKF_AND_R88_SAME_TICK';
                        D.status='EXACT_SAME_EVENT_EKF_AND_KNOWN_GEOMETRY';
                        D.ekf_frame_authenticated=true;return;
                    end
                end
            elseif isempty(idx)
                D.r63_status='R63_NO_EXACT_TIME_VERSION';
            else
                D.r63_status='R63_AMBIGUOUS_EVENT';
            end
        end
    catch ME
        D.r63_status='R63_CAPTURE_REJECTED';
        D.error_identifier=char(ME.identifier);
        D.error_message=char(ME.message);
    end
end
D.status='NO_AUTHENTIC_SAME_EVENT_EKF_FRAME__PHYSICAL_TRIGGER_BLOCKED';
end
function tf=local_has(O,name)
tf=false;
try
    if isstruct(O)
        tf=isfield(O,name);
    else
        tf=isprop(O,name);
    end
catch
end
end
function [ok,reason]=local_validate(F,Z)
ok=false;reason='SOURCE_FRAME_GEOMETRY_INVALID';
M=numel(F.ids);
if ~(isequal(size(F.C),[3,M])&&isequal(size(F.R),[9,M])&& ...
     isequal(size(F.Araw),[3,M])&&numel(F.known_mask)==M&& ...
     numel(F.Dcomp)==3&&all(isfinite([F.pL_EKF;F.pQ_EKF]))&& ...
     all(isfinite([F.C(:);F.R(:);F.Araw(:);F.Dcomp(:)]))&& ...
     all(F.Araw(:)>0)&&all(F.Dcomp>=0)&& ...
     numel(F.pL_EKF)==3&&numel(F.pQ_EKF)==3&& ...
     isfinite(F.cable_length_m)&&F.cable_length_m>0&& ...
     abs(norm(F.pL_EKF-F.pQ_EKF)-F.cable_length_m) ...
         <=1e-5*max(1,F.cable_length_m))
    return;
end
% When saved R88 authority exists, insist on EXACT same known mask, version,
% centers/rotations/axes/ids, not just the same obstacle count.
if ~isempty(Z)
    if ~(isequal(double(Z.reference_version),double(F.reference_version))&& ...
         isequal(double(Z.time_s),double(F.time_s))&& ...
         isequal(logical(Z.known_mask(:).'),logical(F.known_mask(:).'))&& ...
         isequaln(double(Z.obstacle_packet.C),F.C)&& ...
         isequaln(double(Z.obstacle_packet.R),F.R)&& ...
         isequaln(double(Z.obstacle_packet.Araw),F.Araw)&& ...
         isequaln(double(Z.obstacle_packet.ids(:).'),F.ids(:).'))
        reason='FRAME_R88_SAME_EVENT_KNOWN_SET_MISMATCH';return;
    end
    Q=Z.obstacle_packet;
    expectedD=[double(Z.rL_m)+double(Z.d_margin_m)+double(Q.delta_payload_m); ...
               double(Z.rQ_m)+double(Z.d_margin_m)+double(Q.delta_uav_m); ...
               double(Z.rC_m)+double(Z.d_margin_m)+double(Q.delta_cable_m)];
    if ~isequaln(double(expectedD(:)),double(F.Dcomp(:)))
        reason='FRAME_R88_INFLATION_AUTHORITY_MISMATCH';return;
    end
end
ok=true;reason='EXACT_EKF_FRAME_AND_KNOWN_GEOMETRY_VALID';
end
