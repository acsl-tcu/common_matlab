function S=AF7R58R105_C7_SEAM_AUDIT(route,source)
% Evaluate all 0..7 derivatives from BOTH sides at every connector seam
% and from the live phase-shifted nominal source at the final seam.
% This is a numerical implementation audit, not an IEEE continuity proof.
assert(isstruct(route)&&isfield(route,'segments')&& ...
    isfield(route,'knot_jets')&&isfield(route,'times_s')&& ...
    isfield(route,'phase_delay_s'), ...
    'R105:SeamRouteRequired','Real C7 route struct required.');
T=double(route.times_s(:).');K=numel(T);J=double(route.knot_jets);
assert(K>=2&&numel(route.segments)==K-1&& ...
    isequal(size(J),[3,8,K])&&all(isfinite(J(:)))&& ...
    all(diff(T)>0),'R105:SeamSchema','Incorrect C7 knot layout.');
res=zeros(K,8);
for i=1:K
    target=J(:,:,i);
    if i>1
        A=local_jets(route.segments(i-1).payload_cp,T(i)-T(i-1),false);
        res(i,:)=max(res(i,:),max(abs(A-target),[],1));
    end
    if i<K
        B=local_jets(route.segments(i).payload_cp,T(i+1)-T(i),true);
        res(i,:)=max(res(i,:),max(abs(B-target),[],1));
    end
end
Jnom=AF7R58R88_NOMINAL_JETS(source.nominal_derivative_fns, ...
    source.flight_reference_time_origin,T(end)-double(route.phase_delay_s));
Jnom(3,1)=Jnom(3,1)-double(source.cable_length_m);
sourceExactlyMatches=isequaln(Jnom,J(:,:,end));
res(end,:)=max(res(end,:),max(abs(Jnom-J(:,:,end)),[],1));
S=struct('status','NUMERIC_SEAM_AUDITED_NOT_IEEE_PROVED', ...
    'knot_count',K,'max_abs_jet_residual',max(res,[],'all'), ...
    'max_by_order',max(res,[],1), ...
    'finite_nominal_source_matches_last_stored_knot',sourceExactlyMatches, ...
    'all_sections_examined',true,'yaw_c6_mathematically_proved',false, ...
    'physical_execution_certified',false, ...
    'production_commit_authorized',false);
if ~sourceExactlyMatches
    S.status='DISCONTINUOUS_SOURCE_VS_TERMINAL_KNOT';
end
end
function J=local_jets(P,h,begin)
assert(isequal(size(P),[3 16])&&all(isfinite(P(:)))&& ...
    isfinite(h)&&h>0,'R105:SeamBezier','Expected finite 15th degree Bezier.');
J=zeros(3,8);
for d=0:7
    if begin,J(:,d+1)=P(:,1);else,J(:,d+1)=P(:,end);end
    if d<7,P=(size(P,2)-1)/h*diff(P,1,2);end
end
end
