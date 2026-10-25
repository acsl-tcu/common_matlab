function D=AF7R58R95_KNOT_DISPLACEMENT_SCREEN(R,g,L,vLimit)
% Cheap mathematically NECESSARY bound for speed-limited UAV travel
% between C7 knots, using pQ=pL+L*(aL+ge3)/||aL+ge3||.
% This is a generator prefilter, not continuous-time sufficiency.
assert(isstruct(R)&&isfield(R,'knot_jets')&& ...
    isfield(R,'times_s')&&isnumeric(g)&&isscalar(g)&&g>0&& ...
    isnumeric(L)&&isscalar(L)&&L>0&& ...
    isnumeric(vLimit)&&isscalar(vLimit)&&vLimit>0, ...
    'R95:KnotScreenNeedsExternalAuthority');
J=double(R.knot_jets);times=double(R.times_s);K=numel(times);
assert(isequal(size(J),[3,8,K])&&all(diff(times)>0), ...
    'R95:InvalidC7Knots');
Q=zeros(3,K);
D=struct('status','NECESSARY_UAV_KNOT_TRAVEL_PASS__NOT_SUFFICIENT', ...
    'necessary_speed_pass',true,'first_failed_segment',NaN, ...
    'largest_required_average_speed_m_s',NaN, ...
    'production_commit_authorized',false);
for k=1:K
    f=J(:,3,k)+[0;0;g];nf=norm(f);
    if ~(isfinite(nf)&&nf>128*eps(max([1,g,norm(f)])))
        D.necessary_speed_pass=false;
        D.status='INCONCLUSIVE_KNOT_TAUT_DIRECTION';return;
    end
    Q(:,k)=J(:,1,k)+L*f/nf;
end
needed=vecnorm(diff(Q,1,2),2,1)./diff(times);
D.largest_required_average_speed_m_s=max(needed);
idx=find(needed>vLimit+128*eps(max([1,vLimit,needed])),1);
if ~isempty(idx)
    D.necessary_speed_pass=false;
    D.first_failed_segment=idx;
    D.status='NECESSARY_UAV_KNOT_TRAVEL_TIME_VIOLATED';
end
end
