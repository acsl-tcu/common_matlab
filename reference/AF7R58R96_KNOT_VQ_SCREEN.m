function V=AF7R58R96_KNOT_VQ_SCREEN(R,g,L,vLimit)
% Necessary UAV velocity inequality at EVERY internal knot, including
% post-corridor-projection checks. Not a continuous-time speed proof.
assert(isstruct(R)&&isfield(R,'knot_jets')&& ...
    isnumeric(g)&&isscalar(g)&&isfinite(g)&&g>0&& ...
    isnumeric(L)&&isscalar(L)&&isfinite(L)&&L>0&& ...
    isnumeric(vLimit)&&isscalar(vLimit)&&isfinite(vLimit)&&vLimit>0, ...
    'R96:KnotSpeedExternalAuthority','Live g, L and velocity limit required.');
J=R.knot_jets;
assert(size(J,1)==3&&size(J,2)==8, ...
    'R96:KnotJetShape','Expected 3-by-8-by-K jets.');
vmax=0;
V=struct('status','NECESSARY_ALL_KNOT_UAV_SPEED_PASS__NOT_INTERVAL_PROOF', ...
    'necessary_speed_pass',true,'max_speed_m_s',NaN, ...
    'first_failed_knot',NaN,'production_commit_authorized',false);
for k=1:size(J,3)
    v=J(:,2,k);a=J(:,3,k);j=J(:,4,k);
    f=a+[0;0;g];sigma=norm(f);
    if ~isfinite(sigma)||sigma<=128*eps(max([1,g,norm(a)]))
        V.necessary_speed_pass=false;
        V.status='INCONCLUSIVE_TAUT_FORCE_AT_KNOT';return;
    end
    q=f/sigma;qdot=(j-q*(q'*j))/sigma;
    sp=norm(v+L*qdot);vmax=max(vmax,sp);
    if sp>=vLimit-128*eps(max([1,vLimit,sp]))
        V.necessary_speed_pass=false;
        V.first_failed_knot=k;
        V.status='NECESSARY_KNOT_UAV_SPEED_LIMIT_VIOLATED';
        V.max_speed_m_s=vmax;
        return;
    end
end
V.max_speed_m_s=vmax;
end
