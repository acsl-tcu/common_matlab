function R=AF7R58R41_RETURN_SHAPE(candidate,direction)
% Mathematical signed-shape test for any number of Bernstein segments.
% No obstacle/velocity/yaw/cable or floating interval certification.
% Candidate may be R33 two-segment (first/second) or a cell of 3x16
% control polygons. Strictly negative Bernstein controls are not by
% themselves a collision/overshoot witness. Negative evaluated points are.
validateattributes(direction,{'double'},{'real','finite','numel',3});
n=direction(:); assert(norm(n)>0,'[R41] Direction is zero.');
n=n/norm(n);
if isstruct(candidate) && all(isfield(candidate,{'first','second'}))
 B={double(candidate.first.control_points),double(candidate.second.control_points)};
elseif iscell(candidate)
 B=candidate;
else
 error('[R41] Expected exact C7 two-segment candidate or a cell of Bezier polygons.');
end
mins=zeros(1,numel(B)); maxs=mins; samples=mins;
for j=1:numel(B)
 P=double(B{j});
 assert(size(P,1)==3 && size(P,2)>=2 && ...
  all(isfinite(P),'all'),'[R41] Invalid Bezier polygon.');
 c=n.'*P;
 mins(j)=min(c); maxs(j)=max(c);
 % This gives an explicit midpoint witness if negative. It is not
 % a continuous-time certification based on finitely many samples.
 for k=1:size(P,2)-1
  P(:,1:end-k)=0.5*(P(:,1:end-k)+P(:,2:end-k+1));
 end
 samples(j)=n.'*P(:,1);
end
R=struct();
R.schema='AF7R58R41_SIGNED_BEZIER_SHAPE_V1';
R.unit_direction=n;
R.min_signed_controls_m=mins;
R.max_signed_controls_m=maxs;
R.midpoint_samples_m=samples;
R.one_sided_Bernstein_sufficient=all(mins>=0);
R.signed_negative_midpoint_witness=any(samples<0);
R.unknown_if_controls_cross_zero=any(mins<0 & maxs>0);
R.continuous_time_collision_certified=false;
R.finite_precision_outward_rounded=false;
R.c7_jet_identity_checked=false;
R.necessary_overshoot_proved=false;
R.committable=false;
end
