function R=AF7R58R37_RETURN_JET_SIGN_AUDIT(jetLower,jetUpper,direction)
% C7 Taylor-jet necessary local sign test (not geometry or yaw safety).
% jetLower/Upper are certified bounds on 0..7 derivatives, 3x8 each.
% direction points to the POSITIVE half-space for a desired one-sided return.
% Algebraic Taylor sign implication only; numerical rounding is NOT outward-verified.
% A negative result is conditional on valid externally certified jet bounds.
% It does NOT certify global sign, collision safety, or feasibility.
lo=double(jetLower); hi=double(jetUpper);n=double(direction(:));
assert(isequal(size(lo),[3 8])&&isequal(size(hi),[3 8])&& ...
    all(isfinite(lo(:)))&&all(isfinite(hi(:)))&&all(lo(:)<=hi(:)), ...
    'Requires finite 3x8 certified derivative interval bounds.');
assert(numel(n)==3&&all(isfinite(n))&&norm(n)>0, ...
    'Direction must be physically defined, nonzero, real 3-vector.');
n=n/norm(n);
lower=zeros(1,8);upper=zeros(1,8);
for r=1:8
  vlow=lo(:,r);vhigh=hi(:,r);
  lower(r)=sum(n.*( (n>=0).*vlow+(n<0).*vhigh));
  upper(r)=sum(n.*( (n>=0).*vhigh+(n<0).*vlow));
end
first=NaN;sig='INCONCLUSIVE';
for r=1:8
  if upper(r)<0
    first=r-1;sig='NEGATIVE_LOCALLY_UNAVOIDABLE';break
  end
  if lower(r)>0
    first=r-1;sig='POSITIVE_LOCALLY_GUARANTEED';break
  end
  % If the preceding derivative is not PROVEN identically zero, no
  % inference from the sign of a higher derivative is valid.
  if lower(r)~=0 || upper(r)~=0, break;end
end
R=struct('schema','R37_C7_TAYLOR_LOCAL_SIGN_V1', ...
  'sign_result',sig,'first_certified_nonzero_order',first, ...
  'projected_lower',lower,'projected_upper',upper, ...
  'local_only',true,'roundoff_outward_certified',false, ...
  'requires_external_certified_jet_intervals',true, ...
  'general_obstacle_nonintersection_proven',false, ...
  'yaw_C6_proven',false,'global_overshoot_absent_proven',false, ...
  'auto_commit',false);
end
