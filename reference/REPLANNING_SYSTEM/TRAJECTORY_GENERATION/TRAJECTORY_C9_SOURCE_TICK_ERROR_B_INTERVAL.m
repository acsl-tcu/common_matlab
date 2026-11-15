function R=TRAJECTORY_C9_SOURCE_TICK_ERROR_B_INTERVAL(piece,ta,tb, ...
    plant_all_24x2,gravity_world_mps2,cable_length_m)
%TRAJECTORY_C9_SOURCE_TICK_ERROR_B_INTERVAL source-model tick error set.
% Certified *conditional on a valid rigorous plant all-time reach box*.
% Compares physical UAV/payload/entire straight cable to the SAME
% continuous-time nominal flatness curve over [ta,tb], not just samples.
% No arbitrary user-tuned tube radius; INTLAB creates all upper bounds.
% A body B error radius is a SUPREMUM bound, not a covariance/3-sigma.
R=struct('status','UNKNOWN_TICK_B_ERROR_NOT_RUN','valid',false, ...
 'error_radii_m',[NaN NaN NaN], ...
 'nominal_uav_box_3x2',NaN(3,2), ...
 'nominal_payload_box_3x2',NaN(3,2), ...
 'source_plant_reach_assumed',true, ...
 'actual_double_roundoff_proven',false,'safe_to_commit',false);
try
 if exist('intval','file')~=2||exist('infsup','file')~=2
     R.status='UNKNOWN_TICK_B_INTLAB_MISSING';return
 end
 if ~(isstruct(piece)&&isfield(piece,'control_points_m') && ...
      isfield(piece,'t0_s')&&isfield(piece,'duration_s')&& ...
      size(piece.control_points_m,1)==3&& ...
      ismember(size(piece.control_points_m,2),[24 28]) && ...
      all(isfinite(piece.control_points_m(:)))&& ...
      isfinite(piece.duration_s)&&piece.duration_s>0 && ...
      ta>=piece.t0_s&&tb>ta&&tb<=piece.t0_s+piece.duration_s && ...
      isequal(size(plant_all_24x2),[24 2])&& ...
      all(isfinite(plant_all_24x2(:)))&& ...
      all(plant_all_24x2(:,1)<=plant_all_24x2(:,2)) && ...
      numel(gravity_world_mps2)==3&& ...
      isfinite(cable_length_m)&&cable_length_m>0)
     R.status='UNKNOWN_TICK_B_INPUT';return
 end
 T=intval(piece.duration_s);
 a=intval(ta-piece.t0_s)/T;
 b=intval(tb-piece.t0_s)/T;
 if ~(double(inf(a))>=0&&double(sup(b))<=1&& ...
        double(inf(b-a))>0)
     R.status='UNKNOWN_TICK_B_PHASE_BOUNDARY';return
 end
 P=intval(double(piece.control_points_m));
 V=deriv(P,T);W=deriv(V,T);
 Psub=restrict(P,a,b);
 Wsub=restrict(W,a,b);
 g=intval(double(gravity_world_mps2(:)));
 WS=g-Wsub;
 WB=infsup(min(inf(WS),[],2),max(sup(WS),[],2));
 normW=sqrt(sum(WB.^2));
 if double(inf(normW))<=0
     R.status='UNKNOWN_TICK_B_TAUT_DIRECTION';return
 end
 PL=infsup(min(inf(Psub),[],2),max(sup(Psub),[],2));
 U=PL-intval(cable_length_m)*(WB/normW);
 actualU=infsup(plant_all_24x2(1:3,1),plant_all_24x2(1:3,2));
 actualL=infsup(plant_all_24x2(13:15,1),plant_all_24x2(13:15,2));
 eU=radius_upper(actualU-U);
 eL=radius_upper(actualL-PL);
 r=[eU,max(eU,eL),eL];
 if any(~isfinite(r))||any(r<0)
     R.status='UNKNOWN_TICK_B_ERROR_NONFINITE';return
 end
 R.error_radii_m=r;
 R.nominal_uav_box_3x2=[double(inf(U)),double(sup(U))];
 R.nominal_payload_box_3x2=[double(inf(PL)),double(sup(PL))];
 R.valid=true;
 R.status='CONDITIONAL_TICK_B_UAV_CABLE_PAYLOAD_FULLTIME_ERROR_ENCLOSURE__REACH_SOURCE_REQUIRED';
catch ME
 R.status=['UNKNOWN_TICK_B_ERROR_EXCEPTION_' ME.identifier];
 R.error_message=ME.message;
end
end
function z=deriv(P,T)
n=size(P,2)-1;z=(intval(n)/T)*(P(:,2:end)-P(:,1:end-1));
end
function X=restrict(X,a,b)
if double(inf(a))>0
 [~,X]=split(X,a);
end
if double(sup(b))<1
 local=(b-a)/(1-a);
 [X,~]=split(X,local);
end
end
function [L,R]=split(P,s)
n=size(P,2);Q=P;L=intval(zeros(size(P)));R=L;
L(:,1)=Q(:,1);R(:,n)=Q(:,end);
for k=1:n-1
 Q=(1-s)*Q(:,1:end-1)+s*Q(:,2:end);
 L(:,k+1)=Q(:,1);
 R(:,n-k)=Q(:,end);
end
end
function r=radius_upper(E)
Elo=double(inf(E));Ehi=double(sup(E));
a=max(abs(Elo),abs(Ehi));
r=double(sup(sqrt(sum(intval(a).^2))));
end
