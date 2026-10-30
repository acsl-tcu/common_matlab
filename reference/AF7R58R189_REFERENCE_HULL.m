function G=AF7R58R189_REFERENCE_HULL(B,K,mask,rad,obj)
% Independent conservative reference geometry proxy. NOT execution.
% pL sampled-free convex hull control points; pQ proxy= pL+L*[0;0;1].
% This is not a physical taut-cable proof when acceleration is nonzero.
G=struct('sufficient',false,'checked',false,'min_gap_m',NaN,...
 'status','UNKNOWN_INCOMPLETE_REFERENCE_PROXY','physical_execution_proved',false);
try
 mask=logical(mask(:).');inds=find(mask);
 L=double(obj.L_cable);
 assert(isfinite(L)&&L>0&&isfinite(rad)&&rad>=0, ...
  'R189:BadRuntimeRadius');
 gap=Inf;
 for k=1:numel(B.control_points)
   P=double(B.control_points{k});
   assert(isequal(size(P),[3,16])&&all(isfinite(P(:))),'R189:BadBezierHull');
   % Hull includes every point of straight proxy cable swept over time.
   for ii=inds
      R=reshape(double(K.R(:,ii)),3,3);aa=double(K.Araw(:,ii));
      cc=double(K.C(:,ii));
      cmean=mean(P,2)+[0;0;L/2];
      V=P-cmean;
      N=[cc-cmean, P(:,1)-cc, P(:,end)-cc, ...
         [1;0;0],[-1;0;0],[0;1;0],[0;-1;0],[0;0;1],[0;0;-1]];
      ok=false;mg=-Inf;
      for j=1:size(N,2)
         n=N(:,j);d=norm(n);if d==0||~isfinite(d),continue;end
         n=n/d;
         h=n'*cc+norm(aa.*(R'*n));
         % both proxy endpoints at ALL Bézier control vertices; min for
         % the interpolated straight cable and for its entire time hull.
         low=min([n'*P,n'*(P+[0;0;L])],[],'all');
         gm=low-h-rad;
         if gm>mg,mg=gm;end
         if gm>0,ok=true;break;end
      end
      gap=min(gap,mg);
      if ~ok
        G.checked=true;G.min_gap_m=gap;
        G.status='UNKNOWN_PROXY_HULL_NOT_SEPARATED';return;
      end
   end
 end
 G.checked=true;G.sufficient=true;G.min_gap_m=gap;
 G.status='PROXY_BERNSTEIN_HULL_SEPARATED__NOT_PHYSICAL_EXECUTION';
catch ME
 G.status=['UNKNOWN_' char(ME.identifier)];
end
end
