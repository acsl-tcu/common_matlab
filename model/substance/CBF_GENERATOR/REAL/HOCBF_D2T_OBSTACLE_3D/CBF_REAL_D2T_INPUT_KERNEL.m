function K=CBF_REAL_D2T_INPUT_KERNEL(T,q,H,B,varargin)
% REV60: ideal constant-ddT quadratic hold, one-cycle stopping set.
% Bernstein control points b0=T, b1=T+H*q/2, b2=T+H*q+H^2*nu/2.
% b3=b2+H*(q+H*nu)/2. All in [Tmin,Tmax] => hold and stop set.
% Within K_T the two endpoint linear constraints are necessary and sufficient.
% Bounds are conservatively rounded in floating point. No motor bandwidth claim.
validateattributes([T;q;H],{'numeric'},{'real','finite','numel',3});
validateattributes(B,{'numeric'},{'real','finite','numel',2,'nonnegative'});
assert(H>0&&B(2)>B(1),'CBF60:InputBounds','Invalid H or thrust limits');
B=double(B(:));T=double(T);q=double(q);H=double(H);
if ~isempty(varargin)
    validateattributes(varargin{1},{'numeric'},{'scalar','real','finite'});
    K=chain(T,q,H,B,double(varargin{1}));return;
end
K=struct('ok',false,'status','INITIAL_STOP_SET_UNCERTIFIED','hold',[Inf;-Inf], ...
 'nu',[Inf;-Inf],'qRange',[-2*(T-B(1))/H;2*(B(2)-T)/H], ...
 'startMargin',[T+H*q/2-B(1);B(2)-T-H*q/2], 'stop',-q/H);
if T<B(1)||T>B(2),return;end
b1=ia(iv(T),im(iv(.5),im(iv(H),iv(q))));
if b1(1)<B(1)||b1(2)>B(2),return;end
% For initial K_T, b0,b1 inside; hold iff b2 inside (quadratic Bezier).
lo=2*(B(1)-T-H*q)/H^2;hi=2*(B(2)-T-H*q)/H^2;
K.hold=[lo;hi];
lo=max(lo,(B(1)-T)/H^2-1.5*q/H);
hi=min(hi,(B(2)-T)/H^2-1.5*q/H);
stop=K.stop;
if ~all(isfinite([lo;hi;stop]))||lo>hi||~chain(T,q,H,B,stop)
 K.status='STOP_WITNESS_NOT_CERTIFIED';return;end
% Round both ends toward a certified witness. Convexity proves interval.
for side=1:2
 if side==1,a=lo;else,a=hi;end
 if ~chain(T,q,H,B,a)
  bad=a;good=stop;
  for k=1:32
   v=a+sign(stop-a)*eps(max(abs(a),realmin));
   if v==a||abs(v-stop)>abs(a-stop),break;end
   a=v;if chain(T,q,H,B,a),good=a;break;end;bad=a;
  end
  if good==stop
   for k=1:64
    mid=bad+(good-bad)/2;if mid==bad||mid==good,break;end
    if chain(T,q,H,B,mid),good=mid;else,bad=mid;end
   end
  end
  a=good;
 end
 if side==1,lo=a;else,hi=a;end
end
K.nu=[lo;hi];K.ok=lo<=hi;K.status='OK';
end
function ok=chain(T,q,H,B,v)
% Certify the actual binary64 terminal state admits a stopping input, too.
if ~cert(T,q,H,B,v),ok=false;return;end
te=T+H*q+.5*H^2*v;qe=q+H*v;
ok=cert(te,qe,H,B,-qe/H);
end
function ok=cert(T,q,H,B,v)
t=iv(T);q0=iv(q);h=iv(H);nu=iv(v);half=iv(.5);
b1=ia(t,im(half,im(h,q0)));b2=ia(ia(t,im(h,q0)),im(half,im(im(h,h),nu)));
qe=ia(q0,im(h,nu));b3=ia(b2,im(half,im(h,qe)));z=[t;b1;b2;b3];
ok=all(z(:,1)>=B(1)&z(:,2)<=B(2));
end
function r=iv(x),x=double(x(:));r=[x,x];end

function r=out(l,h)
if any(~isfinite([l(:);h(:)]))||any(l(:)>h(:)),error('SD52:Interval','Invalid interval arithmetic');end
% One outward ulp at the computed value (at least realmin*eps relative
% to normal scale; avoids relying on exact directed hardware rounding).
r=[l-eps(max(abs(l),realmin)),h+eps(max(abs(h),realmin))];
if any(~isfinite(r(:))),error('SD52:Interval','Overflow');end
end

function r=ia(a,b)
l=a(:,1)+b(:,1);h=a(:,2)+b(:,2);r=out(l,h);
% All ia callers supply the same row counts. Exact zero is identity.
z=all(a==0,2);r(z,:)=b(z,:);z=all(b==0,2);r(z,:)=a(z,:);
end

function r=isub(a,b),r=ia(a,-flip(b,2));end

function r=im(a,b)
c=[a(:,1).*b(:,1),a(:,1).*b(:,2),a(:,2).*b(:,1),a(:,2).*b(:,2)];r=out(min(c,[],2),max(c,[],2));z=all(a==0,2)|all(b==0,2);r(z,:)=0;
end

