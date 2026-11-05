function R=TRIGGER_ORIGINAL_PLANT_INTERVAL_RHS(X,U,P)
% Audit candidate: original 24-state physical plant RHS (not HLC).
% Exact source SHA256 (R193 used-Flight): 183837c484f6ec8953998aacb20b93120ce0d6fde647c0e54bcc27f5ae02164a
% The physical RHS is EXACTLY the original real-valued algebra.  Three
% Euler-inertia differences are factored to avoid repeated independent
% occurrences of the SAME angular velocity in interval arithmetic, and
% three rotation-matrix coefficients are intersected with the exact
% [-1,1] bound following from a unit quaternion.  These are only
% mathematical interval-extension improvements; the actual plant and
% its controller are NOT modified.  Source hash must be freshly pinned.
% Uses finite outward double interval operations; trigonometry is
% rigorous bounded Taylor for |x|<=1 and [-1,1] elsewhere.
% NOT a closed-loop certificate: HLC, EKF, control holds, trajectory
% event time, mass envelope, tension, source audit, and output authority
% are still missing. Returns safe_to_commit=false unconditionally.
R=struct('status','UNKNOWN_INTERVAL_RHS_NOT_VALIDATED', ...
    'rhs_lower',nan(24,1),'rhs_upper',nan(24,1), ...
    'outward_elementary_operations',false, ...
    'original_rhs_source_identity_audited',false, ...
    'source_sha256','183837c484f6ec8953998aacb20b93120ce0d6fde647c0e54bcc27f5ae02164a', ...
    'safe_to_commit',false,'reason','');
try
    check_box(X,24,'X');check_box(U,4,'U');check_box(P,21,'P');
    assert(P.lower(1)>0 && P.lower(6)>0 && P.lower(7)>0 && ...
        P.lower(8)>0 && P.lower(21)>0 && P.lower(20)>=0, ...
        'TRIGGER_ORIGINAL_PLANT_INTERVAL_RHS:Physics', ...
        'Positive UAV mass, inertias, cable length and nonnegative payload mass needed.');
    in1=pack_box(X,false);in2=pack_box(U,false);in3=pack_box(P,true);
    dx=original_rhs_24_state(in1,in2,in3);
    assert(numel(dx)==24, 'TRIGGER_ORIGINAL_PLANT_INTERVAL_RHS:ResultSize', ...
        '24-dimensional original RHS required.');
    for k=1:24
        R.rhs_lower(k)=dx(k).lo;
        R.rhs_upper(k)=dx(k).hi;
    end
    assert(all(isfinite(R.rhs_lower))&&all(isfinite(R.rhs_upper)) && ...
        all(R.rhs_lower<=R.rhs_upper));
    R.outward_elementary_operations=true;
    R.status='FINITE_OUTWARD_INTERVAL_RHS__NO_CLOSED_LOOP_OR_SOURCE_AUDIT';
    R.reason=['Arithmetic interval extension only. Require original source identity ' ...
        'audit, certified EKF initial set, interval held HLC, time-varying mass ' ...
        'bounds, tension and actuator guards, and Picard enclosure.'];
catch ME
    R.reason=[ME.identifier ' | ' ME.message];
end
end

function dx=original_rhs_24_state(in1,in2,in3)
% Original symbolic MATLAB expression body starts below (verbatim).
cableL = in3(:,21);
dp1 = in1(7,:);
dp2 = in1(8,:);
dp3 = in1(9,:);
dpl1 = in1(16,:);
dpl2 = in1(17,:);
dpl3 = in1(18,:);
gravity = in3(:,9);
jx = in3(:,6);
jy = in3(:,7);
jz = in3(:,8);
m = in3(:,1);
mL = in3(:,20);
o1 = in1(10,:);
o2 = in1(11,:);
o3 = in1(12,:);
ol1 = in1(22,:);
ol2 = in1(23,:);
ol3 = in1(24,:);
pT1 = in1(19,:);
pT2 = in1(20,:);
pT3 = in1(21,:);
pitch = in1(5,:);
roll = in1(4,:);
u1 = in2(1,:);
u2 = in2(2,:);
u3 = in2(3,:);
u4 = in2(4,:);
yaw = in1(6,:);
t2 = cos(pitch);
t3 = cos(roll);
t4 = sin(pitch);
t5 = sin(roll);
t6 = m+mL;
t7 = ol1.*pT2;
t8 = ol2.*pT1;
t9 = ol1.*pT3;
t10 = ol3.*pT1;
t11 = ol2.*pT3;
t12 = ol3.*pT2;
t13 = 1.0./cableL;
t14 = -gravity;
t15 = 1.0./jx;
t16 = 1.0./jy;
t17 = 1.0./jz;
t18 = 1.0./m;
t23 = pitch./2.0;
t24 = roll./2.0;
t25 = yaw./2.0;
t19 = 1.0./t2;
t20 = -t8;
t21 = -t10;
t22 = -t12;
t26 = cos(t23);
t27 = cos(t24);
t28 = cos(t25);
t29 = sin(t23);
t30 = sin(t24);
t31 = sin(t25);
t32 = 1.0./t6;
t33 = t7+t20;
t34 = t9+t21;
t35 = t11+t22;
t39 = t26.*t27.*t28;
t40 = t26.*t27.*t31;
t41 = t26.*t28.*t30;
t42 = t27.*t28.*t29;
t43 = t26.*t30.*t31;
t44 = t27.*t29.*t31;
t45 = t28.*t29.*t30;
t46 = t29.*t30.*t31;
t36 = t33.^2;
t37 = t34.^2;
t38 = t35.^2;
t47 = -t44;
t48 = -t45;
t52 = t39+t46;
t53 = t42+t43;
t49 = t36+t37+t38;
t54 = t40+t48;
t55 = t41+t47;
t56 = t52.^2;
t57 = t53.^2;
t62 = t52.*t53.*2.0;
t50 = cableL.*m.*t49;
t58 = t54.^2;
t59 = t55.^2;
t60 = -t57;
t63 = t52.*t55.*2.0;
t64 = t53.*t54.*2.0;
t66 = t54.*t55.*2.0;
t61 = -t59;
t65 = -t64;
t67 = t62+t66;
% For the original unit quaternion q=[t52,t53,t54,t55],
% t67=2(q0*q1+q2*q3).  By 2|ab|<=a^2+b^2 and sum(q.^2)=1,
% its EXACT real value belongs to [-1,1] for all real Euler angles.
% Intersection of independently valid enclosures preserves inclusion.
t67 = TRIGGER_OUTWARD_INTERVAL(max(-1,t67.lo),min(1,t67.hi));
t68 = t63+t65;
% t68=2(q0*q3-q1*q2) is another unit-rotation coefficient.
t68 = TRIGGER_OUTWARD_INTERVAL(max(-1,t68.lo),min(1,t68.hi));
t69 = pT1.*t67.*u1;
t70 = pT2.*t67.*u1;
t71 = pT3.*t67.*u1;
t77 = t56+t58+t60+t61;
% t77=q0^2+q2^2-q1^2-q3^2, so -1<=t77<=1 exactly.
t77 = TRIGGER_OUTWARD_INTERVAL(max(-1,t77.lo),min(1,t77.hi));
t72 = pT1.*t68.*u1;
t73 = pT2.*t68.*u1;
t74 = pT3.*t68.*u1;
t78 = pT1.*t77.*u1;
t79 = pT2.*t77.*u1;
t80 = pT3.*t77.*u1;
t81 = t70+t72;
t82 = t74+t79;
t85 = -pT1.*t32.*(t50-t69+t73-t80);
t86 = -pT2.*t32.*(t50-t69+t73-t80);
t87 = -pT3.*t32.*(t50-t69+t73-t80);
% EXACT real identities, without changing Euler rigid-body dynamics:
%   (u2+(jy-jz)*o2*o3)/jx,
%   (u3-(jx-jz)*o1*o3)/jy,
%   (u4+(jx-jy)*o1*o2)/jz.
% The original generated RHS duplicated each o_i*o_j term on both sides
% of an interval subtraction, producing wide ARTIFICIAL derivative boxes.
mt1 = [dp1;dp2;dp3;t19.*(o1.*t2+o3.*t3.*t4+o2.*t4.*t5);o2.*t3-o3.*t5;t19.*(o3.*t3+o2.*t5);t85-cableL.*(ol2.*t33+ol3.*t34-pT2.*t13.*t18.*t81-pT3.*t13.*t18.*(t71-t78));t86-cableL.*(-ol1.*t33+ol3.*t35+pT1.*t13.*t18.*t81+pT3.*t13.*t18.*t82);t14+t87+cableL.*(ol1.*t34+ol2.*t35+pT2.*t13.*t18.*t82-pT1.*t13.*t18.*(t71-t78));t15.*(u2+(jy-jz).*o2.*o3);t16.*(u3-(jx-jz).*o1.*o3);t17.*(u4+(jx-jy).*o1.*o2);dpl1;dpl2;dpl3;t85;t86;t14+t87;t35;-t9+t10;t33;-t13.*t18.*t82];
mt2 = [-t13.*t18.*(t71-t78);t13.*t18.*t81];
dx = [mt1;mt2];
end

function check_box(A,n,name)
assert(isstruct(A)&&isscalar(A)&&all(isfield(A,{'lower','upper'})), ...
    'TRIGGER_ORIGINAL_PLANT_INTERVAL_RHS:Fields','%s: interval bounds required',name);
lo=A.lower;hi=A.upper;
assert(isnumeric(lo)&&isnumeric(hi)&&isreal(lo)&&isreal(hi)&& ...
    isequal(size(lo),[n 1])&&isequal(size(hi),[n 1])&& ...
    all(isfinite(lo))&&all(isfinite(hi))&&all(lo<=hi), ...
    'TRIGGER_ORIGINAL_PLANT_INTERVAL_RHS:Bounds','%s: invalid interval box',name);
end

function B=pack_box(A,asRow)
n=numel(A.lower);
if asRow
    B=repmat(TRIGGER_OUTWARD_INTERVAL(0,0),1,n);
else
    B=repmat(TRIGGER_OUTWARD_INTERVAL(0,0),n,1);
end
for k=1:n
    B(k)=TRIGGER_OUTWARD_INTERVAL(double(A.lower(k)),double(A.upper(k)));
end
end
