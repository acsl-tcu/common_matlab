function R=AF7R58R43_RETURN_C7_LP(J0,J1,T1,T2,n)
% Two degree-15 Bernstein segments with exact C7 endpoint/junction linear
% equations and convex-hull one-sided sufficient condition. Diagnostic only.
% J0,J1 are 8x3 physical correction time derivatives d^0..d^7 at ends.
assert(isequal(size(J0),[8 3])&&isequal(size(J1),[8 3]), ...
    'Endpoint jets must be 8x3.');
J0=double(J0);J1=double(J1);n=double(n(:));
assert(numel(n)==3 && all(isfinite([J0(:);J1(:);n])) && norm(n)>0 && ...
    isscalar(T1)&&isscalar(T2)&&isfinite(T1)&&isfinite(T2)&&T1>0&&T2>0, ...
    'Invalid jet, direction, or durations.');
n=n/norm(n); D0=zeros(8,16);D1=zeros(8,16); ndeg=15;
Aeq=zeros(72,96);beq=zeros(72,1);
for r=0:7
 for j=0:r
  coeff=(-1)^(r-j)*nchoosek(r,j);
  D0(r+1,j+1)=coeff;
  D1(r+1,ndeg-r+j+1)=coeff;
 end
end
for ax=1:3
 ix1=(ax-1)*16+(1:16);ix2=(ax+2)*16+(1:16);
 rows=(ax-1)*24+(1:8);Aeq(rows,ix1)=D0;
 rowsM=(ax-1)*24+9+(0:7);
 for r=0:7
  Aeq(rowsM(r+1),ix1)=D1(r+1,:);
  Aeq(rowsM(r+1),ix2)=-(T1/T2)^r*D0(r+1,:);
 end
 rowsE=(ax-1)*24+17+(0:7);Aeq(rowsE,ix2)=D1;
 fall=1;
 for r=0:7
  if r>0, fall=fall*(ndeg-r+1);end
  beq((ax-1)*24+r+1)=J0(r+1,ax)*T1^r/fall;
  beq((ax-1)*24+17+r)=J1(r+1,ax)*T2^r/fall;
 end
end
Ai=zeros(32,96);
for piece=1:2
 for j=1:16
  row=(piece-1)*16+j;
  for ax=1:3
   ix=(piece-1)*48+(ax-1)*16+j;
   Ai(row,ix)=-n(ax); % n'*B_{k,j} >= 0 for all control points
  end
 end
end
R=struct('schema','R43_C7_ONE_SIDED_BERNSTEIN_LP_V1', ...
 'feasible',false,'continuous_one_sided_proved',false, ...
 'wholebody_collision_certified',false, ...
 'speed_heading_certified',false,'automatic_commit',false, ...
 'n',n,'T1',T1,'T2',T2);
assert(exist('linprog','file')==2,'MATLAB Optimization Toolbox linprog required.');
opt=optimoptions('linprog','Display','none');
[x,~,exitflag,output]=linprog(zeros(96,1),Ai,zeros(32,1),Aeq,beq,[],[],opt);
R.exitflag=exitflag;R.solver_output=output;
if exitflag<=0||isempty(x)
 R.status='LP_DID_NOT_PROVE_FEASIBILITY_IN_THIS_TWO_SEGMENT_FAMILY';return;
end
B1=reshape(x(1:48),16,3); B2=reshape(x(49:96),16,3);
eqError=max(abs(Aeq*x-beq));signMin=min(-Ai*x);
R.max_scaled_equality_residual=eqError;
R.min_signed_control_point_m=signMin;
R.B1=B1;R.B2=B2;
% Report float residual; this is not outward-rounded rigorous arithmetic.
R.feasible=eqError<=1e-7 && signMin>=-1e-9;
R.continuous_one_sided_proved=false;
R.status='FLOATING_POINT_CANDIDATE_ONLY_REQUIRE_INTERVAL_CERTIFICATION';
end
