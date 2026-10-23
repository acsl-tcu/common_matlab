function R=AF7R58R45_REBUILD_RETURN_LP(snapshotFile,tSplit)
% Independent MATLAB LP reconstruction from ACTUAL saved 2-layer model.
% Two degree-15 correction segments; start C7 jet unchanged; end jets
% cancel existing second layer; signed combined convex-hull >=0; minimize
% maximum control-polygon signed velocity derivative (no tuned weights).
% Offline diagnostic only. The source end time is inherited, NOT a proposed
% production fixed maneuver duration. Requires Optimization Toolbox.
root=fileparts(mfilename('fullpath'));
if nargin<1 || isempty(snapshotFile)
    snapshotFile=fullfile(root,'AF7R58R44R2_ACTIVE_SNAPSHOT.mat');
end
if nargin<2 || isempty(tSplit)
    cf=fullfile(root,'AF7R58R45_NUMERICAL_RETURN_CANDIDATE.mat');
    assert(isfile(cf),'[R45] Split not given and numerical candidate missing.');
    D=load(cf,'Candidate');tSplit=double(D.Candidate.segment_t1(1));
end
assert(isfile(snapshotFile),'[R45] Saved actual snapshot missing.');
D=load(snapshotFile,'Snapshot');S=D.Snapshot;
assert(strcmp(char(S.mode),'REPAIRED')&&numel(S.corrections)==2&& ...
    (~isfield(S,'phase_layers')||isempty(S.phase_layers)), ...
    '[R45] Incompatible actual model or phase layers.');
G1=S.corrections(1);G2=S.corrections(2);
P0=local_get_cp(G1,numel(G1.segment_t0));
P2=local_get_cp(G2,numel(G2.segment_t0));
a=double(G1.segment_t0(end));b=double(G1.segment_t1(end));
a2=double(G2.segment_t0(end));b2=double(G2.segment_t1(end));
assert(b==b2&&a>=a2&&tSplit>a&&tSplit<b, ...
    '[R45] LP split not valid for the actual last two polygons.');
assert(exist('linprog','file')==2,'[R45] MATLAB linprog unavailable.');
N=15;D0=zeros(8,16);D1=zeros(8,16);
for r=0:7
    for j=0:r
        v=(-1)^(r-j)*nchoosek(r,j);
        D0(r+1,j+1)=v;D1(r+1,16-r+j)=v;
    end
end
Told=b-a;T1=tSplit-a;T2=b-tSplit;
J0=zeros(8,1);
for r=0:7
    f=prod((16-r):15);
    if r==0,d=P0(1,:);else,d=diff(P0(1,:),r,2);end
    J0(r+1)=d(1)*f/Told^r;
end
Jend=zeros(8,1);Jend(1)=-P2(1,end);
Aeq=zeros(24,33);beq=zeros(24,1);
for r=0:7
    f=prod((16-r):15);
    Aeq(r+1,1:16)=D0(r+1,:);
    beq(r+1)=J0(r+1)*T1^r/f;
    Aeq(9+r,1:16)=D1(r+1,:);
    Aeq(9+r,17:32)=-(T1/T2)^r*D0(r+1,:);
    Aeq(17+r,17:32)=D1(r+1,:);
    beq(17+r)=Jend(r+1)*T2^r/f;
end
F0=local_restrict(P2,(a-a2)/(b2-a2),(tSplit-a2)/(b2-a2));
F1=local_restrict(P2,(tSplit-a2)/(b2-a2),1);
F=[F0(1,:) F1(1,:)];
% Signed direction = -x is source-bound to R44R2 measured diagnostic.
assert(isequal(size(P0),[3 16]) && ...
    isequal(size(P2),[3 16]),'[R45] Invalid real control-point shape.');
Ai=zeros(32+60,33);bi=zeros(32+60,1);
for i=1:32
    Ai(i,i)=1;
    bi(i)=-F(i);
end
k=32;
for piece=1:2
    if piece==1,dt=T1;ix=0;fcp=F0(1,:);
    else,dt=T2;ix=16;fcp=F1(1,:);end
    for jj=1:15
        for sg=[1 -1]
            k=k+1;
            Ai(k,ix+jj)=-sg*15/dt;
            Ai(k,ix+jj+1)=sg*15/dt;
            Ai(k,33)=-1;
            bi(k)=-sg*15/dt*(fcp(jj+1)-fcp(jj));
        end
    end
end
f=zeros(33,1);f(33)=1;
lb=[-inf(32,1);0];ub=inf(33,1);
opt=optimoptions('linprog','Display','none');
[z,fun,flag,output]=linprog(f,Ai,bi,Aeq,beq,lb,ub,opt);
R=struct('schema','R45_SOURCE_BOUND_RETURN_REBUILD_V1', ...
    'status','FLOATING_POINT_SHAPE_SEARCH_ONLY','exitflag',flag, ...
    'solver_output',output,'source_commit_count',double(S.commit_count), ...
    'candidate_committed',false,'wholebody_collision_certified',false, ...
    'interval_proved',false,'t0',a,'tm',tSplit,'t1',b);
if isempty(z)||flag<=0
    R.status='NO_LP_CANDIDATE__NOT_FEASIBILITY_THEOREM';return;
end
% Lexicographic second LP: among solutions at the minimum signed
% velocity bound, minimize the maximum absolute 7th time derivative of
% the combined signed correction. No arbitrary weighted objective terms.
vAllowed=fun+1e-8; % numerical feasibility slack, NOT a physical margin
A7=zeros(32+60+36,33);b7=zeros(32+60+36,1);
A7(1:32,:)=Ai(1:32,:);b7(1:32)=bi(1:32);
for j=33:92
    A7(j,:)=Ai(j,:);A7(j,33)=0;
    b7(j)=bi(j)+vAllowed;
end
q=92; ord=7;
for piece=1:2
    if piece==1,dt=T1;ix=0;fcp=F0(1,:);
    else,dt=T2;ix=16;fcp=F1(1,:);end
    falling=prod((16-ord):15);
    dc=diff(fcp,ord,2)*falling/dt^ord;
    for jj=1:(16-ord)
        for sg=[1 -1]
            q=q+1;
            for l=0:ord
                A7(q,ix+jj+l)=sg*(-1)^(ord-l)* ...
                    nchoosek(ord,l)*falling/dt^ord;
            end
            A7(q,33)=-1;
            b7(q)=-sg*dc(jj);
        end
    end
end
f7=zeros(33,1);f7(end)=1;
[z7,fun7,flag7,out7]=linprog(f7,A7,b7,Aeq,beq,lb,ub,opt);
R.smoothness_exitflag=flag7;
R.smoothness_solver_output=out7;
if flag7<=0||isempty(z7)
    R.status='SEVENTH_ORDER_MINIMAX_LP_FAILED__NO_CANDIDATE';return;
end
z=z7;
R.minimax_seventh_derivative_cp_bound=fun7;
C=zeros(3,16,2);
C(1,:,1)=z(1:16);C(1,:,2)=z(17:32);
for ax=2:3
    % Unique degree-15 polynomial with zero initial 0..7 jet and final
    % position canceling the actual second correction, final r1..r7=0.
    B=[zeros(1,8) repmat(-P2(ax,end),1,8)];
    Q0=local_restrict(B,0,(tSplit-a)/(b-a));
    Q1=local_restrict(B,(tSplit-a)/(b-a),1);
    C(ax,:,1)=Q0;C(ax,:,2)=Q1;
end
Candidate=struct('schema','R45_NUMERICAL_SHAPE_CANDIDATE_ONLY', ...
    'source_snapshot_commit_count',double(S.commit_count), ...
    'source_snapshot_version',double(S.version), ...
    'segment_t0',[a tSplit],'segment_t1',[tSplit b], ...
    'control_points',C,'signed_axis',[-1;0;0], ...
    'source_original_end_control_points',P0(:,end), ...
    'source_original_start_8cp',P0(:,1:8), ...
    'source_second_layer_last_16cp',P2, ...
    'source_second_layer_terminal_cp',P2(:,end), ...
    'signed_speed_cp_bound_m_s',vAllowed, ...
    'minimax_seventh_derivative_cp_bound',fun7, ...
    'wholebody_collision_certified',false, ...
    'heading_C6_certified',false, ...
    'automatic_commit',false);
R.candidate=Candidate;
R.max_scaled_equality_residual=max(abs(Aeq*z-beq));
R.max_inequality_residual=max(A7*z-b7);
R.signed_cp_speed_bound_m_s=fun;
fn=fullfile(root,'AF7R58R45_REBUILT_RETURN.mat');
save(fn,'Candidate');R.saved_file=fn;
R.status='SOURCE_BOUND_FLOAT_CANDIDATE_SAVED__NOT_CERTIFIED';
fprintf('[R45] LP feasible=%d | signed speed bound %.9g m/s | r7 bound %.9g | saved %s\n',flag7>0,vAllowed,fun7,fn);
end
function P=local_get_cp(G,j)
if iscell(G.control_points),P=double(G.control_points{j});
elseif isnumeric(G.control_points)&&ndims(G.control_points)==3
    P=double(G.control_points(:,:,j));
else,error('[R45] Unsupported snapshot control-point layout.');end
end
function V=local_restrict(Q,u0,u1)
u0=max(0,min(1,u0));u1=max(0,min(1,u1));
if u0>0
    [~,Q]=local_split(Q,u0);
    u1=(u1-u0)/(1-u0);
end
if u1<1,[Q,~]=local_split(Q,u1);end
V=Q;
end
function [A,B]=local_split(P,u)
n=size(P,2);Q=P;A=zeros(size(P));B=zeros(size(P));
A(:,1)=Q(:,1);B(:,end)=Q(:,end);
for j=1:n-1
    Q=(1-u)*Q(:,1:end-1)+u*Q(:,2:end);
    A(:,j+1)=Q(:,1);B(:,n-j)=Q(:,end);
end
end
