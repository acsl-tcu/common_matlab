function R = AF7R58R45_VERIFY_ACTUAL_RETURN(snapshotFile,candidateFile)
% R45 read-only source-bound verification of the actual two-layer C7
% correction and a replacement last segment. No production mutation.
% Continuous signed shape: Bernstein convex-hull sufficient condition,
% numerically tested (NOT interval/outward-rounded certification).
root=fileparts(mfilename('fullpath'));
if nargin<1 || isempty(snapshotFile)
    snapshotFile=fullfile(root,'AF7R58R44R2_ACTIVE_SNAPSHOT.mat');
end
if nargin<2 || isempty(candidateFile)
    candidateFile=fullfile(root,'AF7R58R45_NUMERICAL_RETURN_CANDIDATE.mat');
end
assert(isfile(snapshotFile),'[R45] Actual R44R2 snapshot missing: %s',snapshotFile);
assert(isfile(candidateFile),'[R45] Actual R45 candidate missing: %s',candidateFile);
U=load(snapshotFile,'Snapshot','R'); V=load(candidateFile,'Candidate');
assert(isfield(U,'Snapshot')&&isfield(V,'Candidate'),'[R45] MAT schema absent.');
S=U.Snapshot; C=V.Candidate;
assert(isfield(S,'schema') && strcmp(char(S.schema),'AF7R58R44R2_ACTIVE_SNAPSHOT_V1'), ...
    '[R45] Unexpected snapshot source schema.');
assert(isfield(S,'mode')&&strcmpi(char(S.mode),'REPAIRED') && ...
    double(S.commit_count)==2 && isstruct(S.corrections) && numel(S.corrections)==2, ...
    '[R45] Requires the actual two-commit REPAIRED model.');
assert(~isfield(S,'phase_layers') || isempty(S.phase_layers), ...
    '[R45] Phase time-warp layers present: unsupported, fail closed.');
assert(isfield(C,'schema') && strcmp(char(C.schema),'R45_NUMERICAL_SHAPE_CANDIDATE_ONLY'), ...
    '[R45] Unexpected candidate schema.');
assert(double(C.source_snapshot_commit_count)==double(S.commit_count) && ...
       double(C.source_snapshot_version)==double(S.version), ...
    '[R45] Candidate and snapshot version mismatch.');
G1=S.corrections(1); G2=S.corrections(2);
assert(numel(G1.segment_t0)==3 && numel(G2.segment_t0)==5, ...
    '[R45] Different segment count: do not use snapshot-bound candidate.');
G1cp=local_polygon_cell(G1);G2cp=local_polygon_cell(G2);
cc=double(C.control_points);
assert(isequal(size(cc),[3 16 2]) && all(isfinite(cc(:))), ...
    '[R45] Invalid new two-piece degree-15 control-point array.');
a=double(G1.segment_t0(end));b=double(G1.segment_t1(end));
assert(isequal(double(C.segment_t0(:).'),[a,double(C.segment_t1(1))]) && ...
    isequal(double(C.segment_t1(:).'),[double(C.segment_t1(1)),b]), ...
    '[R45] Candidate does not replace the existing final segment clock.');
% R45R1: MAT serialization may store a 3-vector as 1x3 (Python scipy)
% or 3x1 (MATLAB). Compare canonical COLUMN vectors exactly, while
% explicitly requiring an actual 3-element vector on both sides.
% Do NOT relax the control-point numerical identity check.
assert(local_exact_vec3(C.source_original_end_control_points,G1cp{end}(:,end)) && ...
    local_exact_vec3(C.source_second_layer_terminal_cp,G2cp{end}(:,end)), ...
    '[R45] Source control-point VALUES differ from the current snapshot.');
% Keep the first two actual segments exactly, replace only the last one.
segments1=local_segments(G1);
segments1(end)=[];
segments1(end+1)=struct('a',a,'b',double(C.segment_t1(1)),'cp',cc(:,:,1));
segments1(end+1)=struct('a',double(C.segment_t1(1)),'b',b,'cp',cc(:,:,2));
segments2=local_segments(G2);
% Same knots for both correction groups; common refinements ensure that
% the 15th-degree Bernstein convex-hull implication applies to their sum.
knots=unique([arrayfun(@(x)x.a,[segments1(:);segments2(:)]), ...
              arrayfun(@(x)x.b,[segments1(:);segments2(:)])]);
rows=zeros(0,5);
minSigned=inf; maxSigned=-inf; maxDcp=0; replacedMaxDcp=0;
axis=double(C.signed_axis(:));
assert(isequal(size(axis),[3 1]) && norm(axis)>0 && all(isfinite(axis)), ...
    '[R45] Invalid source-fixed signed axis.');
axis=axis/norm(axis);
assert(isfield(U,'R')&&isfield(U.R,'return_hull')&& ...
    isfield(U.R.return_hull,'axis') && ...
    isequal(axis,double(U.R.return_hull.axis(:))), ...
    '[R45] Signed direction does not match the actual R44R2 authority.');
for k=1:(numel(knots)-1)
    ta=knots(k);tb=knots(k+1);
    if tb<=ta,continue;end
    Q=zeros(3,16); active=0;
    for group={segments1,segments2}
        Vg=group{1};
        jj=find(arrayfun(@(z)z.a<=ta && tb<=z.b,Vg),1,'first');
        if isempty(jj),continue;end
        active=active+1;
        P=Vg(jj);
        u0=(ta-P.a)/(P.b-P.a);u1=(tb-P.a)/(P.b-P.a);
        Q=Q+local_restrict(P.cp,u0,u1);
    end
    d=axis.'*Q;
    mn=min(d);mx=max(d);
    minSigned=min(minSigned,mn);maxSigned=max(maxSigned,mx);
    vbound=max(sqrt(sum((15/(tb-ta)*diff(Q,1,2)).^2,1)));
    maxDcp=max(maxDcp,vbound);
    if ta>=a,replacedMaxDcp=max(replacedMaxDcp,vbound);end
    rows(end+1,:)=[ta,tb,active,mn,mx]; %#ok<AGROW>
end
% Physical-time derivative continuity, orders 0..7. Test source final
% knot, new knot and total correction returning to zero at 49 sec.
assert(isequal(double(C.source_original_start_8cp),G1cp{end}(:,1:8)) && ...
    isequal(double(C.source_second_layer_last_16cp),G2cp{end}), ...
    '[R45] Snapshot control points changed; candidate is not source-bound.');
Estart=zeros(8,1);Emid=zeros(8,1);Eend=zeros(8,1);
Esource=0;
for gg={G1,G2}
    Gr=gg{1}; Pc=local_polygon_cell(Gr);
    for j=1:(numel(Pc)-1)
        for r=0:7
            de=local_jet(Pc{j},double(Gr.segment_t1(j)-Gr.segment_t0(j)),r,'end');
            ds=local_jet(Pc{j+1},double(Gr.segment_t1(j+1)-Gr.segment_t0(j+1)),r,'start');
            Esource=max(Esource,norm(de-ds));
        end
    end
end
for r=0:7
    src=local_jet(G1cp{end},b-a,r,'start');
    new=local_jet(cc(:,:,1),double(C.segment_t1(1))-a,r,'start');
    Estart(r+1)=norm(src-new);
    j1=local_jet(cc(:,:,1),double(C.segment_t1(1))-a,r,'end');
    j2=local_jet(cc(:,:,2),b-double(C.segment_t1(1)),r,'start');
    Emid(r+1)=norm(j1-j2);
    j3=local_jet(cc(:,:,2),b-double(C.segment_t1(1)),r,'end');
    j4=local_jet(G2cp{end},b-double(G2.segment_t0(end)),r,'end');
    Eend(r+1)=norm(j3+j4);
end
originalEnd=G1cp{end}(:,end)+G2cp{end}(:,end);
candidateEnd=cc(:,end,2)+G2cp{end}(:,end);
R=struct('schema','AF7R58R45_REAL_TWO_LAYER_SHAPE_AUDIT_V1', ...
    'status','SOURCE_BOUND_NUMERICAL_CANDIDATE__NOT_SAFETY_CERTIFIED', ...
    'source_vector_orientation_compatible',true, ...
    'snapshot_used',char(snapshotFile),'candidate_used',char(candidateFile), ...
    'active_version',double(S.version),'commits',double(S.commit_count), ...
    'replaced_from_s',a,'candidate_knot_s',double(C.segment_t1(1)), ...
    'inherited_end_s',b,'signed_axis',axis, ...
    'minimum_signed_combined_control_point_m',minSigned, ...
    'maximum_signed_combined_control_point_m',maxSigned, ...
    'max_combined_correction_velocity_cp_bound_m_s',maxDcp, ...
    'replaced_interval_velocity_cp_bound_m_s',replacedMaxDcp, ...
    'old_terminal_correction_m',originalEnd, ...
    'new_terminal_correction_m',candidateEnd, ...
    'source_segment_junction_C7_residual_max',Esource, ...
    'C7_start_residual_by_order',Estart, ...
    'C7_internal_residual_by_order',Emid, ...
    'C7_terminal_combined_residual_by_order',Eend, ...
    'continuous_one_sided_if_exact_cp',(minSigned>=0), ...
    'floating_point_hull_candidate',(minSigned>=-1e-10), ...
    'candidate_reference_committed',false, ...
    'wholebody_continuous_collision_certified',false, ...
    'yaw_C6_certified',false,'actual_HLC_tracking_certified',false, ...
    'hard_rt_4ms_proved',false, ...
    'outward_rounded_interval_proof',false);
R.intervals=array2table(rows,'VariableNames', ...
    {'t0_s','t1_s','active_layers','signed_min_CP_m','signed_max_CP_m'});
fprintf('[R45] actual %d layers, %d intervals | signed CP min=%+.12g m\n', ...
    numel(S.corrections),size(rows,1),minSigned);
fprintf('[R45] old/new terminal correction norms %.9g / %.9g m\n', ...
    norm(originalEnd),norm(candidateEnd));
fprintf('[R45] max C7 start/junction/end residual %.3g / %.3g / %.3g\n', ...
    max(Estart),max(Emid),max(Eend));
fprintf('[R45] This is a floating-point SHAPE candidate, NOT obstacle safety or commit.\n');
end

function tf=local_exact_vec3(x,y)
% Serialization-invariant exact comparison for shape 1x3 / 3x1 only.
% Matrix reshapes, NaNs, mismatched values or wrong lengths are rejected.
tf=isnumeric(x) && isnumeric(y) && numel(x)==3 && numel(y)==3 && ...
    isvector(x) && isvector(y) && all(isfinite(x(:))) && ...
    all(isfinite(y(:))) && isequal(double(x(:)),double(y(:)));
end

function S=local_segments(G)
P=local_polygon_cell(G);a=double(G.segment_t0(:));b=double(G.segment_t1(:));
S=repmat(struct('a',0,'b',0,'cp',zeros(3,16)),numel(a),1);
for k=1:numel(a),S(k)=struct('a',a(k),'b',b(k),'cp',P{k});end
end
function C=local_polygon_cell(G)
P=G.control_points;
if iscell(P),C=P;
elseif isnumeric(P)&&ndims(P)==3
    C=arrayfun(@(j)P(:,:,j),1:size(P,3),'UniformOutput',false);
elseif isnumeric(P)&&isequal(size(P),[3 16]),C={P};
else,error('[R45] Unsupported canonical polygon layout.');end
assert(numel(C)==numel(G.segment_t0),'[R45] Wrong polygon count.');
for k=1:numel(C)
    assert(isequal(size(C{k}),[3 16])&&all(isfinite(C{k}(:))), ...
        '[R45] Invalid canonical polygon.');
    C{k}=double(C{k});
end
end
function V=local_restrict(Q,u0,u1)
assert(u0>=-1e-12&&u1<=1+1e-12&&u0<=u1, ...
    '[R45] Segment interval outside authority.');
u0=max(0,min(1,u0));u1=max(0,min(1,u1));
if u0>0
    [~,Q]=local_split(Q,u0);
    u1=(u1-u0)/(1-u0);
end
if u1<1
    [Q,~]=local_split(Q,u1);
end
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
function d=local_jet(P,T,r,which)
if r==0,V=double(P);else,V=diff(double(P),r,2);end
f=prod((16-r):15); % (15)!/(15-r)!, empty product = 1 when r=0
if strcmp(which,'start'),d=V(:,1)*f/T^r;
else,d=V(:,end)*f/T^r;end
end
