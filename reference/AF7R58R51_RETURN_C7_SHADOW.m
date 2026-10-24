function W = AF7R58R51_RETURN_C7_SHADOW(snapshotFile)
% Source-bound, READ-ONLY C7 correction-shape witness from the R44R2 snapshot.
% Not a complete nominal/phase/whole-body/controller certificate. NEVER commit.
if nargin ~= 1 || ~(ischar(snapshotFile) || isstring(snapshotFile))
    error('R51:Input','Pass the exact R44R2 snapshot filename.');
end
D = load(char(snapshotFile),'Snapshot');
assert(isfield(D,'Snapshot'),'R51:Snapshot','Snapshot variable is missing.');
S=D.Snapshot;
assert(isfield(S,'corrections') && numel(S.corrections)>=1, ...
    'R51:Layers','No stored correction control points.');
C=S.corrections;
if iscell(C), C=[C{:}]; end
assert(isstruct(C) && numel(C)>=2,'R51:Layers','Expected the stored two-layer snapshot.');
assert(isfield(S,'phase_layers') && isempty(S.phase_layers), ...
    'R51:Phase','This witness is for the historical no-phase snapshot only.');
startTimes=nan(1,numel(C));
for k=1:numel(C)
    assert(isfield(C(k),'segment_t0') && isfield(C(k),'segment_t1') && ...
        isfield(C(k),'control_points'),'R51:Schema','Missing Bezier fields.');
    startTimes(k)=double(C(k).segment_t0(end));
end
% Choose the latest last-segment start from the ACTUAL stored piecewise data.
t0=max(startTimes);
J=zeros(3,8);
for k=1:numel(C)
    a=double(C(k).segment_t0(:)); b=double(C(k).segment_t1(:));
    cp=C(k).control_points;
    if ~iscell(cp), error('R51:ControlPoints','Expected MATLAB cell control points.'); end
    idx=find(a<=t0+64*eps(max(1,abs(t0))) & b>=t0-64*eps(max(1,abs(t0))),1,'last');
    assert(~isempty(idx),'R51:Coverage','No segment covers the connection time.');
    P=double(cp{idx});
    assert(isequal(size(P),[3,16]) && all(isfinite(P(:))), ...
        'R51:ControlPoints','Requires 3x16 finite Bernstein controls.');
    span=b(idx)-a(idx); assert(span>0,'R51:Span','Invalid segment duration.');
    u=(t0-a(idx))/span;
    for r=0:7
        Q=P;
        for d=1:r
            Q=(16-d)*diff(Q,1,2)/span;
        end
        J(:,r+1)=J(:,r+1)+AF7R58R51_bern_eval(Q,u);
    end
end
W=struct();
W.schema='AF7R58R51_SOURCE_BOUND_C7_RETURN_WITNESS_V1';
W.source='R44R2_ACTUAL_TWO_LAYER_SNAPSHOT';
W.connection_time_s=t0;
W.composite_entry_jets_0_to_7=J;
W.old_offset_m=J(:,1);
W.old_offset_norm_m=norm(J(:,1));
W.existing_phase_layers=numel(S.phase_layers);
W.production_changed=false;
W.valid_for_commit=false;
W.wholebody_certified=false;
W.heading_c6_certified=false;
W.no_same_clock_catchup_certified=false;
W.outward_rounded=false;
W.generated=false;
W.status='NO_ALGEBRAIC_WITNESS';
% Choose a data-derived EXAMPLE duration, not a maneuver cap or requirement.
% It may fail for another snapshot. A production search must optimize time
% freely while respecting live towing, phase, obstacle and controller bounds.
if norm(J(:,1))==0
    W.status='ZERO_ENTRY_OFFSET__NO_STITCH_NEEDED'; return;
end
n=-J(:,1)/norm(J(:,1));
towardVelocity=dot(n,J(:,2));
W.toward_velocity_m_s=towardVelocity;
if ~(isfinite(towardVelocity)&&towardVelocity>0)
    W.status='NO_POSITIVE_ENTRY_RETURN_VELOCITY__NEEDS_GENERAL_SOLVER'; return;
end
T=2*norm(J(:,1))/towardVelocity;
assert(isfinite(T)&&T>0,'R51:Duration','Invalid data-derived witness duration.');
P=zeros(3,16);
for i=0:7
    for r=0:i
        falling=1;
        for h=0:r-1, falling=falling*(15-h); end
        P(:,i+1)=P(:,i+1)+nchoosek(i,r)*T^r/falling*J(:,r+1);
    end
end
% Last eight control points stay zero => terminal position and jets 1..7 zero.
entry=zeros(3,8); Q=P;
for r=0:7
    if r>0,Q=(16-r)*diff(Q,1,2)/T;end
    entry(:,r+1)=Q(:,1);
end
W.candidate_control_points=P;
W.witness_duration_s=T;
W.witness_end_s=t0+T;
W.start_jet_residual_max=max(abs(entry-J),[],'all');
W.end_jet_residual_max=max(abs(P(:,end-7:end)),[],'all');
s=P.'*n;
W.projected_CP=s;
W.projected_CP_increments=diff(s);
W.projected_min_increment=min(diff(s));
W.projected_min_m=min(s);
W.projected_max_m=max(s);
W.correction_speed_Bernstein_upper_m_s=max(vecnorm(15*diff(P,1,2)/T,2,1));
W.directional_no_overshoot_screen=logical(...
    min(diff(s))>=-1e-12 && max(s)<=1e-12 && min(s)>=-norm(J(:,1))-1e-12);
W.generated=logical(W.directional_no_overshoot_screen && ...
    W.start_jet_residual_max <1e-8 && W.end_jet_residual_max <1e-8);
if W.generated
    W.status='C7_MONOTONE_OFFSET_WITNESS__NOT_PHASE_OR_WHOLEBODY_SAFE';
else
    W.status='WITNESS_FAILED__GENERAL_OPTIMIZATION_REQUIRED';
end
end
function v = AF7R58R51_bern_eval(P,u)
% de Casteljau with actual polynomial degree, stable for 0<=u<=1.
assert(u>=-1e-9&&u<=1+1e-9,'R51:Eval','Time outside stored segment.');
u=max(0,min(1,u));
Q=P;
for k=1:size(P,2)-1
    Q=(1-u)*Q(:,1:end-1)+u*Q(:,2:end);
end
v=Q(:,1);
end
