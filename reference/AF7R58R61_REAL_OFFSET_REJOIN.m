function OUT=AF7R58R61_REAL_OFFSET_REJOIN(snapshotPath, maxTrials)
% Reads actual committed R44R2 two-layer Bernstein control points, not synthetic trajectory.
% Constructs a C7 correction-to-zero SHADOW. Not a delayed nominal return,
% NOT a reference/actuator/obstacle proof, and cannot be committed.
% maxTrials limits diagnostic CPU search only; NEVER a maximum flight duration.
assert(ischar(snapshotPath)||isstring(snapshotPath),'R61:Snapshot','Snapshot path required');
assert(isscalar(maxTrials)&&isfinite(maxTrials)&&maxTrials>=3&&fix(maxTrials)==maxTrials,'R61:Budget','Positive integer search budget required');
S=load(snapshotPath,'Snapshot');
assert(isfield(S,'Snapshot')&&isfield(S.Snapshot,'corrections'),'R61:Snapshot','Invalid R44R2 snapshot');
L=S.Snapshot.corrections;
assert(numel(L)==2,'R61:Layers','Source-bound test requires this historical two-layer snapshot');
% Historical witness selects the first layer final knot, from the source data.
% This is NOT a hardcoded production or maneuver time.
t0=double(L(1).segment_t0(end));
J=zeros(3,8);
for a=1:numel(L)
    J=J+localJet(L(a),t0);
end
assert(all(isfinite(J),'all')&&norm(J(:,1))>0,'R61:SourceJets','No finite active correction jets');
% Time scale comes from SOURCE state displacement and derivative.
% If derivative is near zero, use a source segment length as structural seed,
% not a hard-coded external vehicle-speed fallback.
v=norm(J(:,2));
if v>sqrt(eps)*max(1,norm(J(:,1)))
    Tseed=norm(J(:,1))/v;
else
    Tseed=double(L(1).segment_t1(end)-L(1).segment_t0(end));
end
assert(isfinite(Tseed)&&Tseed>0,'R61:SourceTime','Cannot select source-derived search scale');
% Search alternately below/above the source-derived time scale. The explored
% range grows with CPU budget; there is NO fixed maximum maneuver duration.
Tlist=zeros(1,maxTrials);Tlist(1)=Tseed;
for kk=2:maxTrials
    lev=ceil((kk-1)/2);
    if mod(kk,2)==0
        Tlist(kk)=Tseed/(1.25^lev);
    else
        Tlist(kk)=Tseed*(1.25^lev);
    end
end
trials=repmat(struct('T_s',NaN,'one_sided',false,'speed_cp_upper_m_s',NaN, ...
    'join_jet_abs_error',NaN,'end_jet_abs_error',NaN,'worst_cp_progress',NaN,'max_transverse_cp_m',NaN),1,maxTrials);
ixBest=0;best=Inf;bestP=[];
for i=1:maxTrials
    T=Tlist(i); P=zeros(3,16); d=zeros(3,8);
    for k=0:7
        d(:,k+1)=J(:,k+1)*T^k/falling(15,k);
    end
    for j=0:7
        for k=0:j
            P(:,j+1)=P(:,j+1)+nchoosek(j,k)*d(:,k+1);
        end
    end
    q=J(:,1); qnorm=dot(q,q);
    progress=q.'*P/qnorm; % correction multiplier: 1 at start, 0 at end
    transverse=P-q*progress;
    rtol=256*eps(max(1,norm(P,'fro')));
    % This certifies only projection monotonicity, not a straight spatial
    % line: small lateral control-point motion is reported, never hidden.
    oneSided=all(progress>=-rtol) && all(progress<=1+rtol) && ...
       all(diff(progress)<=rtol);
    speedUB=max(vecnorm((15/T)*diff(P,1,2)));
    Js=endpointJets(P,T,false);Je=endpointJets(P,T,true);
    startErr=max(abs(Js(:)-J(:)));endErr=max(abs(Je(:)));
    trials(i)=struct('T_s',T,'one_sided',oneSided,'speed_cp_upper_m_s',speedUB, ...
        'join_jet_abs_error',startErr,'end_jet_abs_error',endErr, ...
        'worst_cp_progress',max(progress), ...
        'max_transverse_cp_m',max(vecnorm(transverse)));
    if oneSided && speedUB<best
        best=speedUB;ixBest=i;bestP=P;
    end
end
OUT=struct('schema','R61_R44R2_SOURCE_SHADOW_V1', ...
    'status','UNRESOLVED_NO_ONE_SIDED_C7_CANDIDATE', ...
    'source_snapshot',char(snapshotPath),'snapshot_version',double(S.Snapshot.version), ...
    'real_source_t0_s',t0,'real_source_offset_jets_0_to_7',J, ...
    'real_source_offset_m',J(:,1),'source_time_scale_s',Tseed, ...
    'candidate_found',ixBest>0,'candidate',struct(), ...
    'trials',trials,'commit_allowed',false,'phase_layer_bound',false, ...
    'yaw_c6_proved',false,'wholebody_continuous_safety_proved',false, ...
    'hlc_forward_proved',false,'robust_tube_installed',false);
if ixBest>0
    b=trials(ixBest);
    OUT.status='REAL_SOURCE_C7_ONE_SIDED_CORRECTION_ONLY__NOT_DELAYED_REJOIN';
    OUT.candidate=struct('t0_s',t0,'T_s',b.T_s,'t1_s',t0+b.T_s, ...
        'degree',15,'control_points',bestP,'initial_jets',J, ...
        'start_jet_error',b.join_jet_abs_error, 'end_jet_error',b.end_jet_abs_error, ...
        'speed_cp_upper_m_s',b.speed_cp_upper_m_s, ...
        'max_transverse_cp_m',b.max_transverse_cp_m, ...
        'one_sided_projected',true);
end
end
function J=localJet(C,t)
    starts=double(C.segment_t0(:));ends=double(C.segment_t1(:));
    idx=find(t>=starts-64*eps(max(1,abs(t))) & t<=ends+64*eps(max(1,abs(t))),1,'last');
    assert(~isempty(idx),'R61:Support','Requested time not in source layer support');
    P=double(C.control_points{idx});
    assert(isequal(size(P),[3,16]),'R61:Degree','Expected actual 3x16 Bezier control points');
    T=ends(idx)-starts(idx); u=(t-starts(idx))/T;
    assert(T>0&&u>=-1e-12&&u<=1+1e-12,'R61:Time','Invalid segment');
    u=min(1,max(0,u));J=zeros(3,8);D=P;
    for k=0:7
        x=D;
        for h=1:(15-k)
            x=(1-u)*x(:,1:end-1)+u*x(:,2:end);
        end
        J(:,k+1)=x(:,1)*falling(15,k)/T^k;
        D=diff(D,1,2);
    end
end
function J=endpointJets(P,T,atEnd)
    J=zeros(3,8);D=P;
    for k=0:7
        if atEnd,z=D(:,end);else,z=D(:,1);end
        J(:,k+1)=z*falling(15,k)/T^k;
        D=diff(D,1,2);
    end
end
function f=falling(n,k)
    f=1;
    for j=0:k-1, f=f*(n-j);end
end
