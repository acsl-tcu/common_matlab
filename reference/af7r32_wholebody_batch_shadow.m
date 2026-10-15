function G = af7r32_wholebody_batch_shadow(replanner,CTX,P)
% Breadth-batched implementation of the SAME continuous whole-body theorem.
% Shadow only.  Acceptance still uses static midpoint support margins and
% strict h < margin/speed over every leaf.

G=struct('mathematical_pass',false,'certified',false, ...
    'continuous_time',true,'whole_body',true, ...
    'all_detected_obstacles',true,'static_obstacles_only',true, ...
    'sampled_acceptance',false,'finite_cover_speed_tube',true, ...
    'support_function_separation',true, ...
    'corrected_flatness_cable_direction',true, ...
    'ieee754_outward_closed',false,'reason_code',0, ...
    'min_margin_m',NaN,'min_payload_margin_m',NaN, ...
    'min_uav_margin_m',NaN,'min_cable_margin_m',NaN, ...
    'leaf_count',0,'accepted_leaf_count',0,'split_count',0, ...
    'max_depth_used',0,'max_depth_cap',Inf, ...
    'min_specific_force_norm_lb',NaN,'max_qdot_ub',NaN, ...
    'max_q_arc_radius',NaN,'max_q_arc_inflation_m',NaN, ...
    'resource_exhausted',false,'static_separation_open',false, ...
    'representability_open',false,'failure_time_s',NaN, ...
    'min_static_payload_margin_m',NaN,'min_static_uav_margin_m',NaN, ...
    'min_static_cable_margin_m',NaN, ...
    'min_theorem_certifiable_radius_s',NaN, ...
    'cover_theorem','MIDPOINT_STATIC_SUPPORT_MINUS_SPEED_TIMES_HALFWIDTH', ...
    'tracking_inflation_m',0,'reference_only_authority',true, ...
    'final_system_authority',false);

try
    O=CTX.obstacles; LIV=CTX.live;
    if double(O.count)<1, G.reason_code=101; return; end
    dt=double(LIV.dt);
    if ~(isfinite(dt)&&dt>0), G.reason_code=102; return; end

    SC=P.reference_speed_certificate;
    if ~isstruct(SC)||~isfield(SC,'certified')||~logical(SC.certified)
        G.reason_code=102; return;
    end
    VL=double(SC.payload_speed_UB_m_s);
    VQ=double(SC.uav_speed_UB_m_s);
    qdotUB=double(SC.qdot_UB_rad_s);
    sigmaLB=double(SC.specific_force_norm_LB_m_s2);
    trackingInfl=0;
    if isfield(P,'execution_tracking_inflation_m')
        trackingInfl=double(P.execution_tracking_inflation_m);
    end
    if any(~isfinite([VL VQ qdotUB sigmaLB trackingInfl])) || ...
            min([VL VQ qdotUB sigmaLB trackingInfl])<0 || sigmaLB<=0
        G.reason_code=102; return;
    end
    VC=max(VL,VQ);

    th=double(P.tm);
    if isfield(P,'hold_end')&&isfinite(double(P.hold_end))
        th=max(double(P.tm),min(double(P.t1),double(P.hold_end)));
    end

    A=zeros(0,1); B=zeros(0,1); D=zeros(0,1);
    if double(P.tm)>double(P.t0)
        A(end+1,1)=double(P.t0); B(end+1,1)=double(P.tm); D(end+1,1)=0; %#ok<AGROW>
    end
    if th>double(P.tm)
        A(end+1,1)=double(P.tm); B(end+1,1)=th; D(end+1,1)=0; %#ok<AGROW>
    end
    if double(P.t1)>th
        A(end+1,1)=th; B(end+1,1)=double(P.t1); D(end+1,1)=0; %#ok<AGROW>
    end

    minL=inf; minQ=inf; minC=inf;
    minStaticL=inf; minStaticQ=inf; minStaticC=inf;
    minRadius=inf;
    leafCount=0; acceptCount=0; splitCount=0; maxDepth=0;

    rhoL0=double(LIV.r_load)+double(LIV.d_margin)+trackingInfl;
    rhoQ0=double(LIV.r_drone)+double(LIV.d_margin)+trackingInfl;
    rhoC0=double(LIV.r_cable)+double(LIV.d_margin)+trackingInfl;

    while ~isempty(A)
        tc=0.5*(A+B); half=0.5*(B-A);
        representable=isfinite(tc)&tc>A&tc<B;
        if any(~representable)
            j=find(~representable,1,'first');
            G.representability_open=true; G.failure_time_s=tc(j);
            break;
        end

        [pL,aL,vflat]=af7r32_proposal_position_accel_batch(replanner,P,tc.');
        sf=aL+[zeros(2,numel(tc));double(LIV.gravity)*ones(1,numel(tc))];
        ns=sqrt(sum(sf.^2,1));
        good=vflat & isfinite(ns) & ns>=sigmaLB;
        if any(~good)
            G.reason_code=108; return;
        end
        q=-sf./ns;
        pQ=pL-double(LIV.cable_length)*q;

        [cellPass,MM]=af7r31_geometry_batch_mex(2,pL,pQ,half.', ...
            double(O.centers),double(O.rotations_flat),double(O.base_axes), ...
            rhoL0,rhoQ0,rhoC0,VL,VQ);
        cellPass=logical(cellPass(:));
        MM=double(MM);

        leafCount=leafCount+numel(tc);
        maxDepth=max(maxDepth,max(D));

        nextA=zeros(0,1); nextB=zeros(0,1); nextD=zeros(0,1);
        for k=1:numel(tc)
            mL0=MM(1,k); mQ0=MM(2,k); mC0=MM(3,k);
            minStaticL=min(minStaticL,mL0);
            minStaticQ=min(minStaticQ,mQ0);
            minStaticC=min(minStaticC,mC0);

            if ~(all(isfinite([mL0 mQ0 mC0]))&&min([mL0 mQ0 mC0])>0)
                G.static_separation_open=true;
                G.failure_time_s=tc(k);
                break;
            end

            rL=safe_radius(mL0,VL);
            rQ=safe_radius(mQ0,VQ);
            rC=safe_radius(mC0,VC);
            rCert=min([rL rQ rC]);
            minRadius=min(minRadius,rCert);

            if cellPass(k)
                cL=mL0-VL*half(k);
                cQ=mQ0-VQ*half(k);
                cC=mC0-VC*half(k);
                if min([cL cQ cC])<=0
                    G.reason_code=110; return;
                end
                acceptCount=acceptCount+1;
                minL=min(minL,cL); minQ=min(minQ,cQ); minC=min(minC,cC);
            else
                mid=tc(k);
                nextA(end+1,1)=A(k);   nextB(end+1,1)=mid; nextD(end+1,1)=D(k)+1; %#ok<AGROW>
                nextA(end+1,1)=mid;    nextB(end+1,1)=B(k); nextD(end+1,1)=D(k)+1; %#ok<AGROW>
                splitCount=splitCount+1;
            end
        end
        if G.static_separation_open, break; end
        A=nextA; B=nextB; D=nextD;
    end

    G.min_payload_margin_m=minL; G.min_uav_margin_m=minQ; G.min_cable_margin_m=minC;
    G.min_margin_m=min([minL minQ minC]);
    G.leaf_count=leafCount; G.accepted_leaf_count=acceptCount;
    G.split_count=splitCount; G.max_depth_used=maxDepth;
    G.min_static_payload_margin_m=minStaticL;
    G.min_static_uav_margin_m=minStaticQ;
    G.min_static_cable_margin_m=minStaticC;
    G.min_theorem_certifiable_radius_s=minRadius;
    G.min_specific_force_norm_lb=sigmaLB;
    G.max_qdot_ub=qdotUB;
    G.max_q_arc_radius=qdotUB*dt/2;
    G.max_q_arc_inflation_m=double(LIV.cable_length)*qdotUB*dt/2;
    G.tracking_inflation_m=trackingInfl;

    if ~G.static_separation_open && ~G.representability_open && isempty(A) && ...
            acceptCount>0 && isfinite(G.min_margin_m) && G.min_margin_m>0
        G.mathematical_pass=true; G.certified=true; G.reason_code=0;
    elseif G.static_separation_open
        G.reason_code=111;
    elseif G.representability_open
        G.reason_code=112;
    else
        G.reason_code=105;
    end
catch
    G.reason_code=199;
    G.mathematical_pass=false; G.certified=false;
end
end

function r=safe_radius(m,V)
if V==0, r=Inf; else, r=m/V; end
end
