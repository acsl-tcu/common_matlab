function R=TRAJECTORY_C9_27_DIRECTED_ALLBODY_FORCE_INTLAB(piece,observed, ...
    radii,authority,geo,force)
% Conservative OUTWARD INTLAB proof for degree27 ideal taut model.
% Checks ALL time, ALL observed convex-hull supported 12 shapes, ALL 3
% bodies, Flight ground, positive tension and all-time nominal thrust.
% Time subdivision occurs only to sharpen inclusion; UNKNOWN on exhaustion.
% No flatness torque, actual HLC, floating-point MATLAB/C++ error, or
% Flight commit is implied.  This is a slow REFERENCE CERTIFIER, not an
% online substitute for the existing 5ms target fused native oracle.
R=struct('status','UNKNOWN_DIRECTED_C9_INPUT','directed_nominal_proven',false, ...
    'geometry_proven',false,'tension_proven',false,'thrust_proven',false, ...
    'safe_to_commit',false,'nodes',0,'leaves',0,'elapsed_ms',NaN, ...
    'unknown_reason','NOT_RUN');
tk=tic;
if exist('infsup','file')~=2 || exist('intval','file')~=2
    R.status='UNKNOWN_DIRECTED_C9_INTLAB_MISSING';R.elapsed_ms=1000*toc(tk);return
end
try
    [ground,phase]=TRAJECTORY_GROUND_SCOPE(geo);
    if ~strcmp(phase,'f') || ~isstruct(piece) || ...
            ~isfield(piece,'control_points_m') || ...
            ~isequal(size(piece.control_points_m),[3 28]) || ...
            any(~isfinite(piece.control_points_m(:))) || ...
            ~(isscalar(piece.duration_s)&&isfinite(piece.duration_s)&&piece.duration_s>0)
        R.status='UNKNOWN_DIRECTED_C9_FLIGHT_OR_PIECE';return
    end
    m=double(authority.uav_mass_kg);
    ml=double(authority.payload_mass_kg);
    L=double(authority.cable_length_m);
    fmin=double(authority.thrust_limits_N(1));
    fmax=double(authority.thrust_limits_N(2));
    g=double(authority.gravity_world_mps2(:));
    tension=double(authority.minimum_tension_N);
    wguard=double(authority.acceleration_guard_mps2);
    fg=double(force.force_guard_N);
    tol=double(geo.numerical_guard_m);
    maxdepth=min(double(geo.max_subdivision_depth),double(force.max_subdivision_depth));
    maxnodes=min(double(geo.max_node_count),double(force.max_node_count));
    rad=double([radii.uav_m,radii.cable_m,radii.payload_m]);
    if geo.robust_tube_enabled
        x=geo.uncertainty_bounds_m;
        rad=rad+double([x.uav_m,x.cable_m,x.payload_m]);
    end
    if ~(all(isfinite([m,ml,L,fmin,fmax,tension,wguard,fg,tol,rad,g.']))&& ...
            m>0&&ml>0&&L>0&&fmin>=0&&fmax>fmin&& ...
            all(rad>=0)&&all([tension,wguard,fg,tol]>=0) && ...
            maxdepth>=0 && maxdepth==floor(maxdepth) && ...
            maxnodes>=1 && maxnodes==floor(maxnodes))
        R.status='UNKNOWN_DIRECTED_C9_AUTHORITY';return
    end
    prepared=TRAJECTORY_OBSTACLE_SUPPORT.prepare(observed);
    PP=infsup(piece.control_points_m,piece.control_points_m);
    [good,R]=branch(PP,intval(piece.duration_s),0,R);
    if good
        R.status='CONDITIONAL_DIRECTED_C9_GEOMETRY_TENSION_THRUST_ALLTIME';
        R.directed_nominal_proven=true;
        R.geometry_proven=true;
        R.tension_proven=true;
        R.thrust_proven=true;
    else
        R.status=['UNKNOWN_DIRECTED_C9_' R.unknown_reason];
    end
catch ME
    R.status=['UNKNOWN_DIRECTED_C9_EXCEPTION_' ME.identifier];
end
R.elapsed_ms=1000*toc(tk);

    function [ok,stats]=branch(P,T,depth,stats)
        ok=false;
        if stats.nodes>=maxnodes
            stats.unknown_reason='NODE_BUDGET';return
        end
        stats.nodes=stats.nodes+1;
        N=size(P,2)-1;
        D1=(N/T)*(P(:,2:end)-P(:,1:end-1));
        D2=((N-1)/T)*(D1(:,2:end)-D1(:,1:end-1));
        D3=((N-2)/T)*(D2(:,2:end)-D2(:,1:end-1));
        D4=((N-3)/T)*(D3(:,2:end)-D3(:,1:end-1));
        W=infsup(g,g)-D2;
        WC=0.5*(min(double(inf(W)),[],2)+max(double(sup(W)),[],2));
        C=infsup(WC,WC);
        rho=0;
        for idx=1:size(W,2)
            rho=max(rho,double(sup(sqrt(sum((W(:,idx)-C).^2)))));
        end
        lowI=sqrt(sum(C.^2))-infsup(rho,rho);
        low=double(inf(lowI));
        required=intval(tension)/intval(ml)+intval(wguard);
        ta=low>double(sup(required));
        fs=false;allbody=false;
        if ta
            maxwI=sqrt(sum(C.^2))+infsup(rho,rho);
            w1=colnorm_max(-D3);w2=colnorm_max(-D4);
            qdd=intval(w2)/lowI+intval(3)*(intval(w1)/lowI)^2;
            fd=intval(m)*intval(L)*qdd;
            flI=(intval(m)+intval(ml))*lowI-fd;
            fhI=(intval(m)+intval(ml))*maxwI+fd;
            fs=double(inf(flI))>=double(sup(intval(fmin)+intval(fg))) && ...
               double(sup(fhI))<=double(inf(intval(fmax)-intval(fg)));
            if fs
                wbox=infsup(min(double(inf(W)),[],2), ...
                           max(double(sup(W)),[],2));
                w2box=sum(wbox.^2);
                if double(inf(w2box))>0
                    q=wbox/sqrt(w2box);
                    pl=infsup(min(double(inf(P)),[],2), ...
                              max(double(sup(P)),[],2));
                    pu=pl-intval(L)*q;
                    allbody=do_body(pu,pl);
                end
            end
        end
        if ta && fs && allbody
            ok=true;stats.leaves=stats.leaves+1;return
        end
        if depth>=maxdepth
            if ~ta, stats.unknown_reason='TENSION';
            elseif ~fs, stats.unknown_reason='THRUST';
            else, stats.unknown_reason='GEOMETRY';end
            return
        end
        [A,B]=split(P);
        [al,stats]=branch(A,T/2,depth+1,stats);
        if ~al,return;end
        [ar,stats]=branch(B,T/2,depth+1,stats);
        ok=al&&ar;
    end
    function v=colnorm_max(X)
        v=0;
        for ii=1:size(X,2)
            v=max(v,double(sup(sqrt(sum(X(:,ii).^2)))));
        end
    end
    function pass=do_body(U,PL)
        pass=false;
        if ground
            rh=double(sup(intval(rad(:))+intval(tol)));
            if ~(double(inf(U(3)))>rh(1) && ...
                 double(inf(PL(3)))>rh(3) && ...
                 min(double(inf(U(3))),double(inf(PL(3))))>rh(2))
                return
            end
        end
        Uc=(double(inf(U))+double(sup(U)))/2;
        Pc=(double(inf(PL))+double(sup(PL)))/2;
        for idx=1:numel(prepared)
            shape=prepared(idx);
            dirs=[Uc-shape.center,Pc-shape.center, ...
                  (Uc+Pc)/2-shape.center,shape.R,-shape.R];
            separated=false(1,3);
            for ni=1:size(dirs,2)
                n=dirs(:,ni);
                if ~(all(isfinite(n))&&any(n~=0)),continue;end
                NI=infsup(n,n);
                h=shape_support_hi(shape,NI);
                normn=sqrt(sum(NI.^2));
                ulo=double(inf(sum(U.*NI)));
                plo=double(inf(sum(PL.*NI)));
                a=[ulo;min(ulo,plo);plo];
                for bi=1:3
                    hi=double(sup(h+(intval(rad(bi))+intval(tol))*normn));
                    if a(bi)>hi,separated(bi)=true;end
                end
                if all(separated),break;end
            end
            if ~all(separated),return;end
        end
        pass=true;
    end
    function h=shape_support_hi(s,n)
        w=infsup(s.R.',s.R.')*n;
        base=sum(infsup(s.center,s.center).*n);
        if ~isempty(s.local_vertices)
            V=infsup(s.local_vertices,s.local_vertices);
            h=max(base+V*w);return
        end
        d=s.dim;
        rr=sqrt(w(1).^2+w(2).^2);
        ww=sqrt(sum(w.^2));
        switch char(s.kind)
            case 'sphere', h=base+intval(d(1))*ww;
            case 'ellipsoid'
                D=infsup(d(:),d(:));h=base+sqrt(sum((D.*w).^2));
            case 'box'
                D=infsup(d(:),d(:));h=base+sum(D.*abs(w))/2;
            case 'cylinder',h=base+intval(d(1))*rr+intval(d(2))*abs(w(3))/2;
            case 'capsule',h=base+intval(d(1))*ww+intval(d(2))*abs(w(3))/2;
            case 'cone'
                h=base+max(intval(d(1))*rr-intval(d(2))*w(3)/2, ...
                           intval(d(2))*w(3)/2);
            case 'frustum'
                h=base+max(intval(d(1))*rr-intval(d(3))*w(3)/2, ...
                           intval(d(2))*rr+intval(d(3))*w(3)/2);
            case 'torus',h=base+intval(d(1))*rr+intval(d(2))*ww;
            otherwise,error('DIRECTED_C9_UNSUPPORTED_SHAPE');
        end
    end
    function [A,B]=split(X)
        nn=size(X,2);A=X;B=X;work=X;
        for jj=1:nn
            A(:,jj)=work(:,1);B(:,nn-jj+1)=work(:,end);
            work=(work(:,1:end-1)+work(:,2:end))/2;
        end
    end
end
