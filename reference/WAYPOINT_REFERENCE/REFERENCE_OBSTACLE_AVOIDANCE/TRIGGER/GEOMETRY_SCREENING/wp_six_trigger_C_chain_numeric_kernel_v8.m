function possible=wp_six_trigger_C_chain_numeric_kernel_v8( ...
    pD,pL,rhoD,rhoL,accD,accL,t,segments,rBodyD,rBodyL,rCable, ...
    safetyMargin,obsC,obsHalf,coarsePossible)
%#codegen
% Fixed numeric kernel for optional MATLAB Coder MEX acceleration.
% NO object/struct, no original HLC, no future oracle. ONE C sphere chain.
% Designed for 41 knots/40 intervals and observed outer AABBs.
% Every separation checks ALL 2+segments+1 body AABBs, as in MATLAB V8.
% It returns possible=true if any body fails sufficient AABB separation.
% Never interprets an unresolved pair as a proven collision.
M=size(pD,2)-1;P=size(obsC,2);seg=double(segments);
possible=coarsePossible;
for k=1:M
    dt=t(k+1)-t(k);
    erD=max(rhoD(k),rhoD(k+1))+accD(k)*dt*dt/8;
    erL=max(rhoL(k),rhoL(k+1))+accL(k)*dt*dt/8;
    cableLength0=sqrt(sum((pL(:,k)-pD(:,k)).^2));
    cableLength1=sqrt(sum((pL(:,k+1)-pD(:,k+1)).^2));
    lengthUpper=max(cableLength0,cableLength1)+erD+erL;
    chainRadius=hypot(lengthUpper/(2*seg),rCable+safetyMargin);
    for j=1:P
        if ~possible(k,j), continue; end
        allSep=true;
        for b=1:seg+3
            if b==1
                v0=pD(:,k);v1=pD(:,k+1);r=erD+rBodyD+safetyMargin;
            elseif b==2
                v0=pL(:,k);v1=pL(:,k+1);r=erL+rBodyL+safetyMargin;
            else
                s=(b-3)/seg;
                v0=(1-s)*pD(:,k)+s*pL(:,k);
                v1=(1-s)*pD(:,k+1)+s*pL(:,k+1);
                r=chainRadius+(1-s)*erD+s*erL;
            end
            sep=false;
            for axis=1:3
                mid=(v0(axis)+v1(axis))*.5;
                half=abs(v1(axis)-v0(axis))*.5+r;
                w=half+obsHalf(axis,j);
                guard=128*eps(max(1,abs(mid)+abs(obsC(axis,j))+w));
                if abs(mid-obsC(axis,j))>w+guard
                    sep=true;break
                end
            end
            if ~sep
                allSep=false;break
            end
        end
        if allSep, possible(k,j)=false; end
    end
end
end
