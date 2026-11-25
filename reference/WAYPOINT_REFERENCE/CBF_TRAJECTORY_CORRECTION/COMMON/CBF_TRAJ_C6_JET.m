function jet = CBF_TRAJ_C6_JET(d,w,mode,omega)
% CBF_TRAJ_C6_JET: 28x1 corrections ordered [xyz;yaw] at orders 0..6.
% At EVERY current control instant, the returned jet is the exact derivative
% sequence of ONE analytic local function c_k(s). No inter-sample continuity
% is assumed or enforced. Yaw correction is zero.
% CARRY: c(s)=d+s*w. RETURN: c(s)=exp(-omega*s)*(d+(w+omega*d)*s).
validateattributes(d,{'numeric'},{'vector','numel',3,'real','finite'});
validateattributes(w,{'numeric'},{'vector','numel',3,'real','finite'});
d=double(d(:));w=double(w(:));mode=upper(char(mode));
jet=zeros(28,1);
switch mode
    case 'NOMINAL'
        return
    case 'CARRY'
        jet(1:3)=d;
        jet(5:7)=w;
    case 'RETURN'
        validateattributes(omega,{'numeric'},{'scalar','real','finite','positive'});
        % c'' +2*omega*c' +omega^2*c=0.  Entire 0..6 jet is exact.
        aPrev2=d;
        aPrev1=w;
        jet(1:3)=d;
        jet(5:7)=w;
        for j=2:6
            a=-2*omega*aPrev1-omega^2*aPrev2;
            ix=4*j+(1:3);
            jet(ix)=a;
            aPrev2=aPrev1;
            aPrev1=a;
        end
    otherwise
        error('CBFTraj:UnknownMode','Unknown target correction mode: %s',mode);
end
end
