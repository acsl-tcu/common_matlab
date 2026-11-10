function [zNew, tauNew, tauDer] = wp_governor_step(z,tau,cmd,dt,T)
%WP_GOVERNOR_STEP 6-stage positive filter for the virtual-time rate.
% T * dz1/dt = cmd-z1; T * dzi/dt = z(i-1)-zi (i=2..6)
% dt advances the previous command, assumed held constant over this step.
% z(1:6) continuous -> virtual time tau(t) is C6 under command switches.
% Exact zero-order-hold integration avoids Euler-induced derivative spikes.
arguments
    z (6,1) double {mustBeFinite}
    tau (1,1) double {mustBeFinite}
    cmd (1,1) double {mustBeFinite,mustBeNonnegative}
    dt (1,1) double {mustBeFinite,mustBeNonnegative}
    T (1,1) double {mustBeFinite,mustBePositive}
end
h=dt/T;
zNew=zeros(6,1);
for i=1:6
    accum=0;
    for k=0:i-1
        accum=accum+(h^k/factorial(k))*(z(i-k)-cmd);
    end
    zNew(i)=cmd+exp(-h)*accum;
end
intRate=cmd*dt;
for k=0:5
    intRate=intRate+T*(z(6-k)-cmd)*gammainc(h,k+1,'lower');
end
tauNew=tau+intRate;
% Exact derivatives of tau: each derivative through order 6 depends only
% on continuous filter states, and hence remains continuous at each tick.
tauDer=zeros(7,1);
tauDer(1)=tauNew;
tauDer(2)=zNew(6);
for k=1:5
    val=0;
    for j=0:k
        val=val+(-1)^j*nchoosek(k,j)*zNew(6-k+j);
    end
    tauDer(k+2)=val/T^k;
end
end
