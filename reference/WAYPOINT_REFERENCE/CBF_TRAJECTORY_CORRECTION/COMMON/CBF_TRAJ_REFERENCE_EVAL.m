function jet=CBF_TRAJ_REFERENCE_EVAL(method,s,H,base28,baseEval)
% Reconstructed five local BASE reference methods from REV08..12 notes.
% All match actual current post-SLOAD 28-vector exactly. No approximation
% of the correction: caller adds the exact analytic correction jet.
validateattributes(method,{'numeric'},{'scalar','integer','>=',1,'<=',5});
validateattributes(s,{'numeric'},{'scalar','finite','nonnegative','<=',H});
if s==0,jet=base28(:);return;end
j0=reshape(base28,4,7);
if method==1,jet=baseEval(s);jet=jet(:);return;end
% Taylor at current time in normalized coordinates.
a=zeros(4,7);
for j=0:6,a(:,j+1)=j0(:,j+1)*H^j/factorial(j);end
if method==2,jet=reshape(CBF_TRAJ_POLY_JET(a,s,H),28,1);return;end
j1=reshape(baseEval(H),4,7);
switch method
    case 3
        a(:,8)=(j1(:,7)-j0(:,7))*H^6/factorial(7);
        jet=CBF_TRAJ_POLY_JET(a,s,H);
    case 4
        if s==H,jet=j1;else,jet=CBF_TRAJ_HERMITE_JET(j0,j1,s,H);end
    case 5
        if s==H,jet=j1;else
            % flatstep polynomial in u=s/H. Blend left and right Taylor jets
            % using Leibniz, avoiding a degree-19 coefficient expansion.
            b=[zeros(1,7),1716,-9009,20020,-24024,16380,-6006,924];
            f=CBF_TRAJ_POLY_JET(b,s,H);
            a1=zeros(4,7);
            for j=0:6,a1(:,j+1)=j1(:,j+1)*H^j/factorial(j);end
            l=CBF_TRAJ_POLY_JET(a,s,H);r=CBF_TRAJ_POLY_JET(a1,s-H,H);
            jet=l;
            for j=0:6
                for i=0:j
                    jet(:,j+1)=jet(:,j+1)+nchoosek(j,i)*f(i+1)*(r(:,j-i+1)-l(:,j-i+1));
                end
            end
        end
end
jet=jet(:);
end
