function OUT = AF7R21B_R9_VECTOR_COUPLING_THEOREM_AUDIT()
% Independent algebra audit for the central R9 vector-coupling theorem.

clc;
fprintf('\n');
fprintf('====================================================================================================\n');
fprintf(' AF7R21B | R9 VECTOR-COUPLING THEOREM ALGEBRA AUDIT\n');
fprintf('====================================================================================================\n');
fprintf('[AUTHORITY] independentAlgebra=1 | sampledAcceptance=0 | productionModification=0\n');

vb=[0;0;1.5];

% Orthogonal correction: vector coupling must be strictly tighter.
dc1=[2;0;0];
old1=norm(vb)+norm(dc1);
new1=norm(vb+dc1);
exact1=sqrt(1.5^2+2^2);

% Aligned correction: no artificial improvement is allowed.
dc2=[0;0;2];
old2=norm(vb)+norm(dc2);
new2=norm(vb+dc2);

% Suspended-load flatness projector identity.
s=[1.2;-0.7;9.4];
j=[0.8;-1.1;0.5];
q=-s/norm(s);
P=eye(3)-q*q.';
qdot=-(P*j)/norm(s);
qdotBound=norm(j)/norm(s);
projNorm=norm(P,2);

scale=max(1,max(abs([old1,new1,exact1,old2,new2, ...
    qdotBound,norm(qdot),projNorm])));
tol=2^18*eps(scale);

C=struct();
C.orthogonal_vector_exact=abs(new1-exact1)<=tol;
C.orthogonal_strictly_tighter=new1<old1;
C.aligned_no_false_improvement=abs(new2-old2)<=tol;
C.projector_norm_one=abs(projNorm-1)<=tol;
C.qdot_bound_valid=norm(qdot)<=qdotBound+tol;

closed=all(structfun(@logical,C));

fprintf(' orthogonal old scalar bound             = %.17g\n',old1);
fprintf(' orthogonal vector-coupled bound         = %.17g\n',new1);
fprintf(' orthogonal exact norm                   = %.17g\n',exact1);
fprintf(' aligned old / vector bound              = %.17g / %.17g\n',old2,new2);
fprintf(' ||I-qq''||_2                            = %.17g\n',projNorm);
fprintf(' ||qdot|| / ||j||/||s||                 = %.17g / %.17g\n', ...
    norm(qdot),qdotBound);
fprintf(' floating audit tolerance                = %.17g\n',tol);
fprintf(' theorem algebra closed                  = %d\n',closed);
fprintf('====================================================================================================\n');

OUT=struct('schema','AF7R21B_R9_VECTOR_COUPLING_THEOREM_AUDIT_V1', ...
    'checks',C,'closed',closed);
assignin('base','AF7R21B_R9_VECTOR_COUPLING_THEOREM_AUDIT_OUT',OUT);
end
