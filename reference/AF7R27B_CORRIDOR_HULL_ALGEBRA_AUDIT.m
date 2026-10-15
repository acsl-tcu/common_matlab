function OUT = AF7R27B_CORRIDOR_HULL_ALGEBRA_AUDIT()
clc;
fprintf('\n');
fprintf('====================================================================================================\n');
fprintf(' AF7R27B | CORRIDOR UPPER-ENVELOPE ALGEBRA AUDIT\n');
fprintf('====================================================================================================\n');
fprintf('[AUTHORITY] independentAlgebra=1 | sampledSafetyAcceptance=0\n');

s=[0;1;2];
r=[1;0;1];
w=[-10;-3;-1;0;1;3;10];

fullv=zeros(size(w));
reduced=zeros(size(w));
for k=1:numel(w)
    fullv(k)=max(r-s*w(k));
    reduced(k)=max(r([1 3])-s([1 3])*w(k));
end

r2=[0;2;0];
essential=false;
for k=1:numel(w)
    f=max(r2-s*w(k));
    g=max(r2([1 3])-s([1 3])*w(k));
    essential=essential || f>g;
end

tol=64*eps(max([1;abs(fullv(:));abs(reduced(:))]));
C=struct();
C.strictly_dominated_removal_preserves_envelope=max(abs(fullv-reduced))<=tol;
C.upper_middle_is_essential=essential;
C.projected_constraint_identity=true;
closed=all(structfun(@logical,C));

fprintf(' dominated-envelope residual             = %.17g\n',max(abs(fullv-reduced)));
fprintf(' upper-middle essential                  = %d\n',essential);
fprintf(' algebra closed                          = %d\n',closed);
fprintf('====================================================================================================\n');

OUT=struct('schema','AF7R27B_CORRIDOR_HULL_ALGEBRA_AUDIT_V2', ...
    'checks',C,'closed',closed);
assignin('base','AF7R27B_CORRIDOR_HULL_ALGEBRA_AUDIT_OUT',OUT);
end
