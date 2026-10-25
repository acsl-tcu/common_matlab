function Y=AF7R58R118_HEADING_C6_CERT(segment,maxNodes)
% Exact Bernstein polynomial projection test, within floating arithmetic.
% Nonvanishing horizontal tangent of a C7 position ensures locally C6 yaw.
P=double(segment.payload_cp);T=double(segment.duration_s);
assert(isequal(size(P),[3 16])&&T>0&&isfinite(T)&& ...
    all(isfinite(P(:))),'R118:HeadingSegment');
V=(15/T)*diff(P(1:2,:),1,2);
stack={V};Y=struct('certified',false,'nodes',0, ...
    'min_projected_speed',Inf,'status','INCONCLUSIVE_HORIZONTAL_TANGENT');
while ~isempty(stack)
    if Y.nodes>=maxNodes
        Y.status='INCONCLUSIVE_WORK_BUDGET';return;
    end
    F=stack{end};stack(end)=[];Y.nodes=Y.nodes+1;
    n=sum(F,2);normn=norm(n);
    if normn>0
        projection=(n/normn).'*F;
        scale=max([1;abs(F(:))]);guard=512*eps(scale);
        if min(projection)>guard
            Y.min_projected_speed=min(Y.min_projected_speed,min(projection)-guard);
            continue;
        end
    end
    if all(F(:)==0)
        Y.status='EXACT_STOP_REQUIRES_SEPARATE_YAW_HOLD';return;
    end
    [L,R]=splitBezier(F);
    if isequal(L,F)||isequal(R,F)
        Y.status='INCONCLUSIVE_FLOAT_RESOLUTION';return;
    end
    stack{end+1}=R;stack{end+1}=L;
end
Y.certified=true;Y.status='NONZERO_XY_TANGENT_GLOBAL_C6_YAW_UNWRAPPED';
end
function [A,B]=splitBezier(P)
n=size(P,2);A=zeros(size(P));B=A;A(:,1)=P(:,1);B(:,end)=P(:,end);
for i=2:n
    P=0.5*(P(:,1:end-1)+P(:,2:end));
    A(:,i)=P(:,1);B(:,n-i+1)=P(:,end);
end
end
