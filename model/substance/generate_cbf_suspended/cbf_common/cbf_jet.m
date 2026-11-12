function y = cbf_jet(op,varargin)
%CBF_JET  一変数Taylor係数の基本演算（高階の数値差分は使わない）。
% 配列の第k+1列は時間Taylor係数 x^(k)(0)/k!。
% 使用: cbf_jet('mul',a,b), 'dot', 'cross', 'sqrt', 'inv', 'const'
% MATLAB組み込み関数との混同を避けるためCBF専用関数とする。
switch lower(op)
    case 'const'
        value=varargin{1}; N=varargin{2};
        y=zeros(numel(value),N+1);y(:,1)=value(:);
    case 'mul'
        a=varargin{1};b=varargin{2};
        assert(size(a,2)==size(b,2),'Taylor次数が一致していません。');
        n=max(size(a,1),size(b,1));
        if size(a,1)==1 && n>1,a=repmat(a,n,1);end
        if size(b,1)==1 && n>1,b=repmat(b,n,1);end
        assert(size(a,1)==size(b,1),'Taylorのベクトル要素数が一致しません。');
        K=size(a,2)-1;y=zeros(n,K+1);
        for k=0:K
            for j=0:k
                y(:,k+1)=y(:,k+1)+a(:,j+1).*b(:,k-j+1);
            end
        end
    case 'dot'
        y=sum(cbf_jet('mul',varargin{1},varargin{2}),1);
    case 'cross'
        a=varargin{1};b=varargin{2};
        assert(size(a,1)==3 && size(b,1)==3,'crossには3次元ベクトルが必要です。');
        y=[cbf_jet('mul',a(2,:),b(3,:))-cbf_jet('mul',a(3,:),b(2,:)); ...
           cbf_jet('mul',a(3,:),b(1,:))-cbf_jet('mul',a(1,:),b(3,:)); ...
           cbf_jet('mul',a(1,:),b(2,:))-cbf_jet('mul',a(2,:),b(1,:))];
    case 'inv'
        a=varargin{1};
        assert(size(a,1)==1,'invはスカラーTaylor級数のみ対応します。');
        assert(isfinite(a(1)) && a(1)~=0,'CBF:JetSingularity','逆数の分母がゼロです。');
        K=size(a,2)-1;y=zeros(1,K+1);y(1)=1/a(1);
        for k=1:K
            total=0;
            for j=1:k,total=total+a(j+1)*y(k-j+1);end
            y(k+1)=-total/a(1);
        end
    case 'sqrt'
        a=varargin{1};
        assert(size(a,1)==1,'sqrtはスカラーTaylor級数のみ対応します。');
        if a(1)<=0 || ~isfinite(a(1))
            error('CBF:JetSingularity','平方根の定数項が正ではありません。');
        end
        K=size(a,2)-1;y=zeros(1,K+1);y(1)=sqrt(a(1));
        for k=1:K
            total=0;
            for j=1:k-1,total=total+y(j+1)*y(k-j+1);end
            y(k+1)=(a(k+1)-total)/(2*y(1));
        end
    otherwise
        error('CBF:JetOperation','不明なTaylor演算: %s',op);
end
end
