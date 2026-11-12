function [A,b] = cbf_high_order_row(model,h,r,k)
%CBF_HIGH_ORDER_ROW 1個のhについて(D+k)^r h>=0 の候補を作る。
% 実装に用いる前に「低次の微分に入力が現れないこと」を検査する。
% 入力係数が特定状態でゼロになる退化と、各時刻でのQP可行性は別途検証。
if ~isscalar(k) || k<=0 || ~isscalar(r) || r<1 || r~=floor(r)
    error('CBF:InvalidOrder','rは正整数、kは正の値にしてください。');
end
x=model.x; f=model.f; g=model.g;
jet=cell(r,1);jet{1}=h;
for j=1:r-1
    early=simplify(jacobian(jet{j},x)*g);
    if ~all(arrayfun(@(z)isequal(z,sym(0)),early(:)))
        error('CBF:EarlyInput', ...
            '%d階で入力が現れました。指定次数%dの通常HOCBFでは生成しません。',j,r);
    end
    jet{j+1}=jacobian(jet{j},x)*f;
end
A=jacobian(jet{r},x)*g;
drift=jacobian(jet{r},x)*f;
poly=1;
for j=1:r,poly=conv(poly,[1 k]);end
extra=sym(0);
for j=0:r-1
    extra=extra+poly(r-j+1)*jet{j+1};
end
b=-drift-extra; % A * u >= b
end
