clear all
n = 21;
syms t z0 v0 te ve ze real
syms a [1,n] real
syms A [1,n] real
tra = 0;
for i = 1:n
tra = tra + a(i)*t^(i-1);
end
m = 4;
T = sym("T",[m,1]);
T(1) = tra;
for i = 1:m
T(i+1) = diff(tra,t,i);
end

[A1, A2, A3, A4, A5, A6, A7, A8, A9, A10, A11, A12, A13, A14, A15, A16, A17, A18, A19, A20,A21]= solve( ...
    [subs(T,t,0) == [z0;v0;zeros(m-1,1)];subs(T,t,te) == [ze;ve;zeros(m-1,1)]],a);
A = [A1, A2, A3, A4, A5, A6, A7, A8, A9, A10, A11, A12, A13, A14, A15, A16, A17, A18, A19, A20,A21];
tra2 = subs(tra,a,A);
matlabFunction(tra2,"File","class"+m+"_interpolation.m","Vars",{t,te,z0,v0,ze,ve});
%%
te = 10;
tp = 0:0.01:te;
func = str2func("class"+m+"_interpolation");
R= func(tp,te,1,0,0,0);
plot(tp,R);
%R2= curve_interpolation_9order(tp',te,1,0,0,0);
% plot(tp,R,tp,R2(1:401))