classdef TRIGGER_OUTWARD_INTERVAL
    % Finite, scalar, outward double interval arithmetic.
    % Independent of interval/toolbox and never used in Flight loop.
    % This class is an AUDIT CANDIDATE. On IEEE754 strict nearest
    % operations, conservative 2*eps widening encloses each elementary exact value.
    % sin/cos do NOT trust libm accuracy: bounded Taylor with analytic
    % remainder is used when |x|<=1, otherwise the exact range [-1,1].
    % Certification still needs an independent source/rounding audit.
    properties (SetAccess=private)
        lo
        hi
    end
    methods
        function obj=TRIGGER_OUTWARD_INTERVAL(lo,hi)
            if nargin==0,lo=0;hi=0;end
            assert(isnumeric(lo)&&isnumeric(hi)&&isscalar(lo)&&isscalar(hi)&&...
                isreal(lo)&&isreal(hi)&&isfinite(lo)&&isfinite(hi)&&lo<=hi,...
                'TRIGGER_OUTWARD_INTERVAL:Bounds','Finite scalar interval required.');
            obj.lo=double(lo);obj.hi=double(hi);
        end
        function out=plus(a,b)
            a=TRIGGER_OUTWARD_INTERVAL.toI(a);b=TRIGGER_OUTWARD_INTERVAL.toI(b);
            out=TRIGGER_OUTWARD_INTERVAL(TRIGGER_OUTWARD_INTERVAL.down(a.lo+b.lo),...
                TRIGGER_OUTWARD_INTERVAL.up(a.hi+b.hi));
        end
        function out=uminus(a)
            out=TRIGGER_OUTWARD_INTERVAL(-a.hi,-a.lo);
        end
        function out=minus(a,b)
            out=a+(-b);
        end
        function out=times(a,b)
            a=TRIGGER_OUTWARD_INTERVAL.toI(a);b=TRIGGER_OUTWARD_INTERVAL.toI(b);
            prod=[a.lo*b.lo,a.lo*b.hi,a.hi*b.lo,a.hi*b.hi];
            assert(all(isfinite(prod)), 'TRIGGER_OUTWARD_INTERVAL:Overflow',...
                'Interval product overflow.');
            out=TRIGGER_OUTWARD_INTERVAL(TRIGGER_OUTWARD_INTERVAL.down(min(prod)),...
                TRIGGER_OUTWARD_INTERVAL.up(max(prod)));
        end
        function out=mtimes(a,b)
            out=times(a,b);
        end
        function out=rdivide(a,b)
            b=TRIGGER_OUTWARD_INTERVAL.toI(b);
            assert(b.lo>0||b.hi<0, 'TRIGGER_OUTWARD_INTERVAL:Singular',...
                'Denominator interval contains zero; chart/plant bound unavailable.');
            rec=TRIGGER_OUTWARD_INTERVAL(...
                TRIGGER_OUTWARD_INTERVAL.down(1/b.hi),...
                TRIGGER_OUTWARD_INTERVAL.up(1/b.lo));
            out=times(a,rec);
        end
        function out=mrdivide(a,b)
            out=rdivide(a,b);
        end
        function out=power(a,n)
            a=TRIGGER_OUTWARD_INTERVAL.toI(a);
            assert(isnumeric(n)&&isscalar(n)&&isfinite(n)&&n>=0&&...
                n==fix(n)&&n<=64, 'TRIGGER_OUTWARD_INTERVAL:Power',...
                'Only bounded nonnegative integer powers are supported.');
            out=TRIGGER_OUTWARD_INTERVAL(1,1);
            for k=1:n
                out=times(out,a);
            end
        end
        function out=mpower(a,n)
            out=power(a,n);
        end
        function out=sin(a)
            out=TRIGGER_OUTWARD_INTERVAL.trig(a,false);
        end
        function out=cos(a)
            out=TRIGGER_OUTWARD_INTERVAL.trig(a,true);
        end
    end
    methods (Static,Access=private)
        function v=toI(v)
            if ~isa(v,'TRIGGER_OUTWARD_INTERVAL')
                v=TRIGGER_OUTWARD_INTERVAL(v,v);
            end
        end
        function a=down(a)
            assert(isfinite(a), 'TRIGGER_OUTWARD_INTERVAL:Overflow',...
                'Unbounded interval result must fail closed.');
            a=double(a)-2.*eps(double(a));
            assert(isfinite(a),'TRIGGER_OUTWARD_INTERVAL:Overflow','Downward widening overflow.');
        end
        function a=up(a)
            assert(isfinite(a), 'TRIGGER_OUTWARD_INTERVAL:Overflow',...
                'Unbounded interval result must fail closed.');
            a=double(a)+2.*eps(double(a));
            assert(isfinite(a),'TRIGGER_OUTWARD_INTERVAL:Overflow','Upward widening overflow.');
        end
        function out=trig(x,wantCos)
            x=TRIGGER_OUTWARD_INTERVAL.toI(x);
            a=max(abs([x.lo,x.hi]));
            if a>1
                out=TRIGGER_OUTWARD_INTERVAL(-1,1);
                return
            end
            xx=x.*x;
            out=TRIGGER_OUTWARD_INTERVAL(0,0);
            % Degree 14 (cos) and 15 (sin) with remainder <= a^16/16!
            % and a^17/17!, respectively, for |x|<=1.
            term=TRIGGER_OUTWARD_INTERVAL(1,1);
            for j=0:7
                if wantCos
                    exponent=2*j;
                else
                    exponent=2*j+1;
                end
                if j>0
                    term=term.*xx;
                end
                coef=TRIGGER_OUTWARD_INTERVAL(1,1) ./ ...
                    TRIGGER_OUTWARD_INTERVAL(factorial(exponent),factorial(exponent));
                if mod(j,2)==1,coef=-coef;end
                out=out+term.*coef;
            end
            if ~wantCos
                out=out.*x;
                remainderDegree=17;
            else
                remainderDegree=16;
            end
            remainder=TRIGGER_OUTWARD_INTERVAL(a,a).^remainderDegree ./ ...
                TRIGGER_OUTWARD_INTERVAL(factorial(remainderDegree),factorial(remainderDegree));
            out=out+TRIGGER_OUTWARD_INTERVAL(-remainder.hi,remainder.hi);
            % Mathematical trigonometric range, no approximate libc call.
            out=TRIGGER_OUTWARD_INTERVAL(max(-1,out.lo),min(1,out.hi));
        end
    end
end
