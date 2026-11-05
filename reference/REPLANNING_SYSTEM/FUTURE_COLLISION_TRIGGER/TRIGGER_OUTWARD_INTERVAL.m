classdef TRIGGER_OUTWARD_INTERVAL
    % Finite, scalar, outward double interval arithmetic.
    % Independent of interval/toolbox and never used in Flight loop.
    % This class is an AUDIT CANDIDATE. On IEEE754 strict nearest
    % operations, conservative 2*eps widening encloses each elementary exact value.
    % sin/cos do NOT trust libm accuracy: outward Taylor with analytic
    % remainder for |x|<=4; otherwise universal range [-1,1].
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
            % The original implementation evaluated a.^n by repeated
            % generic interval multiplication.  That is inclusion-safe
            % but loses the identity x^2 >= 0 whenever x straddles zero.
            % Here the exact range follows from monotonicity on each
            % sign branch. Endpoint powers themselves use outward
            % interval products, not a floating pow() assumption.
            a=TRIGGER_OUTWARD_INTERVAL.toI(a);
            assert(isnumeric(n)&&isscalar(n)&&isfinite(n)&&n>=0&&...
                n==fix(n)&&n<=64, 'TRIGGER_OUTWARD_INTERVAL:Power',...
                'Only bounded nonnegative integer powers are supported.');
            if n==0
                out=TRIGGER_OUTWARD_INTERVAL(1,1);
                return
            elseif n==1
                out=a;
                return
            end
            if mod(n,2)==0
                % Even power has exact mathematical minimum 0 when the
                % interval crosses the origin. Otherwise use the closest
                % endpoint to zero. The maximum is at the farthest end.
                hiAbs=max(abs(a.lo),abs(a.hi));
                maximum=TRIGGER_OUTWARD_INTERVAL.positivePointPower(hiAbs,n);
                if a.lo<=0 && a.hi>=0
                    lower=0;
                else
                    nearAbs=min(abs(a.lo),abs(a.hi));
                    minimum=TRIGGER_OUTWARD_INTERVAL.positivePointPower(nearAbs,n);
                    lower=minimum.lo;
                end
                out=TRIGGER_OUTWARD_INTERVAL(lower,maximum.hi);
            else
                % Odd powers are monotone on all of R, including negatives.
                if a.lo<0
                    l=TRIGGER_OUTWARD_INTERVAL.positivePointPower(-a.lo,n);
                    lower=-l.hi;
                else
                    l=TRIGGER_OUTWARD_INTERVAL.positivePointPower(a.lo,n);
                    lower=l.lo;
                end
                if a.hi<0
                    h=TRIGGER_OUTWARD_INTERVAL.positivePointPower(-a.hi,n);
                    upper=-h.lo;
                else
                    h=TRIGGER_OUTWARD_INTERVAL.positivePointPower(a.hi,n);
                    upper=h.hi;
                end
                out=TRIGGER_OUTWARD_INTERVAL(lower,upper);
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
        function z=positivePointPower(v,n)
            % Every factor is an exactly represented nonnegative binary64
            % endpoint. Each multiplication is outward, so the true
            % integer power is contained despite roundoff/underflow.
            assert(isfinite(v)&&isreal(v)&&v>=0 && ...
                n==fix(n)&&n>=0&&n<=64);
            z=TRIGGER_OUTWARD_INTERVAL(1,1);
            factor=TRIGGER_OUTWARD_INTERVAL(v,v);
            for j=1:n
                z=z.*factor;
            end
            % x^n >= 0 is an exact mathematical identity, including
            % signed zeros; intersect the interval with [0,+inf).
            z=TRIGGER_OUTWARD_INTERVAL(max(0,z.lo),max(0,z.hi));
        end
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
            % The previous threshold a>1 returned [-1,1] even for a
            % SINGLETON Euler pitch of 1.2 rad, despite cos(pitch)>0;
            % this erased the chart denominator in the original plant.
            % Extend the analytic Taylor enclosure to |x|<=4. Above 4,
            % the conservative universal range remains [-1,1].
            if a>4
                out=TRIGGER_OUTWARD_INTERVAL(-1,1);
                return
            end
            xx=x.^2;  % exact nonnegative range, no x*x dependency loss
            out=TRIGGER_OUTWARD_INTERVAL(0,0);
            if wantCos
                term=TRIGGER_OUTWARD_INTERVAL(1,1);
            else
                term=x;
            end
            if a<=1
                lastTerm=7;  % original degree 14/15, faster tight-angle path
            else
                lastTerm=17; % degree 34/35 to cover wider angles
            end
            for j=0:lastTerm
                if j>0
                    if wantCos
                        denom=(2*j-1)*(2*j);
                    else
                        denom=(2*j)*(2*j+1);
                    end
                    term=term.*xx ./ ...
                        TRIGGER_OUTWARD_INTERVAL(denom,denom);
                end
                if mod(j,2)==1
                    out=out-term;
                else
                    out=out+term;
                end
            end
            % cos includes the even degree 2*lastTerm and has zero
            % odd coefficient thereafter; its Taylor remainder is
            % bounded by a^(2*lastTerm+2)/(2*lastTerm+2)!.
            % sin includes degree 2*lastTerm+1, with zero even next
            % coefficient; its remainder uses degree 2*lastTerm+3.
            % Compute reciprocal factorial through successive outward
            % divisions by EXACT small integer denominators, never
            % treat rounded factorial(36/37) as an exact double integer.
            if wantCos
                remainderDegree=2*lastTerm+2;
            else
                remainderDegree=2*lastTerm+3;
            end
            remainder=TRIGGER_OUTWARD_INTERVAL.positivePointPower( ...
                a,remainderDegree);
            for j=2:remainderDegree
                remainder=remainder ./ TRIGGER_OUTWARD_INTERVAL(j,j);
            end
            out=out+TRIGGER_OUTWARD_INTERVAL(-remainder.hi,remainder.hi);
            % The exact elementary range is always [-1,1].
            out=TRIGGER_OUTWARD_INTERVAL(max(-1,out.lo),min(1,out.hi));
        end
    end
end
