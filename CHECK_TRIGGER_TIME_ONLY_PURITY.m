function R=CHECK_TRIGGER_TIME_ONLY_PURITY()
% Does NOT require or modify agent, reference, sensor, logger, Flight.
% Tests conservative eligibility of one-time mathematical source audit.
% This does not establish model/geometry/closed-loop safety.
a=2; b=[3;4];
pure=@(t)[a.*t+1;b(1).*sin(t);b(2).*cos(t)];
mutable=containers.Map('KeyType','char','ValueType','double');
mutable('x')=1;
impure=@(t)mutable('x')+t;
random=@(t)rand(1)+t;
external=@(t)local_unknown(t);
plain=@(t)[t;t.^2;sin(t)];
P=TRIGGER_TIME_ONLY_PURITY(pure);
Q=TRIGGER_TIME_ONLY_PURITY(plain);
M=TRIGGER_TIME_ONLY_PURITY(impure);
D=TRIGGER_TIME_ONLY_PURITY(random);
E=TRIGGER_TIME_ONLY_PURITY(external);
assert(P.eligible && Q.eligible && ~M.eligible && ...
       ~D.eligible && ~E.eligible, ...
       'CHECK_TRIGGER_TIME_ONLY_PURITY:Regression', ...
       'Pure scalar arithmetic must be accepted, dynamic expressions rejected.');
R=struct('status','PASS_STRICT_SOURCE_PURITY_SYNTHETIC_ONLY', ...
    'pure_numeric_capture',P.eligible,'pure_no_capture',Q.eligible, ...
    'mutable_rejected',~M.eligible,'random_rejected',~D.eligible, ...
    'external_function_rejected',~E.eligible, ...
    'continuous_time_safe',false,'real_flight_tested',false);
folder=fileparts(mfilename('fullpath'));
save(fullfile(folder,'TRIGGER_TIME_ONLY_PURITY_TEST.mat'),'R');
fid=fopen(fullfile(folder,'TRIGGER_TIME_ONLY_PURITY_TEST.txt'),'wt');
if fid>=0
    c=onCleanup(@()fclose(fid));
    fprintf(fid,'%s\npure_capture=%d pure_plain=%d mutable_rejected=%d random_rejected=%d external_rejected=%d\n', ...
        R.status,R.pure_numeric_capture,R.pure_no_capture, ...
        R.mutable_rejected,R.random_rejected,R.external_function_rejected);
end
end
function y=local_unknown(x)
y=x;
end
