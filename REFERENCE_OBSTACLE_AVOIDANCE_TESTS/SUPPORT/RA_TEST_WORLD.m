function w=RA_TEST_WORLD(cover,scenario)
% 36 observed outer parts with deterministic geometry. TEST FIXTURE ONLY.
if nargin<2,scenario='near';end
n=36;p=repmat(struct('kind','sphere','center',zeros(3,1), ...
    'R',eye(3),'params',struct('radius',.08,'axes',[.08;.08;.08]), ...
    'obstacleName',''),1,n);
for k=1:n
    x=.1+3.7*(k-1)/(n-1);
    if strcmp(scenario,'distant'),y=20;else,y=.8;end
    p(k).center=[x;y;2];p(k).obstacleName=sprintf('test_%02d',k);
    if strcmp(cover,'multiple_ellipsoids')
        p(k).kind='ellipsoid';p(k).params.axes=[.06;.11;.05];
    end
end
w=struct('parts',p,'approximation',cover);
end
