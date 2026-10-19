function OUT = AF7R58E_ONEFILE_FLOAT_TREE_BOUND(packet)
% AF7R58E: conservative numeric depth/component bound for AF7R36 episode tree.
% Put in common_matlab/reference. Default: source binding + arithmetic regressions.
% Optional explicit packet: struct('t0',...,'t1',...,'dt',...). No runtime guessing.
% Return AF7R58E_REPORT.txt AND AF7R58E_OUT.mat from the printed directory.
% No production edits, MEX, Flight, simulation, or safety acceptance by samples.
% Scope: binary64 round-to-nearest, gradual underflow, no fast-math reassociation.
% It bounds the geometry-independent tree. It does NOT prove 4ms or actuator safety.
if nargin<1, packet=[]; end
root=fileparts(mfilename('fullpath'));
outdir=tempname(root); [ok,msg]=mkdir(outdir); assert(ok,'%s',msg);
f=fopen(fullfile(outdir,'AF7R58E_REPORT.txt'),'wt','n','UTF-8');
assert(f>=0,'[AF7R58E] cannot open report.');
cleanup=onCleanup(@()fclose(f)); %#ok<NASGU>
say(f,'AF7R58E | OUTWARD SCALAR TREE BOUND - REFERENCE-FIRST WORKFLOW');
say(f,' productionModified=0 | noFlight=1 | noMex=1 | sampledSafetyAcceptance=0');
p=fullfile(root,'af7r36_core.hpp');
expected='ad5ce1970583c0313637dcac58b8cf949092fe53e201efc61e55754fb2ea79be';
h=''; if isfile(p), h=file_hash(p); end
sourceBound=strcmp(h,expected);
say(f,' reviewed core SHA match=%d | %s',sourceBound,h);
PROOF={ ...
 'P1: Let M=max(1,abs(t0),abs(t1)). Require finite ordered endpoints and M<=realmax/4.'; ...
 'All strictly interior descendants stay in [-M,M]; their endpoint sums cannot overflow.'; ...
 'For tc=fl(0.5*fl(a+b)), a conservative absolute midpoint error is eta=2*eps(M).'; ...
 'This covers sum rounding and multiplication/underflow; M>=1 dominates subnormal error.'; ...
 'If exact width b-a<=W, either child exact width is <=W/2+eta.'; ...
 'up(x) is the next larger binary64 number, implemented by a positive bit increment.'; ...
 'W0=up(fl(t1-t0)); Wnext=up(fl(up(fl(W/2))+eta)) are upper bounds by induction.'; ...
 'For any node with width<=W, its computed subtraction is <=up(W).'; ...
 'tau is computed as in production: dt+64*eps(1)*max(1,abs(t0),abs(t1),abs(dt)).'; ...
 'Once up(W)<=tau, every such node terminates; a nonsplittable midpoint terminates earlier.'; ...
 'Let K be the first certified depth. The full tree has at most 2^K terminal cells.'; ...
 'All obstacles use the same scalar tree. Safe early exits remove unsafe cells only.'; ...
 'The union of unsafe cells has at most ceil(2^K/2) connected runs (1 when K=0).'; ...
 'Thus C<=2^(max(K-1,0)), and S_group<=2*C+1 for production build_candidate.'; ...
 'The bounds are independent of obstacle centers, poses, axes and finite obstacle count.'; ...
 'M_obstacles still affects episode count and work: E<=M_obstacles*C_bound; branches=2^E.'; ...
 'No obstacle is ignored or accepted safe by this resource bound.'; ...
 'If arithmetic stagnates, overflows or cannot certify a bound, status is OPEN, never SAFE.'; ...
 'The formula is conditional on the stated floating-point environment and reviewed source.'; ...
 'Compiler flags / rounding-mode authority are not verified by this MATLAB-only audit.'; ...
 'Source-bound and numeric-computed do not mean production-admitted or final safety closed.'};
for j=1:numel(PROOF), say(f,'%s',PROOF{j}); end
% These finite cases exercise implementation only. They are not safety evidence,
% performance benchmarks, selected caps or current Flight parameters.
cases=[0 1 .125; -1 1 .3; 1e6 1e6+1 .125; 0 1 .07; ...
       -1e-100 1e-100 1e-103; 2^40 2^40+1 .01];
T=struct('input',{},'bound',{},'enumerated_leaves',{},'enumerated_depth',{}, ...
         'enumerated_components_bound',{},'pass',{});
say(f,'[ARITHMETIC IMPLEMENTATION REGRESSIONS - NOT PRODUCTION INPUTS]');
for i=1:size(cases,1)
    a=cases(i,1); b=cases(i,2); dt=cases(i,3);
    B=tree_bound(a,b,dt);
    [n,d]=enumerate_tree(a,b,dt);
    pass=B.computed && d<=B.depth && n<=B.leaf_bound && ceil(n/2)<=B.component_bound;
    T(i)=struct('input',cases(i,:),'bound',B,'enumerated_leaves',n, ...
       'enumerated_depth',d,'enumerated_components_bound',ceil(n/2),'pass',pass);
    say(f,' case=%d | depth=%g<=%g | leaves=%g<=%g | pass=%d',i,d,B.depth,n,B.leaf_bound,pass);
end
% Invalid inputs must not manufacture a cap.
invalid={ [0 0 .1], [0 1 0], [0 Inf .1], [0 1 NaN], [-realmax realmax .1] };
invalidPass=true;
for i=1:numel(invalid)
    z=invalid{i}; B=tree_bound(z(1),z(2),z(3)); invalidPass=invalidPass && ~B.computed;
end
OUT=struct(); OUT.schema='AF7R58E_OUTWARD_TREE_BOUND_V1';
OUT.source_bound=sourceBound; OUT.core_sha256=h; OUT.derivation=PROOF;
OUT.regressions=T; OUT.invalid_input_rejection_pass=invalidPass;
OUT.implementation_regressions_pass=all([T.pass]) && invalidPass;
OUT.arithmetic_environment_verified=false;
OUT.packet_supplied=~isempty(packet); OUT.packet_bound=struct();
if ~isempty(packet)
    assert(isstruct(packet)&&isscalar(packet)&&all(isfield(packet,{'t0','t1','dt'})), ...
        '[AF7R58E] optional packet needs explicit scalar t0,t1,dt.');
    OUT.packet_bound=tree_bound(packet.t0,packet.t1,packet.dt);
    say(f,'[EXPLICIT PACKET] status=%s | K=%g | C_bound=%g | S_group_bound=%g', ...
        OUT.packet_bound.status,OUT.packet_bound.depth, ...
        OUT.packet_bound.component_bound,OUT.packet_bound.segment_bound);
else
    say(f,' explicit runtime packet not supplied; current production numeric cap is NOT selected.');
end
OUT.production_caps_selected=false; OUT.production_modified=false;
OUT.production_4ms_claimed=false; OUT.actuator_feasibility_closed=false;
OUT.tracking_addon_required_for_reference_path=false;
OUT.failure_backup_deferred=true;
OUT.status='SOURCE_OR_IMPLEMENTATION_OPEN';
if sourceBound && OUT.implementation_regressions_pass
    OUT.status='CONDITIONAL_TREE_BOUND_IMPLEMENTED__LIVE_BINDING_AND_ENVIRONMENT_OPEN';
end
OUT.next_action='BIND_LIVE_INTERVAL_AND_ARITHMETIC__COMMIT_LIFETIME__ACTUATOR_FEASIBILITY';
OUT.required_return_files={'AF7R58E_REPORT.txt','AF7R58E_OUT.mat'};
OUT.output_directory=outdir;
say(f,' status=%s',OUT.status);
say(f,' Gmax/Smax NOT derived: commit/lifetime contract remains separate.');
say(f,' nominal tracking assumed for reference path; certified inflation is optional add-on.');
say(f,' tension/thrust/moments remain REQUIRED reference feasibility obligations.');
say(f,' failure backup deferred; no final system safety completion is claimed.');
save(fullfile(outdir,'AF7R58E_OUT.mat'),'OUT','-v7');
assignin('base','AF7R58E_OUT',OUT);
say(f,' RETURN: %s',fullfile(outdir,'AF7R58E_REPORT.txt'));
say(f,' RETURN: %s',fullfile(outdir,'AF7R58E_OUT.mat'));
end

function B=tree_bound(a,b,dt)
B=struct('computed',false,'status','OPEN_INVALID_INPUT','depth',NaN, ...
    'leaf_bound',NaN,'component_bound',NaN,'segment_bound',NaN, ...
    'leaf_bound_exponent',NaN,'component_bound_exponent',NaN, ...
    'threshold',NaN,'midpoint_error_bound',NaN,'width_bounds',[], ...
    'numeric_counts_exact',false);
if ~(isa(a,'double')&&isa(b,'double')&&isa(dt,'double') && ...
        isreal(a)&&isreal(b)&&isreal(dt)&&isscalar(a)&&isscalar(b)&&isscalar(dt) && ...
        isfinite(a)&&isfinite(b)&&isfinite(dt)&&b>a&&dt>0), return; end
M=max([1 abs(a) abs(b)]);
if M>realmax/4, B.status='OPEN_ARITHMETIC_RANGE'; return; end
eta=2*eps(M);
tol=64*eps(1)*max([1 abs(a) abs(b) abs(dt)]);
tau=dt+tol;
if ~isfinite(tau), B.status='OPEN_THRESHOLD_OVERFLOW'; return; end
W=up(b-a); K=0; hist=W;
while up(W)>tau
    V=up(up(W/2)+eta);
    if ~isfinite(V) || ~(V<W)
        B.status='OPEN_WIDTH_RECURRENCE_NOT_CONTRACTING';
        B.threshold=tau; B.midpoint_error_bound=eta; B.width_bounds=hist; return;
    end
    W=V; K=K+1; hist(end+1)=W; %#ok<AGROW>
end
B.computed=true; B.status='CONDITIONAL_BOUND_COMPUTED'; B.depth=K;
B.leaf_bound_exponent=K; B.component_bound_exponent=max(K-1,0);
B.threshold=tau; B.midpoint_error_bound=eta; B.width_bounds=hist;
% Exact integer powers and 2*C+1 only when exactly representable in binary64.
% Larger exponents remain symbolic, not saturated or rounded down.
if K<=52
    B.leaf_bound=2^K; B.component_bound=2^max(K-1,0);
    B.segment_bound=2*B.component_bound+1; B.numeric_counts_exact=true;
else
    B.status='CONDITIONAL_SYMBOLIC_COUNT_ONLY';
end
end
function y=up(x)
assert(isreal(x)&&isscalar(x)&&x>=0,'[AF7R58E] up needs a nonnegative scalar.');
if isinf(x), y=x; return; end
if x==0, x=0; end % Canonicalize negative zero before positive bit stepping.
u=typecast(double(x),'uint64'); y=typecast(u+uint64(1),'double');
end
function [n,dmax]=enumerate_tree(a,b,dt)
% Test-only exact scalar traversal; safety/geometry code is never evaluated.
tol=64*eps(1)*max([1 abs(a) abs(b) abs(dt)]);
st=[a b 0]; n=0; dmax=0;
while ~isempty(st)
    z=st(end,:); st(end,:)=[]; ta=z(1); tb=z(2); d=z(3);
    tc=.5*(ta+tb); splittable=isfinite(tc)&&tc>ta&&tc<tb;
    if (tb-ta)<=dt+tol || ~splittable
        n=n+1; dmax=max(dmax,d);
    else
        st(end+1,:)=[tc tb d+1]; st(end+1,:)=[ta tc d+1]; %#ok<AGROW>
    end
end
end
function say(f,fmt,varargin)
s=sprintf(fmt,varargin{:}); fprintf(1,'%s\n',s); fprintf(f,'%s\n',s);
end
function h=file_hash(p)
f=fopen(p,'rb'); assert(f>=0,'cannot read %s',p);
c=onCleanup(@()fclose(f)); %#ok<NASGU>
b=fread(f,Inf,'*uint8'); m=java.security.MessageDigest.getInstance('SHA-256'); m.update(b);
d=typecast(m.digest(),'uint8'); h=lower(reshape(dec2hex(d,2).',1,[]));
end
