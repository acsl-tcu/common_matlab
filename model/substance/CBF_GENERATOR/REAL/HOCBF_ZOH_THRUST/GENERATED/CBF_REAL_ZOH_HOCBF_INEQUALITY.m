function out = CBF_REAL_ZOH_HOCBF_INEQUALITY(x, u, parts, P, G, options)
% CBF_REAL_ZOH_HOCBF_INEQUALITY  Experimental r=[4,4,4] ZOH residuals.
% GENERATED locally as CBF_REAL_ZOH_HOCBF_INEQUALITY.m.
% Inputs: x(19x1), real u=[T;tx;ty;tz], parts (single ellipsoid array
% or world struct with .parts), physical P, geometry G, options.
%
% h: conservative nonintersection function for drone sphere, cable
% ellipsoid and load sphere against obstacle ellipsoids, plus ground.
%
% gamma_i = (d/dt + lambda)^i h for i=0,...,r along the fixed-input
% time Taylor jet. Top-order residual >=0 is the candidate inequality.
% r=4 for drone/cable/load is the generic FIRST TORQUE ACTION ORDER.
% Thrust appears at order 2, so r=4 is NOT a classical full-input
% relative degree. This is a diagnostic, NOT a certified HOCBF.
%
% IMPORTANT: If torques act in lower derivatives, this is an experimental
% constant-input high-order surrogate, NOT a certified standard HOCBF.
% Also the constraints can be nonlinear in real inputs (including T*tau);
% do not insert into a linear-QP solver or claim intersample safety.
%
% options.lambda: scalar or [drone,cable,load], default [1 1 1] [1/s].
% Outputs: out.values (1xn), out.stages (1xn cells), out.currentSafe,
% out.lowerStagesSatisfied, out.requestedOrder, out.names, out.derivatives.

if nargin<6 || isempty(options), options=struct(); end
if nargin<5 || isempty(G), G=CBF_COMMON_GEOMETRY(P); end
if isempty(parts), parts=struct([]); end
if isstruct(parts) && isscalar(parts) && isfield(parts,'parts')
    parts=parts.parts;
end
if ~isfield(options,'lambda'), options.lambda=[1 1 1]; end
lambda=options.lambda(:).';
if numel(lambda)==1, lambda=repmat(lambda,1,3); end
validateattributes(lambda,{'numeric'},{'vector','numel',3,'real','finite','positive'});
if any(~isfinite(lambda)), error('CBFRealZOH:Lambda','Invalid lambda'); end
J=CBF_REAL_ZOH_DIAG_EVAL(x,u,parts,P,G,4);
n=numel(J.names);
values=zeros(1,n);
ranks=zeros(1,n);
gains=zeros(1,n);
stages=cell(1,n);
lowerOK=true(1,n);
for j=1:n
    switch J.components{j}
        case 'drone', r=4; lam=lambda(1);
        case 'cable', r=4; lam=lambda(2);
        case 'load', r=4; lam=lambda(3);
        otherwise, error('CBFRealZOH:Component','Unknown component');
    end
    ranks(j)=r;
    gains(j)=lam;
    stage=zeros(r+1,1);
    for i=0:r
        v=0;
        for k=0:i
            v=v+nchoosek(i,k)*lam^(i-k)*J.derivatives(k+1,j);
        end
        stage(i+1)=v;
    end
    stages{j}=stage;
    values(j)=stage(end);
    lowerOK(j)=all(stage(1:end-1)>=0);
end
out=struct('values',values,'names',{J.names},'components',{J.components}, ...
    'kinds',{J.kinds},'requestedOrder',ranks,'gains',gains, ...
    'stages',{stages},'derivatives',J.derivatives, ...
    'h',J.derivatives(1,:), 'currentSafe',J.derivatives(1,:)>=0, ...
    'lowerStagesSatisfied',lowerOK, 'topStageSatisfied',values>=0, ...
    'input',u(:),'state',x(:), ...
    'status','EXPERIMENTAL_ZOH_FOUR_INPUT_NONLINEAR_NOT_SAFETY_CERTIFIED');
end