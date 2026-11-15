function plan=wp_trigger_plan_once(known,cfg,predictNominal,opts)
%WP_TRIGGER_PLAN_ONCE Fixed relative physical-time sites for 4-s trigger.
% Rebuild only on observed obstacle / active reference version change.
% predictNominal(tRel) returns two 3x1 centers, used ONLY to SELECT density.
% It need not be a verified trajectory: it NEVER justifies CLEAR.
% Base sites cover free-space; each part contributes <=maxExtraPerPart
% extra sites. No sites are inserted or moved during an online check.
% If hazard timing drifts after plan construction, the interval certificate
% still checks EVERY interval, including sparse ones.
if nargin<4,opts=struct();end
if ~isfield(opts,'baseStep'),opts.baseStep=0.4;end
if ~isfield(opts,'scanStep'),opts.scanStep=0.05;end
if ~isfield(opts,'maxExtraPerPart'),opts.maxExtraPerPart=16;end
if opts.baseStep<=0 || opts.scanStep<=0 || ...
        opts.maxExtraPerPart<0 || opts.maxExtraPerPart~=floor(opts.maxExtraPerPart)
    error('Trigger:PlanConfig','Invalid sampling budget');
end
% Discretizations are purely COMPUTATIONAL budgets, not safety margins.
base=linspace(0,4,ceil(4/opts.baseStep)+1);
scan=linspace(0,4,ceil(4/opts.scanStep)+1);
if numel(scan)>20001,error('Trigger:PlanSize','Offline plan too large');end
P=known.numParts;
score=-inf(P,numel(scan));
physR=max([cfg.droneRadius,cfg.loadRadius,cfg.cableRadius])+cfg.safetyMargin;
for k=1:numel(scan)
    [d,l]=predictNominal(scan(k));
    if ~isequal(size(d),[3,1]) || ~isequal(size(l),[3,1]) || ...
            any(~isfinite([d;l]))
        error('Trigger:PlanForecast','Need two finite 3x1 reference centers');
    end
    u=l-d;den=dot(u,u);
    if den>0
        s=max(0,min(1,sum(bsxfun(@times,known.center-d,u),1)/den));
    else
        s=zeros(1,P);
    end
    feet=bsxfun(@plus,d,u*s);
    dist=sqrt(sum((known.center-feet).^2,1));
    score(:,k)=(known.outerRadius+physR-dist)';
end
extras=cell(1,P);counts=zeros(1,P);
for j=1:P
    active=find(score(j,:)>=0);
    if isempty(active) || opts.maxExtraPerPart==0,continue;end
    m=min(numel(active),opts.maxExtraPerPart);
    if numel(active)<=m
        ix=active;
    else
        [~,peak]=max(score(j,active));
        picks=unique([1,numel(active),peak]);
        if numel(picks)>m,picks=picks(1:m);end
        while numel(picks)<m
            ds=min(abs((1:numel(active))'-picks(:)'),[],2);
            ds(picks)=-inf;
            [~,pick]=max(ds);
            picks(end+1)=pick; %#ok<AGROW>
        end
        ix=active(sort(picks));
    end
    extras{j}=scan(ix);
    counts(j)=numel(ix);
end
nodes=sort(unique([base,extras{:}]));
plan=struct('timeOffsets',nodes,'baseSites',base,'perPartSites',{extras}, ...
    'perPartSelectedCount',counts,'numParts',P, ...
    'maxExtraPerPart',opts.maxExtraPerPart,'numSites',numel(nodes), ...
    'fixedAcrossControllerTicks',true,'timeHorizonSeconds',4, ...
    'densitySelectionNotSafetyProof',true,'settings',opts);
end
