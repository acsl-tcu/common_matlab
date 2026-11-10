function REPORT = CBF_ANALYZE_PHYSICAL_QP_FAILURE(source)
% CBF_ANALYZE_PHYSICAL_QP_FAILURE
% READ-ONLY: audit the TWO saved S4 failure QPs independently.
% No edits to generator, HLC, EKF, reference, QP, plant or safety limits.
% Default input is the ONE result ZIP in common_matlab. No persistent files.
%
%   REPORT = CBF_ANALYZE_PHYSICAL_QP_FAILURE();
%   REPORT = CBF_ANALYZE_PHYSICAL_QP_FAILURE('CBF_DUAL_RESULTS_UPLOAD.zip');
%   REPORT = CBF_ANALYZE_PHYSICAL_QP_FAILURE('CBF_physical_QP_failure.mat');
%
% Requires Optimization Toolbox for linprog. Optional same-state S3/S4
% derivative comparison requires Generate_CBF_ActuatorAware_6Methods on path.
% This audit cannot establish closed-loop safety or recursive feasibility.
if nargin<1 || isempty(source)
    source='CBF_DUAL_RESULTS_UPLOAD.zip';
end
source=char(source);
assert(exist(source,'file')==2,'CBF:AuditFileMissing', ...
    'Source file not found: %s',source);
[~,~,ext]=fileparts(source);
if strcmpi(ext,'.zip')
    tempRoot=tempname(tempdir);
    mkdir(tempRoot);
    cleanup=onCleanup(@()localRemoveTemp(tempRoot)); %#ok<NASGU>
    unzip(source,tempRoot);
    paths={fullfile(tempRoot,'virtual','CBF_physical_QP_failure.mat'), ...
           fullfile(tempRoot,'physical','CBF_physical_QP_failure.mat')};
    labels={'virtual','physical'};
elseif strcmpi(ext,'.mat')
    paths={source}; labels={'snapshot'};
else
    error('CBF:AuditType','Provide the results ZIP or a QP snapshot MAT.');
end
REPORT=struct('label',{},'time',{},'nCBF',{},'nRows',{}, ...
    'qpExitflag',{},'lpExitflag',{},'minimumCommonSlack',{}, ...
    'worstSingleCBFRow',{},'worstSingleCBFGap',{},'rowReport',{}, ...
    'capsuleSupport',{},'sameStateShapeComparison',{});
options=optimoptions('linprog','Display','off');
for k=1:numel(paths)
    file=paths{k};
    assert(exist(file,'file')==2,'CBF:MissingSnapshot', ...
        'Missing %s in experiment ZIP',file);
    loaded=load(file,'D');
    assert(isfield(loaded,'D'),'CBF:InvalidSnapshot', ...
        'Expected struct D in %s',file);
    D=loaded.D;
    A=double(D.Aineq); b=double(D.bineq(:));
    assert(size(A,2)==4 && numel(b)==size(A,1) && ...
        all(isfinite([A(:);b(:)])),'CBF:InvalidQP', ...
        'QP matrix and right-hand side are not valid.');
    n=size(A,1);nCBF=numel(D.filterResult.rows);
    assert(n>=nCBF,'CBF:InvalidRows','Missing CBF constraint rows.');
    [x,~,flag]=linprog([zeros(4,1);1],[A,-ones(n,1)],b, ...
        [],[],[-inf(4,1);0],[],options);
    commonSlack=NaN;
    if flag>0 && ~isempty(x) && all(isfinite(x))
        commonSlack=max(0,x(end));
    end
    report=struct();
    report.label=labels{k};report.time=double(D.time);
    report.nCBF=nCBF;report.nRows=n;
    report.qpExitflag=double(D.filterResult.qp.exitflag);
    report.lpExitflag=flag;report.minimumCommonSlack=commonSlack;
    report.worstSingleCBFRow=NaN;report.worstSingleCBFGap=NaN;
    report.rowReport=struct('row',{},'obstacleID',{},'component',{}, ...
        'minimumLeftHandSide',{},'requiredUpperBound',{}, ...
        'unavoidableViolation',{},'lpExitflag',{});
    fprintf('\n=== CBF S4 QP audit | %s | t=%.5f s ===\n', ...
        labels{k},report.time);
    fprintf('QP exitflag=%g; minimum common slack=%.9g (LP flag=%g)\n', ...
        report.qpExitflag,commonSlack,flag);
    if n>nCBF
        % Isolate whether ANY one CBF inequality is inconsistent with the
        % SAME actuator bounds. This is a mathematical implication, not an
        % optional relaxation of safety constraints.
        actuatorRows=(nCBF+1):n;
        for i=1:nCBF
            [w,best,oneFlag]=linprog(A(i,:).',A(actuatorRows,:), ...
                b(actuatorRows),[],[],[],[],options); %#ok<ASGLU>
            if oneFlag>0 && ~isempty(w) && all(isfinite(w))
                gap=best-b(i);
            else
                gap=NaN;best=NaN;
            end
            row=D.filterResult.rows(i);
            rr=struct('row',i,'obstacleID',double(row.obstacleID), ...
                'component',char(row.component), ...
                'minimumLeftHandSide',best,'requiredUpperBound',b(i), ...
                'unavoidableViolation',gap,'lpExitflag',oneFlag);
            report.rowReport(end+1)=rr; %#ok<AGROW>
            if isfinite(gap) && (isnan(report.worstSingleCBFGap) || ...
                    gap>report.worstSingleCBFGap)
                report.worstSingleCBFGap=gap;
                report.worstSingleCBFRow=i;
            end
        end
        if isfinite(report.worstSingleCBFRow)
            r=report.rowReport(report.worstSingleCBFRow);
            fprintf(['Most conflicting single CBF row: #%d obs=%g %s, ' ...
                'min(A*w)=%.9g, b=%.9g, forced gap=%.9g\n'], ...
                r.row,r.obstacleID,r.component,r.minimumLeftHandSide, ...
                r.requiredUpperBound,r.unavoidableViolation);
        end
    end
    report.capsuleSupport=struct();
    report.sameStateShapeComparison=struct();
    if isfield(D,'method') && strcmpi(char(D.method),'S4') && ...
            isfield(D,'shapes') && isfield(D.shapes,'capsule')
        x0=double(D.x(:));L=double(D.P(7));
        pL=x0(8:10);pT=x0(14:16);
        e=double(D.shapes.capsule(2));
        report.capsuleSupport.epsilon=e;
        report.capsuleSupport.maxConservatism_m=L*e/2;
        report.capsuleSupport.maxAbsoluteFourthDerivative_wrt_a=3*L/(2*e^3);
        report.capsuleSupport.obstacle=struct('id',{},'a',{}, ...
            'aOverEpsilon',{},'fourthDerivative_wrt_a',{});
        for j=1:numel(D.obsList)
            o=D.obsList(j);
            d=double(o.p_center(:))-(pL-L*pT/2);
            unit=d/norm(d);
            a=unit.'*pT;
            d4=(L/2)*3*e^2*(4*a^2-e^2)/(a^2+e^2)^(7/2);
            oo=struct('id',double(o.id),'a',a,'aOverEpsilon',a/e, ...
                'fourthDerivative_wrt_a',d4);
            report.capsuleSupport.obstacle(end+1)=oo; %#ok<AGROW>
            fprintf('obs=%g: a=n''*pT=%+.8g, a/epsilon=%+.5g, d4s/da4=%+.8g\n', ...
                oo.id,oo.a,oo.aOverEpsilon,oo.fourthDerivative_wrt_a);
        end
        fprintf('The high derivative is a mechanism, not proof of unique causality.\n');
        if exist('Generate_CBF_ActuatorAware_6Methods','file')==2
            % Counterfactual comparison at the EXACT SAME state; no plant run.
            try
                obsIdx=find([D.obsList.id]==3,1,'first');
                if ~isempty(obsIdx) && strcmpi(char(D.filterResult.mode),'physical')
                    aux=double(D.supervisor.aux(:));
                    refZ=D.xd([3,7,11,15,19,23,27]);
                    g3=Generate_CBF_ActuatorAware_6Methods('evaluate', ...
                        'physical','S3','cable',D.x,aux,refZ,D.F1,D.P, ...
                        D.obsList(obsIdx),D.shapes.cable,D.kappa);
                    g4=Generate_CBF_ActuatorAware_6Methods('evaluate', ...
                        'physical','S4','capsule',D.x,aux,refZ,D.F1,D.P, ...
                        D.obsList(obsIdx),D.shapes.capsule,D.kappa);
                    n3=max([1,abs(g3.braw),norm(g3.Araw,inf)]);
                    n4=max([1,abs(g4.braw),norm(g4.Araw,inf)]);
                    report.sameStateShapeComparison=struct( ...
                        'S3_h',g3.h,'S4_h',g4.h, ...
                        'S3_normalized_b',g3.braw/n3, ...
                        'S4_normalized_b',g4.braw/n4, ...
                        'S3_raw_b',g3.braw,'S4_raw_b',g4.braw, ...
                        'S3_raw_A',g3.Araw,'S4_raw_A',g4.Araw);
                    fprintf('SAME-STATE obs3: S3 cable h=%+.9g, raw b=%+.9g, norm b=%+.9g\n', ...
                        g3.h,g3.braw,g3.braw/n3);
                    fprintf('SAME-STATE obs3: S4 capsule h=%+.9g, raw b=%+.9g, norm b=%+.9g\n', ...
                        g4.h,g4.braw,g4.braw/n4);
                end
            catch ME
                fprintf('Same-state S3/S4 comparison unavailable: %s\n',ME.message);
                report.sameStateShapeComparison=struct('error',ME.message);
            end
        end
    end
    REPORT(end+1)=report; %#ok<AGROW>
end
fprintf('\nNo controller commands or project files were changed by this audit.\n');
end

function localRemoveTemp(path)
if exist(path,'dir')==7
    try
        rmdir(path,'s');
    catch ME
        warning('CBF:AuditCleanup','Could not clean temporary audit folder: %s',ME.message);
    end
end
end
