function record = CBF_TRACE_PIPELINE_STEP(t, stage, agent)
% CBF_TRACE_PIPELINE_STEP  Read-only stage-by-stage plant/estimator/input audit.
%
% Place calls at successive stages of run_flight_headless_CBF_PHYSICAL.m:
%   CBF_TRACE_PIPELINE_STEP(time.t,'after_estimator',agent);
%   CBF_TRACE_PIPELINE_STEP(time.t,'after_controller',agent);
%   CBF_TRACE_PIPELINE_STEP(time.t,'after_input_transform',agent);
%   CBF_TRACE_PIPELINE_STEP(time.t,'after_plant',agent);
%
% Only records t in [14.90, 15.025] s. Saves CBF_pipeline_trace.mat
% after EVERY call, so records survive a later fail-closed CBF exception.
% It never changes agent, HLC, estimator, plant or applied control input.
% IMPORTANT: 'input_transform.result' is an intermediate output; it is NOT
% proof of the actually applied physical torque until plant implementation
% and motor/actuator mapping have been examined.

persistent history
record = struct();
if isempty(history), history = {}; end
if ~isnumeric(t) || ~isscalar(t) || ~isfinite(t) || t < 14.90 || t > 15.025
    return
end

record.t = double(t);
record.stage = char(stage);
record.sensor = localComponent(agent,'sensor');
record.sensorMotive = localNestedComponent(agent,'sensor','motive');
record.estimator = localComponent(agent,'estimator');
record.estimatorEKF = localNestedComponent(agent,'estimator','ekf');
record.estimatorLoadstate = localNestedComponent(agent,'estimator','loadstate');
record.plant = localComponent(agent,'plant');
record.controller = localComponent(agent,'controller');
record.inputTransform = localComponent(agent,'input_transform');
record.parameter = struct('mass',localParam(agent,'mass'), ...
    'loadmass',localParam(agent,'loadmass'), ...
    'jx',localParam(agent,'jx'), ...
    'jy',localParam(agent,'jy'), ...
    'jz',localParam(agent,'jz'));

history{end+1,1} = record; %#ok<AGROW>
try
    save('CBF_pipeline_trace.mat','history');
catch ME
    warning('CBF:Audit:SaveFailed','Could not save trace: %s',ME.message);
end
fprintf('\n[PIPELINE] t=%8.5f %-21s\n',record.t,record.stage);
localPrintInput('sensor result',record.sensor);
localPrintInput('sensor MOTIVE',record.sensorMotive);
localPrint('estimated merged',record.estimator);
localPrint('estimated EKF',record.estimatorEKF);
localPrint('true plant result',record.plant);
localPrintInput('controller',record.controller);
localPrintInput('input transform',record.inputTransform);
localPrintInput('plant result',record.plant);
end

function C=localComponent(agent,name)
C=localEmpty();
try
    node=localGet(agent,name);
    if isempty(node),return;end
    result=localGet(node,'result');
    C=localSummarize(result);
catch ME
    C.note=['capture exception: ' ME.message];
end
end

function C=localNestedComponent(agent,parent,name)
C=localEmpty();
try
    node=localGet(localGet(agent,parent),name);
    if isempty(node),return;end
    C=localSummarize(localGet(node,'result'));
catch ME
    C.note=['capture exception: ' ME.message];
end
end

function C=localSummarize(result)
C=localEmpty();
if isempty(result),return;end
C.available=true;
C.resultType=class(result);
C.resultFieldNames=localNames(result);
s=localGet(result,'state');
if ~isempty(s)
    C.stateType=class(s);
    C.stateFieldNames=localNames(s);
    C.q=localNumeric(localGet(s,'q'));
    % getq('compact') yields the scalar-first quaternion used by the HLC.
    try
        qc=s.getq('compact');
        if isnumeric(qc),C.qCompact=double(qc(:));end
    catch
    end
    C.w=localNumeric(localGet(s,'w'));
    C.p=localNumeric(localGet(s,'p'));
    C.pL=localNumeric(localGet(s,'pL'));
    C.pT=localNumeric(localGet(s,'pT'));
end
for k={'input','u','U','command','torque','M','thrust','forces','motor','motors','output','tmp'}
    key=k{1};
    val=localNumeric(localGet(result,key));
    if ~isempty(val) && numel(val)<=32
        C.numericFields.(key)=val;
    end
end
end

function C=localEmpty()
C=struct('available',false,'note','', ...
    'resultType','','resultFieldNames',{{}},'stateType','', ...
    'stateFieldNames',{{}},'q',[],'qCompact',[],'w',[], ...
    'p',[],'pL',[],'pT',[],'numericFields',struct());
end

function val=localGet(obj,name)
val=[];
try
    if isstruct(obj)
        if isfield(obj,name),val=obj.(name);end
    elseif isobject(obj)
        if isprop(obj,name),val=obj.(name);end
    end
catch
end
end

function names=localNames(obj)
names={};
try
    if isstruct(obj)
        names=fieldnames(obj);
    elseif isobject(obj)
        names=properties(obj);
    end
catch
end
end

function val=localNumeric(v)
val=[];
if isnumeric(v) && ~isempty(v) && numel(v)<=64
    val=double(v(:));
end
end

function val=localParam(agent,key)
val=NaN;
try
    node=localGet(agent,'parameter');
    tmp=node.get(key);
    if isnumeric(tmp)&&isscalar(tmp),val=double(tmp);end
catch
end
end

function localPrint(name,C)
if ~C.available
    fprintf('  %-19s unavailable\n',name);return
end
q=C.qCompact;
if isempty(q),q=C.q;end
fprintf('  %-19s q=%s w=%s',name,localFmt(q),localFmt(C.w));
if strcmp(name,'true plant result')
    fprintf(' (check whether plant.result.state is true/post-step)');
end
fprintf('\n');
end

function localPrintInput(name,C)
fields=fieldnames(C.numericFields);
if isempty(fields)
    fprintf('  %-19s no obvious numeric input fields (available=%d; fields: %s)\n', ...
        name,C.available,strjoin(C.resultFieldNames,','));
    return
end
fprintf('  %-19s',name);
for i=1:numel(fields)
    fld=fields{i};
    fprintf(' %s=%s',fld,localFmt(C.numericFields.(fld)));
end
fprintf('\n');
end

function s=localFmt(v)
if isempty(v)
    s='[]';return
end
v=v(:)';
if numel(v)>12
    s=sprintf('[%d numbers]',numel(v));return
end
s=['[' sprintf(' %+.5g',v) ' ]'];
end
