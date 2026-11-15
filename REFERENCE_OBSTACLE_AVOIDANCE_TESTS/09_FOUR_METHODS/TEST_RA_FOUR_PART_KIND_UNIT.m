function T=TEST_RA_FOUR_PART_KIND_UNIT()
% Read exactly the supplied part.kind, with no obstacle re-approximation.
methods={'single_sphere','multiple_spheres','single_ellipsoid','multiple_ellipsoids'};
modeList={'C','B','D'};
agent=RA_TEST_AGENT();cases=0;handOff=0;
for k=1:4
    meth=methods{k};isSphere=contains(meth,'sphere');isSingle=startsWith(meth,'single_');
    if isSphere,kind='sphere';else,kind='ellipsoid';end
    a=fixturePart(kind,'A',[21;0;2],eye(3),1);
    if isSingle
        parts=a;objs=fixtureObjects({'A'});
    else
        % A2 lies outside radius20 but belongs to A: must be passed through.
        a2=fixturePart(kind,'A',[40;0;2],eye(3),.15);
        parts=[a,a2];objs=fixtureObjects({'A'});
    end
    w=struct('parts',parts,'objects',objs,'ground',[], ...
        'approximation',meth,'scene','four_modes_fixture');
    I=ra_obstacle_method_info(w);
    assert(strcmp(I.kind,kind) && I.kindFromParts);
    [observed,rep]=ra_sim_sensor_detect(w,[0;0;2],ra_sim_sensor_config(20));
    assert(strcmp(rep.approximation,meth) && rep.shapeFromOriginalPartKind);
    assert(rep.detectedObjectCount==1 && numel(observed.parts)==numel(parts));
    assert(isequaln(observed.parts,parts)); % exact original definition
    assert(strcmp(observed.approximation,meth));
    for z=1:3
        P=reference_avoidance_prepare_observed(agent,modeList{z},observed,[], ...
            struct('allowMexBuild',false));
        assert(strcmp(P.cfg.obstacleKind,kind));
        assert(P.known.numParts==numel(parts));
        assert(isequal(P.known.center,reshape([parts.center],3,[])));
        for j=1:numel(parts)
            if isSphere,axesExpected=repmat(parts(j).params.radius,3,1);
            else,axesExpected=parts(j).params.axes(:);end
            assert(isequal(P.known.axes(:,j),axesExpected));
            assert(isequal(P.known.R(:,:,j),parts(j).R));
            expected=1+double(~isSphere);
            assert(P.known.kind(j)==expected);
        end
        F=RA_TEST_FORECAST(P.cfg);
        G=wp_six_trigger_evaluate_fast_v8(P.known,P.cfg,F);
        assert(~strcmp(G.status,'UNKNOWN'), 'Core geometry must accept this original kind.');
        handOff=handOff+1;
    end
    % Radius-only config never changes the kind or original axes.
    w2=w;w2.parts(1).kind=alternateKind(kind);
    expectError(@()ra_obstacle_method_info(w2));
    w3=w;w3.approximation=alternateMethod(meth);
    expectError(@()ra_sim_sensor_detect(w3,[0;0;2],ra_sim_sensor_config(20)));
    if isSingle
        w4=w;w4.parts(end+1)=w4.parts(1);
        expectError(@()ra_obstacle_method_info(w4));
    end
    % Source method optional with nonempty parts: infer only a META label.
    wNoMethod=rmfield(w,'approximation');
    J=ra_obstacle_method_info(wNoMethod);
    assert(J.methodInferred && strcmp(J.kind,kind));
    [g,~]=ra_sim_sensor_detect(wNoMethod,[0;0;2],ra_sim_sensor_config(20));
    assert(isequaln(g.parts,w.parts));
    cases=cases+1;
end
% A rotated thin ellipsoid whose enclosing sphere overlaps the range,
% but whose real ellipsoid does not: must not invent a sensor hit.
R90=[0 -1 0;1 0 0;0 0 1];
thin=fixturePart('ellipsoid','thin',[25.5;0;0],R90,1);
thin.params.axes=[6;.2;.2];
w=struct('parts',thin,'objects',fixtureObjects({'thin'}),'ground',[], ...
    'approximation','single_ellipsoid');
[v,rep]=ra_sim_sensor_detect(w,[0;0;0],ra_sim_sensor_config(20));
assert(isempty(v.parts) && rep.detectedObjectCount==0);
T=struct('status','PASS_FOUR_METHODS_BY_ORIGINAL_KIND', ...
    'methodsChecked',4,'systemHandOffs',handOff,'numericGeometryRuns',handOff, ...
    'allOriginalPartsPreserved',true,'rotatedThinEllipsoidNotOverDetected',true, ...
    'realFlightTested',false,'physicalSafetyCertified',false);
end
function p=fixturePart(kind,name,c,R,scale)
if strcmp(kind,'sphere')
    params=struct('radius',scale,'axes',[scale;scale;scale]);
else
    params=struct('radius',scale,'axes',[scale;scale*.65;scale*.4]);
end
p=struct('kind',kind,'center',c,'R',R,'params',params, ...
    'obstacleName',name,'method','','componentIndex',1, ...
    'vertices',zeros(0,3),'H',zeros(0,3),'b',zeros(0,1));
end
function o=fixtureObjects(names)
o=repmat(struct('name','','isDynamic',false,'position',[0 0 0]),1,numel(names));
for i=1:numel(names),o(i).name=names{i};end
end
function y=alternateKind(k)
if strcmp(k,'sphere'),y='ellipsoid';else,y='sphere';end
end
function y=alternateMethod(m)
if contains(m,'sphere'),y='single_ellipsoid';else,y='single_sphere';end
end
function expectError(fn)
raised=false;
try,fn();catch,raised=true;end
assert(raised,'A mismatched shape definition must cause an error.');
end
