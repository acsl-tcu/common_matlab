function R=TEST_TRIGGER_V4_ORIENTED_AABB(nRandom)
%TEST_TRIGGER_V4_ORIENTED_AABB Independent geometric checks for V4.
%
% 1) The derived SO(3) ellipsoid AABB contains every sampled true point.
% 2) V3 CLEAR cannot turn into V4 possible/unknown merely through the
%    additional sufficient separation test (all model inputs unchanged).
% 3) An obstacle that visibly includes a predicted drone point is NEVER
%    cleared, independently for all six mode/cover alternatives.
%
% Numerical sampling is an AUDIT, not a continuous floating-point proof.
if nargin<1,nRandom=150;end
rng(20261112);
R=struct('status','NOT_RUN','sampledEllipsoidPoints',0, ...
    'modeDecisions',0,'oldClearRegressions',0,'witnessFalseClear',0, ...
    'newClearDecisions',0,'newResolvedV3Risks',0, ...
    'contactWitnessesChecked',0, ...
    'physicalHlcCertified',false);
for k=1:nRandom
    % A random orientation via QR, ensuring det=+1.
    [Q,~]=qr(randn(3));
    if det(Q)<0,Q(:,1)=-Q(:,1);end
    a=.01+rand(3,1)*2;
    c=randn(3,1)*5;
    half=sqrt(sum((Q.*repmat(a.',3,1)).^2,2));
    u=randn(3,1);u=u/norm(u);
    p=c+Q*(a.*u);
    assert(all(abs(p-c)<=half+5e-13), 'Ellipsoid AABB missed point');
    R.sampledEllipsoidPoints=R.sampledEllipsoidPoints+1;
end
modes={'C','B','D'};covers={'multiple_spheres','multiple_ellipsoids'};
for mi=1:3
    for ci=1:2
        mode=modes{mi}; cover=covers{ci};
        [~,cfg,F]=selected_test_context(mode,cover);
        for k=1:nRandom
            numParts=randi([1,12]);
            center=[-1+5*rand(1,numParts); -1+3*rand(1,numParts);.2+4*rand(1,numParts)];
            axes=.03+rand(3,numParts)*.4;
            [S,E]=six_test_fixture(center,axes);
            if ci==1, world=S;else,world=E;end
            known=wp_six_trigger_prepare(world,cover);
            old=wp_six_trigger_evaluate_fast_v3(known,cfg,F);
            new=wp_six_trigger_evaluate_fast_v4(known,cfg,F);
            R.modeDecisions=R.modeDecisions+1;
            if strcmp(old.status,'CLEAR_CONDITIONAL') && ...
                    ~strcmp(new.status,'CLEAR_CONDITIONAL')
                R.oldClearRegressions=R.oldClearRegressions+1;
                error('LightTrigger:V4Regression', ...
                    'V4 lost a proven V3 CLEAR: %s %s',mode,cover);
            end
            if strcmp(new.status,'CLEAR_CONDITIONAL')
                R.newClearDecisions=R.newClearDecisions+1;
                if strcmp(old.status,'POSSIBLE_CONTACT')
                    R.newResolvedV3Risks=R.newResolvedV3Risks+1;
                end
            end
        end
        % Construct a guaranteed collision witness at t=0 using the drone.
        centers=F.pDrone(:,1);
        axes=.2*ones(3,1);
        [S,E]=six_test_fixture(centers,axes);
        if ci==1,world=S;else,world=E;end
        known=wp_six_trigger_prepare(world,cover);
        r=wp_six_trigger_evaluate_fast_v4(known,cfg,F);
        if strcmp(r.status,'CLEAR_CONDITIONAL')
            R.witnessFalseClear=R.witnessFalseClear+1;
            error('LightTrigger:FalseClear','Contact witness was cleared');
        end
        R.contactWitnessesChecked=R.contactWitnessesChecked+1;
        % A cable-point contact at a knot: separate from the drone test.
        [S,E]=six_test_fixture([.8;0;2],.18*ones(3,1));
        if ci==1,world=S;else,world=E;end
        known=wp_six_trigger_prepare(world,cover);
        r=wp_six_trigger_evaluate_fast_v4(known,cfg,F);
        if strcmp(r.status,'CLEAR_CONDITIONAL')
            R.witnessFalseClear=R.witnessFalseClear+1;
            error('LightTrigger:FalseClear','Cable contact witness was cleared');
        end
        R.contactWitnessesChecked=R.contactWitnessesChecked+1;
    end
end
R.status='PASS_ORIENTED_BOX_SUPPORT_AND_SIX_INDEPENDENT_MODES__CONDITIONAL';
fprintf('V4 AABB: %d ellipsoid points, %d six-mode independent tests, %d resolved risks, 0 known-contact false clears.\n', ...
    R.sampledEllipsoidPoints,R.modeDecisions,R.newResolvedV3Risks);
end
