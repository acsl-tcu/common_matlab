function R=TEST_TRIGGER_V3_GEOMETRY_AUDIT(nRandom)
%TEST_TRIGGER_V3_GEOMETRY_AUDIT V2 vs V3 exact-status + enclosure audit.
% Every C/B/D x sphere/ellipsoid test uses ONE mode per call.
if nargin<1,nRandom=100;end
modes={'C','B','D'};covers={'multiple_spheres','multiple_ellipsoids'};
R=struct('status','NOT_RUN','compared',0,'falseClear',0, ...
    'differentDecision',0,'verifiedPrescreenClearPairs',0, ...
    'referenceCodeFound',false);
rng(25);
for m=1:3
    for c=1:2
        mode=modes{m};cover=covers{c};
        [~,cfg,F]=selected_test_context(mode,cover);
        % Deterministic geometric cases: empty / far / near / cable.
        for si=1:4
            switch si
                case 1, centers=zeros(3,0);a=zeros(3,0);
                case 2, centers=[40+(1:36);15*ones(1,36);2*ones(1,36)];a=.15*ones(3,36);
                case 3, centers=[.3+.09*(0:35);.38*ones(1,36);2*ones(1,36)];a=repmat([.20;.12;.09],1,36);
                case 4, centers=[.8;0;2];a=.18*ones(3,1);
            end
            [S,E]=six_test_fixture(centers,a);
            known=choose(S,E,cover);
            compareOnce(known,cfg,F);
        end
        % Conditional ground case and malformed-state cases must retain
        % exactly the V2 fail-closed behavior.
        [S,E]=six_test_fixture(zeros(3,0),zeros(3,0),true);
        compareOnce(choose(S,E,cover),cfg,F);
        Fbad=rmfield(F,'rhoDrone');
        compareOnce(choose(S,E,cover),cfg,Fbad);
        for trial=1:nRandom
            % Randomized center positions, orientation and dimensions
            % within existing test fixture support.
            np=randi([1,12]);
            centers=[-1+6*rand(1,np); -1+2*rand(1,np); .2+4*rand(1,np)];
            a=.03+.35*rand(3,np);
            [S,E]=six_test_fixture(centers,a);
            known=choose(S,E,cover);
            compareOnce(known,cfg,F);
            if np>=1 && mod(trial,4)==0
                verifyPrescreen(known,cfg,F);
            end
        end
    end
end
R.status='PASS_V3_V2_EXACT_DECISION_PARITY_AND_PRESCREEN_INCLUSION';
    function compareOnce(known,cfg,F)
        old=wp_six_trigger_evaluate(known,cfg,F);
        new=wp_six_trigger_evaluate_fast_v3(known,cfg,F);
        R.compared=R.compared+1;
        if ~strcmp(old.status,new.status)
            R.differentDecision=R.differentDecision+1;
            if strcmp(new.status,'CLEAR_CONDITIONAL') && ~strcmp(old.status,'CLEAR_CONDITIONAL')
                R.falseClear=R.falseClear+1;
            end
            error('LightTrigger:Parity', ...
                'Status changed for %s x %s old %s new %s', ...
                cfg.systemMode,cfg.obstacleApproximation,old.status,new.status);
        end
        if strcmp(old.status,'POSSIBLE_CONTACT')
            assert(isequal(old.firstInterval,new.firstInterval));
            assert(strcmp(old.firstObstacle,new.firstObstacle));
        end
    end
    function verifyPrescreen(known,cfg,F)
        ids=1:known.numParts;
        [possible,~]=wp_six_trigger_interval_broadphase(known,cfg,F,ids);
        for k=1:numel(F.timeOffsets)-1
            if all(possible(k,:)),continue;end
            b=wp_six_trigger_make_sweep(cfg,F,k,cfg.systemMode);
            for j=find(~possible(k,:))
                center=known.center(:,j);rr=known.outerRadius(j);
                for bi=1:numel(b)
                    low=min(b(bi).vertices,[],2)-b(bi).radius;
                    high=max(b(bi).vertices,[],2)+b(bi).radius;
                    gap=max(max(low-center,center-high),0);
                    % Prior to numerical tolerance: actual body must be
                    % AABB-separated whenever the fast outer box is clear.
                    assert(norm(gap)>rr-1e-10);
                    R.verifiedPrescreenClearPairs=R.verifiedPrescreenClearPairs+1;
                end
            end
        end
    end
end
function known=choose(S,E,c)
if strcmp(c,'multiple_spheres'), known=wp_six_trigger_prepare(S,c);
else,known=wp_six_trigger_prepare(E,c);end
end
