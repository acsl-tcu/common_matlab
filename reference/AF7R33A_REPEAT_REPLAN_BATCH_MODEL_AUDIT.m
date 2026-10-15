function OUT = AF7R33A_REPEAT_REPLAN_BATCH_MODEL_AUDIT(replanner)
clc;
fprintf('\n====================================================================================================\n');
fprintf(' AF7R33A | REPEATED-REPLAN FLAT-MODEL EQUIVALENCE AUDIT\n');
fprintf('====================================================================================================\n');
fprintf('[AUTHORITY] representationEquivalence=1 | sampledSafetyAcceptance=0 | productionModification=0\n');

E=replanner.get_first_trigger_event();
assert(isstruct(E)&&isfield(E,'valid')&&logical(E.valid));
CTX=af7r32_replay_context(replanner,E);
P=replanner.builtin_bezier_d15_c7_proposal(CTX);
assert(isstruct(P)&&isfield(P,'success')&&logical(P.success), ...
    '[AF7R33A] true-trigger proposal replay did not close.');

model1=af7r33_make_batch_model_from_proposal( ...
    replanner,P,double(E.reference.version)+1);

tg=unique([double(P.t0),double(P.t1), ...
    linspace(double(P.t0),double(P.t1),257)]);
[Y1,v1]=af7r33_flat_jet_batch_model(replanner,tg,model1);
assert(all(v1));

Yr=zeros(21,numel(tg));
for k=1:numel(tg)
    x=double(replanner.evaluate_bezier_proposal_flat_reference(tg(k),P));
    x=x(:);
    for r=0:6
        Yr(3*r+(1:3),k)=x(4*r+(1:3));
    end
end

d1=max(abs(Y1-Yr),[],'all');

% Algebraic stacking test: append the same correction a second time.  This
% does not claim a physical second replan; it verifies that the explicit
% correction-chain representation is additive and does not silently fall
% back to NOMINAL after a commit.
model2=model1;
model2.corrections(end+1)=model1.corrections(end);
[Y2,v2]=af7r33_flat_jet_batch_model(replanner,tg,model2);
[Y0,v0]=af7r33_flat_jet_batch_model(replanner,tg,af7r33_nominal_batch_model(replanner));
assert(all(v2)&&all(v0));
d2=max(abs(Y2-(Y0+2*(Y1-Y0))),[],'all');

scale=max([1,max(abs(Yr),[],'all'),max(abs(Y2),[],'all')]);
tol=4096*eps(scale);
closed=d1<=tol&&d2<=tol;

fprintf(' true-trigger proposal success            = %d\n',P.success);
fprintf(' committed correction count model1        = %d\n',numel(model1.corrections));
fprintf(' max |model1 - proposal flat|             = %.17g\n',d1);
fprintf(' max |stacked - algebraic sum|            = %.17g\n',d2);
fprintf(' equivalence tolerance                    = %.17g\n',tol);
fprintf(' repeated-replan flat model closed        = %d\n',closed);
fprintf(' production source modified               = 0\n');
fprintf('====================================================================================================\n');

if closed
    src=which('REPLANNING_SYSTEM'); root=fileparts(src);
    token=struct('schema','AF7R33_REPEAT_MODEL_TOKEN_V1','closed',true, ...
        'max_single_diff',d1,'max_stack_diff',d2,'tol',tol);
    save(fullfile(root,'AF7R33_REPEAT_MODEL_TOKEN.mat'),'token');
end
OUT=struct('schema','AF7R33A_REPEAT_REPLAN_BATCH_MODEL_AUDIT_V1', ...
    'closed',closed,'single_diff',d1,'stack_diff',d2,'tol',tol);
assignin('base','AF7R33A_REPEAT_REPLAN_BATCH_MODEL_AUDIT_OUT',OUT);
end
