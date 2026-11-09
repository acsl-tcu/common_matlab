function R=CHECK_TRAJECTORY_C9_HLC_TUBE_AND_FOUR_INPUT()
%CHECK_TRAJECTORY_C9_HLC_TUBE_AND_FOUR_INPUT
% A MATHEMATICAL fixture test of 25ms sample/hold virtual chain,
% separate body geometry tube, and the ACTUAL-HLC beta2\rhs perturbation
% formula. This DOES NOT certify any real Flight or grant COMMIT.
% Test source is in common_matlab/ directly; results saved there, pwd-free.
root=fileparts(mfilename('fullpath'));
name='TRAJECTORY_C9_HLC_TUBE_AND_FOUR_INPUT_RESULT';
R=struct('status','FAIL_NOT_STARTED','stage','INIT','error_message','', ...
    'scope','SAMPLED_VIRTUAL_AND_INPUT_LEMMAS_ONLY__NOT_REAL_HLC_PROOF', ...
    'safe_to_commit',false,'actual_HLC_tracking_proven',false, ...
    'real_HLC_four_input_proven',false,'numerical_interval_proven',false, ...
    'virtual_tube_status','NOT_RUN','physical_tube_status','NOT_RUN', ...
    'input_lemma_status','NOT_RUN','sample_transition_parity_max',NaN, ...
    'intersample_sampled_norm_upper_gap_min',NaN, ...
    'tube_radii_m',[NaN NaN NaN], ...
    'conditional_four_input_headroom',NaN, ...
    'missing_model_rejected',false,'singular_beta_rejected',false, ...
    'unproven_normalization_rejected',false, ...
    'elapsed_ms',NaN,'zip_path',fullfile(root,[name '_FOR_CHATGPT.zip']));
clock=tic;
try
    addpath(fullfile(root,'reference','REPLANNING_SYSTEM', ...
        'TRAJECTORY_GENERATION'),'-begin');
    % TEST-ONLY mathematically stable gains. In Flight, these MUST instead
    % be read from TRAJECTORY_C9_HLC_FORCE_AUTHORITY.bind(...).
    F=struct('F1',[9 6],'F2',[1 6 15 20 15 6], ...
        'F3',[1 6 15 20 15 6],'F4',[9 6]);
    dt=.025;H=.3;
    E0=[1e-5;1e-5;1e-5;1e-5];
    D=[1e-7;1e-7;1e-7;1e-7];
    R.stage='SAMPLED_VIRTUAL_INCLUSIVE_INTERSAMPLE_BOUND';
    V=TRAJECTORY_C9_HLC_SAMPLED_VIRTUAL_TUBE(F,dt,H,E0,D);
    R.virtual_tube_status=V.status;
    assert(strcmp(V.status, ...
        'CONDITIONAL_SAMPLED_VIRTUAL_TUBE__REAL_HLC_MODEL_PENDING'), ...
        'C9_HLC_LEMMA:Virtual','Virtual sampled-data lemma unavailable.');
    assert(all(isfinite(V.max_intersample_error_2norm))&& ...
        all(V.max_intersample_error_2norm>=E0)&& ...
        ~V.safe_to_commit&&~V.actual_HLC_equivalence_proven, ...
        'C9_HLC_LEMMA:NotProof','A virtual lemma became a real proof.');
    errs=zeros(1,4);gap=Inf;
    names={'F1','F2','F3','F4'};
    for ix=1:4
        gg=F.(names{ix}); gain=gg(:).';n=numel(gain);
        A=diag(ones(n-1,1),1);B=zeros(n,1);B(n)=1;
        C=zeros(n,n);Ad=eye(n);Bd=zeros(n,1);
        for k=1:n
            if k<n
                C=A^k*dt^k/factorial(k);
            else
                C=zeros(n);
            end
            Ad=Ad+C;
            Bd=Bd+A^(k-1)*B*dt^k/factorial(k);
        end
        truth=Ad-Bd*gain;
        errs(ix)=max(abs(V.sample_step_transition{ix}-truth),[],'all');
        for frac=linspace(0,1,21)
            h=dt*frac;Ad1=eye(n);Bd1=zeros(n,1);
            for k=1:n
                if k<n,Ad1=Ad1+A^k*h^k/factorial(k);end
                Bd1=Bd1+A^(k-1)*B*h^k/factorial(k);
            end
            gap=min(gap,V.max_Bernstein_transition_norm(ix)- ...
                norm(Ad1-Bd1*gain,2));
        end
    end
    R.sample_transition_parity_max=max(errs);
    R.intersample_sampled_norm_upper_gap_min=gap;
    assert(R.sample_transition_parity_max<1e-10&&gap>-1e-11, ...
        'C9_HLC_LEMMA:Transition','Sampled transition or Bernstein hull mismatch.');
    R.stage='PHYSICAL_BODY_TUBE_IMPLICATION';
    P=TRAJECTORY_C9_HLC_PHYSICAL_TUBE_LIFT(V,9.0,1e-5,2.0);
    R.physical_tube_status=P.status;
    assert(strcmp(P.status, ...
        'CONDITIONAL_SEPARATE_UAV_CABLE_PAYLOAD_TUBE__EXECUTION_UNPROVEN'), ...
        'C9_HLC_LEMMA:Body','Unable to lift independent body errors.');
    assert(P.uav_m>=P.payload_m&&P.cable_m==P.uav_m&& ...
        all(isfinite([P.uav_m P.cable_m P.payload_m]))&& ...
        ~P.safe_to_commit,'C9_HLC_LEMMA:Tube','Body bounds invalid.');
    R.tube_radii_m=[P.uav_m P.cable_m P.payload_m];
    bad=TRAJECTORY_C9_HLC_PHYSICAL_TUBE_LIFT(V, ...
        0.5*P.payload_acceleration_error_mps2,1e-5,2.0);
    R.unproven_normalization_rejected=strcmp(bad.status, ...
        'UNKNOWN_TAUT_DIRECTION_NORMALIZATION_NOT_PROVEN');
    assert(R.unproven_normalization_rejected, ...
        'C9_HLC_LEMMA:Degeneracy','Missing tautness proof was accepted.');
    R.stage='REAL_HLC_BETA2_INVERSE_PERTURBATION_LEMMA';
    low=[0;-1;-1;-1];high=[20;1;1;1];
    nom=[2 3;-.1 .1;-.1 .1;-.1 .1];
    I=TRAJECTORY_C9_HLC_FOUR_INPUT_PERTURBATION( ...
        low,high,nom,.004,.5,.02,.005,.1);
    R.input_lemma_status=I.status;
    assert(I.conditional_headroom_all_positive&& ...
        ~I.actual_HLC_4_input_certified&&~I.safe_to_commit, ...
        'C9_HLC_LEMMA:Preclamp','Input implication or fail-closed flag wrong.');
    R.conditional_four_input_headroom=I.minimum_conditional_headroom;
    invalid=TRAJECTORY_C9_HLC_FOUR_INPUT_PERTURBATION( ...
        low,high,nom,.004,2,1,.005,.1);
    R.singular_beta_rejected=strcmp(invalid.status, ...
        'UNKNOWN_HLC_BETA2_INVERTIBILITY_NOT_PROVEN');
    assert(R.singular_beta_rejected, ...
        'C9_HLC_LEMMA:Beta2','Singular beta2 was accepted.');
    invalid=TRAJECTORY_C9_HLC_SAMPLED_VIRTUAL_TUBE( ...
        F,dt,H,[],D);
    R.missing_model_rejected=strcmp(invalid.status, ...
        'UNKNOWN_INVALID_VIRTUAL_TUBE_INPUT');
    assert(R.missing_model_rejected, ...
        'C9_HLC_LEMMA:Evidence','Missing initial error proof accepted.');
    R.stage='NO_REAL_FLIGHT_CERTIFICATION_ALLOWED';
    assert(~V.safe_to_commit&&~P.safe_to_commit&&~I.safe_to_commit, ...
        'C9_HLC_LEMMA:Commit','Conditional lemmas must not authorize Flight.');
    R.status='PASS_SAMPLED_VIRTUAL_AND_HLC_INPUT_LEMMAS__REAL_HLC_PROOF_PENDING';
catch ME
    R.status='FAIL_HLC_TUBE_OR_INPUT_LEMMA';
    R.error_message=[ME.identifier ': ' ME.message];
end
R.elapsed_ms=1000*toc(clock);
try
    save(fullfile(root,[name '.mat']),'R');
    f=fopen(fullfile(root,[name '.txt']),'w');
    if f>=0
        fprintf(f,'status=%s\nstage=%s\nerror=%s\n', ...
            R.status,R.stage,R.error_message);
        fprintf(f,'virtual_tube=%s\nphysical_tube=%s\ninput_lemma=%s\n', ...
            R.virtual_tube_status,R.physical_tube_status,R.input_lemma_status);
        fprintf(f,'transition_parity=%g\nintersample_gap=%g\n', ...
            R.sample_transition_parity_max,R.intersample_sampled_norm_upper_gap_min);
        fprintf(f,'conditional_tube_U_C_L=%s\n',mat2str(R.tube_radii_m,15));
        fprintf(f,'conditional_four_input_headroom=%g\n',R.conditional_four_input_headroom);
        fprintf(f,'safe_to_commit=0 actual_HLC_tracking_proven=0\n');
        fprintf(f,'elapsed_ms=%g\n',R.elapsed_ms);
        fclose(f);
    end
    zip(R.zip_path,{[name '.mat'],[name '.txt']},root);
catch ME
    R.status=['FAIL_SAVE_RESULT_' ME.identifier];
end
end
