function R=TRAJECTORY_C9_HLC_SOURCE_TICK_PLANT_CHAIN(agent,time,x0_plant_24x2)
%TRAJECTORY_C9_HLC_SOURCE_TICK_PLANT_CHAIN Current-tick HLC -> held plant tube.
% Research/source-bound lemma, NOT called from Flight and NOT a future-tick
% execution certificate. Call after the ACTIVE HLC.do and BEFORE plant.do,
% with a certified 24x2 plant-state enclosure at the beginning of the tick.
% The HLC x/xd are taken from actual controller.result.x/.xd (AFTER yaw
% branch compensation). The ORIGINAL HLC methods and its 4 clamps are
% interval evaluated, then the ORIGINAL plant RHS is Picard-contained for
% the entire ZOH tick. No nominal flatness torque or guessed error tube.
%
% Per the sixCkidou IDEAL MODEL convention, latest estimated mL is used as
% the model truth for this lemma. If actual plant.param(20) differs, the
% resulting enclosure refers to the idealized mass-updated plant model,
% not the physical simulator with its different true plant mass.
R=struct('status','UNKNOWN_HLC_SOURCE_TICK_CHAIN_NOT_RUN', ...
 'actual_HLC_source_input_evaluated',false, ...
 'four_preclamp_inputs_in_bounds',false, ...
 'ideal_plant_source_tick_reach_proven',false, ...
 'full_future_HLC_EKF_tube_proven',false, ...
 'safe_to_commit',false,'ideal_payload_mass_kg',NaN, ...
 'sim_plant_mass_equal_to_estimate',false, ...
 'pre_input_box_4x2',NaN(4,2),'plant_box_all_24x2',NaN(24,2), ...
 'plant_box_end_24x2',NaN(24,2), ...
 'hlc_status','NOT_RUN','plant_status','NOT_RUN', ...
 'elapsed_ms',NaN);
tic0=tic;
if exist('infsup','file')~=2 || exist('intval','file')~=2
 R.status='UNKNOWN_HLC_INTLAB_NOT_AVAILABLE';R.elapsed_ms=1000*toc(tic0);return
end
if ~(isnumeric(x0_plant_24x2)&&isequal(size(x0_plant_24x2),[24 2])&& ...
   all(isfinite(x0_plant_24x2(:)))&& ...
   all(x0_plant_24x2(:,1)<=x0_plant_24x2(:,2)))
 R.status='UNKNOWN_PLANT_INITIAL_STATE_BOX';R.elapsed_ms=1000*toc(tic0);return
end
try
 if ~strcmp(char(agent.cha),'f')
    R.status='UNKNOWN_HLC_NOT_FLIGHT';R.elapsed_ms=1000*toc(tic0);return
 end
 ctrl=agent.controller.hlc_suspended;
 plant=agent.plant;
 if ~(isa(ctrl,'HLC_SUSPENDED_LOAD') && isa(plant,'MODEL_CLASS') && ...
      isa(plant.method,'function_handle') && ...
      strcmp(func2str(plant.method),'with_load_model_euler_for_HL'))
    R.status='UNKNOWN_ACTIVE_HLC_PLANT_SOURCE_NOT_MATCHED';
    R.elapsed_ms=1000*toc(tic0);return
 end
 % This is exactly the ZOH used by MODEL_CLASS.do. A changed plant
 % implementation requires source review, not an automatic certification.
 source=which('MODEL_CLASS');
 if isempty(source) || ~isfile(source)
    R.status='UNKNOWN_ZOH_PLANT_CLASS_SOURCE_MISSING';
    R.elapsed_ms=1000*toc(tic0);return
 end
 src=fileread(source);
 if ~(contains(src,'u = obj.self.controller.result.input') && ...
       contains(src,'obj.method(x, u, obj.param)'))
    R.status='UNKNOWN_ZOH_PLANT_SOURCE_IDENTITY';
    R.elapsed_ms=1000*toc(tic0);return
 end
 x=double(ctrl.result.x(:)); xd=double(ctrl.result.xd(:));
 if ~(numel(x)==19&&numel(xd)==28 && ...
      all(isfinite([x;xd])))
    R.status='UNKNOWN_ACTIVE_HLC_TICK_DATA_MISSING';
    R.elapsed_ms=1000*toc(tic0);return
 end
 gains=ctrl.param;
 if ~(isstruct(gains)&&isfield(gains,'dt')&& ...
      isequal(double(gains.dt),double(time.dt)))
    R.status='UNKNOWN_ACTIVE_HLC_SAMPLE_TIME_MISMATCH';
    R.elapsed_ms=1000*toc(tic0);return
 end
 keys={'F1','F2','F3','F4'};
 for k=1:4
    if ~isfield(gains,keys{k})
      R.status='UNKNOWN_ACTIVE_HLC_GAINS_INVALID';
      R.elapsed_ms=1000*toc(tic0);return
    end
    gainvec=gains.(keys{k});
    if any(~isfinite(gainvec(:)))
      R.status='UNKNOWN_ACTIVE_HLC_GAINS_INVALID';
      R.elapsed_ms=1000*toc(tic0);return
    end
 end
 P=double([agent.parameter.get(["mass","jx","jy","jz", ...
            "gravity","loadmass","cableL"]),0,0]);
 if ~(numel(P)==9&&all(isfinite(P))&&P(1)>0&& ...
        P(2)>0&&P(3)>0&&P(4)>0&&P(6)>=0&&P(7)>0)
    R.status='UNKNOWN_HLC_LIVE_PHYSICS_INVALID';
    R.elapsed_ms=1000*toc(tic0);return
 end
 R.ideal_payload_mass_kg=P(6);
 [lo,hi]=TRAJECTORY_C9_HLC_FORCE_AUTHORITY.source_input_limits();
 XI=infsup(x,x);XDI=infsup(xd,xd);PI=infsup(P,P);
 vf=ctrl.Vfd_SuspendedLoadxyDst(gains.dt,XI,XDI.',gains.F1);
 vs=ctrl.Vs_SuspendedLoadxyDst(XI,XDI.',vf,PI, ...
       gains.F2,gains.F3,gains.F4);
 beta=ctrl.Beta2_SuspendedLoadxyDst(XI,XDI.',vf,PI);
 rhs=ctrl.V2_alpha2_SuspendedLoadxyDst(XI,XDI.',vf,vs',PI);
 uf=ctrl.Uf_SuspendedLoadxyDst(XI,XDI.',vf,PI);
 I=TRAJECTORY_C9_HLC_FOUR_INPUT_INTERVAL_INTLAB( ...
       lo,hi,beta,rhs,uf(1));
 R.hlc_status=I.status;
 R.pre_input_box_4x2=I.input_enclosure_4x2;
 if ~I.conditional_4input_bound_proven
    R.status=['UNKNOWN_HLC_PREINPUT_' I.status];
    R.elapsed_ms=1000*toc(tic0);return
 end
 R.actual_HLC_source_input_evaluated=true;
 R.four_preclamp_inputs_in_bounds=true;
 % Model-mass authority comes from CURRENT HLC estimated mL, NOT a
 % frozen Sim initial truth. No claim of physical mismatch certification.
 plantP=double(plant.param(:).');
 if ~(numel(plantP)==21&&all(isfinite(plantP)))
    R.status='UNKNOWN_PLANT_PARAMETER_SOURCE_MISSING';
    R.elapsed_ms=1000*toc(tic0);return
 end
 R.sim_plant_mass_equal_to_estimate=isequal(plantP(20),P(6));
 plantP(20)=P(6);
 reach=TRAJECTORY_C9_HLC_PLANT_TICK_REACH_INTLAB( ...
    plant.method,plantP,x0_plant_24x2, ...
    I.input_enclosure_4x2,[lo,hi],double(time.dt));
 R.plant_status=reach.status;
 if reach.intersample_reach_proven
    R.ideal_plant_source_tick_reach_proven=true;
    R.plant_box_all_24x2=reach.box_all_24x2;
    R.plant_box_end_24x2=reach.box_end_24x2;
    R.status= ...
      'CONDITIONAL_IDEAL_HLC_FOUR_INPUT_AND_PLANT_TICK_REACH__FUTURE_EKF_PENDING';
 else
    R.status=['UNKNOWN_HLC_PLANT_TICK_' reach.status];
 end
catch ME
 R.status=['UNKNOWN_HLC_TICK_SOURCE_EXCEPTION_' ME.identifier];
end
R.elapsed_ms=1000*toc(tic0);
end
