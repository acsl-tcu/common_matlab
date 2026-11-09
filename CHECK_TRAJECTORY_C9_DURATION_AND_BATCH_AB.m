function R=CHECK_TRAJECTORY_C9_DURATION_AND_BATCH_AB()
% CHECK-only: fixed physical C9 endpoint jets under variable duration;
% analytic duration derivatives and paired bulk Bernstein point evaluation.
% NEVER an all-time input/collision or actual HLC tracking certificate.
% All results are written to the common_matlab directory, not pwd.
root=fileparts(mfilename('fullpath'));
name='TRAJECTORY_C9_DURATION_AND_BATCH_AB_RESULT';
R=struct('status','FAIL_NOT_STARTED','stage','INIT', ...
 'error_identifier','','error_message','','error_stack',{{}}, ...
 'duration_dcontrols_max_abs',NaN, ...
 'duration_djets_relative_max',NaN, ...
 'duration_fixed_physical_time_djets_relative_max',NaN, ...
 'duration_endpoint_jet_invariance_max',NaN, ...
 'fixed_free_controls_dT_zero',false, ...
 'time_rescale_changes_nonzero_start_velocity',false, ...
 'batch_same_valid_mask',false,'batch_same_outside_mask',false, ...
 'batch_nominal_torque_max_rel_difference',NaN, ...
 'batch_angular_rate_max_rel_difference',NaN, ...
 'median_ms_scalar',NaN,'median_ms_batch',NaN, ...
 'p95_ms_scalar',NaN,'p95_ms_batch',NaN, ...
 'max_ms_scalar',NaN,'max_ms_batch',NaN, ...
 'safe_to_commit',false,'actual_hlc_tracking_proven',false, ...
 'scope','ANALYTIC_TIME_SENSITIVITY_AND_NUMERIC_POINT_AB__NOT_ALL_TIME_PROOF', ...
 'zip_path',fullfile(root,[name '_FOR_CHATGPT.zip']));
try
  addpath(fullfile(root,'controller'),fullfile(root,'model'),'-begin');
  addpath(fullfile(root,'reference','REPLANNING_SYSTEM', ...
      'TRAJECTORY_GENERATION'),'-begin');
  R.stage='VARIABLE_DURATION_FIXED_PHYSICAL_ENDPOINT_JETS';
  % A reproducible CHECK fixture; no production dynamics/flight parameter.
  L=zeros(3,10);E=zeros(3,10);
  L(:,1)=[-1;0;3];E(:,1)=[1;0;3];
  L(:,2)=[0.5;0.02;0];E(:,2)=[0.5;-0.03;0];
  L(:,3)=[0.04;0;0];E(:,3)=[0;0.03;0];
  for k=4:10
      L(:,k)=0.03^(k-2)*[1;-0.5;0.25];
      E(:,k)=0.03^(k-2)*[-0.3;0.25;0.4];
  end
  free=[-1+(10:13)*2/23;[0.01 0.05 -0.04 0.02];3*ones(1,4)];
  T=6;t0=8;
  phases=[0.17 0.39 0.77];
  S=TRAJECTORY_C9_DURATION_SENSITIVITY(t0,T,L,E,free,phases);
  assert(~S.safe_to_commit&&S.piece.degree==23, ...
      'C9_DURATION_AB:Scope','Duration sensitivity cannot authorize COMMIT.');
  h=1e-4*T; % finite-difference TEST step, not a planner threshold.
  Pp=TRAJECTORY_C9_FLAT_BERNSTEIN.build(t0,T+h,L,E,free);
  Pm=TRAJECTORY_C9_FLAT_BERNSTEIN.build(t0,T-h,L,E,free);
  fd=(Pp.control_points_m-Pm.control_points_m)/(2*h);
  R.duration_dcontrols_max_abs= ...
      max(abs(fd-S.dcontrol_points_dT_mps),[],'all');
  assert(R.duration_dcontrols_max_abs<1e-8, ...
      'C9_DURATION_AB:Controls', 'Duration derivative differs from finite difference.');
  Dp=TRAJECTORY_C9_FLAT_BERNSTEIN.actual_derivative_controls(Pp);
  Dm=TRAJECTORY_C9_FLAT_BERNSTEIN.actual_derivative_controls(Pm);
  worst=0;
  for k=1:7
      % Compare the physical derivatives at the SAME NORMALIZED phase,
      % so that no physical-time chain-rule term is silently omitted.
      for u=[0.17 0.39 0.77]
          n=size(Dp{k},2)-1;
          w=zeros(n+1,1);
          for j=0:n
              w(j+1)=nchoosek(n,j)*u^j*(1-u)^(n-j);
          end
          actual=S.dphysical_derivative_controls_dT{k}*w;
          numeric=((Dp{k}-Dm{k})/(2*h))*w;
          rel=norm(actual-numeric)/max(1,norm(actual));
          worst=max(worst,rel);
      end
  end
  R.duration_djets_relative_max=worst;
  % A physical-time derivative needs the extra -u*p^(r+1) chain term.
  worst_fixed=0;
  for it=1:numel(phases)
      tPhysical=t0+phases(it)*T;
      jp=TRAJECTORY_C9_FLAT_BERNSTEIN.evaluate(Pp,tPhysical);
      jm=TRAJECTORY_C9_FLAT_BERNSTEIN.evaluate(Pm,tPhysical);
      ref=(jp(:,1:7)-jm(:,1:7))/(2*h);
      prediction=S.dphysical_jets_dT_fixed_time(:,1:7,it);
      worst_fixed=max(worst_fixed,norm(prediction(:)-ref(:))/ ...
          max(1,norm(prediction(:))));
  end
  R.duration_fixed_physical_time_djets_relative_max=worst_fixed;
  assert(worst_fixed<1e-5, ...
      'C9_DURATION_AB:FixedTime','Physical-time chain-rule duration derivative failed.');
  assert(worst<1e-5, ...
      'C9_DURATION_AB:Jets','Duration physical-jet derivative parity failed.');
  R.duration_endpoint_jet_invariance_max=0;
  for k=1:5
      V=S.dphysical_derivative_controls_dT{k};
      R.duration_endpoint_jet_invariance_max=max( ...
          R.duration_endpoint_jet_invariance_max, ...
          max(abs(V(:,[1 end])),[],'all'));
  end
  assert(R.duration_endpoint_jet_invariance_max<1e-7, ...
      'C9_DURATION_AB:Endpoint','Fixed physical endpoint jet derivative must vanish.');
  R.fixed_free_controls_dT_zero= ...
      all(S.dcontrol_points_dT_mps(:,11:14)==0,'all');
  assert(R.fixed_free_controls_dT_zero, ...
      'C9_DURATION_AB:Free','Free controls must be independent in the partial T derivative.');
  % Uniform re-timing t->t/gamma scales p^(r) by gamma^(-r).
  % Therefore any nonzero prescribed start velocity loses its join.
  gamma=1.5;
  R.time_rescale_changes_nonzero_start_velocity= ...
      norm(L(:,2)/gamma-L(:,2))>0;
  assert(R.time_rescale_changes_nonzero_start_velocity, ...
      'C9_DURATION_AB:NaiveScale','A nonzero physical start velocity must change under naive retiming.');

  R.stage='NUMERIC_BATCH_BERNSTEIN_POINT_TORQUE_AB';
  % Independent same-equation baseline, using the actual controller limit
  % reader. FIXTURE mass is permitted only in this temporary CHECK.
  [lo,hi]=TRAJECTORY_C9_HLC_FORCE_AUTHORITY.source_input_limits();
  param=DRONE_PARAM_SUSPENDED_LOAD("DIATONE");
  param.set("loadmass",0.14);
  d=double(param.get(["mass","jx","jy","jz", ...
      "gravity","cableL","loadmass"]));
  d=d(:);lo=double(lo(:));hi=double(hi(:));
  assert(numel(d)==7&&all(isfinite(d))&&all(d>0)&& ...
      numel(lo)==4&&numel(hi)==4&&all(hi>lo), ...
      'C9_DURATION_AB:Authority','Missing flight fixture physical authority.');
  a=struct('uav_mass_kg',d(1),'payload_mass_kg',d(7), ...
      'cable_length_m',d(6),'uav_inertia_kgm2',d(2:4), ...
      'gravity_world_mps2',[0;0;-d(5)], ...
      'torque_limits_Nm',[lo(2:4) hi(2:4)]);
  opts=struct('minimum_force_z_N',0,'minimum_planar_speed_mps',0, ...
      'torque_guard_Nm',zeros(3,1), ...
      'use_precomputed_point_derivative_polygons',false, ...
      'use_vectorized_bernstein_point_jets',false);
  L0=zeros(3,10);E0=zeros(3,10);
  L0(:,1)=[-1;0;3];E0(:,1)=[1;0;3];
  L0(1,2)=0.5;E0(1,2)=0.5;
  free0=[-1+(10:13)*2/23;zeros(1,4);3*ones(1,4)];
  guide=TRAJECTORY_C9_FLAT_BERNSTEIN.build(8,6,L0,E0,free0);
  ts=guide.t0_s+guide.duration_s*linspace(0,1,33);
  U=TRAJECTORY_C9_TORQUE_POINT_EVALUATE(guide,a,opts,ts);
  batch=opts;batch.use_vectorized_bernstein_point_jets=true;
  V=TRAJECTORY_C9_TORQUE_POINT_EVALUATE(guide,a,batch,ts);
  R.batch_same_valid_mask=isequal(U.valid,V.valid);
  R.batch_same_outside_mask=isequal(U.outside_limits,V.outside_limits);
  assert(R.batch_same_valid_mask&&R.batch_same_outside_mask&&any(U.valid), ...
      'C9_DURATION_AB:BatchMasks','Batch point masks must equal scalar point masks.');
  uval=U.torque_Nm(:,U.valid);vval=V.torque_Nm(:,U.valid);
  R.batch_nominal_torque_max_rel_difference= ...
      max(abs(uval-vval),[],'all')/max(1,max(abs(uval),[],'all'));
  omega0=U.angular_velocity_body_radps(:,U.valid);
  omega1=V.angular_velocity_body_radps(:,U.valid);
  R.batch_angular_rate_max_rel_difference= ...
      max(abs(omega0-omega1),[],'all')/max(1,max(abs(omega0),[],'all'));
  assert(R.batch_nominal_torque_max_rel_difference<1e-9&& ...
      R.batch_angular_rate_max_rel_difference<1e-9, ...
      'C9_DURATION_AB:BatchValues','Batch point torque parity failed.');
  N=7;told=zeros(1,N);tnew=zeros(1,N);
  for rep=1:N
      if mod(rep,2)
          tic;TRAJECTORY_C9_TORQUE_POINT_EVALUATE(guide,a,opts,ts);
          told(rep)=1000*toc;
          tic;TRAJECTORY_C9_TORQUE_POINT_EVALUATE(guide,a,batch,ts);
          tnew(rep)=1000*toc;
      else
          tic;TRAJECTORY_C9_TORQUE_POINT_EVALUATE(guide,a,batch,ts);
          tnew(rep)=1000*toc;
          tic;TRAJECTORY_C9_TORQUE_POINT_EVALUATE(guide,a,opts,ts);
          told(rep)=1000*toc;
      end
  end
  R.median_ms_scalar=median(told);R.median_ms_batch=median(tnew);
  so=sort(told);sn=sort(tnew);
  R.p95_ms_scalar=so(ceil(0.95*N));
  R.p95_ms_batch=sn(ceil(0.95*N));
  R.max_ms_scalar=max(told);R.max_ms_batch=max(tnew);
  R.status='PASS_DURATION_DERIVATIVE_AND_BATCH_POINT_PARITY__NO_FLIGHT_CERTIFICATE';
catch ME
  R.status='FAIL_C9_DURATION_AND_BATCH_AB';
  R.error_identifier=ME.identifier;R.error_message=ME.message;
  R.error_stack=arrayfun(@(x)sprintf('%s:%d',x.name,x.line), ...
      ME.stack,'UniformOutput',false);
end
save(fullfile(root,[name '.mat']),'R');
fid=fopen(fullfile(root,[name '.txt']),'w');
if fid>=0
  fprintf(fid,'status=%s stage=%s error=%s %s\n', ...
      R.status,R.stage,R.error_identifier,R.error_message);
  for j=1:numel(R.error_stack)
      fprintf(fid,'stack=%s\n',R.error_stack{j});
  end
  fprintf(fid,'duration_dcontrols_max_abs=%.12g\n', ...
      R.duration_dcontrols_max_abs);
  fprintf(fid,'duration_djets_relative_max=%.12g\n', ...
      R.duration_djets_relative_max);
  fprintf(fid,'duration_fixed_physical_time_djets_relative_max=%.12g\n', ...
      R.duration_fixed_physical_time_djets_relative_max);
  fprintf(fid,'duration_endpoint_jet_invariance_max=%.12g\n', ...
      R.duration_endpoint_jet_invariance_max);
  fprintf(fid,'fixed_free_dT_zero=%d naive_time_rescale_changes_join=%d\n', ...
      R.fixed_free_controls_dT_zero, ...
      R.time_rescale_changes_nonzero_start_velocity);
  fprintf(fid,'batch_valid/outside_mask_parity=%d/%d\n', ...
      R.batch_same_valid_mask,R.batch_same_outside_mask);
  fprintf(fid,'batch_torque_relative_error=%.12g omega_relative_error=%.12g\n', ...
      R.batch_nominal_torque_max_rel_difference, ...
      R.batch_angular_rate_max_rel_difference);
  fprintf(fid,'scalar/batch_median_ms=%.9f %.9f\n', ...
      R.median_ms_scalar,R.median_ms_batch);
  fprintf(fid,'scalar/batch_p95_ms=%.9f %.9f\n', ...
      R.p95_ms_scalar,R.p95_ms_batch);
  fprintf(fid,'scalar/batch_max_ms=%.9f %.9f\n', ...
      R.max_ms_scalar,R.max_ms_batch);
  fprintf(fid,'safe_to_commit=%d actual_hlc_tracking_proven=%d\n', ...
      R.safe_to_commit,R.actual_hlc_tracking_proven);
  fclose(fid);
end
zip(R.zip_path,{[name '.mat'],[name '.txt']},root);
fprintf('[C9 DURATION & BATCH] %s\n',R.status);
end
