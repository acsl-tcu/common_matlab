function OUT=AF7R58ZZY_R22_REPORT
% Print/save R22 outer->reference->stage accounting for the one R21 replan.
P=evalin('base','runtime_profile');
obj=evalin('base','replanner_instance');
assert(isa(obj,'REPLANNING_SYSTEM_R22_ACCOUNTING_DIAG'),'[R22 REPORT] active replanner is not R22 diagnostic class.');
S=obj.get_staged_pipeline_status(); D=obj.get_r22_stage_status();
tc=double(obj.last_replan_commit_time); dt=double(obj.prediction_dt);
assert(isfinite(tc),'[R22 REPORT] no successful staged commit found.');

mb=P.phase=='f' & P.t>=tc-0.70 & P.t<=tc-0.20;
base=median(1e3*P.online_before_plant_s(mb),'omitnan');
win=P.phase=='f' & P.t>=tc-0.075-1e-12 & P.t<=tc+0.125+1e-12;
idx=find(win);

fn=fullfile(pwd,'AF7R58ZZY_R22_REPORT.txt');
fid=fopen(fn,'w'); assert(fid>=0); c=onCleanup(@()fclose(fid));
say(fid,'====================================================================================================\n');
say(fid,' AF7R58ZZY-R22 | ONLINE -> REFERENCE -> STAGE ACCOUNTING\n');
say(fid,'====================================================================================================\n');
say(fid,'[AUTHORITY] diagnosticOnly=1 | optimization=0 | R21BehaviorCopied=1 | controllerModified=0\n');
say(fid,' commit time                         = %.6f s\n',tc);
say(fid,' steady online median                = %.6f ms\n',base);
say(fid,' Stage1/2/3 scheduled                = %.6f / %.6f / %.6f s\n',tc-2*dt,tc-dt,tc);
say(fid,'\n[A. OUTER ONLINE ACCOUNTING]\n');
say(fid,' t[s]    online   excess   acq    sensor  estim   replnr  sload  rmerge refsRes diag   ctrl   input  OUTres\n');
for ii=idx(:).'
    online=1e3*P.online_before_plant_s(ii); acq=1e3*P.sensor_acquisition_s(ii);
    vals=1e3*[P.r22_sensor_prop_s(ii) P.r22_estimator_prop_s(ii) P.r22_replanner_s(ii) P.r22_sload_s(ii) P.r22_reference_merge_s(ii) P.r22_diag_snapshot_s(ii) P.r22_controller_prop_s(ii) P.r22_input_transform_prop_s(ii)];
    refres=1e3*P.r22_reference_prop_s(ii)-sum(vals(3:5));
    outres=online-(acq+vals(1)+vals(2)+1e3*P.r22_reference_prop_s(ii)+vals(6)+vals(7)+vals(8));
    say(fid,'%6.3f %8.3f %8.3f %6.3f %7.3f %7.3f %7.3f %6.3f %7.3f %7.3f %6.3f %6.3f %6.3f %7.3f\n', ...
        P.t(ii),online,online-base,acq,vals(1),vals(2),vals(3),vals(4),vals(5),refres,vals(6),vals(7),vals(8),outres);
end

say(fid,'\n[B. REPLANNER DO ACCOUNTING]\n');
say(fid,' t[s]   replWall lightE2E observer front load dep  base/cert/geom sens/trig out/state | refPkt trigger replanStat writeOut innerRes\n');
for ii=idx(:).'
    rw=1e3*P.r22_replanner_s(ii); light=P.r22_rt_light_e2e_ms(ii);
    front=P.r22_spike_front_ms(ii); loadm=P.r22_spike_loadmass_ms(ii); dep=P.r22_spike_dependency_ms(ii);
    bcg=P.r22_spike_base_cert_geom_ms(ii); stg=P.r22_spike_sensor_trigger_ms(ii); outs=P.r22_spike_output_state_ms(ii);
    innerRes=light-sum([front loadm dep bcg stg outs],'omitnan');
    say(fid,'%6.3f %8.3f %8.3f %8.3f %5.2f %5.2f %5.2f %14.3f %9.3f %9.3f | %6.3f %7.3f %10.3f %8.3f %8.3f\n', ...
        P.t(ii),rw,light,P.r22_rt_observer_ms(ii),front,loadm,dep,bcg,stg,outs, ...
        P.r22_rt_reference_packet_ms(ii),P.r22_rt_trigger_ms(ii),P.r22_rt_replan_status_ms(ii),P.r22_rt_output_write_ms(ii),innerRes);
end

say(fid,'\n[C. STAGE1 INTERNAL]\n');
say(fid,' recorded Stage1                     = %.6f ms\n',S.last_stage1_ms);
say(fid,' full start_staged method            = %.6f ms\n',D.stage1.method_total_ms);
say(fid,' future activation reference packet  = %.6f ms\n',D.stage1.future_packet_ms);
say(fid,' context/materialization             = %.6f ms\n',D.stage1.context_ms);
say(fid,' native proposal wall                = %.6f ms\n',D.stage1.native_wall_ms);
say(fid,' native proposal internal            = %.6f ms\n',D.stage1.native_internal_ms);
say(fid,' AFTER recorded Stage1 timer         = %.6f ms\n',D.stage1.post_recorded_ms);
say(fid,' Stage1 method residual              = %.6f ms\n',D.stage1.residual_ms);

say(fid,'\n[D. STAGE2 INTERNAL]\n');
say(fid,' recorded Stage2                     = %.6f ms\n',S.last_stage2_ms);
say(fid,' reference cert eval                 = %.6f ms\n',D.reference_certificate_ms);
say(fid,' prepare total                       = %.6f ms\n',D.stage2.prepare_total_ms);
say(fid,'  guard/certificate/spacing          = %.6f ms\n',D.stage2.guard_cert_ms);
say(fid,'  batch model                        = %.6f ms\n',D.stage2.batch_model_ms);
say(fid,'  packet reuse gate                  = %.6f ms\n',D.stage2.packet_reuse_gate_ms);
say(fid,'  packet action                      = %.6f ms\n',D.stage2.packet_action_ms);
say(fid,'  handles                            = %.6f ms\n',D.stage2.handles_ms);
say(fid,'  reference metadata                 = %.6f ms\n',D.stage2.metadata_ms);
say(fid,'  bundle                             = %.6f ms\n',D.stage2.bundle_ms);
say(fid,'  prepare residual                   = %.6f ms\n',D.stage2.residual_ms);
say(fid,' recorded-stage unexplained          = %.6f ms\n',S.last_stage2_ms-D.reference_certificate_ms-D.stage2.prepare_total_ms);
say(fid,' runtime packet reuse                = %d\n',D.stage2.runtime_packet_reuse);

say(fid,'\n[E. STAGE3 INTERNAL]\n');
say(fid,' recorded Stage3                     = %.6f ms\n',S.last_stage3_ms);
say(fid,' activate method total               = %.6f ms\n',D.stage3.total_ms);
say(fid,'  schema/time guard                  = %.6f ms\n',D.stage3.guard_ms);
say(fid,'  cache grid                         = %.6f ms\n',D.stage3.grid_ms);
say(fid,'  position/accel eval                = %.6f ms\n',D.stage3.eval_ms);
say(fid,'  validation + q                     = %.6f ms\n',D.stage3.validate_q_ms);
say(fid,'  cache struct                       = %.6f ms\n',D.stage3.cache_struct_ms);
say(fid,'  pre-deadline                       = %.6f ms\n',D.stage3.predeadline_ms);
say(fid,'  Bfull materialization              = %.6f ms\n',D.stage3.bfull_ms);
say(fid,'  final freshness                    = %.6f ms\n',D.stage3.freshness_ms);
say(fid,'  rollback snapshot                  = %.6f ms\n',D.stage3.snapshot_ms);
say(fid,'  authority assignment               = %.6f ms\n',D.stage3.authority_assign_ms);
say(fid,'  hot certificate assert             = %.6f ms\n',D.stage3.hot_cert_ms);
say(fid,'  deadline check                     = %.6f ms\n',D.stage3.deadline_ms);
say(fid,'  cache installation                 = %.6f ms\n',D.stage3.cache_install_ms);
say(fid,'  status/counters                    = %.6f ms\n',D.stage3.status_ms);
say(fid,'  Stage3 residual                    = %.6f ms\n',D.stage3.residual_ms);
say(fid,' recorded-vs-activate residual       = %.6f ms\n',S.last_stage3_ms-D.stage3.total_ms);
say(fid,' table used                          = %d\n',D.stage3.table_used);

% Key stage-cycle cross-accounting.
say(fid,'\n[F. STAGE-CYCLE EXCESS RECONCILIATION]\n');
target=[tc-2*dt tc-dt tc]; sn={'Stage1','Stage2','Stage3'}; sm=[S.last_stage1_ms S.last_stage2_ms S.last_stage3_ms];
for j=1:3
    [~,ii]=min(abs(P.t-target(j)));
    online=1e3*P.online_before_plant_s(ii); excess=online-base;
    say(fid,' %-6s @ %.3f | online %.6f | excess %.6f | recorded %.6f | excess-recorded %.6f | replanner wall %.6f | replanStatus %.6f\n', ...
        sn{j},P.t(ii),online,excess,sm(j),excess-sm(j),1e3*P.r22_replanner_s(ii),P.r22_rt_replan_status_ms(ii));
end

OUT=struct('commit_time',tc,'steady_online_ms',base,'stage_status',S,'diagnostic',D,'window_indices',idx,'report_file',fn);
assignin('base','AF7R58ZZY_R22_OUT',OUT);
save('AF7R58ZZY_R22_OUT.mat','OUT','P','-v7.3');
say(fid,'\nSaved: AF7R58ZZY_R22_OUT.mat\n');
say(fid,'Saved: %s\n',fn);
end

function say(fid,varargin)
fprintf(varargin{:});
fprintf(fid,varargin{:});
end
