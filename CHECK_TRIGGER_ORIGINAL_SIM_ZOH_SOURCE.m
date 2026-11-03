function R=CHECK_TRIGGER_ORIGINAL_SIM_ZOH_SOURCE()
% Original HLC/plant source identification, not a flight safety theorem.
R=TRIGGER_ORIGINAL_SIM_ZOH_SOURCE_SCOPE();
f=fopen('TRIGGER_ORIGINAL_SIM_ZOH_RESULT.txt','w');
if f>=0
    fprintf(f,'status=%s\nmodel_sha256=%s\nmodel_pinned=%d\nhlc_pinned=%d\noriginal_sim_zoh=%d\nplant_mass_refresh_proven=%d\nflight_safe=%d\nreason=%s\n', ...
        R.status,R.model_sha256,R.model_source_pinned, ...
        R.hlc_source_pinned,R.sim_zero_order_hold_source, ...
        R.sim_parameter_refresh_each_tick,R.safe_to_commit,R.reason);
    fclose(f);
end
end
