function H=CHECK_TRIGGER_ORIGINAL_HLC_ACTUATOR_SCOPE()
% Disposable source-only verification. Never reads Flight observations.
H=TRIGGER_ORIGINAL_HLC_ACTUATOR_SCOPE();
save('TRIGGER_HLC_ACTUATOR_SCOPE_RESULT.mat','H');
fid=fopen('TRIGGER_HLC_ACTUATOR_SCOPE_RESULT.txt','w');
if fid>=0
    fprintf(fid,'status=%s\nsource_verified=%d\nsha256=%s\nactuator_lower=%s\nactuator_upper=%s\nreal_wiring_verified=%d\nreason=%s\n', ...
        H.status,H.source_verified,H.source_sha256,mat2str(H.lower.'), ...
        mat2str(H.upper.'),H.actual_applied_input_wiring_verified,H.reason);
    fclose(fid);
end
end
