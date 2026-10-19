function model = af7r33_nominal_batch_model(replanner)
F=replanner.nominal_flat_derivative_fn;
assert(iscell(F)&&numel(F)>=7&& ...
    all(cellfun(@(f)isa(f,'function_handle'),F(1:7))), ...
    '[AF7R33] nominal flat derivative handles unavailable.');
S=cellfun(@func2str,F(1:7),'UniformOutput',false);
model=struct();
model.schema='AF7R33_BATCH_FLAT_MODEL_V1';
model.certified=true;
model.root='NOMINAL';
model.nominal_signature=sha256_text(strjoin(S,char(10)));
model.flight_reference_time_origin=double(replanner.flight_reference_time_origin);
model.corrections=struct('t0',{},'t1',{},'segment_t0',{}, ...
    'segment_t1',{},'control_points',{});
model.reference_version=double(replanner.active_reference_version);
end

function h=sha256_text(s)
md=java.security.MessageDigest.getInstance('SHA-256');
md.update(uint8(unicode2native(char(s),'UTF-8')));
d=typecast(md.digest(),'uint8');
h=lower(reshape(dec2hex(d,2).',1,[]));
end
