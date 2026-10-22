function Q19 = AF7R58R19_PREFLIGHT()
% AF7R58R19: non-mutating source-contract check for R16 MEX flag type.
% Research-only, no trajectory generation, no gate weakening.
Q17=AF7R58R17_PREFLIGHT();
s=fileread(Q17.source);
expected='pkt.r58r16_reference_bridge=double(logical(obj.ENABLE_AF7R58R16_REFERENCE_BRIDGE));';
assert(numel(strfind(s,expected))==1, ...
    '[R19] Active MATLAB source is missing the numeric-double R16 packet fix.');
legacy='pkt.r58r16_reference_bridge=logical(obj.ENABLE_AF7R58R16_REFERENCE_BRIDGE);';
assert(isempty(strfind(s,legacy)), ...
    '[R19] Legacy logical-valued bridge packet is still present.');
assert(exist('AF7R58R18_AUTO_RESULT','file')==2, ...
    '[R19] R18 result helper is missing. Keep the existing R18 helper installed.');
Q19=Q17;
Q19.bridge_packet_real_double=true;
Q19.source_check_only=true;
Q19.rebuild_mex_required=false;
Q19.reference_activation_forbidden=true;
fprintf('AF7R58R19 PREFLIGHT PASS | double scalar bridge packet | NO COMMIT\n');
fprintf('source: %s\n',Q19.source);
end
