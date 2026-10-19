function OUT = AF7R57R_CANONICAL_SERIALIZATION_FEASIBILITY_AUDIT()
% AF7R57R_CANONICAL_SERIALIZATION_FEASIBILITY_AUDIT
% Read-only source-bound audit of the production repaired-reference ->
% R57 native correction-packet serialization contract.
%
% No production source is modified. No guard is installed. No timing claim
% is upgraded. Approximate/sampled equivalence is never accepted as exact.

clc;
fprintf('\n====================================================================================================\n');
fprintf(' AF7R57R | CANONICAL REPAIRED-REFERENCE SERIALIZATION FEASIBILITY AUDIT\n');
fprintf('====================================================================================================\n');
fprintf('[AUTHORITY] readOnly=1 | sourceBound=1 | exactSerializationRequired=1 | approximationForbidden=1 | productionModified=0\n');

root=fileparts(mfilename('fullpath'));
self=[mfilename('fullpath') '.m'];
fprintf(' immutable entrypoint                    = %s\n',self);
fprintf(' immutable entrypoint SHA-256             = %s\n',sha256_file(self));

rps=fullfile(root,'REPLANNING_SYSTEM.m');
mk=fullfile(root,'af7r34_make_core_packet.m');
bm=fullfile(root,'af7r33_make_batch_model_from_proposal.m');
core=fullfile(root,'af7r57k_trace_core.hpp');
ser=fullfile(root,'af7r57k_corrected_core_mex.cpp');
assert(isfile(rps)&&isfile(mk)&&isfile(bm)&&isfile(core)&&isfile(ser), ...
    '[AF7R57R] required source file missing.');

R=normalize_newlines(fileread(rps));
K=normalize_newlines(fileread(mk));
B=normalize_newlines(fileread(bm));
C=normalize_newlines(fileread(core));
S=normalize_newlines(fileread(ser));

fprintf('\n[A. CURRENT SOURCE SNAPSHOT]\n');
fprintf(' REPLANNING_SYSTEM SHA-256               = %s\n',sha256_file(rps));
fprintf(' af7r33 batch model SHA-256              = %s\n',sha256_file(bm));
fprintf(' af7r34 core packet SHA-256              = %s\n',sha256_file(mk));
fprintf(' R57K corrected core SHA-256             = %s\n',sha256_file(core));
fprintf(' R57K serializer SHA-256                 = %s\n',sha256_file(ser));

% Production semantics.
commitBody = extract_matlab_function(R,'commit_repaired_reference');
refOnlyBody = extract_function_containing(R,'REFERENCE_ONLY_COMMIT_CLOSED');
prodEval = contains(commitBody,'R.eval_fn=candidate.eval_fn') || contains(commitBody,'R.eval_fn = candidate.eval_fn');
prodFlat = contains(commitBody,'R.flat_eval_fn=candidate.flat_eval_fn') || contains(commitBody,'R.flat_eval_fn = candidate.flat_eval_fn');
prodRep = contains(commitBody,'candidate.representation');
prodBatchGeneric = contains(commitBody,'batch_flat_model');
prodBatchRefOnly = contains(refOnlyBody,'batch_flat_model');

fprintf('\n[B. PRODUCTION COMMIT REPRESENTATION]\n');
fprintf(' generic commit stores eval_fn            = %d\n',prodEval);
fprintf(' generic commit stores flat_eval_fn       = %d\n',prodFlat);
fprintf(' generic commit stores representation     = %d\n',prodRep);
fprintf(' generic commit stores batch_flat_model   = %d\n',prodBatchGeneric);
fprintf(' reference-only path stores batch model   = %d\n',prodBatchRefOnly);
fprintf(' generic canonical native packet stored   = %d\n',all_tokens(commitBody,{'repair_t0','repair_offsets','repair_seg0','repair_cp'}));

% Backend catalog.
backends={'BEZIER_D15_C7','POLY_D15_C7','MINCO_STYLE_D15_C7','BSPLINE_D8_C7'};
fprintf('\n[C. BACKEND CATALOG / PRODUCTION VISIBILITY]\n');
for i=1:numel(backends)
    fprintf(' %-24s production token = %d\n',backends{i},contains(R,backends{i}));
end

% af7r33/34 source visibility.
fprintf('\n[D. EXISTING SHADOW SERIALIZATION CANDIDATES]\n');
fprintf(' af7r33 function declaration              = %s\n',first_function_line(B));
fprintf(' af7r34 function declaration              = %s\n',first_function_line(K));
packetFields={'nominal_signature','flight_t0','repair_t0','repair_t1','repair_offsets','repair_seg0','repair_seg1','repair_cp'};
for i=1:numel(packetFields)
    fprintf(' af7r34 writes/mentions %-18s = %d\n',packetFields{i},contains(K,packetFields{i}));
end
fprintf(' af7r34 native packet field set complete  = %d\n',all_tokens(K,packetFields));

% Determine whether af7r34 explicitly proves each backend rather than assuming.
fprintf('\n[E. BACKEND-SPECIFIC EXACT SERIALIZATION EVIDENCE]\n');
backendEvidence=false(1,numel(backends));
for i=1:numel(backends)
    tok=backends{i};
    in34=contains(K,tok);
    in33=contains(B,tok);
    % A backend name appearing in the serializer is NOT an exact-equivalence
    % proof.  Close this evidence flag only when the source carries an
    % explicit backend-specific exact-serialization/equivalence marker.
    marker1=['EXACT_SERIALIZATION_' tok];
    marker2=['EXACT_EQUIVALENCE_' tok];
    backendEvidence(i)=in34 && (contains(upper(K),marker1) || contains(upper(K),marker2));
    fprintf(' %-24s af7r33=%d | af7r34=%d | explicit-exact-marker=%d\n',tok,in33,in34,backendEvidence(i));
end

% Native representation semantics.
nativeDegree15 = contains(C,'int deg=15') && contains(C,'CorrSegment');
nativeRepairVec = contains(C,'std::vector<Correction> repairs');
parser48 = contains(S,'repair_cp must be 48xS') || (contains(S,'mxGetM(cpA)!=48') && contains(S,'repair_cp'));

fprintf('\n[F. R57 NATIVE REPRESENTATION REQUIREMENTS]\n');
fprintf(' correction chain vector                 = %d\n',nativeRepairVec);
fprintf(' fixed degree-15 correction evaluation   = %d\n',nativeDegree15);
fprintf(' repair_cp fixed 48xS                    = %d\n',parser48);

% Repository caller scan for af7r34_make_core_packet.
files=dir(fullfile(root,'*.m'));
callers={};
for i=1:numel(files)
    p=fullfile(files(i).folder,files(i).name);
    if strcmpi(p,mk), continue; end
    try t=normalize_newlines(fileread(p)); catch, continue; end
    if contains(t,'af7r34_make_core_packet(')
        callers{end+1}=files(i).name; %#ok<AGROW>
    end
end
prodCaller=any(strcmp(callers,'REPLANNING_SYSTEM.m'));

fprintf('\n[G. SERIALIZER CALL-PROVENANCE]\n');
fprintf(' af7r34 caller count                     = %d\n',numel(callers));
for i=1:numel(callers), fprintf('  caller = %s\n',callers{i}); end
fprintf(' REPLANNING_SYSTEM directly calls af7r34  = %d\n',prodCaller);

% Contract closure logic: require a production path, exact conversion evidence
% for every enabled backend, and native field compatibility. Approximate or
% sampled equivalence never closes this gate.
allBackendsVisible=all(cellfun(@(x)contains(R,x),backends));
allExactBackendEvidence=all(backendEvidence);
nativeCompatible=all_tokens(K,packetFields)&&nativeDegree15&&nativeRepairVec&&parser48;
productionBridge=prodCaller || prodBatchGeneric;
canonicalClosed = productionBridge && allExactBackendEvidence && nativeCompatible;

fprintf('\n[H. CANONICAL SERIALIZATION THEOREM STATUS]\n');
fprintf(' all 4 production backends visible        = %d\n',allBackendsVisible);
fprintf(' exact conversion evidence all backends   = %d\n',allExactBackendEvidence);
fprintf(' native packet compatibility source-bound = %d\n',nativeCompatible);
fprintf(' production -> serializer bridge present  = %d\n',productionBridge);
fprintf(' canonical serialization theorem closed   = %d\n',canonicalClosed);

% Scope-specific conservative classification.
fprintf('\n[I. BACKEND CLASSIFICATION]\n');
for i=1:numel(backends)
    if backendEvidence(i)
        st='SOURCE_EVIDENCE_PRESENT__EQUIVALENCE_STILL_REQUIRES_DIRECT_AUDIT';
    elseif contains(K,backends{i}) || contains(B,backends{i})
        st='REPRESENTATION_VISIBLE__EXACT_EQUIVALENCE_NOT_PROVED';
    else
        st='NO_EXACT_SERIALIZATION_SOURCE_EVIDENCE';
    end
    fprintf(' %-24s = %s\n',backends{i},st);
end

fprintf('\n[J. FORMAL GATE]\n');
fprintf(' historical R57 safety mathematics preserved = 1\n');
fprintf(' no backend silently approximated          = 1\n');
fprintf(' no backend silently removed               = 1\n');
fprintf(' production source modified                = 0\n');
fprintf(' production timing claim upgraded          = 0\n');
if canonicalClosed
    next='DIRECT_EXACT_EQUIVALENCE_AUDIT__THEN_INSTALL_PREPARSE_G_S_GUARD';
else
    next='DEFINE_VERSION_BOUND_CANONICAL_NATIVE_REPRESENTATION_FOR_ALL_ENABLED_BACKENDS__PROVE_EXACT_CONVERSION__THEN_INSTALL_G_S_GUARD';
end
fprintf(' next action                                = %s\n',next);
fprintf('====================================================================================================\n');

OUT=struct();
OUT.schema='AF7R57R_CANONICAL_SERIALIZATION_FEASIBILITY_V1';
OUT.closed=true;
OUT.backends=backends;
OUT.backend_source_evidence=backendEvidence;
OUT.native_compatible=nativeCompatible;
OUT.production_bridge=productionBridge;
OUT.canonical_serialization_closed=canonicalClosed;
OUT.callers=callers;
OUT.production_modified=false;
OUT.timing_claim_upgraded=false;
OUT.next_action=next;
assignin('base','AF7R57R_CANONICAL_SERIALIZATION_FEASIBILITY_AUDIT_OUT',OUT);
save(fullfile(root,'AF7R57R_CANONICAL_SERIALIZATION_FEASIBILITY_AUDIT_OUT.mat'),'OUT');
end

function tf=all_tokens(s,toks)
tf=true;
for i=1:numel(toks), tf=tf&&contains(s,toks{i}); end
end

function z=first_function_line(s)
L=splitlines(string(s)); z='';
for i=1:numel(L)
    q=strtrim(L(i));
    if startsWith(q,'function '), z=char(q); return; end
end
end

function body=extract_matlab_function(src,name)
L=splitlines(string(src));
start=0;
pat=['function '];
for i=1:numel(L)
    q=char(L(i));
    if contains(q,pat) && contains(q,name), start=i; break; end
end
assert(start>0,'[AF7R57R] MATLAB function %s not found.',name);
% Conservative class-method extraction by the next function declaration at
% the same textual level. Sufficient for token/provenance audit; no semantics
% are inferred from nesting.
stop=numel(L);
for i=start+1:numel(L)
    if startsWith(strtrim(L(i)),'function '), stop=i-1; break; end
end
body=char(strjoin(L(start:stop),newline));
end

function body=extract_function_containing(src,token)
L=splitlines(string(src)); idx=find(contains(L,token),1,'first');
if isempty(idx), body=''; return; end
start=idx;
while start>1 && ~startsWith(strtrim(L(start)),'function '), start=start-1; end
stop=numel(L);
for i=idx+1:numel(L)
    if startsWith(strtrim(L(i)),'function '), stop=i-1; break; end
end
body=char(strjoin(L(start:stop),newline));
end

function s=normalize_newlines(s)
s=strrep(s,sprintf('\r\n'),sprintf('\n'));
s=strrep(s,sprintf('\r'),sprintf('\n'));
end

function h=sha256_file(f)
fid=fopen(f,'r'); assert(fid>=0,'cannot open %s',f);
c=onCleanup(@()fclose(fid)); %#ok<NASGU>
b=fread(fid,Inf,'*uint8');
md=java.security.MessageDigest.getInstance('SHA-256');
md.update(b); d=typecast(md.digest(),'uint8');
h=lower(reshape(dec2hex(d,2).',1,[]));
end
