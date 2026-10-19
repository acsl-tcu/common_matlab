function OUT = AF7R57S_BACKEND_EXACT_REPRESENTABILITY_AUDIT()
clc;
fprintf('\n====================================================================================================\n');
fprintf(' AF7R57S | FOUR-BACKEND EXACT REPRESENTABILITY / CANONICAL NATIVE CONTRACT AUDIT\n');
fprintf('====================================================================================================\n');
fprintf('[AUTHORITY] readOnly=1 | exactBasisOnly=1 | sampledFitForbidden=1 | noGuardInstall=1 | productionModified=0\n');

root=fileparts(mfilename('fullpath'));
selfFile=[mfilename('fullpath') '.m'];
selfSHA=sha256_file(selfFile);
fprintf(' immutable entrypoint                    = %s\n',selfFile);
fprintf(' immutable entrypoint SHA-256             = %s\n',selfSHA);

rps=fullfile(root,'REPLANNING_SYSTEM.m');
a33=fullfile(root,'af7r33_make_batch_model_from_proposal.m');
a34=fullfile(root,'af7r34_make_core_packet.m');
old=fullfile(root,'G27V10R2_FOUR_BACKEND_EXACT_C7_AUDIT.m');
core=fullfile(root,'af7r57k_trace_core.hpp');
ser=fullfile(root,'af7r57k_corrected_core_mex.cpp');
req={rps,a33,a34,core,ser};
for k=1:numel(req)
    assert(isfile(req{k}),'[AF7R57S] required source missing: %s',req{k});
end
S=normnl(fileread(rps)); A33=normnl(fileread(a33)); A34=normnl(fileread(a34));
C=normnl(fileread(core)); M=normnl(fileread(ser));
if isfile(old), O=normnl(fileread(old)); else, O=''; end

fprintf('\n[A. CURRENT SOURCE SNAPSHOT]\n');
fprintf(' REPLANNING_SYSTEM SHA-256               = %s\n',sha256_file(rps));
fprintf(' af7r33 SHA-256                          = %s\n',sha256_file(a33));
fprintf(' af7r34 SHA-256                          = %s\n',sha256_file(a34));
fprintf(' R57K core SHA-256                       = %s\n',sha256_file(core));
fprintf(' R57K serializer SHA-256                 = %s\n',sha256_file(ser));
fprintf(' G27V10R2 source available               = %d\n',isfile(old));

B={'BEZIER_D15_C7','POLY_D15_C7','MINCO_STYLE_D15_C7','BSPLINE_D8_C7'};
fprintf('\n[B. CURRENT PRODUCTION BACKEND CONTRACT]\n');
for k=1:4, fprintf(' %-24s production-visible = %d\n',B{k},contains(S,B{k})); end
genericEval=contains(S,'R.eval_fn=candidate.eval_fn')||contains(S,'R.eval_fn = candidate.eval_fn');
genericFlat=contains(S,'R.flat_eval_fn=candidate.flat_eval_fn')||contains(S,'R.flat_eval_fn = candidate.flat_eval_fn');
repOptional=contains(S,'if isfield(candidate,''representation'')');
refBatch=contains(S,'R.batch_flat_model=af7r33_make_batch_model_from_proposal')||contains(S,'R.batch_flat_model = af7r33_make_batch_model_from_proposal');
fprintf(' generic commit eval_fn                  = %d\n',genericEval);
fprintf(' generic commit flat_eval_fn             = %d\n',genericFlat);
fprintf(' generic representation optional         = %d\n',repOptional);
fprintf(' reference-only batch model stored       = %d\n',refBatch);
fprintf(' canonical_native_representation field   = %d\n',contains(S,'canonical_native_representation'));

fprintf('\n[C. NATIVE TARGET REPRESENTATION]\n');
fprintf(' piecewise correction chain              = %d\n',contains(C,'std::vector<Correction> repairs'));
fprintf(' fixed degree-15 correction evaluator    = %d\n',contains(C,'int deg=15'));
fprintf(' serializer repair_cp = 48xS             = %d\n',contains(M,'repair_cp must be 48xS'));
fprintf(' target family                           = PIECEWISE_BERNSTEIN_D15_TRANSLATION_CORRECTION\n');

fprintf('\n[D. EXACT BASIS-IDENTITY SELF TESTS]\n');
n=15; a=((1:16).').^2/137; b=zeros(n+1,1);
for k=0:n
    for j=0:k, b(k+1)=b(k+1)+a(j+1)*nchoosek(k,j)/nchoosek(n,j); end
end
u=(0:32)'/32; pv=zeros(size(u)); bv=zeros(size(u));
for j=0:n, pv=pv+a(j+1)*u.^j; end
for k=0:n, bv=bv+b(k+1)*nchoosek(n,k)*u.^k.*(1-u).^(n-k); end
ePB=max(abs(pv-bv)); passPB=ePB<2e-12;
fprintf(' power D15 -> Bernstein D15 identity      = %d | maxerr=%.3e\n',passPB,ePB);

m=8; N=15; c=((1:m+1).').^3/211; ce=zeros(N+1,1);
for k=0:N
    lo=max(0,k-(N-m)); hi=min(m,k);
    for j=lo:hi
        ce(k+1)=ce(k+1)+c(j+1)*nchoosek(m,j)*nchoosek(N-m,k-j)/nchoosek(N,k);
    end
end
p8=zeros(size(u)); p15=zeros(size(u));
for k=0:m, p8=p8+c(k+1)*nchoosek(m,k)*u.^k.*(1-u).^(m-k); end
for k=0:N, p15=p15+ce(k+1)*nchoosek(N,k)*u.^k.*(1-u).^(N-k); end
eElev=max(abs(p8-p15)); passElev=eElev<2e-12;
fprintf(' Bernstein D8 -> D15 degree elevation     = %d | maxerr=%.3e\n',passElev,eElev);

A=zeros(16,16); row=0;
for r=0:7, row=row+1; A(row,r+1)=factorial(r); end
for r=0:7
    row=row+1;
    for j=r:15, A(row,j+1)=factorial(j)/factorial(j-r); end
end
jetRank=rank(A); jetUnique=(jetRank==16);
fprintf(' endpoint jets 0..7 -> unique D15 poly    = %d | rank=%d\n',jetUnique,jetRank);
basisClosed=passPB&&passElev&&jetUnique;
fprintf(' exact basis mathematics closed          = %d\n',basisClosed);

fprintf('\n[E. HISTORICAL REPRESENTATION SEMANTICS]\n');
oldBez=~isempty(O)&&contains(O,'piecewise degree-15 Bezier representation');
oldPoly=~isempty(O)&&contains(O,'Exact power-basis representation of the SAME degree-15 curve');
oldMinco=~isempty(O)&&contains(O,'Exact endpoint-jet representation/reconstruction of the SAME')&&contains(O,'piecewise degree-15 curve');
oldBs=~isempty(O)&&contains(O,'Independent degree-8 clamped B-spline repair family');
fprintf(' BEZIER D15 exact family evidence        = %d\n',oldBez);
fprintf(' POLY D15 same-polynomial evidence       = %d\n',oldPoly);
fprintf(' MINCO endpoint-jet D15 evidence         = %d\n',oldMinco);
fprintf(' BSPLINE degree-8 family evidence        = %d\n',oldBs);

fprintf('\n[F. PRODUCTION METADATA SUFFICIENCY]\n');
repMandatory=contains(S,'assert(isfield(candidate,''representation'')')||contains(S,'canonical_native_representation');
fprintf(' generic representation mandatory        = %d\n',repMandatory);
fprintf(' function handles alone sufficient        = 0\n');
fields={'repair_t0','repair_t1','repair_offsets','repair_seg0','repair_seg1','repair_cp'};
a34Complete=true; for k=1:numel(fields), a34Complete=a34Complete&&contains(A34,fields{k}); end
fprintf(' af7r34 native packet fields complete     = %d\n',a34Complete);
backendSwitch=contains(A33,'POLY_D15_C7')||contains(A33,'MINCO_STYLE_D15_C7')||contains(A33,'BSPLINE_D8_C7')||contains(A34,'POLY_D15_C7')||contains(A34,'MINCO_STYLE_D15_C7')||contains(A34,'BSPLINE_D8_C7');
fprintf(' af7r33/34 backend-specific exact switch  = %d\n',backendSwitch);

fprintf('\n[G. BACKEND CLASSIFICATION]\n');
theory=[oldBez, oldPoly&&passPB, oldMinco&&jetUnique&&passPB, oldBs&&passElev];
meta=false(1,4);
meta(1)=refBatch&&a34Complete;
meta(2)=repMandatory&&(contains(S,'power_coeff')||contains(S,'coefficients'));
meta(3)=repMandatory&&(contains(S,'endpoint_jet')||contains(S,'endpoint_jets'));
meta(4)=repMandatory&&contains(S,'knots')&&(contains(S,'control_points')||contains(S,'P_ctrl'));
CLS=cell(1,4);
CLS{1}=classify(theory(1),meta(1),'EXACT_NATIVE_D15_BERNSTEIN');
CLS{2}=classify(theory(2),meta(2),'EXACT_POWER_D15_TO_BERNSTEIN_D15');
CLS{3}=classify(theory(3),meta(3),'EXACT_ENDPOINT_JETS_TO_D15_BERNSTEIN');
CLS{4}=classify(theory(4),meta(4),'EXACT_BSPLINE_D8_SPANS_TO_D15_BERNSTEIN');
for k=1:4, fprintf(' %-24s = %s\n',B{k},CLS{k}); end

fprintf('\n[H. REQUIRED VERSION-BOUND CANONICAL NATIVE CONTRACT]\n');
CF={'schema','reference_version','source_backend','nominal_signature','flight_t0','repair_t0','repair_t1','repair_offsets','repair_seg0','repair_seg1','repair_cp_48xS','exact_conversion_method','exact_conversion_certificate','source_dependency_signature'};
for k=1:numel(CF), fprintf(' required field %-28s = 1\n',CF{k}); end
fprintf(' sampled reconstruction allowed          = 0\n');
fprintf(' approximate least-squares fit allowed   = 0\n');
fprintf(' silent backend downgrade allowed        = 0\n');

allTheory=all(theory); allMeta=all(meta); contractInstalled=contains(S,'canonical_native_representation');
closed=allTheory&&allMeta&&contractInstalled;
fprintf('\n[I. FORMAL GATE]\n');
fprintf(' exact representability theory all 4     = %d\n',allTheory);
fprintf(' production metadata sufficient all 4    = %d\n',allMeta);
fprintf(' canonical native contract installed      = %d\n',contractInstalled);
fprintf(' canonical serialization theorem closed   = %d\n',closed);
fprintf(' historical R57 safety mathematics changed= 0\n');
fprintf(' production source modified                = 0\n');
fprintf(' timing claim upgraded                     = 0\n');
fprintf(' next action                                = INSTALL_VERSION_BOUND_CANONICAL_NATIVE_CONTRACT_AND_EXACT_CONVERTERS__THEN_G_S_PREPARSE_GUARD\n');
fprintf('====================================================================================================\n');

OUT=struct('schema','AF7R57S_BACKEND_EXACT_REPRESENTABILITY_V1','backend_names',{B},'classification',{CLS},'theory',theory,'metadata',meta,'basis_closed',basisClosed,'canonical_closed',closed,'production_modified',false,'entrypoint',selfFile,'entrypoint_sha256',selfSHA);
assignin('base','AF7R57S_BACKEND_EXACT_REPRESENTABILITY_AUDIT_OUT',OUT);
save(fullfile(root,'AF7R57S_BACKEND_EXACT_REPRESENTABILITY_AUDIT_OUT.mat'),'OUT');
end

function s=classify(th,meta,label)
if th&&meta, s=[label '__PRODUCTION_METADATA_PRESENT']; elseif th, s=[label '__THEOREM_ONLY__PRODUCTION_METADATA_MISSING']; else, s='SOURCE_SEMANTICS_NOT_CLOSED'; end
end
function s=normnl(s), s=strrep(s,sprintf('\r\n'),sprintf('\n')); s=strrep(s,sprintf('\r'),sprintf('\n')); end
function h=sha256_file(f)
fid=fopen(f,'r'); assert(fid>=0,'cannot open %s',f); c=onCleanup(@()fclose(fid)); b=fread(fid,Inf,'*uint8'); md=java.security.MessageDigest.getInstance('SHA-256'); md.update(b); d=typecast(md.digest(),'uint8'); h=lower(reshape(dec2hex(d,2).',1,[]));
end
