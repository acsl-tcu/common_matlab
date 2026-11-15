function G=TRAJECTORY_C9_25D_BATCH_LOCAL_PLANES(S,shapes,rad,L,margin)
%TRAJECTORY_C9_25D_BATCH_LOCAL_PLANES SEARCH-only QP linearization.
% Exactly the same (k,shape,body) constraint order and near-cable lambda
% as the prior scalar implementation. For each observed shape, evaluate
% all normalized 3D directions in ONE true-shape support call.
% The output is not a full-time collision certificate. Final candidate
% always goes through independent continuous-time validation.
start=tic;
% Determine allocation size without dereferencing an unverified field.
% A malformed external fixture must produce a structured UNKNOWN, not an
% unhandled MATLAB:nonExistentField before the guarded validation.
N=0;
if isstruct(S)&&isscalar(S)&&isfield(S,'phase')
    N=numel(S.phase);
end
J=numel(shapes);
G=struct('valid',false,'status','UNKNOWN_BATCH_PLANES_NOT_STARTED', ...
 'A',zeros(25,3*N*J),'b',zeros(1,3*N*J), ...
 'normal',zeros(3,3*N*J),'support_batches',0,'elapsed_ms',NaN, ...
 'error_identifier','','error_message','');
try
    % Fail with an explicit module-level status if a synthetic fixture or
    % external caller omits a field required by the EXACT 25D C9 tangent.
    % Do not synthesize missing physical derivatives in production.
    required={'phase','payload_m','cable_q', ...
              'payload_J_fixed_phase','cable_q_J_fixed_phase'};
    if ~isstruct(S)||~isscalar(S)
        G.status='UNKNOWN_BATCH25_INVALID_TANGENT_STRUCT';
        G.elapsed_ms=1000*toc(start);return
    end
    for f=1:numel(required)
        if ~isfield(S,required{f})
            G.status=['UNKNOWN_BATCH25_MISSING_TANGENT_' required{f}];
            G.elapsed_ms=1000*toc(start);return
        end
    end
    if ~(isnumeric(S.phase)&&isreal(S.phase)&&isvector(S.phase)&& ...
         numel(S.phase)==N&&all(isfinite(S.phase(:))) && ...
         isequal(size(S.payload_m),[3 N])&& ...
         isequal(size(S.cable_q),[3 N])&& ...
         size(S.payload_J_fixed_phase,1)==3&& ...
         size(S.payload_J_fixed_phase,2)==25&& ...
         size(S.payload_J_fixed_phase,3)==N&& ...
         size(S.cable_q_J_fixed_phase,1)==3&& ...
         size(S.cable_q_J_fixed_phase,2)==25&& ...
         size(S.cable_q_J_fixed_phase,3)==N&& ...
         all(isfinite(S.payload_m(:)))&& ...
         all(isfinite(S.cable_q(:)))&& ...
         all(isfinite(S.payload_J_fixed_phase(:)))&& ...
         all(isfinite(S.cable_q_J_fixed_phase(:))))
        G.status='UNKNOWN_BATCH25_INVALID_TANGENT_DIMENSION_OR_NONFINITE';
        G.elapsed_ms=1000*toc(start);return
    end
    if ~isstruct(shapes)||~all(isfield(shapes, ...
              {'center','R','kind','dim','local_vertices'}))
        G.status='UNKNOWN_BATCH25_INVALID_PREPARED_SHAPES';
        G.elapsed_ms=1000*toc(start);return
    end
    assert(isscalar(L)&&isfinite(L)&&L>0 && ...
        isequal(size(rad),[1 3])&&all(isfinite(rad))&& ...
        isfinite(margin)&&margin>=0,'C9_BATCH25:Physics', ...
        'Finite cable length, radii and separation margin are necessary.');
    for j=1:J
        shape=shapes(j);
        nm=zeros(3,3*N);centers=zeros(3,3*N);
        dg=zeros(25,3*N);
        for k=1:N
            p=S.payload_m(:,k);q=S.cable_q(:,k);
            Jp=S.payload_J_fixed_phase(:,:,k);
            Jq=S.cable_q_J_fixed_phase(:,:,k);
            lamNear=max(0,min(1,(q.'*(p-shape.center))/L));
            lam=[1 lamNear 0];
            for body=1:3
                index=(k-1)*3+body;
                center=p-lam(body)*L*q;
                v=center-shape.center;
                nv=norm(v);
                if ~(isfinite(nv)&&nv>0)
                    G.status='UNKNOWN_BATCH25_UNDEFINED_SEARCH_NORMAL';
                    G.elapsed_ms=1000*toc(start);return
                end
                n=v/nv;
                nm(:,index)=n;
                centers(:,index)=center;
                dg(:,index)=(Jp-lam(body)*L*Jq).'*n;
            end
        end
        h=TRAJECTORY_C9_SUPPORT_BATCH(shape,nm);
        G.support_batches=G.support_batches+1;
        for k=1:N
            for body=1:3
                local=(k-1)*3+body;
                globalindex=(k-1)*(J*3)+(j-1)*3+body;
                G.normal(:,globalindex)=nm(:,local);
                G.A(:,globalindex)=dg(:,local);
                G.b(globalindex)=h(local)+rad(body)+margin- ...
                    nm(:,local).'*centers(:,local);
            end
        end
    end
    if ~(all(isfinite(G.A(:))) && all(isfinite(G.b(:))))
        G.status='UNKNOWN_BATCH25_NONFINITE_LINEARIZATION';
    else
        G.valid=true;
        G.status='LOCAL_25D_BATCH_SUPPORT_PLANES_READY__NOT_FULLTIME_PROOF';
    end
catch ME
    G.status=['UNKNOWN_BATCH25_EXCEPTION_' ME.identifier];
    G.error_identifier=ME.identifier;
    G.error_message=ME.message;
end
G.elapsed_ms=1000*toc(start);
end
