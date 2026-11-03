function A=CHECK_TRIGGER_TAUT_REDUCED_DIRECTIONAL_ENCLOSURE()
% Common-root removable fixture: reconstructed cable/full-body support.
% Numerical sampling is NOT proof of enclosure or actual Flight safety.
A=struct('status','NOT_RUN','tested_samples',0,'violations',0, ...
    'taut_identity_samples',0,'never_commit',true,'reason','');
try
    previous=rng;cleanup=onCleanup(@()rng(previous)); %#ok<NASGU>
    rng(116,'twister');
    X=struct();
    X.q=struct('lower',[-0.2;-0.1;0.9],'upper',[0.3;0.2;1.2]);
    X.v=struct('lower',[-0.3;-0.4;-0.2],'upper',[0.4;0.5;0.3]);
    X.d=struct('lower',[-0.2;-0.2;-1.0],'upper',[0.2;0.2;-0.9]);
    X.w=struct('lower',[-0.5;-0.4;-0.3],'upper',[0.4;0.5;0.3]);
    L=struct('lower',1.9,'upper',2.1);
    radii=struct('uav',0.15,'cable',0.01,'payload',0.12);
    normals=[eye(3),-eye(3),randn(3,8)];
    shape=struct('type','ellipsoid','rotation',eye(3), ...
        'dimensions',[0.3;0.6;0.9], ...
        'center0',[6;0;0]);
    R=TRIGGER_TAUT_REDUCED_DIRECTIONAL_ENCLOSURE(X,L,radii,normals,shape);
    assert(R.conditional_enclosure && ~R.safe_to_commit && ...
        any(R.conditional_separated), ...
        'CHECK_TRIGGER_TAUT_REDUCED:Output', ...
        'Expected conditional support and some separating normal.');
    for k=1:500
        q=X.q.lower+rand(3,1).*(X.q.upper-X.q.lower);
        v=X.v.lower+rand(3,1).*(X.v.upper-X.v.lower);
        a=-0.2+0.4*rand();b=-0.2+0.4*rand();
        d=[a;b;-sqrt(1-a^2-b^2)]; % exactly taut fixture
        w=X.w.lower+rand(3,1).*(X.w.upper-X.w.lower);
        len=L.lower+rand()*(L.upper-L.lower);
        p=q+len*d;vl=v+len*cross(w,d);
        assert(all(p>=R.payload_position_lower-1e-12)&& ...
            all(p<=R.payload_position_upper+1e-12), ...
            'CHECK_TRIGGER_TAUT_REDUCED:Position', 'Payload escaped enclosure.');
        assert(all(vl>=R.payload_velocity_lower-1e-12)&& ...
            all(vl<=R.payload_velocity_upper+1e-12), ...
            'CHECK_TRIGGER_TAUT_REDUCED:Velocity','Velocity escaped enclosure.');
        A.taut_identity_samples=A.taut_identity_samples+1;
        for j=1:size(normals,2)
            n=normals(:,j);n=n/norm(n);
            for s=[0 0.2 0.5 0.9 1]
                bodyval=n.'*q+radii.uav;
                cableval=n.'*(q+s*(p-q))+radii.cable;
                loadval=n.'*p+radii.payload;
                assert(max([bodyval,cableval,loadval])<= ...
                    R.union_support_upper(j)+1e-12, ...
                    'CHECK_TRIGGER_TAUT_REDUCED:Support', ...
                    'Exact taut union escaped directional support.');
                A.tested_samples=A.tested_samples+1;
            end
        end
    end
    assert(A.taut_identity_samples==500,'CHECK_TRIGGER_TAUT_REDUCED:Fixture', ...
        'Expected 500 unit-direction fixture samples.');
    invalid=X;invalid.d.lower=[2;2;2];invalid.d.upper=[3;3;3];
    K=TRIGGER_TAUT_REDUCED_DIRECTIONAL_ENCLOSURE( ...
        invalid,L,radii,normals,shape);
    assert(~K.conditional_enclosure && ~K.safe_to_commit, ...
        'CHECK_TRIGGER_TAUT_REDUCED:InvalidDirection', ...
        'Impossible taut direction must fail closed.');
    A.status='PASS_CONDITIONAL_TAUT_RECONSTRUCTION_FIXTURES__NOT_FLIGHT_SAFETY';
catch ME
    A.status='FAIL_TAUT_RECONSTRUCTION_FIXTURES';
    A.reason=[ME.identifier ' | ' ME.message];
end
save('TRIGGER_REDUCED_TAUT_RESULT.mat','A');
fid=fopen('TRIGGER_REDUCED_TAUT_RESULT.txt','w');
if fid>=0
 fprintf(fid,'status=%s\nidentity_samples=%d\nsupport_samples=%d\nviolations=%d\nreason=%s\n', ...
    A.status,A.taut_identity_samples,A.tested_samples,A.violations,A.reason);
 fclose(fid);
end
end
