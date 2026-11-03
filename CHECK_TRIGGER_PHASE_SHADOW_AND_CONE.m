function R=CHECK_TRIGGER_PHASE_SHADOW_AND_CONE(replanner_instance)
% Disposable common_matlab CHECK. No Flight modifications, no unsafe commit.
R=struct('status','NOT_RUN','reason','', ...
    'phase_shadow_frame_calls',0,'phase_shadow_authority_calls',0, ...
    'phase_shadow_max_ms',NaN,'cone_cases',0, ...
    'cone_sample_violations',0,'safe_to_commit',false);
try
    S=replanner_instance.get_status();
    R.phase_shadow_frame_calls=S.phase_shadow_frame_calls;
    R.phase_shadow_authority_calls=S.phase_shadow_authority_calls;
    R.phase_shadow_max_ms=S.phase_shadow_max_ms;
    assert(S.phase_shadow_frame_calls>0, ...
        'CHECK_TRIGGER_SHADOW:NoNonFlight', ...
        'No Arming/Takeoff shadow frames were executed.');
    axis=[0;0;-1]; theta0=0.10;omega=0.4;h=0.025;
    qlo=[-0.2;-0.15;1.8];qhi=[0.2;0.15;2.2];
    n=[eye(3),-eye(3),[1;1;1],[-1;2;0.5]];
    C=struct('uav_lower',qlo,'uav_upper',qhi, ...
        'initial_direction_axis',axis, ...
        'initial_direction_angle_upper_rad',theta0, ...
        'angular_speed_upper_rad_s',omega, ...
        'interval_duration_s',h,'cable_length_lower_m',2.0, ...
        'cable_length_upper_m',2.0,'r_uav_m',0.2, ...
        'r_cable_m',0.02,'r_payload_m',0.1,'normals',n);
    G=TRIGGER_TAUT_DIRECTION_CONE_SUPPORT(C);
    assert(G.conditional_enclosure && ~G.safe_to_commit,...
        'CHECK_TRIGGER_CONE:NoEnclosure','Conditional cone evaluator failed.');
    theta=theta0+omega*h;
    for k=1:1000
        alpha=theta*mod(k*0.618033988749895,1);
        beta=2*pi*mod(k*0.75487766624669,1);
        d=[sin(alpha)*cos(beta);sin(alpha)*sin(beta);-cos(alpha)];
        q=qlo+(qhi-qlo)*mod(k*0.414213562373095,1);
        for col=1:size(n,2)
            nn=n(:,col);u=nn.'*q;
            for s=[0,0.3,0.7,1]
                cable=nn.'*(q+s*2*d)+0.02*norm(nn);
                if cable>G.cable_support_upper(col)+1e-10
                    R.cone_sample_violations=R.cone_sample_violations+1;
                end
            end
            load=nn.'*(q+2*d)+0.1*norm(nn);
            if load>G.payload_support_upper(col)+1e-10
                R.cone_sample_violations=R.cone_sample_violations+1;
            end
            if u+0.2*norm(nn)>G.uav_support_upper(col)+1e-10
                R.cone_sample_violations=R.cone_sample_violations+1;
            end
        end
        R.cone_cases=R.cone_cases+1;
    end
    assert(R.cone_sample_violations==0, ...
        'CHECK_TRIGGER_CONE:Escaped','A sampled point escaped conditional support.');
    if R.phase_shadow_authority_calls>0
        R.status='PASS_OBSERVED_PHASE_SHADOW_AND_CONE__NO_JIT_OR_SAFETY_PROOF';
    else
        R.status='PARTIAL_PHASE_SHADOW_FRAME_ONLY__AUTHORITY_NOT_WARMED';
        R.reason=['The tested phase never reached a positive-mass, ' ...
            'static-bound real authority path. Do not claim MCOS warmup.'];
    end
catch ME
    R.status='NOT_COMPLETED_PHASE_SHADOW_OR_CONE';
    R.reason=[ME.identifier ' | ' ME.message];
end
fid=fopen('TRIGGER_PHASE_SHADOW_CONE_RESULT.txt','w');
if fid>=0
    fprintf(fid,'status=%s\nshadow_frame_calls=%d\nshadow_authority_calls=%d\nshadow_max_ms=%.6f\ncone_cases=%d\nviolations=%d\nreason=%s\n', ...
        R.status,R.phase_shadow_frame_calls,R.phase_shadow_authority_calls, ...
        R.phase_shadow_max_ms,R.cone_cases,R.cone_sample_violations,R.reason);
    fclose(fid);
end
save('TRIGGER_PHASE_SHADOW_CONE_RESULT.mat','R');
end
