function E=AF7R58R106_PACK_EKF(st,time_s,version,knownMask,L)
% EKF field order is [p;q;v;w;pL;vL;pT;wL].
% IMPORTANT: st.q is UAV attitude/YAW, NOT the suspended cable.
% st.pT (indices 19:21) is the physical cable direction.
% Do not change x24 ordering; existing HLC and plant depend on it.
E=struct('available',false,'status','EKF_EVENT_NOT_CAPTURED', ...
    'time_s',double(time_s),'reference_version',double(version), ...
    'known_mask',logical(knownMask(:).'), ...
    'state_x24',nan(24,1),'q_norm_error',NaN, ...
    'cable_relation_error_m',NaN);
try
    assert(isstruct(st)||isobject(st),'R106:EKFStateSchema');
    assert(isscalar(time_s)&&isfinite(time_s)&& ...
        isscalar(version)&&isfinite(version)&& ...
        isscalar(L)&&isfinite(L)&&L>0, ...
        'R106:EKFAuthority');
    fields={'p','q','v','w','pL','vL','pT','wL'};
    x24=zeros(24,1);
    for k=1:8
        v=st.(fields{k});
        a=double(v(:));
        assert(numel(a)==3&&all(isfinite(a)), ...
            'R106:IncompleteEKFState');
        x24(3*k-2:3*k)=a;
    end
    E.state_x24=x24;
    E.q_norm_error=abs(norm(x24(19:21))-1);
    E.cable_relation_error_m=norm(x24(13:15)-x24(1:3)-L*x24(19:21));
    if E.q_norm_error<=1e-5 && ...
            E.cable_relation_error_m<=1e-5*max(1,L)
        E.available=true;
        E.status='AUTHENTIC_SAME_TICK_EKF_24';
    else
        E.status='EKF_CABLE_GEOMETRY_MISMATCH';
    end
catch ME
    E.status=['EKF_CAPTURE_FAILED__' char(ME.identifier)];
end
end
