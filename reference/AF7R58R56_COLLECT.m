function R56 = AF7R58R56_COLLECT()
% R56 same-event native hotpath + HLC frame + EKF-initialized kinematic SHADOW.
% Read-only AFTER Flight. No source/control/commit changes. No error tube.
root=fileparts(mfilename('fullpath'));
R56=struct();
R56.schema='AF7R58R56_THREE_TRACK_SOURCE_BOUND_R1';
R56.production_source=which('REPLANNING_SYSTEM');
R56.raw_obstacles_sensor_only=true;
R56.robust_error_tube_installed=false;
R56.live_trigger_modified=false;
R56.return_commit_modified=false;
R56.controller_24_95_in_scope=false;
R56.mat_file=fullfile(root,'AF7R58R56_OUT.mat');
R56.report_file=fullfile(root,'AF7R58R56_REPORT.txt');
R56.capture_available=false;
R56.r58_stage_events=[];
R56.frame_events=[];
R56.kinematic_shadow=[];
R56.shadow_labels={'time_s','reference_hit','reference_ttc_s', ...
    'EKF_anchored_shadow_hit','EKF_anchored_shadow_ttc_s', ...
    'shadow_obstacle_id','shadow_component_code', ...
    'EKF_cable_residual_m','anchor_error_m'};
R56.warning='Shadow is a geometrically coherent rigid rotation/translation of reference, NOT HLC/plant physical prediction or continuous-time certificate.';
D=struct();
if isappdata(0,'AF7R58R17_LAST_CAPTURE')
    D=getappdata(0,'AF7R58R17_LAST_CAPTURE');
end
if isstruct(D) && isfield(D,'r54_stage_ring') && ...
        isfield(D,'r55_trigger_packets')
    R56.capture_available=true;
    n=min(64,double(D.r54_stage_ring_count));
    if n>0 && isfield(D,'r56_r58_ring')
        ix=mod((double(D.r54_stage_ring_head)-n:double(D.r54_stage_ring_head)-1),64)+1;
        R56.r58_stage_events=double(D.r56_r58_ring(ix,:));
    end
    n=min(64,double(D.r55_trigger_ring_count));
    if n>0
        ix=mod((double(D.r55_trigger_ring_head)-n:double(D.r55_trigger_ring_head)-1),64)+1;
        packets=D.r55_trigger_packets(ix);
        rings=double(D.r55_trigger_ring(ix,:));
        R56.frame_events=nan(numel(packets),7);
        R56.kinematic_shadow=nan(numel(packets),9);
        for i=1:numel(packets)
            P=packets{i};
            if ~isstruct(P),continue;end
            t=rings(i,1);
            if isfield(P,'xd_active_now') && isfield(P,'xd_nom_now')
                xa=double(P.xd_active_now(:));xn=double(P.xd_nom_now(:));
                if numel(xa)>=3 && numel(xn)>=3
                    R56.frame_events(i,:)=[t, norm(xa(1:3)-P.pL_ref(:,1)), ...
                        norm(xa(1:3)-P.pQ_EKF(:)),norm(xa(1:3)-P.pL_EKF(:)), ...
                        norm(xn(1:3)-P.pL_ref(:,1)), ...
                        norm(xa(1:3)-(P.pL_ref(:,1)-P.L*P.q_ref(:,1))), ...
                        rings(i,2)];
                end
            end
            if ~isfield(P,'active') || ~any(logical(P.active)),continue;end
            if exist('g26_future_trigger_kernel_mex','file')~=3,continue;end
            try
                L=double(P.L);H=double(P.H);dt=double(P.dt);
                N=size(P.pL_ref,2);
                a=double(P.q_ref(:,1)); a=a/norm(a);
                b=double(P.q_EKF(:)); b=b/norm(b);
                co=max(-1,min(1,a.'*b)); v=cross(a,b);
                if co<=-1+1e-8 || ~all(isfinite(v)),continue;end
                K=[0 -v(3) v(2);v(3) 0 -v(1);-v(2) v(1) 0];
                Rot=eye(3)+K+K*K/(1+co);
                qsur=Rot*double(P.q_ref);
                qsur=qsur./vecnorm(qsur,2,1);
                pLsur=double(P.pL_EKF(:)) + double(P.pL_ref)-double(P.pL_ref(:,1));
                [href,jref]=g26_future_trigger_kernel_mex(H,dt,L, ...
                    P.pL_ref,P.q_ref,P.centers,P.rotations,P.axes, ...
                    P.radii,P.ids,double(P.active));
                [hs,js,oid,code]=g26_future_trigger_kernel_mex(H,dt,L, ...
                    pLsur,qsur,P.centers,P.rotations,P.axes, ...
                    P.radii,P.ids,double(P.active));
                prefpQ=double(P.pL_EKF(:))-L*b;
                R56.kinematic_shadow(i,:)=[t,double(href),R56_ttc(href,jref,dt), ...
                    double(hs),R56_ttc(hs,js,dt),double(oid),double(code), ...
                    norm(double(P.pL_EKF(:))-double(P.pQ_EKF(:))-L*b), ...
                    norm(prefpQ-double(P.pQ_EKF(:)))];
            catch
                % Non-authoritative diagnostic failure is explicitly NaN.
            end
        end
    end
end
save(R56.mat_file,'R56','-v7');
f=fopen(R56.report_file,'wt');
assert(f>0,'R56:IO','Could not write report to %s',root);
closer=onCleanup(@()fclose(f)); %#ok<NASGU>
fprintf(f,'AF7R58R56 | CORRIDOR ROW PRECONDITION / HLC FRAME / EKF SHADOW\n');
fprintf(f,'capture=%d | production_trigger_changed=0 | return_commit_changed=0 | robust_tube=DEFERRED\n',R56.capture_available);
fprintf(f,'controller_24_95s_spike_ignored=1 | continuous_wholebody_proved=0 | simulation_speedup_proved=0\n');
fprintf(f,'[STAGE1] time total r58_build r58_homotopy r58_certificate r58_shadow qp_solves nodes full_assignments r16_connected\n');
for i=1:size(R56.r58_stage_events,1)
    z=R56.r58_stage_events(i,:);
    if z(2)==1
        fprintf(f,'%.6f %.5f %.5f %.5f %.5f %.5f %.0f %.0f %.0f %.0f\n',z(1),z(3),z(4),z(5),z(6),z(7),z(8),z(9),z(10),z(11));
    end
end
fprintf(f,'[FRAME] time |xd_HLC_cmd_xyz - future_packet_pL0| |xd_cmd - EKF_Q| |xd_cmd - EKF_L| |xd_nom - packet_pL0| |xd_cmd - ref_implied_Q| version\n');
for i=1:size(R56.frame_events,1)
    z=R56.frame_events(i,:);
    fprintf(f,'%.6f %.6f %.6f %.6f %.6f %.6f %.0f\n',z);
end
fprintf(f,'[SHADOW] time ref_hit ref_ttc now_anchored_hit anchored_ttc obs_id component cable_residual init_error\n');
for i=1:size(R56.kinematic_shadow,1)
    fprintf(f,'%.6f %.0f %.6f %.0f %.6f %.0f %.0f %.6g %.6g\n',R56.kinematic_shadow(i,:));
end
fprintf(f,'LIMIT: EKF-anchored future is constant-rigid-rotation/translation surrogate, NOT HLC model or rigorous future motion. Do not use for production trigger or commits.\n');
fprintf(f,'RETURN: old R58R16 branch accepts held terminal before natural phase; still unchanged. Free-time C7 delayed rejoin remains unproved.\n');
fprintf(f,'MAT=%s\nTXT=%s\n',R56.mat_file,R56.report_file);
fprintf('R56 output written directly to %s\n',root);
end
function ttc=R56_ttc(hit,j,dt)
ttc=NaN;
if logical(hit)&&isfinite(double(j))
    ttc=(double(j)-1)*dt;
end
end
