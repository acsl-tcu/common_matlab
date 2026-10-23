function R = AF7R58R33_RETURN_VECTOR_AUDIT(time_s,correction_xyz)
% Signed 3D return-shape diagnostic, never an obstacle-safety certificate.
% Distinguishes distance-to-nominal norm from a true direction reversal.
validateattributes(time_s,{'double'},{'real','finite','vector'});
validateattributes(correction_xyz,{'double'},{'real','finite','2d'});
t=time_s(:);
if size(correction_xyz,1)==3 && size(correction_xyz,2)==numel(t)
    C=correction_xyz.';
elseif size(correction_xyz,2)==3 && size(correction_xyz,1)==numel(t)
    C=correction_xyz;
else
    error('AF7R58R33:Shape','correction_xyz must be N-by-3 or 3-by-N.');
end
assert(all(diff(t)>0), 'Time stamps must be strictly increasing.');
mag=sqrt(sum(C.^2,2));
[peak,i]=max(mag);
assert(peak>0, 'No nonzero correction exists; signed return is undefined.');
u=C(i,:).'/peak;
signed=C*u;
% A projection can vanish even when the vector is nonzero and orthogonal;
% classification below reports sign, not geometric/path safety.
[minSigned,j]=min(signed);
zeroGuard=64*eps(max(1,peak));
R=struct('schema','AF7R58R33_RETURN_VECTOR_AUDIT_V1', ...
    'reference_time_s',t(i),'max_correction_m',peak, ...
    'signed_axis',u,'min_signed_projection_m',minSigned, ...
    'min_signed_projection_time_s',t(j), ...
    'opposite_side_sample_observed',minSigned < -zeroGuard, ...
    'orientation_definition','DIRECTION_OF_LARGEST_CORRECTION', ...
    'continuous_time_no_overshoot_proven',false, ...
    'obstacle_safety_certified',false, ...
    'time_s',t,'correction_xyz',C, ...
    'correction_norm_m',mag,'signed_projection_m',signed);
figure(34); clf;
plot(t,mag,'DisplayName','||active - nominal|| [m]'); hold on;
plot(t,signed,'DisplayName','Projection along principal correction [m]');
yline(0,'k:');
grid on; xlabel('Time [s]'); ylabel('Correction [m]');
legend('Location','best'); title('R33 Signed correction vs correction norm');
fprintf('[R33] peak=%.6f m at %.6f s; min signed projection=%.6f m at %.6f s; reversed=%d\n', ...
    peak,t(i),minSigned,t(j),R.opposite_side_sample_observed);
end
