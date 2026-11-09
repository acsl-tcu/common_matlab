classdef TRAJECTORY_C7_BERNSTEIN
    % 3D degree-15 Hermite/Bernstein segment with exact 0..7 endpoint jets.
    % BUILD and EVALUATE are representation primitives, NOT a collision,
    % tracking, cable, thrust, torque, or commit certificate.
    % All physical durations, t0 and endpoint jets come from the caller.
    methods (Static)
        function piece=build(t0_s,T_s,start_jets,end_jets)
            assert(isnumeric(t0_s)&&isreal(t0_s)&&isscalar(t0_s)&&isfinite(t0_s), ...
                'TRAJECTORY_C7_BERNSTEIN:Time','The source time must be finite.');
            assert(isnumeric(T_s)&&isreal(T_s)&&isscalar(T_s)&&isfinite(T_s)&&T_s>0, ...
                'TRAJECTORY_C7_BERNSTEIN:Duration','A positive external duration is required.');
            assert(isnumeric(start_jets)&&isequal(size(start_jets),[3 8])&& ...
                isreal(start_jets)&&all(isfinite(start_jets(:)))&& ...
                isnumeric(end_jets)&&isequal(size(end_jets),[3 8])&& ...
                isreal(end_jets)&&all(isfinite(end_jets(:))), ...
                'TRAJECTORY_C7_BERNSTEIN:JetSize', ...
                'Both boundary jets must be finite real 3x8 arrays (orders 0..7).');
            n=15;
            % Reverse-time endpoint derivative k carries sign (-1)^k.
            left=TRAJECTORY_C7_BERNSTEIN.end_controls(start_jets,T_s,n);
            reversed_end=end_jets;
            reversed_end(:,2:2:8)=-reversed_end(:,2:2:8);
            right_reversed=TRAJECTORY_C7_BERNSTEIN.end_controls(reversed_end,T_s,n);
            P=[left fliplr(right_reversed)];
            dc=cell(1,8);dc{1}=P;
            for k=1:7
                dc{k+1}=(n-k+1)/T_s*diff(dc{k},1,2);
            end
            piece=struct('t0_s',double(t0_s),'duration_s',double(T_s), ...
                'degree',15,'control_points_m',P,'derivative_controls',{dc}, ...
                'start_jets',start_jets,'end_jets',end_jets, ...
                'safety_status','NOT_CERTIFIED_DO_NOT_COMMIT');
        end
        function jets=evaluate(piece,t_s)
            assert(isstruct(piece)&&isscalar(piece)&& ...
                isfield(piece,'degree')&&isequal(piece.degree,15)&& ...
                isfield(piece,'derivative_controls')&&numel(piece.derivative_controls)==8, ...
                'TRAJECTORY_C7_BERNSTEIN:Piece','Invalid degree-15 piece.');
            assert(isnumeric(t_s)&&isreal(t_s)&&isscalar(t_s)&&isfinite(t_s), ...
                'TRAJECTORY_C7_BERNSTEIN:Time','Evaluation time must be finite.');
            u=(double(t_s)-piece.t0_s)/piece.duration_s;
            assert(u>=0&&u<=1,'TRAJECTORY_C7_BERNSTEIN:Domain', ...
                'Extrapolation is not permitted. Select a certified segment.');
            if u==0,jets=piece.start_jets;return;end
            if u==1,jets=piece.end_jets;return;end
            jets=zeros(3,8);
            for k=1:8
                c=piece.derivative_controls{k};
                for j=size(c,2)-1:-1:1
                    c(:,1:j)=(1-u)*c(:,1:j)+u*c(:,2:j+1);
                end
                jets(:,k)=c(:,1);
            end
        end
        function [ref28,yaw_jets,position_jets]=reference28(piece,t_s,yaw_lift_anchor_rad)
            % Heading-exact jets for vxy~=0, using power series of log(vx+i vy).
            % The caller must maintain/verify the continuous yaw branch along
            % the whole segment; nearest 2*pi lift alone is NOT a winding proof.
            assert(isnumeric(yaw_lift_anchor_rad)&&isreal(yaw_lift_anchor_rad)&& ...
                isscalar(yaw_lift_anchor_rad)&&isfinite(yaw_lift_anchor_rad), ...
                'TRAJECTORY_C7_BERNSTEIN:YawAnchor', ...
                'A finite yaw branch anchor supplied by the active reference is required.');
            position_jets=TRAJECTORY_C7_BERNSTEIN.evaluate(piece,t_s);
            q=complex(position_jets(1,2:8),position_jets(2,2:8));
            q0=q(1);
            assert(isfinite(real(q0))&&isfinite(imag(q0))&&abs(q0)>0, ...
                'TRAJECTORY_C7_BERNSTEIN:HeadingSingular', ...
                'Horizontal speed zero: heading-exact yaw is not defined without a certified extension.');
            % q_n are Taylor coefficients of horizontal velocity for n=0..6.
            qcoef=q./factorial(0:6);
            g=zeros(1,7); % coefficients of log(q/q0), except for constant
            for n=1:6
                accum=0;
                for k=1:n-1
                    accum=accum+k*g(k+1)*qcoef(n-k+1);
                end
                g(n+1)=(n*qcoef(n+1)-accum)/(n*qcoef(1));
            end
            yaw_jets=imag(g).*factorial(0:6);
            principal=atan2(imag(q0),real(q0));
            yaw_jets(1)=principal+2*pi*round((yaw_lift_anchor_rad-principal)/(2*pi));
            ref28=reshape([position_jets(:,1:7);yaw_jets],28,1);
            assert(all(isfinite(ref28)), ...
                'TRAJECTORY_C7_BERNSTEIN:Reference','Nonfinite 28-vector.');
        end
        function [same,delta]=join_jet_check(left,right,tolerance)
            % Diagnostic check only: floating comparison is not an exact proof.
            if nargin<3,tolerance=0;end
            assert(isnumeric(tolerance)&&isscalar(tolerance)&&isfinite(tolerance)&&tolerance>=0);
            delta=left.end_jets-right.start_jets;
            same=all(abs(delta(:))<=tolerance);
        end
    end
    methods (Static,Access=private)
        function P=end_controls(J,T,n)
            % For Bernstein degree n, d^k B/dt^k at 0 =
            % n!/(n-k)! / T^k * Delta^k P_0. Solve triangularly.
            P=zeros(3,8);
            for k=0:7
                scale=1;
                for m=0:k-1,scale=scale*(n-m);end
                delta=J(:,k+1)*(T^k)/scale;
                previous=zeros(3,1);
                for j=0:k-1
                    previous=previous+((-1)^(k-j))*nchoosek(k,j)*P(:,j+1);
                end
                P(:,k+1)=delta-previous;
            end
        end
    end
end
