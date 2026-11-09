classdef TRAJECTORY_C7_FREE_BERNSTEIN
    % Degree-23 Bernstein representation: two endpoint 7-jets and eight
    % genuinely FREE 3-D interior controls. No specified physical default.
    % Separate segments can share an identical boundary 7-jet => C7.
    % Not a dynamical/collision certificate on its own.
    methods (Static)
        function piece=build(t0_s,duration_s,start_jets,end_jets,interior_controls)
            assert(isnumeric(t0_s)&&isscalar(t0_s)&&isfinite(t0_s) && ...
                isnumeric(duration_s)&&isscalar(duration_s)&&isfinite(duration_s)&&duration_s>0, ...
                'TRAJECTORY_C7_FREE_BERNSTEIN:Time','Finite time and positive duration required.');
            assert(isequal(size(start_jets),[3 8])&&isequal(size(end_jets),[3 8])&& ...
                isequal(size(interior_controls),[3 8])&& ...
                isnumeric(start_jets)&&isreal(start_jets)&&all(isfinite(start_jets(:)))&& ...
                isnumeric(end_jets)&&isreal(end_jets)&&all(isfinite(end_jets(:)))&& ...
                isnumeric(interior_controls)&&isreal(interior_controls)&&all(isfinite(interior_controls(:))), ...
                'TRAJECTORY_C7_FREE_BERNSTEIN:Jets','Require finite 3x8 endpoint jets and free controls.');
            n=23; rev=end_jets;rev(:,2:2:8)=-rev(:,2:2:8);
            P=[TRAJECTORY_C7_FREE_BERNSTEIN.left(start_jets,duration_s,n), ...
                double(interior_controls), fliplr(TRAJECTORY_C7_FREE_BERNSTEIN.left(rev,duration_s,n))];
            controls=cell(1,8); controls{1}=P;
            for k=1:7
                controls{k+1}=(n-k+1)/duration_s*diff(controls{k},1,2);
            end
            piece=struct('t0_s',double(t0_s),'duration_s',double(duration_s), ...
                'degree',n,'control_points_m',P,'derivative_controls',{controls}, ...
                'start_jets',double(start_jets),'end_jets',double(end_jets), ...
                'safety_status','NOT_CERTIFIED_DO_NOT_COMMIT');
        end
        function J=evaluate(piece,t_s)
            assert(isstruct(piece)&&isscalar(piece)&&isfield(piece,'degree')&& ...
                isequal(piece.degree,23)&&isfield(piece,'derivative_controls')&& ...
                numel(piece.derivative_controls)==8, ...
                'TRAJECTORY_C7_FREE_BERNSTEIN:Piece','Expected degree-23 piece.');
            u=(double(t_s)-piece.t0_s)/piece.duration_s;
            assert(isscalar(u)&&isfinite(u)&&u>=0&&u<=1, ...
                'TRAJECTORY_C7_FREE_BERNSTEIN:Domain','Outside time domain.');
            if u==0,J=piece.start_jets;return;end
            if u==1,J=piece.end_jets;return;end
            J=zeros(3,8);
            for k=1:8
                C=piece.derivative_controls{k};
                for m=size(C,2)-1:-1:1
                    C(:,1:m)=(1-u)*C(:,1:m)+u*C(:,2:m+1);
                end
                J(:,k)=C(:,1);
            end
        end
        function [ref28,yaw_jets,position_jets]=reference28(piece,t_s,yaw_anchor_rad)
            % Heading-exact Taylor jet; caller MUST enforce a continuous
            % 2*pi branch between ticks, and full-path nonzero speed.
            assert(isnumeric(yaw_anchor_rad)&&isreal(yaw_anchor_rad)&& ...
                isscalar(yaw_anchor_rad)&&isfinite(yaw_anchor_rad), ...
                'TRAJECTORY_C7_FREE_BERNSTEIN:Yaw','Finite external yaw anchor required.');
            position_jets=TRAJECTORY_C7_FREE_BERNSTEIN.evaluate(piece,t_s);
            velocity=complex(position_jets(1,2:8),position_jets(2,2:8));
            assert(abs(velocity(1))>0, ...
                'TRAJECTORY_C7_FREE_BERNSTEIN:HeadingSingular', ...
                'Heading undefined at zero horizontal speed.');
            a=velocity./factorial(0:6);
            g=zeros(1,7);
            for n=1:6
                accum=0;
                for k=1:n-1
                    accum=accum+k*g(k+1)*a(n-k+1);
                end
                g(n+1)=(n*a(n+1)-accum)/(n*a(1));
            end
            yaw_jets=imag(g).*factorial(0:6);
            main=atan2(imag(velocity(1)),real(velocity(1)));
            yaw_jets(1)=main+2*pi*round((yaw_anchor_rad-main)/(2*pi));
            ref28=reshape([position_jets(:,1:7);yaw_jets],28,1);
            assert(all(isfinite(ref28)), ...
                'TRAJECTORY_C7_FREE_BERNSTEIN:Reference','Nonfinite reference.');
        end
    end
    methods (Static,Access=private)
        function P=left(J,T,n)
            P=zeros(3,8);
            for k=0:7
                scale=prod(n-(0:k-1));
                delta=J(:,k+1)*T^k/scale;
                previous=zeros(3,1);
                for j=0:k-1
                    previous=previous+(-1)^(k-j)*nchoosek(k,j)*P(:,j+1);
                end
                P(:,k+1)=delta-previous;
            end
        end
    end
end
