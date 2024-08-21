%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%  Code modified by Francisco Vasconcelos from
%%%%
%%%%  Drone class, feel free to add functionality as you see fit
%%%%  Author: Daniel Butters
%%%%  Date: 16/11/17
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
classdef Drone_q3b < handle
    properties (Constant)
        %width, length, height offset between centre and rotors
        body = [0.6 0.6 0.0];
        
        %time interval for simulation (seconds)
        %time_interval = 0.02;
        
        % size of floating window that follows drone
        axis_size = 2.;
        
        %colours of each component of drone model
        colours = [[.8 .3 .1];[.2 .2 .5];[.8 .1 .3];[.9 .6 .8];[.9 .2 .4]];
        
        %Follows the drone within the figure
        %Don't use if you are simulating more than one drone!
        %Switch to false to see the overall world view
        drone_follow = true;          
        
        %time step used in simulation
        dt = 0.1;

        % Number of points in the simulation.
        m = 0.2;
        g = 9.2;
        kd = 0.1;
        k = 1;
        I = [1 0 0;0 1 0;0 0 0.5];
        L = 0.2;
        b = 0.1;

    end
    properties
        %axis to draw on
        axis
        pos_axis
        theta_axis
%         posdot_axis
        
        %length of one side of the flight arena
        spaceDim
        
        %limits of flight arena
        spaceLimits
        
        %drone position
        pos
        posdot
        measured_pos
        measured_posdot 

        %angular velocity
        omega

        %drone orientation
        theta
        thetadot
        measured_theta
        measured_thetadot
        measured_omega

        %drone rotation matrix
        R

        %input vector
        i

        %Simulation time
        time
        
        %parameter to start drone in random position
        pos_offset
        
        %number of drones
        num_drones

        % defining arrays required for plotting the graphs for quadcopter trajectory
        plt_x
        plt_y
        plt_z
        plt_roll
        plt_pitch
        plt_yaw
%         plt_xdot
%         plt_ydot
%         plt_zdot
        plt_time

        %Returned parameters from delinearise.m
        disc_sys
        U
        input
        x_0
        u_0

        %%%%%%%%%%%%%%%%%%%%%%%%%%
        %% Defining parameters for use with the feedback controller
        ref1
        ref1_reached
        ref1_wait_time
        ref2
        ref2_reached        
        ref3
        ref3_reached
        ref4
        ref4_reached
        ref5
        ref5_reached
        ref6
        ref6_reached
        K_eigenvalues
        K
        X
        measured_X
        %%%%%%%%%%%%%%%%%%%%%%%%%%

    end
    methods
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %INSTANTIATION OF CLASS
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function obj = Drone_q3b(axis1, axis2, axis3, spaceDim, num_drones)
            if nargin > 1
                obj.axis = axis1;
                %Axis for plotting position trajectories
                obj.pos_axis = axis2;
                %Axis for plotting orientation trajectories
                obj.theta_axis = axis3;
%                 %Axis for plotting linear velocity trajectories
%                 obj.posdot_axis = axis4;
                
                obj.spaceDim = spaceDim;
                
                obj.spaceLimits = [(-spaceDim/2)+10 (spaceDim/2)-10 (-spaceDim/2)+10 (spaceDim/2)-10 10 spaceDim-10];
                
                %initializing array containing values of position for the quadcopter
                obj.pos = [0;0;0];

                % Measured position from simulated sensors
                obj.measured_pos = zeros(3, 1);

                %initializing array containing values of linear velocity for the quadcopter
                obj.posdot = zeros(3, 1);

                % Calculated velocity from simulated sensors
                obj.measured_posdot = zeros(3, 1);

                %initializing array containing values of inputs for the quadcopter (gamma)
                obj.i = [1,1,1,1]*0.5;
                
                obj.pos_offset = [5.*(rand - 0.5),5.*(rand - 0.5),2.5.*(rand)];

                %initializing array containing values of angle about their axis for the quadcopter
                obj.theta = zeros(3, 1);

                % Measured angles from simulated sensors
                obj.measured_theta = zeros(3, 1);
               
                %thetadot
                obj.thetadot = zeros(3, 1);

                %omega
                obj.measured_omega = zeros(3,1);

                %angular velocity
                obj.omega = zeros(3, 1);

                % Measured angular velocity from simulated sensors
                obj.measured_thetadot = zeros(3, 1);
                
                %initializing array for the rotation matrix to change from quadcopter body frame to the inertial frame
                obj.R = [1,0,0;0,1,0;0,0,1];
                
                % Saves current simulation time
                obj.time = 0;

                %Recieves linearization data for the linearized LTI system
                [obj.disc_sys,obj.U,obj.input,obj.x_0,obj.u_0] = dlinearise(obj.dt);

                obj.num_drones = num_drones;

                % initializing arrays required for plotting the graphs for quadcopter trajectory
                obj.plt_x = [0];
                obj.plt_y = [0];
                obj.plt_z = [0];
                obj.plt_roll = [0];
                obj.plt_pitch = [0];
                obj.plt_yaw = [0];
%                 obj.plt_xdot = [0];
%                 obj.plt_ydot = [0];
%                 obj.plt_zdot = [0];
                obj.plt_time = [0];


                % Setting a random set of eigenvalues in the stable range for the feedback controller
                obj.K_eigenvalues = [0.91; 0.86; 0.96; 0.75; 0.87; 0.94; 0.88; 0.98; 0.95; 0.79; 0.90; 0.89];

                % reference locations for Q3a
                % a. Starts at (0,0,0)
                % b. Moves up to (5,5,5)
                % c. Stays at (5,5,5) for 10 seconds.
                % d. Passes through (5,-5,10), (-5,-5,6), (-5,5,2), (0,0,2).
                % e. Lands at (0,0,0) safely (less than 0.1 m/s linear velocity when hitting the ground).
                obj.ref1 = [5;5;5;0;0;0;0;0;0;0;0;0];
                obj.ref2 = [5;-5;10;0;0;0;0;0;0;0;0;0];
                obj.ref3 = [-5;-5;6;0;0;0;0;0;0;0;0;0];
                obj.ref4 = [-5;5;2;0;0;0;0;0;0;0;0;0];
                obj.ref5 = [0;0;2;0;0;0;0;0;0;0;0;0];
                obj.ref6 = [0;0;0;0;0;0;0;0;0;0;0;0];
                obj.ref1_reached = false;
                obj.ref2_reached = false;
                obj.ref3_reached = false;
                obj.ref4_reached = false;
                obj.ref5_reached = false;
                obj.ref6_reached = false;
                obj.ref1_wait_time = 0;

                %Initializing the array to store the state data for the quadcopter.
                obj.X = zeros(12,1);
                %Initializing the array to store the noisy state data for
                %the quadcopter used for calculating feedback.
                obj.measured_X = zeros(12,1);
                
                % This gives us the feedback gain parameters
                % using pole placement of that equation, the eigenvalues are self-defined.
                obj.K = place(obj.disc_sys.A, obj.disc_sys.B, obj.K_eigenvalues); 

                
            else
                error('Drone not initialised correctly')
            end
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %DRAWING OF DRONE TO FIGURE
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function draw(obj)
            %how big should the moving window be
            cL = obj.axis_size;
            
            %set to false if you want to see world view
            %if(obj.drone_follow)
            %    axis([obj.pos(1)-cL obj.pos(1)+cL obj.pos(2)-cL obj.pos(2)+cL obj.pos(3)-cL obj.pos(3)+cL]);
            %end
            
            %create middle sphere
            [X Y Z] = sphere(8);
            %[X Y Z] = (obj.body(1)/5.).*[X Y Z];
            X = (obj.body(1)/5.).*X + obj.pos(1);
            Y = (obj.body(1)/5.).*Y + obj.pos(2);
            Z = (obj.body(1)/5.).*Z + obj.pos(3);
            s = surf(obj.axis,X,Y,Z);
            set(s,'edgecolor','none','facecolor',obj.colours(1,:));
            
            %create side spheres
            %front, right, back, left
            hOff = obj.body(3)/2;
            Lx = obj.body(1)/2;
            Ly = obj.body(2)/2;
            rotorsPosBody = [...
                0    Ly    0    -Ly;
                Lx    0    -Lx   0;
                hOff hOff hOff hOff];
            rotorsPosInertial = zeros(3,4);
            for i = 1:4
                rotorPosBody = rotorsPosBody(:,i);
                rotorsPosInertial(:,i) = bodyToInertial(obj,rotorPosBody);
                [X Y Z] = sphere(8);
                X = (obj.body(1)/8.).*X + obj.pos(1) + rotorsPosInertial(1,i);
                Y = (obj.body(1)/8.).*Y + obj.pos(2) + rotorsPosInertial(2,i);
                Z = (obj.body(1)/8.).*Z + obj.pos(3) + rotorsPosInertial(3,i);
                s = surf(obj.axis,X,Y,Z);
                set(s,'edgecolor','none','facecolor',obj.colours(i+1,:));
            end
            obj.axis.Title.String = ['Sim Time = ',num2str(obj.time,'%f'),' seconds'];
        end
        
        function vectorInertial = bodyToInertial(obj, vectorBody)
            vectorInertial = obj.R*vectorBody; %%%%%%%%%%%%%%%%%%%% USE SELF DEFINED ROTATION AS OBJ.R!!!!!!!!!!!!!!!!
        end
        

        function draw_plots(obj)
            
            % Check how to make the scale of x axis time
            % label my axis

            obj.plt_time = [obj.plt_time, obj.time];


            %% Plotting for Position
            obj.plt_x = [obj.plt_x, obj.pos(1)];
            plot(obj.pos_axis, obj.plt_time,obj.plt_x);
            %set(plt,'edgecolor','none','facecolor',obj.colours(1,:));
            %hold on;
            hold(obj.pos_axis,'on')
            
            obj.plt_y = [obj.plt_y, obj.pos(2)];
            plot(obj.pos_axis, obj.plt_time,obj.plt_y);
            %set(plt,'edgecolor','none','facecolor',obj.colours(1,:));
            
            obj.plt_z = [obj.plt_z, obj.pos(3)];
            plot(obj.pos_axis, obj.plt_time,obj.plt_z);
            %set(plt,'edgecolor','none','facecolor',obj.colours(1,:));

            xlim(obj.pos_axis,[0 obj.time]);
            legend(obj.pos_axis,{'x','y','z'});
            title(obj.pos_axis,'Position Tracking');
            xlabel(obj.pos_axis,"Time (s)");
            ylabel(obj.pos_axis,"Position");
            grid(obj.pos_axis,"minor");
            hold(obj.pos_axis,'off');


            %% Plotting for Orientation
            obj.plt_roll = [obj.plt_roll, obj.theta(1)];
            plot(obj.theta_axis, obj.plt_time,obj.plt_roll);
            %set(plt,'edgecolor','none','facecolor',obj.colours(1,:));
            hold(obj.theta_axis,'on')
            
            obj.plt_pitch = [obj.plt_pitch, obj.theta(2)];
            plot(obj.theta_axis, obj.plt_time,obj.plt_pitch);
            %set(plt,'edgecolor','none','facecolor',obj.colours(1,:));

            obj.plt_yaw = [obj.plt_yaw, obj.theta(3)];
            plot(obj.theta_axis, obj.plt_time,obj.plt_yaw);
            %set(plt,'edgecolor','none','facecolor',obj.colours(1,:));

            xlim(obj.theta_axis,[0 obj.time]);
            legend(obj.theta_axis,{'roll', 'pitch', 'yaw'});
            title(obj.theta_axis,'Orientation Tracking');
            xlabel(obj.theta_axis,"Time (s)");
            ylabel(obj.theta_axis,"Orientation");
            grid(obj.theta_axis,"minor");
            hold(obj.theta_axis,'off');

%             %% Plotting for Linear Speed
%             obj.plt_xdot = [obj.plt_xdot, obj.posdot(1)];
%             plot(obj.posdot_axis, obj.plt_time,obj.plt_xdot);
%             %set(plt,'edgecolor','none','facecolor',obj.colours(1,:));
%             hold(obj.posdot_axis,'on')
%             
%             obj.plt_ydot = [obj.plt_ydot, obj.posdot(2)];
%             plot(obj.posdot_axis, obj.plt_time,obj.plt_ydot);
%             %set(plt,'edgecolor','none','facecolor',obj.colours(1,:));
% 
%             obj.plt_zdot = [obj.plt_zdot, obj.posdot(3)];
%             plot(obj.posdot_axis, obj.plt_time,obj.plt_zdot);
%             %set(plt,'edgecolor','none','facecolor',obj.colours(1,:));
% 
%             xlim(obj.posdot_axis,[0 obj.time]);
%             legend(obj.posdot_axis,{'xdot', 'ydot', 'zdot'});
%             title(obj.posdot_axis,'Linear Speed Tracking');
%             xlabel(obj.posdot_axis,"Time (s)");
%             ylabel(obj.posdot_axis,"Linear Speed");
%             grid(obj.posdot_axis,"minor");
%             hold(obj.posdot_axis,'off');
               
        end


        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %SIMULATION FUNCTIONS
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        %code to show varying position and rotation

        function obj = change_pos_and_orientation(obj)
            

            % calculating sin and cos of quadcopter orientations for use in calculations.
            rol = obj.theta(1);
            pitc = obj.theta(2);
            ya = obj.theta(3);
            cos_phi = cos(rol);
            cos_si = cos(ya);
            cos_theta = cos(pitc);
            sin_phi = sin(rol);
            sin_si = sin(ya);
            sin_theta = sin(pitc);
            
            %converting the rate of change of quadcopter orientation to its
            %angular velocity
            obj.omega = thetadot2omega(obj.thetadot, obj.theta);
            
            %Getting the required inputs, used to control the quadcopter which are the square of quadcopter angular velocities.
            obj.i = calculate_inputs(cos_theta, cos_phi, obj.m, obj.g, obj.k, obj.time);

            % Compute linear and angular accelerations.
            a = acceleration(obj.i, obj.theta, obj.posdot, obj.m, obj.g, obj.k, obj.kd);
            omegadot = angular_acceleration(obj.i, obj.omega, obj.I, obj.L, obj.b, obj.k);

            %Updating Quadcopter states
            obj.omega = obj.omega + obj.dt * omegadot;
            obj.thetadot = omega2thetadot(obj.omega, obj.theta);
            obj.theta = obj.theta + obj.dt * obj.thetadot;
            obj.posdot = obj.posdot + obj.dt * a;
            obj.pos = obj.pos + obj.dt * obj.posdot;
            %obj.pos = [0;0;5];
            
            %Function adds noise to real readings to simulate reading
            %real quadcopter states through sensors.
            noisy_measurements = add_gauss_noise();
            obj.measured_pos = [noisy_measurements(1),noisy_measurements(2),noisy_measurements(3)];
            obj.measured_posdot = [noisy_measurements(4),noisy_measurements(5),noisy_measurements(6)];
            obj.measured_theta = [noisy_measurements(7),noisy_measurements(8),noisy_measurements(9)];
            obj.measured_thetadot = [noisy_measurements(10),noisy_measurements(11),noisy_measurements(12)];
            obj.measured_omega = thetadot2omega(transpose(obj.measured_thetadot), transpose(obj.measured_theta));

            obj.X = [obj.pos(1),obj.pos(2),obj.pos(3),obj.posdot(1),obj.posdot(2),obj.posdot(3),obj.theta(1),...
                obj.theta(2),obj.theta(3),obj.thetadot(1),obj.thetadot(2),obj.thetadot(3)];
            % Measured state values from sensors used to control the drone
            obj.measured_X = [obj.measured_pos(1),obj.measured_pos(2),obj.measured_pos(3),obj.measured_posdot(1),...
                obj.measured_posdot(2),obj.measured_posdot(3),obj.measured_theta(1),obj.measured_theta(2),...
                obj.measured_theta(3),obj.measured_omega(1),obj.measured_omega(2),obj.measured_omega(3)];

            
            %Function to convert angular velocity of the quadcopter to the
            %rateof change of its orientation in space.
            function thetadot = omega2thetadot(omega,theta)
                rol = theta(1);
                pitc = theta(2);
                ya = theta(3);
                cos_phi = cos(rol);
                cos_si = cos(ya);
                cos_theta = cos(pitc);
                sin_phi = sin(rol);
                sin_si = sin(ya);
                sin_theta = sin(pitc);

                matrix = [1 0 -sin_theta; 0 cos_phi cos_theta*sin_phi; 0 -sin_phi (cos_theta)*cos_phi];
                if (det(matrix) ~= 0)
                    thetadot = inv(matrix)*omega;
                end

            end

            %Function to convert from the rateof change of the quadcopter
            %orientation in space to its angular velocity vector.
            function omega = thetadot2omega(thetadot,theta)
                rol = theta(1);
                pitc = theta(2);
                ya = theta(3);
                cos_phi = cos(rol);
                cos_si = cos(ya);
                cos_theta = cos(pitc);
                sin_phi = sin(rol);
                sin_si = sin(ya);
                sin_theta = sin(pitc);

                matrix = [1 0 -sin_theta; 0 cos_phi cos_theta*sin_phi; 0 -sin_phi (cos_theta)*cos_phi];
                omega = matrix*thetadot;
            end
            
            % Compute thrust given current inputs and thrust coefficient.
            function T = thrust(inputs, k)
                % Inputs are values for wi
                
                T = [0; 0; k * sum(inputs)];
            end
            
            % Compute torques, given current inputs, length, drag coefficient, and thrust coefficient.
            function tau = torques(inputs, L, b, k)
                % Inputs are values for wi
                
                tau = [
                    L * k * (inputs(1) - inputs(3))
                    L * k * (inputs(2) - inputs(4))
                    b * (inputs(1) - inputs(2) + inputs(3) - inputs(4))
                ];
            end
            
            %Rotation matrix to convert from quadcopter's body frame to the
            %world inertial frame
            function R = rotation(theta)
                rol = theta(1);
                pitc = theta(2);
                ya = theta(3);
                cos_phi = cos(rol);
                cos_si = cos(ya);
                cos_theta = cos(pitc);
                sin_phi = sin(rol);
                sin_si = sin(ya);
                sin_theta = sin(pitc);

                Rx = [1 0 0;0 cos_phi -sin_phi;0 sin_phi cos_phi];
                Ry = [cos_theta 0 sin_theta;0 1 0;-sin_theta 0 cos_theta];
                Rz = [cos_si -sin_si 0;sin_si cos_si 0;0 0 1];

                R = Rz*Ry*Rx;

%                 R = [((cos_phi * cos_si)-(cos_theta*sin_si*sin_phi)) ((-cos_si * sin_phi)-(cos_phi*cos_theta*cos_si)) (sin_theta * sin_si);...
%                     ((cos_theta*cos_si*sin_phi)+(cos_phi*sin_si)) ((cos_phi*cos_theta*cos_si)-(sin_phi*sin_si)) (-cos_si*sin_theta);...
%                     (sin_phi*sin_theta) (cos_phi*sin_theta) (cos_theta)];
            end

            % Function to compute acceleration of the quadcopter
            function a = acceleration(inputs, angles, xdot, m, g, k, kd)
                gravity = [0; 0; -g];
                obj.R = rotation(angles);
                T = obj.R * thrust(inputs, k);
                Fd = -kd * xdot;
                a = gravity + 1 / m * T + Fd;
            end
            
            %Function to compute the angular acceleration of the quadcopter
            function omegadot = angular_acceleration(inputs, omega, I, L, b, k)
                tau = torques(inputs, L, b, k);
                omegadot = inv(I) * (tau - cross(omega, I * omega));
            end
            
            function noisy_measurements = add_gauss_noise()

                % The function normrnd(m,sigu) adds gaussian noise with mean
                % mu and standard diviation sigma.
                pos_noise_magnitude = 0.0005;
                gaussian_noise_pos = pos_noise_magnitude * normrnd(0,1); 
                x_sm = obj.pos(1) + gaussian_noise_pos;
                y_sm = obj.pos(2) + gaussian_noise_pos;
                z_sm = obj.pos(3) + gaussian_noise_pos;

                theta_noise_magnitude = 0.0001;
                gaussian_noise_theta = theta_noise_magnitude * normrnd(0,1);            
                roll_sm = obj.theta(1) + gaussian_noise_theta;
                pitch_sm = obj.theta(2) + gaussian_noise_theta;
                yaw_sm = obj.theta(3) + gaussian_noise_theta;

                xdot_sm = (x_sm - obj.measured_pos(1))/obj.dt;
                ydot_sm = (y_sm - obj.measured_pos(2))/obj.dt;
                zdot_sm = (z_sm - obj.measured_pos(3))/obj.dt;

                rolldot_sm = (roll_sm - obj.measured_theta(1))/obj.dt;
                pitchdot_sm = (pitch_sm - obj.measured_theta(2))/obj.dt;
                yawdot_sm = (yaw_sm - obj.measured_theta(3))/obj.dt;
                
                noisy_measurements = [x_sm,y_sm,z_sm, xdot_sm, ydot_sm,zdot_sm, roll_sm, pitch_sm, yaw_sm,...
                    rolldot_sm,pitchdot_sm,yawdot_sm];

            end

            %This function is used to calculate the change in input
            %required after each time step to send the quadcopter to the
            %set reference point
            function delta_u = feedback_controller(ref)

                %% Controlability Matrix
                
%                 Co = ctrb(obj.disc_sys.A,obj.disc_sys.B);
%                 
%                 if rank(Co,10e-10) == min(size(Co))
%                     'Controlability matrix is full rank. System is controllable'
%                 else
%                     'Controlability matrix is rank deficient. System is not controllable'
%                 end
                
                % %% Observability Matrix 
                % % We modify C, D because we only have a single measurement (d)
                % C = [1 0 0];
                % D = 0;
                % cont_sys = ss(A,B,C,D);
                % disc_sys = c2d(cont_sys,dt,'zoh');
                % 
                % Ob = obsv(disc_sys.A,disc_sys.C);
                % if rank(Ob,10e-10) == min(size(Ob))
                %     'Observability matrix is full rank. System is observable'
                % else
                %     'Observability matrix is rank deficient. System is not observable'
                % end

                %% Calculating system feedback
                
                % Here we calculate the error between the current position
                % and the reference:
                e = obj.measured_X-transpose(ref);
                
                % Here we replace the input to be a function of our state
                % and use system dynamics defined in Q1 to calculate the
                % updated state parameters.
                delta_u = -obj.K*transpose(e);

                %% Problems:
                %Our system is not controllable as found using the ctrb funtion above.
                %Calculate inputs for each motor? As only one input generated currently!
                %Is anything other than new input and feedback parameters required to be calculated for 3a?
                %Implement PID if we cannot make the system controllable  by the end?


            end

            %This function is used to check if the quadcopter is within
            %fixed tollerances of the reference point to ensure the desired final
            %state has been reached
            function result = within_tolerance(reference)
                dev = 0.05;
                % Making sure target is reached.
                at = obj.pos(1) >= reference(1)-dev && obj.pos(1) <= reference(1)+dev;
                bt = obj.pos(2) >= reference(2)-dev && obj.pos(2) <= reference(2)+dev;
                ct = obj.pos(3) >= reference(3)-dev && obj.pos(3) <= reference(3)+dev;
                % Making sure the linear velocities, angles and angular velocities are close to zero to ensure
                % quadcopter stays true to its linear approximation and can
                % be controlled on the same equations.
                dt = obj.posdot(1) >= reference(4)-dev && obj.posdot(1) <= reference(4)+dev;
                et = obj.posdot(2) >= reference(5)-dev && obj.posdot(2) <= reference(5)+dev;
                ft = obj.posdot(3) >= reference(6)-dev && obj.posdot(3) <= reference(6)+dev;
                gt = obj.theta(1) >= reference(7)-dev && obj.theta(1) <= reference(7)+dev;
                ht = obj.theta(2) >= reference(8)-dev && obj.theta(2) <= reference(8)+dev;
                it = obj.theta(3) >= reference(9)-dev && obj.theta(3) <= reference(9)+dev;
                jt = obj.omega(1) >= reference(10)-dev && obj.omega(1) <= reference(10)+dev;
                kt = obj.omega(2) >= reference(11)-dev && obj.omega(2) <= reference(11)+dev;
                lt = obj.omega(3) >= reference(12)-dev && obj.omega(3) <= reference(12)+dev;

                result = at*bt*ct*dt*et*ft;
            end

            %Function to decide the inputs to provide to the quadcopter.
            function inputs = calculate_inputs(cos_tetha, cos_phi, m, g, k, time)

                %This is the input provided to each motor at equillibrium and all
                %feedback from the fsf controller will be added to this
                %value.
                gamma = ((m*g) / (4*k*cos_tetha*cos_phi));
                %gamma = [0,0,0,0];

                if ~within_tolerance(obj.ref1) && (obj.ref1_reached == false) % Checking if the first reference has been reached and if not set that as the target
                    delta_u = feedback_controller(obj.ref1);
                    if (obj.pos(3)<0.5) && (obj.posdot(3)<-0.1) % Safety Measure to stop Quadcopter landing with a linear velocity of more than 0.1 and not hitting the ground on landing
                        delta_u = delta_u*5; 
                    end
                    inputs = [gamma+delta_u(1), gamma+delta_u(2), gamma+delta_u(3), gamma+delta_u(4)];
                
                elseif within_tolerance(obj.ref1) && (obj.ref1_reached == false) % Saving the sim time when the quadcopter just reaches the first target
                    obj.ref1_wait_time = obj.time;
                    obj.ref1_reached = true;
                    delta_u = feedback_controller(obj.ref1);
                    if (obj.pos(3)<0.5) && (obj.posdot(3)<-0.1) % Safety Measure to stop Quadcopter landing with a linear velocity of more than 0.1 and not hitting the ground on landing
                        delta_u = delta_u*5; 
                    end
                    inputs = [gamma+delta_u(1), gamma+delta_u(2), gamma+delta_u(3), gamma+delta_u(4)];
                
                elseif (obj.time<=(obj.ref1_wait_time+10)) && ~within_tolerance(obj.ref2) %Waiting for 10 seconds since the recorded time at ref 1
                    delta_u = feedback_controller(obj.ref1);
                    if (obj.pos(3)<0.5) && (obj.posdot(3)<-0.1) % Safety Measure to stop Quadcopter landing with a linear velocity of more than 0.1 and not hitting the ground on landing
                        delta_u = delta_u*5; 
                    end
                    inputs = [gamma+delta_u(1), gamma+delta_u(2), gamma+delta_u(3), gamma+delta_u(4)];
                
                elseif ~within_tolerance(obj.ref2) && (obj.ref1_reached == true) && (obj.ref2_reached == false)  % Checking if the second reference has been reached and if not set that as the target
                    delta_u = feedback_controller(obj.ref2);
                    if (obj.pos(3)<0.5) && (obj.posdot(3)<-0.1) % Safety Measure to stop Quadcopter landing with a linear velocity of more than 0.1 and not hitting the ground on landing
                        delta_u = delta_u*5; 
                    end
                    inputs = [gamma+delta_u(1), gamma+delta_u(2), gamma+delta_u(3), gamma+delta_u(4)];
                
                elseif ~within_tolerance(obj.ref3) && (obj.ref1_reached == true) && (obj.ref3_reached == false)  % Checking if the third reference has been reached and if not set that as the target
                    obj.ref2_reached = true;
                    delta_u = feedback_controller(obj.ref3);
                    if (obj.pos(3)<0.5) && (obj.posdot(3)<-0.1) % Safety Measure to stop Quadcopter landing with a linear velocity of more than 0.1 and not hitting the ground on landing
                        delta_u = delta_u*5; 
                    end
                    inputs = [gamma+delta_u(1), gamma+delta_u(2), gamma+delta_u(3), gamma+delta_u(4)];

                elseif ~within_tolerance(obj.ref4) && (obj.ref2_reached == true) && (obj.ref4_reached == false)  % Checking if the fourth reference has been reached and if not set that as the target
                    obj.ref3_reached = true;
                    delta_u = feedback_controller(obj.ref4);
                    if (obj.pos(3)<0.5) && (obj.posdot(3)<-0.1) % Safety Measure to stop Quadcopter landing with a linear velocity of more than 0.1 and not hitting the ground on landing
                        delta_u = delta_u*5; 
                    end
                    inputs = [gamma+delta_u(1), gamma+delta_u(2), gamma+delta_u(3), gamma+delta_u(4)];
                    
                elseif ~within_tolerance(obj.ref5) && (obj.ref3_reached == true) && (obj.ref5_reached == false)  % Checking if the fifth reference has been reached and if not set that as the target
                    obj.ref4_reached = true;
                    delta_u = feedback_controller(obj.ref5);
                    if (obj.pos(3)<0.5) && (obj.posdot(3)<-0.1) % Safety Measure to stop Quadcopter landing with a linear velocity of more than 0.1 and not hitting the ground on landing
                        delta_u = delta_u*5; 
                    end
                    inputs = [gamma+delta_u(1), gamma+delta_u(2), gamma+delta_u(3), gamma+delta_u(4)];

                elseif ~within_tolerance(obj.ref6) && (obj.ref4_reached == true) && (obj.ref6_reached == false)  % Checking if the sixth reference has been reached and if not set that as the target
                    obj.ref5_reached = true;
                    delta_u = feedback_controller(obj.ref6);
                    if (obj.pos(3)<0.5) && (obj.posdot(3)<-0.1) % Safety Measure to stop Quadcopter landing with a linear velocity of more than 0.1 and not hitting the ground on landing
                        delta_u = delta_u*5; 
                    end
                    inputs = [gamma+delta_u(1), gamma+delta_u(2), gamma+delta_u(3), gamma+delta_u(4)];
                else
                    obj.ref6_reached = true;
                    %inputs = [gamma, gamma, gamma, gamma];


                end
                %disp(inputs);


            end


        end
        
        
        function update(obj)
            %update simulation time
            obj.time = obj.time + obj.dt;
            
            %change position and orientation of drone
            obj = change_pos_and_orientation(obj);
            
            %draw drone on figure
            draw(obj);

            %create plots
            draw_plots(obj);
        end
    end
end
