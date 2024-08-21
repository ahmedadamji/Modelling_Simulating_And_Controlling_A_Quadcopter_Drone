%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%  Code modified by Francisco Vasconcelos from
%%%%
%%%%  Drone class, feel free to add functionality as you see fit
%%%%  Author: Daniel Butters
%%%%  Date: 16/11/17
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
classdef Drone_q1 < handle
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

        % Defining constants specified
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
        %drone linear velocity
        posdot 
        %angular velocity
        omega
        %drone orientation
        theta
        %drone rate of change of orientation
        thetadot

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

    end
    methods
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %INSTANTIATION OF CLASS
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function obj = Drone_q1(axis1, axis2, axis3, spaceDim, num_drones)
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
                obj.pos = [0;0;5];
                
                %initializing array containing values of linear velocity for the quadcopter
                obj.posdot = zeros(3, 1);
                
                %initializing array containing values of inputs for the quadcopter (gamma)
                obj.i = [1,1,1,1]*0.5;
                
                obj.pos_offset = [5.*(rand - 0.5),5.*(rand - 0.5),2.5.*(rand)];

                %thetadot
                obj.thetadot = zeros(3, 1);

                %angular velocity
                obj.omega = zeros(3, 1);
                
                %initializing array containing values of angle about their axis for the quadcopter
                obj.theta = zeros(3, 1);
                
                %initializing array for the rotation matrix to change from quadcopter body frame to the inertial frame
                obj.R = [1,0,0;0,1,0;0,0,1];
                
                % Saves current simulation time
                obj.time = 0;
                
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
            if(obj.drone_follow)
               axis([obj.pos(1)-cL obj.pos(1)+cL obj.pos(2)-cL obj.pos(2)+cL obj.pos(3)-cL obj.pos(3)+cL]);
            end
            
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
            vectorInertial = obj.R*vectorBody;
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

            % The following plot can be uncommented if needed -->

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
        
        %code to show varying position and orientation
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
            
            %Function to decide the inputs to provide to the quadcopter.
            function inputs = calculate_inputs(cos_tetha, cos_phi, m, g, k, time)

                gamma = (m*g) / (4*k*cos_tetha*cos_phi);

                if (0 < obj.time) && (obj.time < 2)
                    inputs = [gamma, gamma, gamma, gamma];

                elseif (2 < obj.time) && (obj.time < 4)
                    inputs = [gamma, gamma, gamma, gamma]*1.2;
                    

                elseif (4 < obj.time) && (obj.time < 8)
                    inputs = [gamma, gamma, gamma, 0]*1.2;


                end
              
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
