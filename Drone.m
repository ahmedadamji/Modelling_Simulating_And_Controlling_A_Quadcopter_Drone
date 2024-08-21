%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%  Code modified by Francisco Vasconcelos and Ahmed Adamjee from
%%%%
%%%%  Drone class, feel free to add functionality as you see fit
%%%%  Author: Daniel Butters
%%%%  Date: 16/11/17
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
classdef Drone < handle
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

%         t = obj.time;
%             
            
        dt = 0.1;

        % Number of points in the simulation.
        m = 0.2;
        g = 9.2;
        kd = 0.1;
        k = 1;
        I = [1 0 0;0 1 0;0 0 0.5];
        L = 0.2;
        b = 0.1;
        % Initial simulation state.
%         x = [0; 0; 10];
%         xdot = zeros(3, 1);
%         theta = zeros(3, 1);
        
        % Simulate some disturbance in the angular velocity.
        % The magnitude of the deviation is in radians / second.
%         deviation = 100;
%         thetadot = deg2rad(2 * deviation * rand(3, 1) - deviation);
    end
    properties
        %axis to draw on
        axis
        axis2
        axis3
        
        %length of one side of the flight arena
        spaceDim
        
        %limits of flight arena
        spaceLimits
        
        %drone position
        pos
        posdot 

        %drone orientation
        theta
        thetadot


        %drone rotation matrix
        R


        i
        
        %Simulation time
        time
        
        %parameter to start drone in random position
        pos_offset
        
        %number of drones
        num_drones

        %arrays for graph
        plt_x
        plt_y
        plt_z
        plt_roll
        plt_pitch
        plt_yaw
    end
    methods
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %INSTANTIATION OF CLASS
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function obj = Drone(axis1, axis2, axis3, spaceDim, num_drones)
            if nargin > 1
                obj.axis = axis1;

                obj.axis2 = axis2;

                obj.axis3 = axis3;
                
                obj.spaceDim = spaceDim;
                
                obj.spaceLimits = [(-spaceDim/2)+10 (spaceDim/2)-10 (-spaceDim/2)+10 (spaceDim/2)-10 10 spaceDim-10];
                
                obj.pos = [0;0;5];

                obj.posdot = zeros(3, 1);

                obj.i = [1,1,1,1]*0.5;
                
                obj.pos_offset = [5.*(rand - 0.5),5.*(rand - 0.5),2.5.*(rand)];

                obj.theta = zeros(3, 1);

                obj.thetadot = zeros(3, 1);
                
                obj.R = [1,0,0;0,1,0;0,0,1];
                
                obj.time = 0;
                
                obj.num_drones = num_drones;

                obj.plt_x = [];
                obj.plt_y = [];
                obj.plt_z = [];
                obj.plt_roll = [];
                obj.plt_pitch = [];
                obj.plt_yaw = [];
                Q3(obj);

                
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
            vectorInertial = obj.R*vectorBody;
        end
        

        function draw_plots(obj)
            
            % Check how to make the scale of x axis time
            % label my axis
            % Create different graph to show orientation?
            % Fix problem of replacing previous graph



            %x_axis = linspace(0,obj.time);
            %disp(x_axis)


            obj.plt_x = [obj.plt_x, obj.pos(1)];
            plt = plot(obj.axis2,obj.plt_x);
            %set(plt,'edgecolor','none','facecolor',obj.colours(1,:));
            %hold on;
            hold(obj.axis2,'on')
            
            obj.plt_y = [obj.plt_y, obj.pos(2)];
            plt = plot(obj.axis2,obj.plt_y);
            %set(plt,'edgecolor','none','facecolor',obj.colours(1,:));
            
            obj.plt_z = [obj.plt_z, obj.pos(3)];
            plt = plot(obj.axis2,obj.plt_z);
            %set(plt,'edgecolor','none','facecolor',obj.colours(1,:));
            legend(obj.axis2,{'x','y','z'})
            hold(obj.axis2,'off')

            obj.axis2.Title.String = ['Position Tracking'];



            obj.plt_roll = [obj.plt_roll, obj.theta(1)];
            plt = plot(obj.axis3,obj.plt_roll);
            %set(plt,'edgecolor','none','facecolor',obj.colours(1,:));
            hold(obj.axis3,'on')
            
            obj.plt_pitch = [obj.plt_pitch, obj.theta(2)];
            plt = plot(obj.axis3,obj.plt_pitch);
            %set(plt,'edgecolor','none','facecolor',obj.colours(1,:));

            obj.plt_yaw = [obj.plt_yaw, obj.theta(3)];
            plt = plot(obj.axis3,obj.plt_yaw);
            %set(plt,'edgecolor','none','facecolor',obj.colours(1,:));

            %xlim([0 obj.time]);
            legend(obj.axis3,{'roll', 'pitch', 'yaw'})
            hold(obj.axis3,'off')
            obj.axis3.Title.String = ['Orientation Tracking'];
               
        end


        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %SIMULATION FUNCTIONS
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        %code to show varying position and rotation

        function obj = change_pos_and_orientation(obj)
            
            times = 0:obj.dt:15;
            N = numel(times);
            % Step through the simulation, updating the state.

            % Take input from our controller.
            rol = obj.theta(1);
            pitc = obj.theta(2);
            ya = obj.theta(3);
            cos_phi = cos(rol);
            cos_si = cos(ya);
            cos_theta = cos(pitc);
            sin_phi = sin(rol);
            sin_si = sin(ya);
            sin_theta = sin(pitc);
            
            omega = thetadot2omega(obj.thetadot, obj.theta);
            
            % Compute linear and angular accelerations.
            obj.i = calculate_inputs(cos_theta, cos_phi, obj.m, obj.g, obj.k, obj.time);
            a = acceleration(obj.i, obj.theta, obj.posdot, obj.m, obj.g, obj.k, obj.kd);
            omegadot = angular_acceleration(obj.i, omega, obj.I, obj.L, obj.b, obj.k);
            
            omega = omega + obj.dt * omegadot;
            obj.thetadot = omega2thetadot(omega, obj.theta);
            obj.theta = obj.theta + obj.dt * obj.thetadot;
            obj.posdot = obj.posdot + obj.dt * a;
            obj.pos = obj.pos + obj.dt * obj.posdot;
            %obj.pos = [0;0;5];

            
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

            function a = acceleration(inputs, angles, xdot, m, g, k, kd)
                gravity = [0; 0; -g];
                obj.R = rotation(angles);
                T = obj.R * thrust(inputs, k);
                Fd = -kd * xdot;
                a = gravity + 1 / m * T + Fd;
            end
            
            function omegadot = angular_acceleration(inputs, omega, I, L, b, k)
                tau = torques(inputs, L, b, k);
                omegadot = inv(I) * (tau - cross(omega, I * omega));
            end
            
% 
%             % Compute system inputs and updated state.
%             % Note that input = [g1, . . ., g4]
%             function [input, state] = pd_controller(state, thetadot)
%                 % Controller gains, tuned by hand and intuition.
%                 Kd = 4;
%                 Kp = 3;
%                 
%                 % Initialize the integral if necessary.
%                 if ˜isfield(state, ’integral’)
%                     params.integral = zeros(3, 1);
%                 end
%                 
%                 % Compute total thrust
%                 total = state.m * state.g / state.k / (cos(state.integral(1)) * cos(state.integral(15
%                 
%                 % Compute errors
%                 e = Kd * thetadot + Kp * params.integral;
%                 
%                 % Solve for the inputs, gi
%                 input = error2inputs(params, accels, total);
%                 
%                 % Update the state
%                 params.integral = params.integral + params.dt .* thetadot;
%             end
% 
% 
% 
%             % Compute system inputs and updated state.
%             % Note that input = [g1, . . ., g4]
%             function [input, state] = pid_controller(state, thetadot)
%                 % Controller gains, tuned by hand and intuition.
%                 Kd = 4;
%                 Kp = 3;
%                 Ki = 5.5;
%                 
%                 % Initialize the integral if necessary.
%                 if ˜isfield(state, ’integral’)
%                     params.integral = zeros(3, 1);
%                     params.integral2 = zeros(3, 1);
%                 end
%                 
%                 % Prevent wind-up
%                 if max(abs(params.integral2)) > 0.01
%                     params.integral2(:) = 0;
%                 end
%                 
%                 % Compute total thrust
%                 total = state.m * state.g / state.k / (cos(state.integral(1)) * cos(state.integral(22
%                 % Compute errors
%                 
%                 e = Kd * thetadot + Kp * params.integral - Ki * params.integral2;
%                 
%                 % Solve for the inputs, gi
%                 input = error2inputs(params, accels, total);
%                 
%                 % Update the state
%                 params.integral = params.integral + params.dt .* thetadot;
%                 params.integral2 = params.integral2 + params.dt .* params.integral;
%                 % We can use this function to approximate a derivative with respect to a gain:
%                 % Compute derivative with respect to first parameter.
%                 delta = 0.01;
%                 var = [1, 0, 0];
%                 params.derivative = (cost(theta + delta * var) - cost(theta - delta * var)) / (2 * delta);   
%             end
% 
% 
% 
%             function J = cost(theta)
%                 % Create a controller using the given gains.
%                 control = controller(’pid’, theta(1), theta(2), theta(3));
%                 
%                 % Perform a simulation.
%                 data = simulate(control);
%                 
%                 % Compute the integral of e(t)^2 
%                 t0 = 0;
%                 tf = 1;
%                 J = 1/(tf - t0) * sum(data.theta(data.t >= t0 & data.t <= tf) .ˆ 2) * data.dt;
%             end
% 
%                    

            function inputs = calculate_inputs(cos_tetha, cos_phi, m, g, k, time)

                if (0 < obj.time) && (obj.time < 2)
                    gamma = (m*g) / (4*k*cos_tetha*cos_phi);
                    inputs = [gamma, gamma, gamma, gamma];

                elseif (2 < obj.time) && (obj.time < 4)
                    gamma = ((m*g) / (4*k*cos_tetha*cos_phi))*1.2;
                    inputs = [gamma, gamma, gamma, gamma];
                    

                elseif (4 < obj.time) && (obj.time < 8)
                    gamma = ((m*g) / (4*k*cos_tetha*cos_phi))*1.2;
                    inputs = [gamma, gamma, gamma, 0];

                elseif obj.pos(3) <= 0
                    obj.pos(3) = 0;
                    obj.posdot(3) = 0;

                end
%                 %Inputs are values for wi
%                 while time < 20
%                     
%                 if (0 < obj.time) && (obj.time < 20)   
%                     gamma = (m*g) / (4*k*cos_tetha*cos_phi);
%                     inputs = [gamma, gamma, gamma, gamma];
% 
%                     if obj.time > 2
%                         gamma = ((m*g) / (4*k*cos_tetha*cos_phi));
%                         inputs = [gamma, gamma, gamma, gamma]*1.2;
%                         if obj.time > 4 
%                             gamma = (m*g) / (4*k*cos_tetha*cos_phi);
%                             inputs = [gamma, gamma, gamma, 0];
%                         end
%                     end     

              


                    
%                    
%                 else

            end



            
            %vary orientation
            pitch = obj.theta(2);
            roll = obj.theta(1);
            yaw = obj.theta(3);
            obj.R = eul2rotm([yaw, roll, pitch]);



            
            
            %vary position
%             pos_mat = [2.6*sin(t/2), 2.1*cos(t/5),(2.4*sin(t/3)*sin(t/3))] + obj.pos_offset;
%             
%             %vary orientation
%             pitch = 0.3*sin(t*15.2);
%             roll = 0.1*cos(t*33.1 + 0.5);
%             yaw = 2.*pi*sin(t);
%             rot_mat = eul2rotm([yaw, roll, pitch]);
            %update position and rotation matrix of drone
%             obj.R = eul2rotm(tetha.');
%             obj.posdot = obj.posdot + obj.dt*a;
%             obj.pos = obj.pos + obj.dt * obj.posdot;
       
            
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
