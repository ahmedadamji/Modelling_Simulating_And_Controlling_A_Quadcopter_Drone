function [disc_sys,U,input,x_0,u_0] = dlinearise(dt)
%initialise drone constants
    m = 0.2;
    g = 9.2;
    kd = 0.1;
    k = 1;
    I = [1 0 0;0 1 0;0 0 0.5];
    Ixx = 1;
    Iyy = 1;
    Izz = 0.5;
    L = 0.2;
    b = 0.1;
    i = [1,1,1,1]*0.5;

    %create symbolic variables for all parameters in X and Xdot
    input = sym('gamma',[4 1]);
    x = sym('x');
    y = sym('y');
    z= sym('z');
    xdot = sym('xdot');
    ydot = sym('ydot');
    zdot= sym('zdot');
    th = sym('th');
    phi = sym('phi');
    si = sym('si');
    thdot = sym('thdot');
    phidot = sym('phidot');
    sidot = sym('sidot');
    wx = sym('wx');
    wy = sym('wy');
    wz = sym('wz');

    %compute systen dynamics
    ang = [phi;th;si];
    position = [xdot;ydot;zdot];
    omega = [wx;wy;wz];
    gamma_values = (m*g)/(4*k);
    x1dot = [xdot;ydot;zdot];
    x2dot = acceleration(input,ang,position,m, g, k, kd);
    x3dot = omega2thetadot(omega,ang);
    x4dot = angular_acceleration(input, omega, I, L, b, k);
    
    Xdot = [x1dot;x2dot;x3dot;x4dot];

    X = [x;y;z;xdot;ydot;zdot;phi;th;si;omega];
    U = input;
    
    %Derive jacobian matrices
    Bj  = jacobian(Xdot,U);
    Aj = jacobian(Xdot,X);
    
    %operating points
    x_0 = transpose([0,0,5,0,0,0,0,0,0,0,0,0]); 
    u_0 = [gamma_values;gamma_values;gamma_values;gamma_values];
    
    %compputing A,B,C,D matrices
    B = double(subs(Bj,[x,y,z,xdot,ydot,zdot,phi,th,si,wx,wy,wz,input(1,1),input(2,1),input(3,1),input(4,1)],[0,0,5,0,0,0,0,0,0,0,0,0,gamma_values,gamma_values,gamma_values,gamma_values]));
    A = subs(Aj,[x,y,z,xdot,ydot,zdot,phi,th,si,wx,wy,wz,input(1,1),input(2,1),input(3,1),input(4,1)],[0,0,5,0,0,0,0,0,0,0,0,0,gamma_values,gamma_values,gamma_values,gamma_values]);
    A = double(A);
    C = eye(12);
    D = 0 * ones(12,4);
    
    cont_sys = ss(A,B,C,D); %Derive conitinous system from A,B,C,D matrices
    disc_sys = c2d(cont_sys,dt,'zoh'); %discretise continous system
    
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
    
        end
    
        function a = acceleration(inputs, angles, xdot, m, g, k, kd)
            gravity = [0; 0; -g];
            R = rotation(angles);
            T = R * thrust(inputs, k);
            Fd = -kd * xdot;
            a = gravity + 1 / m * T + Fd/m;
        end
    
        function omegadot = angular_acceleration(inputs, omega, I, L, b, k)
            tau = torques(inputs, L, b, k);
            omegadot = inv(I) * (tau - cross(omega, I * omega));
        end
end