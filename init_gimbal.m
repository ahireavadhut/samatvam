% DC Motor Parameters
Kt = 0.05;      % Motor torque constant (N*m/A)
L  = 0.002;     % Armature inductance (H)
R  = 1.5;       % Armature resistance (Ohms)

% Gimbal Mechanical Parameters
J  = 0.01;      % Moment of inertia (kg*m^2)
B  = 0.02;      % Viscous damping coefficient (N*m*s/rad)

% PID Controller Gains (starting values for tuning)
Kp = 15;
Ki = 5;
Kd = 1.2;