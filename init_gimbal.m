% DC Motor Parameters
Kt = 0.05;      % Motor torque constant (N*m/A)
L  = 0.002;     % Armature inductance (H)
R  = 1.5;       % Armature resistance (Ohms)

% Gimbal Mechanical Parameters
J  = 0.01;      % Moment of inertia (kg*m^2)
B  = 0.02;      % Viscous damping coefficient (N*m*s/rad)

% PID Controller Gains (starting values for tuning)
% Kp = 15;
% Ki = 5;
% Kd = 1.2;

G = tf(Kt, conv([L R], [J B 0]));

figure
rlocus(G)
grid on

disp('Open-loop poles:')
disp(pole(G))

Mp = 10; %max overshoot allowed (selected by us)

zeta_min = sqrt(log(100/Mp)^2 / (pi^2 + log(100/Mp)^2)); %assuming as 2nd order since third pole (-750) lies far. dies out quick.

% setting characteristic equation equal to zero and solving we get s^2 + (B/J)s + KpKt/RJ
Kp_max = R*B^2 / (4*J*Kt*zeta_min^2);

fprintf('Minimum damping ratio: %.4f\n', zeta_min);
fprintf('Approximate gain range: 0 < Kp < %.4f\n', Kp_max);

% Test a gain within the range using the full third-order plant
Kp_test = 0.8;
T = feedback(Kp_test*G, 1);

disp('Closed-loop poles:')
disp(pole(T))

info = stepinfo(T);
fprintf('Full-model overshoot: %.2f%%\n', info.Overshoot);
fprintf('Settling time: %.2f s\n', info.SettlingTime);

figure
step(T)
grid on

motor = tf(Kt, [L R]);
gimbal = tf(1, [J B 0]);

% now, if we add integral control to improve steady state response, we get
% the characteristic equation as: Js^3 + Bs^2 + (Kt*Kp / R)s + (Kt*Ki / R) = 0

t = 0:0.01:60;  % defining of time points to check system response at
Td = 0.1;
Kp = Kp_test;                   % Use the P gain selected above
Ki_values = [0 0.05 0.1 0.2 0.4 0.6 0.8 1.0];

figure
hold on

for Ki = Ki_values
    C = tf([Kp Ki], [1 0]);

    T_dist = gimbal / (1 + C*motor*gimbal);

    step(Td*T_dist, t)
end

title('Disturbance response for different Ki values')
xlabel('Time (s)')
ylabel('Angle deviation (rad)')
legend('P only', 'Ki = 0.05', 'Ki = 0.1','Ki = 0.2', 'Ki = 0.4', 'Ki = 0.6', 'Ki = 0.8', 'Ki = 1.0')
hold off