function [q, q_dot] = double_supportALPHA (T, Ts, q0_right, q0_left, delta_right, delta_left, h)
global xd_right xd_dot_right
global Rd_right wd_right
% inverse kinematics settings
Kp = 1e-3*eye(3);
Ko = 2e-3*eye(3);


q0 = [q0_right; q0_left];
iterations = round_to_Ts(T/Ts,Ts);
q = zeros(12,iterations);
q_dot = zeros(12,iterations);
q(:,1) = q0;

%***********************
% supporting right leg *
%***********************

SupportFoot='Right';

delta_x_right = delta_right(1);
delta_y_right = delta_right(2);
delta_z_right = delta_right(3);

% initial conditions
pose0_right = h.RF_T_w (q0);              % Initial position of the right supporting leg
p0_right = pose0_right(1:3);

% determine the desired trajectory for the supporting right leg 
[xd_right, xd_dot_right] = trajectory_double_soporte (T, Ts, p0_right, delta_x_right, delta_y_right, delta_z_right);

% determine the desired orientations for the supporting right leg
Rd0_right = pose0_right(4:7); %DOMINGO --> TIENE LA MISMA ORIENTACION
[Rd_right, wd_right] = desired_orientations (T, Ts, Rd0_right);

% inverse kinematics


[q, q_dot, p_right, R_right, error_p, error_o] = inverse_kinematics_rightALPHA (Ts, T, q, q_dot, Kp, Ko, xd_right, xd_dot_right, Rd_right, wd_right, h, SupportFoot);



%********************
% floating left leg *
%********************
delta_x_left = delta_left(1);
delta_y_left = delta_left(2);
delta_z_left = delta_left(3);

% initial conditions
pose0_left = h.w_T_LF (q0);              % Initial position of the left floating leg
p0_left = pose0_left(1:3);

% determine the desired trajectory for the floating left leg
global xd_left
[xd_left, xd_dot_left] = trajectory_double_soporte (T, Ts, p0_left, delta_x_left, delta_y_left, delta_z_left);

% determine the desired orientations for the supporting right leg
Rd0_left = pose0_left(4:7); %DOMINGO --> TIENE LA MISMA ORIENTACION
[Rd_left, wd_left] = desired_orientations (T, Ts, Rd0_left);

% inverse kinematics
[q, q_dot, p_left, R_left, error_p, error_o] = inverse_kinematics_leftALPHA (Ts, T, q, q_dot, Kp, Ko, xd_left, xd_dot_left, Rd_left, wd_left, h, SupportFoot);

% q_left = Rl_m2r*q_left_m;
% q_dot_left = Rl_m2r*q_dot_left_m;

%%%%%%%%%%%%%
% FUNCTIONS %
%%%%%%%%%%%%%

%**********************************
% trajectory double support phase *
%**********************************
function [xd, xd_dot] = trajectory_double_soporte (T, Ts, p0, delta_x, delta_y, delta_z)
time = 0:Ts:T-Ts;

% sagital plane
% x axis
xd(1,:) = interpolation (T, time, p0(1), p0(1)+delta_x, 0, 0);

% frontal plane
% y axis
xd(2,:) = interpolation (T, time, p0(2), p0(2)+delta_y, 0, 0);

% z axis
xd(3,:) = interpolation (T, time, p0(3), p0(3)+delta_z, 0, 0);

xd_dot = [zeros(3,1),diff(xd,1,2)/Ts];

function[Rd, wd] = desired_orientations (T, Ts, Rd0)
iterations = T/Ts;
Rd = zeros(4, iterations);
for ii=1:iterations
    Rd(:,ii) = Rd0;
end
wd = zeros(3,iterations);