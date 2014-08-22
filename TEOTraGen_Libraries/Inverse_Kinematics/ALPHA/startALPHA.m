% inverse kinematics
clc
% close all
% clear all
global  q_ds q_dot_ds
h = TEO_kinematics_library;

% initial conditions of the robot
q0_right = [0; 0.0033; 0.3086; 0.7964; 0.4849; 0.0279];         %initial position of the links in the right supporting leg
q0_left = [0; 0.0033; 0.3086; 0.7964; 0.4849; 0.0279];          %initial position of the links in the left floating leg


%******************
% GAIT GENERATION *
%******************

% gait generation parameters
% time conditions
Ts = 1e-3;          % Sampling time;
Tss = 3.5;          % Time for the step in single support
Tds = 0.8*Tss;      % Time for the double support
T = 0;

% Step conditions
L = 0.05;           %Step length

%************************************
% double support phase on right leg *
%************************************
T = T+Tds;        %total time of simulation

alfa = 0;           %Desired orientation in degrees

% desired position conditions for the right supporting leg
delta_right(1) = 0;                 % Waist displacement in x (frontal plane lenght, i.e. direction of the step)
delta_right(2) = 0.01;              % Waist displacement in y (sagital plane)
delta_right(3) = 0;                 % Waist displacement in z (frontal plane height)-> determine how much I elevate the waist


% desired position conditions for the left floating leg
delta_left(1) = 0;                  % Waist displacement in x (frontal plane lenght, i.e. direction of the step)
delta_left(2) = -0.01;              % Waist displacement in y (sagital plane)
delta_left(3) = 0;                  % Waist displacement in z (frontal plane height)-> determine how much I elevate the waist

% trajectory generation
[q_ds, q_dot_ds] = double_supportALPHA (Tds, Ts, q0_right, q0_left, delta_right, delta_left, h);

% q_right = q_right_ds;
% q_left = q_left_ds;
% q_dot_right = q_dot_right_ds;
% q_dot_left = q_dot_left_ds;
% % 
% % %************************************
% % % single support phase on right leg *
% % %************************************
% % T = T+Tss;        %total time of simulation
% % [a, dur] = size (q_right);
% % q0_right_ss = q_right(:,dur);
% % q0_left_ss = q_left(:,dur);
% % % desired position conditions for the right supporting leg
% % delta_right(1) = 0.01;          %waist displacement in x (sagital plane)
% % delta_right(2) = 0;            %waist displacement in y (frontal plane height)-> determine how much I elevate the waist
% % delta_right(3) = 0.01;            %waist displacement in z (frontal plane lenght, i.e. direction of the step)
% % 
% % % desired position conditions for the left floating leg
% % delta_left(1) = 0;               %leg displacement in x (sagital plane)
% % delta_left(2) = -L;              %leg displacement in y (frontal plane lenght, i.e. opposite direction of the step)
% % delta_left(3) = 0.04;            %leg displacement in z (frontal plane height)-> determine how much I elevate the foot
% % 
% % % desired orientation for the right supporting leg and for the left floating leg
% % alfa = 0;                        %desired orientation in degrees
% % [Rd0_right, Rd0_left] = determine_matrixes_rotation (alfa*pi/180);
% % 
% % 
% % [q_right_ss, q_dot_right_ss, p_right_ss, R_right_ss, q_left_ss, q_dot_left_ss, p_left_ss, R_left_ss] = right_leg_support (Tss, Ts, q0_right_ss, q0_left_ss, delta_right, Rd0_right, delta_left, Rd0_left);
% % 
% % q_right = [q_right, q_right_ss];
% % q_left = [q_left, q_left_ss];
% % q_dot_right = [q_dot_right, q_dot_right_ss];
% % q_dot_left = [q_dot_left,q_dot_left_ss];
% % 
% % 
% % 
% % %********************************
% % % Back to initial configuration *
% % %********************************
% % T = T+Tds;        %total time of simulation
% % [a, dur] = size (q_right);
% % q0_right_ds = q_right(:,dur);
% % q0_left_ds = q_left(:,dur);
% % 
% % [q_right_ds, q_dot_right_ds, q_left_ds, q_dot_left_ds] = interpolate_to_configuration (Tds, Ts, q0_right_ds, q0_left_ds, q0_right, q0_left);
% % 
% % q_right = [q_right, q_right_ds];
% % q_left = [q_left, q_left_ds];
% % q_dot_right = [q_dot_right, q_dot_right_ds];
% % q_dot_left = [q_dot_left,q_dot_left_ds];
% % 
% % 
% % 
% % R_right =   [   0, 1, 0, 0, 0, 0;
% %                 1, 0, 0, 0, 0, 0;
% %                 0, 0,-1, 0, 0, 0;
% %                 0, 0, 0, 1, 0, 0;
% %                 0, 0, 0, 0, 1, 0;
% %                 0, 0, 0, 0, 0,-1];
% % 
% % R_left =    [   1, 0, 0, 0, 0, 0;
% %                 0, 1, 0, 0, 0, 0;
% %                 0, 0, 1, 0, 0, 0;
% %                 0, 0, 0,-1, 0, 0;
% %                 0, 0, 0, 0, 0,-1;
% %                 0, 0, 0, 0,-1, 0];
% %             
% % q_right_1=R_right'*q_right;
% % q_left_1=R_left'*q_left;
% % 
% % show_q (Ts, T, q_right, q_left, q_dot_right, q_dot_left);