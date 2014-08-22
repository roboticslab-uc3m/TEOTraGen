function [q, dq, ddq] = ik_full_body_v2(q0, d_traj, h, parameters)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%  Full Body - Inverse Kinematics Algorith - Unit Quaternion %%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% Author: Domingo Esteban

%%% INPUT DATA %%%
% q0: Configuraci�n articular en el tiempo inicial
% traj: operational poses trajectory (with RPY rates)
% d_traj: operational velocity trajectory (with RPY rates)
% dd_traj: operational acceleration trajectory (with RPY rates)
% h: Structure with humanoid kinematics library

%%% OUTPUT DATA %%%
% q: trajectory in joints space
% dq: velocity trajectory in joints space
% ddq: acceleration in joints space


num_elements = size(d_traj.time,2);

% error = zeros(6*5, num_elements);
q = zeros(size(q0,1), num_elements);
dq = zeros(size(q0,1), num_elements);

% Parameters
Ts = d_traj.Ts; % Time step

% IK Gains
% Kp = parameters.kp;
Kp=30;%5010,  0.2
% Ko = parameters.ko;
Ko=1;

% Location of the body parts in Full Body joints structure
rl_local = 1:6; % Location of Right Leg joints
ll_local = 7:12; % Location of Left Leg joints in Full Body joints structure
w_local = 13:14; % Location of Waist joints in Full Body joints structure
ra_local = 15:20; % Location of Right Arm joints in Full Body joints structure
la_local = 21:26; % Location of Left Arm joints in Full Body joints structure

%   figure(666),plot(traj.CoM(3,:))
%   figure(777), plot(traj.SF)

% Define quaternions rates
Q_rate.RF = zeros(4,num_elements);
Q_rate_dot.RF = zeros(4,num_elements);
Q_rate.LF = zeros(4,num_elements);
Q_rate_dot.LF = zeros(4,num_elements);
Q_rate.RF_CoM = zeros(4,num_elements);
Q_rate_dot.RF_CoM = zeros(4,num_elements);
Q_rate.LF_CoM = zeros(4,num_elements);
Q_rate_dot.LF_CoM = zeros(4,num_elements);
Q_rate.RH = zeros(4,num_elements);
Q_rate_dot.RH = zeros(4,num_elements);
Q_rate.LH = zeros(4,num_elements);
Q_rate_dot.LH = zeros(4,num_elements);

% Define angular velocities
% ang_vel.RH = zeros(3,num_elements);
% ang_vel.LH = zeros(3,num_elements);
% ang_vel.RF = zeros(3,num_elements);
% ang_vel.LF = zeros(3,num_elements);
% ang_vel.RF_CoM = zeros(3,num_elements);
% ang_vel.LF_CoM = zeros(3,num_elements);
% ang_acc.RH = zeros(3,num_elements);
% ang_acc.LH = zeros(3,num_elements);
% ang_acc.RF = zeros(3,num_elements);
% ang_acc.LF = zeros(3,num_elements);
% ang_acc.RF_CoM = zeros(3,num_elements);
% ang_acc.LF_CoM = zeros(3,num_elements);


trajectory.RF = zeros(7,num_elements);
trajectory.LF = zeros(7,num_elements);
trajectory.RF_CoM = zeros(7,num_elements);
trajectory.LF_CoM = zeros(7,num_elements);
trajectory.RH = zeros(7,num_elements);
trajectory.LH = zeros(7,num_elements);

d_trajectory.RF = zeros(6,num_elements);
d_trajectory.LF = zeros(6,num_elements);
d_trajectory.RF_CoM = zeros(6,num_elements);
d_trajectory.LF_CoM = zeros(6,num_elements);
d_trajectory.RH = zeros(6,num_elements);
d_trajectory.LH = zeros(6,num_elements);


% Assign first configuration
q(:,1) = q0;  

trajectory.RF(:,1) = real(h.w_T_RF(q(:,1)));
trajectory.LF(:,1) = real(h.w_T_LF(q(:,1)));
trajectory.RF_CoM(:,1) = real(h.RF_T_CoM(q(:,1)));
trajectory.LF_CoM(:,1) = real(h.LF_T_CoM(q(:,1)));
trajectory.RH(:,1) = real(h.CoM_T_RH(q(:,1)));
trajectory.LH(:,1) = real(h.CoM_T_LH(q(:,1)));


% Calculate the Inverse Kinematics for the operational trajectory
for i = 1:num_elements - 1,
  
  % RF
%   trajectory.RF(:,i) = real(h.w_T_RF(q(:,i)));
  trajectory.RF(:,i+1) = pose_rpy2quat(pose_quat2rpy(trajectory.RF(:,i)) + d_traj.RF(:,i)*Ts);
  % Calculate quaternions rates
  [Q_rate.RF(:,i)] = quat_rate([trajectory.RF(4:end,i), trajectory.RF(4:end,i+1)], Ts);
%   quat_rate([trajectory.RF(4:end,i), trajectory.RF(4:end,i+1)], Ts);
  % Convert quaternions rates to angular velocities
  ang_vel.RF(:,i) = quat_rate2ang_vel(trajectory.RF(:,i), Q_rate.RF(:,i));
  d_trajectory.RF(:,i) = [(trajectory.RF(1:3,i+1) - trajectory.RF(1:3,i))/Ts; ang_vel.RF(:,i)];
  % Convert quaternions second rates to angular accelerations
%   ang_acc.RF(:,i) = quat_rate_dot2ang_acc(trajectory.RF(:,i), Q_rate_dot.RF(:,i));
  
  % LF
%   trajectory.LF(:,i) = real(h.w_T_LF(q(:,i)));
  trajectory.LF(:,i+1) = pose_rpy2quat(pose_quat2rpy(trajectory.LF(:,i)) + d_traj.LF(:,i)*Ts);
  [Q_rate.LF(:,i)] = quat_rate([trajectory.LF(4:end,i), trajectory.LF(4:end,i+1)], Ts);
  ang_vel.LF(:,i) = quat_rate2ang_vel(trajectory.LF(:,i), Q_rate.LF(:,i));
  d_trajectory.LF(:,i) = [(trajectory.LF(1:3,i+1) - trajectory.LF(1:3,i))/Ts; ang_vel.LF(:,i)];
  % Convert quaternions second rates to angular accelerations
%   ang_acc.LF(:,i) = quat_rate_dot2ang_acc(trajectory.LF(:,i), Q_rate_dot.LF(:,i));

  % RH
%   trajectory.RH(:,i) = h.CoM_T_RH(q(:,i));
  trajectory.RH(:,i+1) = pose_rpy2quat(pose_quat2rpy(trajectory.RH(:,i)) + d_traj.RH(:,i)*Ts);
  [Q_rate.RH(:,i)] = quat_rate([trajectory.RH(4:end,i), trajectory.RH(4:end,i+1)], Ts);
  ang_vel.RH(:,i) = quat_rate2ang_vel(trajectory.RH(:,i), Q_rate.RH(:,i));
  d_trajectory.RH(:,i) = [(trajectory.RH(1:3,i+1) - trajectory.RH(1:3,i))/Ts; ang_vel.RH(:,i)];
  % Convert quaternions second rates to angular accelerations
%   ang_acc.RH(:,i) = quat_rate_dot2ang_acc(trajectory.RH(:,i), Q_rate_dot.RH(:,i));

  % LH
%   trajectory.LH(:,i) = real(h.CoM_T_LH(q(:,i)));
  trajectory.LH(:,i+1) = pose_rpy2quat(pose_quat2rpy(trajectory.LH(:,i)) + d_traj.LH(:,i)*Ts);
  [Q_rate.LH(:,i)] = quat_rate([trajectory.LH(4:end,i), trajectory.LH(4:end,i+1)], Ts);
  ang_vel.LH(:,i) = quat_rate2ang_vel(trajectory.LH(:,i), Q_rate.LH(:,i));
  d_trajectory.LH(:,i) = [(trajectory.LH(1:3,i+1) - trajectory.LH(1:3,i))/Ts; ang_vel.LH(:,i)];
  % Convert quaternions second rates to angular accelerations
%   ang_acc.LH(:,i) = quat_rate_dot2ang_acc(trajectory.LH(:,i), Q_rate_dot.LH(:,i));

  % RF_CoM
%   trajectory.RF_CoM(:,i) = real(h.RF_T_CoM(q(:,i)));
  trajectory.RF_CoM(:,i+1) = pose_rpy2quat(pose_quat2rpy(trajectory.RF_CoM(:,i)) + d_traj.CoM(:,i)*Ts);
  [Q_rate.RF_CoM(:,i)] = quat_rate([trajectory.RF_CoM(4:end,i), trajectory.RF_CoM(4:end,i+1)], Ts);
  ang_vel.RF_CoM(:,i) = quat_rate2ang_vel(trajectory.RF_CoM(:,i), Q_rate.RF_CoM(:,i));
  d_trajectory.RF_CoM(:,i) = [(trajectory.RF_CoM(1:3,i+1) - trajectory.RF_CoM(1:3,i))/Ts; ang_vel.RF_CoM(:,i)];
  % Convert quaternions second rates to angular accelerations
%   ang_acc.RF_CoM(:,i) = quat_rate_dot2ang_acc(trajectory.RF_CoM(:,i), Q_rate_dot.RF_CoM(:,i));

  % LF_CoM
%   trajectory.LF_CoM(:,i) = real(h.LF_T_CoM(q(:,i)));
  trajectory.LF_CoM(:,i+1) = pose_rpy2quat(pose_quat2rpy(trajectory.LF_CoM(:,i)) + d_traj.CoM(:,i)*Ts);
  [Q_rate.LF_CoM(:,i)] = quat_rate([trajectory.LF_CoM(4:end,i), trajectory.LF_CoM(4:end,i+1)], Ts);
  ang_vel.LF_CoM(:,i) = quat_rate2ang_vel(trajectory.LF_CoM(:,i), Q_rate.LF_CoM(:,i));
  d_trajectory.LF_CoM(:,i) = [(trajectory.LF_CoM(1:3,i+1) - trajectory.LF_CoM(1:3,i))/Ts; ang_vel.LF_CoM(:,i)];
  % Convert quaternions second rates to angular accelerations
%   ang_acc.LF_CoM(:,i) = quat_rate_dot2ang_acc(trajectory.LF_CoM(:,i), Q_rate_dot.LF_CoM(:,i));

  %%% LEGS %%%
  switch d_traj.SF(i)
   case 0      % Double Support
     % Left Leg is floating
     [ q([ll_local w_local],i+1), dq([ll_local w_local],i), error(7:12,i)] = inverse_differential_kinematics_quat(trajectory.LF_CoM(:,i), d_trajectory.LF_CoM(:,i), q(:,i), h.LF_T_CoM, h.LF_J_CoM, Ts, Kp, Ko, [ll_local w_local]);
     
     % Right Foot is the Support Foot  
     [ q([rl_local w_local],i+1), dq([rl_local w_local],i), error(1:6,i)] = inverse_differential_kinematics_quat(trajectory.RF_CoM(:,i), d_trajectory.RF_CoM(:,i), q(:,i), h.RF_T_CoM, h.RF_J_CoM, Ts, Kp, Ko, [rl_local w_local]);

   case -1     % Right Foot Support
     % Left Leg is floating
     [ q(ll_local,i+1), dq(ll_local,i), error(7:12,i)] = inverse_differential_kinematics_quat(trajectory.LF(:,i), d_trajectory.LF(:,i), q(:,i), h.w_T_LF, h.w_J_LF, Ts, Kp, Ko, ll_local);
     
     % Right Foot is the Support Foot  
     [ q([rl_local w_local],i+1), dq([rl_local w_local],i), error(1:6,i)] = inverse_differential_kinematics_quat(trajectory.RF_CoM(:,i), d_trajectory.RF_CoM(:,i), q(:,i), h.RF_T_CoM, h.RF_J_CoM, Ts, Kp, Ko, [rl_local w_local]);

   case 1      % Left Foot Support
     % Right Leg is floating
     [ q(rl_local,i+1), dq(rl_local,i), error(1:6,i)] = inverse_differential_kinematics_quat(trajectory.RF(:,i), d_trajectory.RF(:,i), q(:,i), h.w_T_RF, h.w_J_RF, Ts, Kp, Ko, rl_local);

     % Left Foot is the Support Foot
     [ q([ll_local w_local],i+1), dq([ll_local w_local],i), error(7:12,i)] = inverse_differential_kinematics_quat(trajectory.LF_CoM(:,i), d_trajectory.LF_CoM(:,i), q(:,i), h.LF_T_CoM, h.LF_J_CoM, Ts, Kp, Ko, [ll_local w_local]);

  end

  %%% WAIST %%%
  % This joints values are calculated in CoM movement
%   [ q(w_local,i+1), dq(w_local,i), error(13:18,i)] = inverse_differential_kinematics_quat(trajectory.CoM(:,i), d_trajectory.CoM(:,i), q(:,i), h.w_T_CoM, h.w_J_CoM, Ts, Kp, Ko, w_local);

  %%% ARMS %%%
  % Right Arm
  [ q(ra_local,i+1), dq(ra_local,i), error(19:24,i)] = inverse_differential_kinematics_quat(trajectory.RH(:,i), d_trajectory.RH(:,i), q(:,i), h.CoM_T_RH, h.CoM_J_RH, Ts, Kp, Ko, ra_local);

  % Left Arm
  [ q(la_local,i+1), dq(la_local,i), error(25:30,i)] = inverse_differential_kinematics_quat(trajectory.LH(:,i), d_trajectory.LH(:,i), q(:,i), h.CoM_T_LH, h.CoM_J_LH, Ts, Kp, Ko, la_local);
end

% Joints Acceleration
ddq = [diff(dq,1,2) zeros(size(dq,1),1)]/Ts;

end