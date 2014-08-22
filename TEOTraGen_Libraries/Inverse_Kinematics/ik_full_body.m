function [q, dq, ddq] = ik_full_body(q0, traj, d_traj, h, parameters)

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



num_elements = size(traj.time,2);

error = zeros(6*5, num_elements);
q = zeros(size(q0,1), num_elements);
dq = zeros(size(q0,1), num_elements);

% Parameters
Ts = traj.Ts; % Time step

% IK Gains
Kp = parameters.kp;
Ko = parameters.ko;

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
Q_rate.CoM = zeros(4,num_elements);
Q_rate_dot.CoM = zeros(4,num_elements);
Q_rate.RH = zeros(4,num_elements);
Q_rate_dot.RH = zeros(4,num_elements);
Q_rate.LH = zeros(4,num_elements);
Q_rate_dot.LH = zeros(4,num_elements);


% Convert trajectory with RPY to trajectory with Quaternions
trajectory.RF = pose_rpy2quat(traj.RF);
trajectory.LF = pose_rpy2quat(traj.LF);
trajectory.CoM = pose_rpy2quat(traj.CoM);
trajectory.RH = pose_rpy2quat(traj.RH);
trajectory.LH = pose_rpy2quat(traj.LH);

% Calculate quaternions rates
[Q_rate.RF, Q_rate_dot.RF] = quat_rate(trajectory.RF(4:end,:), Ts);
[Q_rate.LF, Q_rate_dot.LF] = quat_rate(trajectory.LF(4:end,:), Ts);
[Q_rate.CoM, Q_rate_dot.CoM] = quat_rate(trajectory.CoM(4:end,:), Ts);
[Q_rate.RH, Q_rate_dot.RH] = quat_rate(trajectory.RH(4:end,:), Ts);
[Q_rate.LH, Q_rate_dot.LH] = quat_rate(trajectory.LH(4:end,:), Ts);

% Convert quaternions rates to angular velocities
ang_vel.RF = quat_rate2ang_vel(trajectory.RF, Q_rate.RF);
ang_vel.LF = quat_rate2ang_vel(trajectory.LF, Q_rate.LF);
ang_vel.CoM = quat_rate2ang_vel(trajectory.CoM, Q_rate.CoM);
ang_vel.RH = quat_rate2ang_vel(trajectory.RH, Q_rate.RH);
ang_vel.LH = quat_rate2ang_vel(trajectory.LH, Q_rate.LH);

% Convert quaternions second rates to angular accelerations
ang_acc.RF = quat_rate_dot2ang_acc(trajectory.RF, Q_rate_dot.RF);
ang_acc.LF = quat_rate_dot2ang_acc(trajectory.LF, Q_rate_dot.LF);
ang_acc.CoM = quat_rate_dot2ang_acc(trajectory.CoM, Q_rate_dot.CoM);
ang_acc.RH = quat_rate_dot2ang_acc(trajectory.RH, Q_rate_dot.RH);
ang_acc.LH = quat_rate_dot2ang_acc(trajectory.LH, Q_rate_dot.LH);

% Replace rpy rates of the input structure with angular velocities
% calculated
d_traj.RF(4:6,:) = ang_vel.RF;
d_traj.LF(4:6,:) = ang_vel.LF;
d_traj.CoM(4:6,:) = ang_vel.CoM;
d_traj.RH(4:6,:) = ang_vel.RH;
d_traj.LH(4:6,:) = ang_vel.LH;
   

% Assign first configuration
q(:,1) = q0;  

% Calculate the Inverse Kinematics for the operational trajectory
for i = 1:num_elements - 1,
  %%% LEGS %%%
  switch traj.SF(i)
   case 0      % Double Support
     % Considero igual RF como Soporte (evito la falta de precision de la formula)
     % Left Leg is floating
%      [ q([ll_local w_local],i+1), dq([ll_local w_local],i), error(7:12,i)] = inverse_differential_kinematics_quat(trajectory.CoM(:,i), d_traj.CoM(:,i), q(:,i), h.LF_T_CoM, h.LF_J_CoM, Ts, Kp, Ko, [ll_local w_local]);
     [ q(ll_local,i+1), dq(ll_local,i), error(7:12,i)] = inverse_differential_kinematics_quat(trajectory.LF(:,i), d_traj.LF(:,i), q(:,i), h.w_T_LF, h.w_J_LF, Ts, Kp, Ko, ll_local);
     
     % Right Foot is the Support Foot  
     [ q([rl_local w_local],i+1), dq([rl_local w_local],i), error(1:6,i)] = inverse_differential_kinematics_quat(trajectory.CoM(:,i), d_traj.CoM(:,i), q(:,i), h.RF_T_CoM, h.RF_J_CoM, Ts, Kp, Ko, [rl_local w_local]);

   case -1     % Right Foot Support
     % Left Leg is floating
     [ q(ll_local,i+1), dq(ll_local,i), error(7:12,i)] = inverse_differential_kinematics_quat(trajectory.LF(:,i), d_traj.LF(:,i), q(:,i), h.w_T_LF, h.w_J_LF, Ts, Kp, Ko, ll_local);
     
     % Right Foot is the Support Foot  
     [ q([rl_local w_local],i+1), dq([rl_local w_local],i), error(1:6,i)] = inverse_differential_kinematics_quat(trajectory.CoM(:,i), d_traj.CoM(:,i), q(:,i), h.RF_T_CoM, h.RF_J_CoM, Ts, Kp, Ko, [rl_local w_local]);

   case 1      % Left Foot Support
     % Right Leg is floating
     [ q(rl_local,i+1), dq(rl_local,i), error(1:6,i)] = inverse_differential_kinematics_quat(trajectory.RF(:,i), d_traj.RF(:,i), q(:,i), h.w_T_RF, h.w_J_RF, Ts, Kp, Ko, rl_local);

     % Left Foot is the Support Foot
     [ q([ll_local w_local],i+1), dq([ll_local w_local],i), error(7:12,i)] = inverse_differential_kinematics_quat(trajectory.CoM(:,i), d_traj.CoM(:,i), q(:,i), h.LF_T_CoM, h.LF_J_CoM, Ts, Kp, Ko, [ll_local w_local]);

  end

  %%% WAIST %%%
  % This joints values are calculated in CoM movement
%   [ q(w_local,i+1), dq(w_local,i), error(13:18,i)] = inverse_differential_kinematics_quat(trajectory.CoM(:,i), d_traj.CoM(:,i), q(:,i), h.w_T_CoM, h.w_J_CoM, Ts, Kp, Ko, w_local);

  %%% ARMS %%%
  % Right Arm
  [ q(ra_local,i+1), dq(ra_local,i), error(19:24,i)] = inverse_differential_kinematics_quat(trajectory.RH(:,i), d_traj.RH(:,i), q(:,i), h.CoM_T_RH, h.CoM_J_RH, Ts, Kp, Ko, ra_local);

  % Left Arm
  [ q(la_local,i+1), dq(la_local,i), error(25:30,i)] = inverse_differential_kinematics_quat(trajectory.LH(:,i), d_traj.LH(:,i), q(:,i), h.CoM_T_LH, h.CoM_J_LH, Ts, Kp, Ko, la_local);
end

% Joints Acceleration
ddq = [diff(dq,1,2) zeros(size(dq,1),1)]/Ts;

end