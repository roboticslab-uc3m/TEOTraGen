function [q, dq, ddq] = inverse_ds_ss_jacobian_quat(q0, trajectory, d_trajectory, h, parameters)

%INVERSE_DS_SS_TEO Inverse Differential Kinematics using Jacobian with unit quaternion algorithm for a humanoid with
%any foot as support leg
%   [Q, DQ, DDQ] = 
%   INVERSE_DS_SS_JACOBIAN_QUAT(Q0, TRAJECTORY, H)
%   returns the joint trajectory, joint velocity and joint acceleration for
%   the selected trajectory.
%
%   INPUT:
%       Q0 = Initial configuration
%       TRAJECTORY = Trajectory for the humanoid parts
%       H = humanoids equations
%
%   NOTE: This algorithm doesn't allow to control velocities neither accelerations
%   TODO: Return accelerations (ddq) considering velocities (dq) diff.
%   TODO2: It doesn't consider CoM movement yet
%   See also INVERSE_DS_SS_JACOBIAN_QUAT.

%   Author: Domingo Esteban
%   References from: P. Pierro
%   RoboticsLab - Universidad Carlos III de Madrid
%   $Revision: 1.0 $  $Date: 2013/11/05 $
% *************************************************************************


%        ______      _           _   _            _           _     
%        | ___ \    | |         | | (_)          | |         | |    
%        | |_/ /___ | |__   ___ | |_ _  ___ ___  | |     __ _| |__  
%        |    // _ \| '_ \ / _ \| __| |/ __/ __| | |    / _` | '_ \ 
%        | |\ \ (_) | |_) | (_) | |_| | (__\__ \ | |___| (_| | |_) |
%        \_| \_\___/|_.__/ \___/ \__|_|\___|___/ \_____/\__,_|_.__/ 

                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                %%%     INVERSE_DS_SS_JACOBIAN_QUAT    %%%
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


% Default gains parameters
if nargin < 5,
  parameters.kp = 20;
  parameters.ko = 1;
end
                

% Inverse Kinematics for Left Leg Support in Simple Support (Using
% Jacobian with unit quaternions)

% Parameters and Variables needed
L = length(trajectory.time);
q  = zeros(size(q0,1), L);
dq = zeros(size(q0,1), L);
% ddq = zeros(size(q0,1), L); % Return accelerations (ddq) considering
% velocities (dq) diff, at bottom
Ts = trajectory.Ts;

% Position and Orientation Gain
Kp = parameters.kp;
Ko = parameters.ko;
    
% Errors Preallocation
e_p_LF = zeros(3,L);
e_o_LF = zeros(3,L);
e_p_RF = zeros(3,L);
e_o_RF = zeros(3,L);
e_p_RH = zeros(3,L);
e_o_RH = zeros(3,L);
e_p_LH = zeros(3,L);
e_o_LH = zeros(3,L);

    
% Initial positions
LF_p0_w = pose_quat2rpy(h.LF_T_w(q0));
RF_p0_w = pose_quat2rpy(h.RF_T_w(q0));
w_p0_RF = pose_quat2rpy(h.w_T_RF(q0));
w_p0_LF = pose_quat2rpy(h.w_T_LF(q0));
CoM_p0_RH = pose_quat2rpy(h.CoM_T_RH(q0));
CoM_p0_LH = pose_quat2rpy(h.CoM_T_LH(q0));


% Initial configuration
q(:,1) = q0;
    

for jj = 1:L-1
  if jj == 125
    disp('borrarXDTS');
  end
  % Inverse differential kinematics for Legs
  switch trajectory.SF(jj)
    case 0      % double support
      if (jj > 1) && (trajectory.SF(jj-1) == 1)
        RF_p0_w = pose_quat2rpy(h.RF_T_w(q(:,jj))) - trajectory.CoM(:,jj);
      end
      if (jj > 1) && (trajectory.SF(jj-1) == -1)
        LF_p0_w = pose_quat2rpy(h.LF_T_w(q(:,jj))) - trajectory.CoM(:,jj);
      end
      
      % Errors signals
      [e_p_LF(:,jj), e_o_LF(:,jj)] = determine_error (trajectory.CoM(:,jj) + LF_p0_w, ...
                                                     pose_quat2rpy(real(h.LF_T_w(q(:,jj)))));
      [e_p_RF(:,jj), e_o_RF(:,jj)] = determine_error (trajectory.CoM(:,jj) + RF_p0_w, ...
                                                     pose_quat2rpy(real(h.RF_T_w(q(:,jj)))));

      % Control signals
      u_L = [d_trajectory.CoM(1:3,jj) + Kp*e_p_LF(:,jj); d_trajectory.CoM(4:6,jj) + Ko*e_o_LF(:,jj)];
      u_R = [d_trajectory.CoM(1:3,jj) + Kp*e_p_RF(:,jj); d_trajectory.CoM(4:6,jj) + Ko*e_o_RF(:,jj)];

      % Joints velocities Output
      dq(7:12, jj) = invert_kinematics_standard (q(:,jj), h.LF_J_w, u_L, 1:6, 1:6);
      dq(1:6, jj) = invert_kinematics_standard (q(:,jj), h.RF_J_w, u_R, 1:6, 1:6);  

    case 1     % left foot support
      if (jj > 1) && (trajectory.SF(jj-1) == 0)
        w_p0_RF = pose_quat2rpy(h.w_T_RF(q(:,jj)));
      end
      % Errors signals for support foot
      [e_p_LF(:,jj), e_o_LF(:,jj)] = determine_error (trajectory.CoM(:,jj) + LF_p0_w, ...
                                                     pose_quat2rpy(real(h.LF_T_w(q(:,jj)))));

      [e_p_RF(:,jj), e_o_RF(:,jj)] = determine_error (trajectory.RF(:,jj) + w_p0_RF, ...
                                                     pose_quat2rpy(real(h.w_T_RF(q(:,jj)))));

      % Control signals for support foot
      u_L = [d_trajectory.CoM(1:3,jj) + Kp*e_p_LF(:,jj); d_trajectory.CoM(4:6,jj) + Ko*e_o_LF(:,jj)];
      u_R = [d_trajectory.RF(1:3,jj) + Kp*e_p_RF(:,jj); d_trajectory.RF(4:6,jj) + Ko*e_o_RF(:,jj)];


      % Joints velocities Output for support leg
      dq(7:12, jj) = invert_kinematics_standard (q(:,jj), h.LF_J_w, u_L, 1:6, 1:6);
      dq(1:6, jj) = invert_kinematics_standard (q(:,jj), h.w_J_RF, u_R, 1:6, 1:6);
      
    case -1     % right foot support
      if (jj > 1) && (trajectory.SF(jj-1) == 0)
        w_p0_LF = pose_quat2rpy(h.w_T_LF(q(:,jj)));
      end
      % Errors signals for support foot
      [e_p_RF(:,jj), e_o_RF(:,jj)] = determine_error (trajectory.CoM(:,jj) + RF_p0_w, ...
                                                     pose_quat2rpy(real(h.RF_T_w(q(:,jj)))));

      [e_p_LF(:,jj), e_o_LF(:,jj)] = determine_error (trajectory.LF(:,jj) + w_p0_LF, ...
                                                     pose_quat2rpy(real(h.w_T_LF(q(:,jj)))));

      % Control signals for support foot
      u_R = [d_trajectory.CoM(1:3,jj) + Kp*e_p_RF(:,jj); d_trajectory.CoM(4:6,jj) + Ko*e_o_RF(:,jj)];
      u_L = [d_trajectory.LF(1:3,jj) + Kp*e_p_LF(:,jj); d_trajectory.LF(4:6,jj) + Ko*e_o_LF(:,jj)];


      % Joints velocities Output for support leg
      dq(1:6, jj) = invert_kinematics_standard (q(:,jj), h.RF_J_w, u_R, 1:6, 1:6);
      dq(7:12, jj) = invert_kinematics_standard (q(:,jj), h.w_J_LF, u_L, 1:6, 1:6);
      
  end
    
  % Inverse differential kinematics for Torso -> We assume it doesn't move
  dq(13:14, jj) = 0;
  
  % Inverse differential kinematics for Arms
  % Errors signals for support foot
  [e_p_RH(:,jj), e_o_RH(:,jj)] = determine_error (trajectory.RH(:,jj) + CoM_p0_RH, pose_quat2rpy(h.CoM_T_RH(q(:,jj))));
  [e_p_LH(:,jj), e_o_LH(:,jj)] = determine_error (trajectory.LH(:,jj) + CoM_p0_LH, pose_quat2rpy(h.CoM_T_LH(q(:,jj))));
  
  % Control signals for support foot
  u_RH = [d_trajectory.RH(1:3,jj) + Kp*e_p_RH(:,jj); d_trajectory.RH(4:6,jj) + Ko*e_o_RH(:,jj)];
  u_LH = [d_trajectory.LH(1:3,jj) + Kp*e_p_LH(:,jj); d_trajectory.LH(4:6,jj) + Ko*e_o_LH(:,jj)];

  % Joints velocities Output for arms               
  dq(15:20, jj) = invert_kinematics_standard (q(:,jj), h.CoM_J_RH, u_RH, 1:6, 1:6); % -> Antes era q(:,jj), h.CoM_J_RH, u_RH, 1:3, 1:6
  dq(21:26, jj) = invert_kinematics_standard (q(:,jj), h.CoM_J_LH, u_LH, 1:6, 1:6); % -> Antes era q(:,jj), h.CoM_J_LH, u_LH, 1:3, 1:6


  % Joints positions Output for support legs and arms
  q(:, jj+1) = integrate_vector (q(:,jj), dq(:,jj), Ts);
    
end

% Joints Acceleration
ddq = [diff(dq,1,2) zeros(size(dq,1),1)]/Ts;

end


function ddq = invert_kinematics_standard (q_act, J, e, m, n)
J1 = J(q_act);
ddq = J1(m, n)\e(m); %A\B matrix division of A into B. Same as INV(A)*B
end

function x_next = integrate_vector (x, dx, Ts)
x_next = x + dx * Ts;
end

function [error_p, error_o] = determine_error (pd, p)
% Return position and orientation errors
desired_pose = pose_rpy2quat(pd);
real_pose = pose_rpy2quat(p);

eta = real_pose(4);
eps = real_pose(5:7);
eta_d = desired_pose(4);
eps_d = desired_pose(5:7);

error_p = real(desired_pose(1:3) - real_pose(1:3));
error_o = real(eta*eps_d - eta_d*eps - skew_matrix(eps_d)*eps);
end

function S = skew_matrix(v)
% Return skew vector
  if isvec(v,3)
    % SO(3) case
    S = [  0   -v(3)  v(2)
          v(3)  0    -v(1)
         -v(2) v(1)   0];
  elseif isvec(v,1)
      % SO(2) case
    S = [0 -v; v 0];
  else
    error('argument must be a 1- or 3-vector');
  end
end