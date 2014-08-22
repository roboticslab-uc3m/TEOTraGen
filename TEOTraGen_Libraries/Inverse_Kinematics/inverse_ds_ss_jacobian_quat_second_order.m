function [q, dq, ddq] = inverse_ds_ss_jacobian_quat_second_order(q0, trajectory, d_trajectory, dd_trajectory, h, parameters)

%INVERSE_DS_SS_JACOBIAN_QUAT_SECOND_ORDER Inverse Differential Kinematics using Jacobian with unit quaternion second-order algorithm
%for a humanoid with any foot as support leg
%   [Q, DQ, DDQ] = 
%   INVERSE_DS_SS_JACOBIAN_QUAT(Q0, TRAJECTORY, H)
%   returns the joint trajectory, joint velocity and joint acceleration for
%   the selected trajectory.
%
%   INPUT:
%       Q0 = Initial configuration
%       TRAJECTORY = Pose trajectory for the humanoid parts
%       D_TRAJECTORY = Velocity trajectory for the humanoid parts
%       DD_TRAJECTORY = Acceleration trajectory for the humanoid parts
%       H = humanoids equations
%
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

           %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
           %%%     INVERSE_DS_SS_JACOBIAN_QUAT_SECOND_ORDER    %%%
           %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

                
if nargin < 6,
  parameters.kp = 1.0000e-03;
  parameters.ko = 0.392699081698724;
end
                
            
% Inverse Kinematics for Left Leg Support in Simple Support (Using
% Jacobian with unit quaternions second-order algorythm)

% Parameters and Variables needed
    L = length(trajectory.time);
    q  = zeros(size(q0,1), L);
    dq = zeros(size(q0,1), L);
   ddq = zeros(size(q0,1), L);
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
    
    
%%%%%%%%
Jd.RF = zeros(6,6,L);
Jd.LF = zeros(6,6,L);
Jd.RH = zeros(6,6,L);
Jd.LH = zeros(6,6,L);
J.RF = zeros(6,6,L);
J.LF = zeros(6,6,L);
J.RH = zeros(6,6,L);
J.LH = zeros(6,6,L);

derror = zeros(26,L);

KP = [Kp  0  0  0  0  0;
       0 Kp  0  0  0  0;
       0  0 Kp  0  0  0;
       0  0  0 Ko  0  0;
       0  0  0  0 Ko  0;
       0  0  0  0  0 Ko];
KD = Kp./Ts;

for jj = 1:L-1
  % Inverse differential kinematics for Legs
  switch trajectory.SF(jj)
    case 0      % double support
      % Jacobians for legs                                             
      J.RF(:,:,jj) = h.RF_J_w(q(:,jj));
      J.LF(:,:,jj) = h.LF_J_w(q(:,jj));
      
      if (jj > 1)
        if (trajectory.SF(jj-1) == 1)
          Jd.RF(:,:,jj) = zeros(6,6);
          RF_p0_w = pose_quat2rpy(h.RF_T_w(q(:,jj))) - trajectory.CoM(:,jj);
        else
          Jd.RF(:,:,jj) = (J.RF(:,:,jj) - J.RF(:,:,jj-1))/Ts;
        end
      else
        Jd.RF(:,:,jj) = zeros(6,6);
      end
      if (jj > 1)
        if (trajectory.SF(jj-1) == -1)
          Jd.LF(:,:,jj) = zeros(6,6);
          LF_p0_w = pose_quat2rpy(h.LF_T_w(q(:,jj))) - trajectory.CoM(:,jj);
        else
          Jd.LF(:,:,jj) = (J.LF(:,:,jj) - J.LF(:,:,jj-1))/Ts;
        end
      else
        Jd.LF(:,:,jj) = zeros(6,6);
      end
      
      % Errors signals
%       [e_p_LF(:,jj) e_o_LF(:,jj)] = determine_error (trajectory.CoM(:,jj) + LF_p0_w, ...
%                                                      pose_quat2rpy(real(h.LF_T_w(q(:,jj)))));
%       [e_p_RF(:,jj) e_o_RF(:,jj)] = determine_error (trajectory.CoM(:,jj) + RF_p0_w, ...
%                                                      pose_quat2rpy(real(h.RF_T_w(q(:,jj)))));
      [e_p_LF(:,jj) e_o_LF(:,jj)] = determine_error (trajectory.CoM(:,jj) + LF_p0_w, ...
                                                     h.LF_T_w(q(:,jj)));
      [e_p_RF(:,jj) e_o_RF(:,jj)] = determine_error (trajectory.CoM(:,jj) + RF_p0_w, ...
                                                     h.RF_T_w(q(:,jj)));

      if (jj > 1)
        if (trajectory.SF(jj-1) == 1)
          derror(1:6,jj) = zeros(6,1);
        else
          derror(1:6,jj) = [e_p_RF(:,jj)-e_p_RF(:,jj-1); e_o_RF(:,jj)-e_o_RF(:,jj-1)]./Ts;
        end
      else
        derror(1:6,jj) = zeros(6,1);
      end
      if (jj > 1) 
        if (trajectory.SF(jj-1) == -1)
          derror(7:12,jj) = zeros(6,1);
        else
          derror(7:12,jj) = [e_p_LF(:,jj)-e_p_LF(:,jj-1); e_o_LF(:,jj)-e_o_LF(:,jj-1)]./Ts;
        end
      else
        derror(7:12,jj) = zeros(6,1);
      end                                        

    ddq(1:6, jj) = J.RF(:,:,jj)\(dd_trajectory.CoM(:,jj) + KD*derror(1:6,jj) + KP*[e_p_RF(:,jj); e_o_RF(:,jj)] - Jd.RF(:,:,jj)*dq(1:6, jj));
    ddq(7:12, jj) = J.LF(:,:,jj)\(dd_trajectory.CoM(:,jj) + KD*derror(7:12,jj) + KP*[e_p_LF(:,jj); e_o_LF(:,jj)] - Jd.LF(:,:,jj)*dq(7:12, jj));
      
    case 1     % left foot support
      % Jacobians for legs
      J.RF(:,:,jj) = h.w_J_RF(q(:,jj));
      J.LF(:,:,jj) = h.LF_J_w(q(:,jj));
      
      if (jj > 1) 
        if (trajectory.SF(jj-1) == 0)
          Jd.RF(:,:,jj) = zeros(6,6);
          w_p0_RF = pose_quat2rpy(h.w_T_RF(q(:,jj)));
        else
          Jd.RF(:,:,jj) = (J.RF(:,:,jj) - J.RF(:,:,jj-1))/Ts;
        end
        Jd.LF(:,:,jj) = (J.LF(:,:,jj) - J.LF(:,:,jj-1))/Ts;
      else
        Jd.RF(:,:,jj) = zeros(6,6);
        Jd.LF(:,:,jj) = zeros(6,6);
      end
      
      
      % Errors signals for support foot
%       [e_p_LF(:,jj) e_o_LF(:,jj)] = determine_error (trajectory.CoM(:,jj) + LF_p0_w, ...
%                                                      pose_quat2rpy(real(h.LF_T_w(q(:,jj)))));
% 
%       [e_p_RF(:,jj) e_o_RF(:,jj)] = determine_error (trajectory.RF(:,jj) + w_p0_RF, ...
%                                                      pose_quat2rpy(real(h.w_T_RF(q(:,jj)))));
      [e_p_LF(:,jj) e_o_LF(:,jj)] = determine_error (trajectory.CoM(:,jj) + LF_p0_w, ...
                                                     h.LF_T_w(q(:,jj)));

      [e_p_RF(:,jj) e_o_RF(:,jj)] = determine_error (trajectory.RF(:,jj) + w_p0_RF, ...
                                                     h.w_T_RF(q(:,jj)));
                                                   
      if (jj > 1) 
        if (trajectory.SF(jj-1) == 0)
          derror(1:6,jj) = zeros(6,1);
        else
          derror(1:6,jj) = [e_p_RF(:,jj)-e_p_RF(:,jj-1); e_o_RF(:,jj)-e_o_RF(:,jj-1)]./Ts;
        end
        derror(7:12,jj) = [e_p_LF(:,jj)-e_p_LF(:,jj-1); e_o_LF(:,jj)-e_o_LF(:,jj-1)]./Ts;
      else
        derror(1:6,jj) = zeros(6,1);
        derror(7:12,jj) = zeros(6,1);
      end
        
    ddq(1:6, jj) = J.RF(:,:,jj)\(dd_trajectory.RF(:,jj) + KD*derror(1:6,jj) + KP*[e_p_RF(:,jj); e_o_RF(:,jj)] - Jd.RF(:,:,jj)*dq(1:6, jj));
    ddq(7:12, jj) = J.LF(:,:,jj)\(dd_trajectory.CoM(:,jj) + KD*derror(7:12,jj) + KP*[e_p_LF(:,jj); e_o_LF(:,jj)] - Jd.LF(:,:,jj)*dq(7:12, jj));
      
    case -1     % right foot support
      J.RF(:,:,jj) = h.RF_J_w(q(:,jj));
      J.LF(:,:,jj) = h.w_J_LF(q(:,jj));
      
      if (jj > 1) 
        if(trajectory.SF(jj-1) == 0)
          Jd.LF(:,:,jj) = zeros(6,6);
          w_p0_LF = pose_quat2rpy(h.w_T_LF(q(:,jj)));
        else
          Jd.LF(:,:,jj) = (J.LF(:,:,jj) - J.LF(:,:,jj-1))/Ts;
        end
        Jd.RF(:,:,jj) = (J.RF(:,:,jj) - J.RF(:,:,jj-1))/Ts;
      else
        Jd.RF(:,:,jj) = zeros(6,6);
        Jd.LF(:,:,jj) = zeros(6,6);
      end
      
      
      % Errors signals for support foot
%       [e_p_RF(:,jj) e_o_RF(:,jj)] = determine_error (trajectory.CoM(:,jj) + RF_p0_w, ...
%                                                      pose_quat2rpy(real(h.RF_T_w(q(:,jj)))));
% 
%       [e_p_LF(:,jj) e_o_LF(:,jj)] = determine_error (trajectory.LF(:,jj) + w_p0_LF, ...
%                                                      pose_quat2rpy(real(h.w_T_LF(q(:,jj)))));
      [e_p_RF(:,jj) e_o_RF(:,jj)] = determine_error (trajectory.CoM(:,jj) + RF_p0_w, ...
                                                     h.RF_T_w(q(:,jj)));

      [e_p_LF(:,jj) e_o_LF(:,jj)] = determine_error (trajectory.LF(:,jj) + w_p0_LF, ...
                                                     h.w_T_LF(q(:,jj)));
                                                   
      if (jj > 1) 
        if (trajectory.SF(jj-1) == 0)
          derror(7:12,jj) = zeros(6,1);
        else
          derror(7:12,jj) = [e_p_LF(:,jj)-e_p_LF(:,jj-1); e_o_LF(:,jj)-e_o_LF(:,jj-1)]./Ts;
        end
        derror(1:6,jj) = [e_p_RF(:,jj)-e_p_RF(:,jj-1); e_o_RF(:,jj)-e_o_RF(:,jj-1)]./Ts;
      else
        derror(1:6,jj) = zeros(6,1);
        derror(7:12,jj) = zeros(6,1);
      end
      
    ddq(1:6, jj) = J.RF(:,:,jj)\(dd_trajectory.CoM(:,jj) + KD*derror(1:6,jj) + KP*[e_p_RF(:,jj); e_o_RF(:,jj)] - Jd.RF(:,:,jj)*dq(1:6, jj));
    ddq(7:12, jj) = J.LF(:,:,jj)\(dd_trajectory.LF(:,jj) + KD*derror(7:12,jj) + KP*[e_p_LF(:,jj); e_o_LF(:,jj)] - Jd.LF(:,:,jj)*dq(7:12, jj));
      
  end
    
  % Jacobians for arms
  J.RH(:,:,jj) = h.CoM_J_RH(q(:,jj));
  J.LH(:,:,jj) = h.CoM_J_LH(q(:,jj));

  if jj > 1
    Jd.RH(:,:,jj) = (J.RH(:,:,jj) - J.RH(:,:,jj-1))/Ts;
    Jd.LH(:,:,jj) = (J.LH(:,:,jj) - J.LH(:,:,jj-1))/Ts;
  end
  
  % Errors signals for arms
%   [e_p_RH(:,jj) e_o_RH(:,jj)] = determine_error (trajectory.RH(:,jj) + CoM_p0_RH, pose_quat2rpy(h.CoM_T_RH(q(:,jj))));
%   [e_p_LH(:,jj) e_o_LH(:,jj)] = determine_error (trajectory.LH(:,jj) + CoM_p0_LH, pose_quat2rpy(h.CoM_T_LH(q(:,jj))));
  [e_p_RH(:,jj) e_o_RH(:,jj)] = determine_error (trajectory.RH(:,jj) + CoM_p0_RH, h.CoM_T_RH(q(:,jj)));
  [e_p_LH(:,jj) e_o_LH(:,jj)] = determine_error (trajectory.LH(:,jj) + CoM_p0_LH, h.CoM_T_LH(q(:,jj)));
  
  % Error difference for arms
  if jj > 1
    derror(15:20,jj) = [e_p_RH(:,jj)-e_p_RH(:,jj-1); e_o_RH(:,jj)-e_o_RH(:,jj-1)]./Ts;
    derror(21:26,jj) = [e_p_LH(:,jj)-e_p_LH(:,jj-1); e_o_LH(:,jj)-e_o_LH(:,jj-1)]./Ts;
  end

  
  % Inverse differential kinematics full body
  ddq(13:14, jj) = 0; % Inverse differential kinematics for Torso -> We assume it doesn't move
  ddq(15:20, jj) = J.RH(:,:,jj)\(dd_trajectory.RH(:,jj) + KD*derror(15:20,jj) + KP*[e_p_RH(:,jj); e_o_RH(:,jj)] - Jd.RH(:,:,jj)*dq(15:20, jj));
  ddq(21:26, jj) = J.LH(:,:,jj)\(dd_trajectory.LH(:,jj) + KD*derror(21:26,jj) + KP*[e_p_LH(:,jj); e_o_LH(:,jj)] - Jd.LH(:,:,jj)*dq(21:26, jj));
  
  dq(:, jj+1) = integrate_vector (dq(:,jj), ddq(:,jj), Ts);
  q(:, jj+1) = integrate_vector (q(:,jj), dq(:,jj), Ts);
    
end


end


function x_next = integrate_vector (x, dx, Ts)
x_next = x + dx * Ts;
end

% function [error_p error_o] = determine_error (pd, p)
function [error_p error_o] = determine_error (pd, real_pose)
% Return position and orientation errors
desired_pose = pose_rpy2quat(pd);
% real_pose = pose_rpy2quat(p);
% real_pose = p;

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