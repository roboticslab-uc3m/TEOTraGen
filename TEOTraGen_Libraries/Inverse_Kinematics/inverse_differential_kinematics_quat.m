function [ q_next, q_dot, error ] = inverse_differential_kinematics_quat( pose_d, pose_dot_d, q0, T, J, Ts, Kp, Ko, body_pos)
%INVERSE_DIFFERENTIAL_KINEMATICS Summary of this function goes here
%   Detailed explanation goes here

% RETURN q(i+1), q_dot(i), error(i)

% Quaternions
% - pose_d: Desired pose
% - pose_e: End-effector pose


% Initial configuration
q = q0(body_pos);

  % End-effector pose
  pose_e = T(q0);

  % Position error
  error_pos = real(pose_d(1:3) - pose_e(1:3));

  % Orientation error
  error_orient = real(pose_e(4)*pose_d(5:7) - pose_d(4)*pose_e(5:7) - skew_matrix(pose_d(5:7))*pose_e(5:7));

  % Joints velocities
%     size(J(q(:,i)) \ [pose_dot_d(1:3,i) + Kp*error_pos; pose_dot_d(4:6,i) + Ko*error_orient])
  q_dot = J(q0) \ [pose_dot_d(1:3) + Kp*error_pos; pose_dot_d(4:6) + Ko*error_orient];

  
  % Euler integration
  q_next = q + q_dot*Ts;

  % Error values
  error = [error_pos; error_orient];

end



function S = skew_matrix(v)
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