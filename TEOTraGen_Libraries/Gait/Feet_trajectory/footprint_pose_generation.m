function [ prev_footprint_pose, current_footprint_pose ] = footprint_pose_generation(h, q0 , xvar, step_d, theta_incr, floating_foot)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
% Domingo Esteban

  footprint_R0 = pose_quat2rpy(real(h.w_T_RF(q0)));
  footprint_L0 = pose_quat2rpy(real(h.w_T_LF(q0)));
  if strcmp(floating_foot, 'Right'),
    prev_footprint_pose = -footprint_L0 + footprint_R0;
    yvar_d = - step_d*2;
  elseif strcmp(floating_foot, 'Left'),
    prev_footprint_pose = -footprint_R0 + footprint_L0;
    yvar_d = step_d*2;
  else
    error('ErrorTEOTraGen:wrongOption', 'Wrong Floating Foot option'); 
  end

  yvar = yvar_d;
  zvar = 0;
  current_footprint_pose(1:3) = rotz(theta_incr)*[xvar; yvar; zvar];
  current_footprint_pose(4:5) = 0;
  current_footprint_pose(6) = theta_incr;
  
  
%   yvar = yvar_d - prev_footprint_pose(2);
%   zvar = 0;
%   current_footprint_pose(1:3) = prev_footprint_pose(1:3) + rotz(theta_incr)*[xvar; yvar; zvar];
%   current_footprint_pose(4:5) = prev_footprint_pose(4:5);
%   current_footprint_pose(6) = prev_footprint_pose(6) + theta_incr;

end

