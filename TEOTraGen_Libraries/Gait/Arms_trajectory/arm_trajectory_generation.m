function [arm_traj, darm_traj, ddarm_traj] = arm_trajectory_generation(next_footprint_pose, prev_footprint_pose, Ts,  step_times, floating_foot, arm, arms_movement_option)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here


% Default arms_interpolation
if nargin < 7
  arms_movement_option = 'Polynomial 5'; 
end


switch arms_movement_option
  case 'Polynomial 5'
    [arm_traj, darm_traj, ddarm_traj] = arm_polynomial_trajectory(next_footprint_pose, prev_footprint_pose, Ts,  step_times, floating_foot, arm, '5');

  case 'Polynomial 3'
    [arm_traj, darm_traj, ddarm_traj] = arm_polynomial_trajectory(next_footprint_pose, prev_footprint_pose, Ts,  step_times, floating_foot, arm, '3');

  case 'Polynomial 7'
    [arm_traj, darm_traj, ddarm_traj] = arm_polynomial_trajectory(next_footprint_pose, prev_footprint_pose, Ts,  step_times, floating_foot, arm, '7');
      
  otherwise
    error('ErrorTEOTraGen:optNoImplemented', 'This arm motion is still not implemented.');
end


end

