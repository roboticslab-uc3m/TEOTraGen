function [ foot_traj, dfoot_traj, ddfoot_traj ] = foot_trajectory_generation(next_footprint_pose, prev_footprint_pose, Ts,  step_times, step_height, foot_option)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

% Default foot_interpolation
if nargin < 6
  foot_option = 'Polynomial 5'; 
end


switch foot_option
  case 'Polynomial 5'
    [foot_traj, dfoot_traj, ddfoot_traj] = foot_polynomial_trajectory(next_footprint_pose, prev_footprint_pose, Ts,  step_times, step_height, '5');

  case 'Polynomial 3'
    [foot_traj, dfoot_traj, ddfoot_traj] = foot_polynomial_trajectory(next_footprint_pose, prev_footprint_pose, Ts,  step_times, step_height, '3');

  case 'Polynomial 7'
    [foot_traj, dfoot_traj, ddfoot_traj] = foot_polynomial_trajectory(next_footprint_pose, prev_footprint_pose, Ts,  step_times, step_height, '7');
      
  otherwise
    error('ErrorTEOTraGen:optNoImplemented', 'This foot motion is still not implemented.');
end