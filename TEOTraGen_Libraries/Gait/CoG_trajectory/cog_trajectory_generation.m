function [ cog_traj, dcog_traj, ddcog_traj ] = cog_trajectory_generation( next_footprint_pose, prev_footprint_pose, step_times, Ts, alpha, beta, lambda, cog_option, zc, g)

% Default Cog Option
if nargin < 8
  cog_option = 'Polynomial 5';
end

switch cog_option
  case 'Polynomial 5'
    [cog_traj, dcog_traj, ddcog_traj] = cog_polynomial_trajectory(next_footprint_pose, prev_footprint_pose, Ts,  step_times, alpha, beta, lambda, '5');

  case 'Polynomial 3'    
    [cog_traj, dcog_traj, ddcog_traj] = cog_polynomial_trajectory(next_footprint_pose, prev_footprint_pose, Ts,  step_times, alpha, beta, lambda, '3');
  
  case 'Polynomial 7'
    [cog_traj, dcog_traj, ddcog_traj] = cog_polynomial_trajectory(next_footprint_pose, prev_footprint_pose, Ts,  step_times, alpha, beta, lambda, '7');
  
  case '3D LIPM'
%     [cog_traj, dcog_traj, ddcog_traj] = cog_3d_LIPM_trajectory(footprints_poses, median_zmp, Ts, ss_time, ds_time, t0, zc, beta, lambda, g);
    [cog_traj, dcog_traj, ddcog_traj] = cog_3d_LIPM_trajectoryMODIFICADO(next_footprint_pose, prev_footprint_pose, Ts, step_times, beta, lambda, zc, g);
  
  case 'Cart Table - Kajita'
    zmp_traj = zmp_trajectory(next_footprint_pose, prev_footprint_pose, Ts, step_times, alpha, beta, lambda);
    [cog_traj, dcog_traj, ddcog_traj] = cog_cart_table_trajectory('Kajita', zmp_traj.data, Ts, step_times, zc, beta, lambda, g, zmp_traj.time);
    
  case 'Cart Table - Wieber'
    zmp_traj = zmp_trajectory(next_footprint_pose, prev_footprint_pose, Ts, step_times, alpha, beta, lambda);
    [cog_traj, dcog_traj, ddcog_traj] = cog_cart_table_trajectory('Wieber', zmp_traj.data, Ts, step_times, zc, beta, lambda, g, zmp_traj.time);
    
  otherwise
    % - Cart Table
    error('ErrorTEOTraGen:optNoImplemented', 'This CoG motion is still not implemented.');
    return;
end