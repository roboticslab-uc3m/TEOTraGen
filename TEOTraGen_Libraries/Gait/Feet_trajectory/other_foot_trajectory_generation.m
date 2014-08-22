function [ other_traj, dother_traj, ddother_traj ] = other_foot_trajectory_generation( h, q0, step_times, Ts, floating_foot)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here


if strcmp(floating_foot, 'Right'),
  foot_pose = pose_quat2rpy(h.w_T_LF(q0));
elseif strcmp(floating_foot, 'Left'),
  foot_pose = pose_quat2rpy(h.w_T_RF(q0));
else
  error('ErrorTEOTraGen:wrongOption', 'Wrong Floating Foot option'); 
end


n = round((step_times.Tend - step_times.Tinit)/Ts + 1);

trajectory = zeros(size(foot_pose,1), n);
dtrajectory = zeros(size(foot_pose,1), n);
ddtrajectory = zeros(size(foot_pose,1), n);

for ii = 1:size(foot_pose,1),
  trajectory(ii,:) = foot_pose(ii);
end

time_SF = step_times.Tinit:Ts:step_times.Tend;

% Convert to trajectory structure
other_traj  = create_trajectory_structure(trajectory, Ts, time_SF);
dother_traj = create_trajectory_structure(dtrajectory, Ts, time_SF);
ddother_traj = create_trajectory_structure(ddtrajectory, Ts, time_SF);

end