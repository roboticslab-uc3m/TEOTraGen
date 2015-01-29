function [ sf_traj, dsf_traj, ddsf_traj ] = support_foot_trajectory_generation( step_times, Ts, floating_foot)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here


if strcmp(floating_foot, 'Right'),
  SF = 1;
elseif strcmp(floating_foot, 'Left'),
  SF = -1;
else
  error('ErrorTEOTraGen:wrongOption', 'Wrong Floating Foot option'); 
end



n1 = length(step_times.Tinit:Ts:step_times.TDS1)-1;
n2 = length(step_times.TDS1:Ts:step_times.TDS2)-1;
n3 = length(step_times.TDS2:Ts:step_times.Tend);

traj1 = 0*ones(1, n1);
traj2 = SF*ones(1, n2);
traj3 = 0*ones(1, n3);
trajectory = [traj1 traj2 traj3];

time_SF = step_times.Tinit:Ts:step_times.Tend;

% n1 = round((round_to_Ts(step_times.TDS1 - step_times.Tinit)/Ts);
% n2 = round((round_to_Ts(step_times.TDS2 - step_times.TDS1)/Ts);
% n3 = round((round_to_Ts(step_times.Tend - step_times.TDS2)/Ts + 1);
% 
% traj1 = 0*ones(1, n1);
% traj2 = SF*ones(1, n2);
% traj3 = 0*ones(1, n3);
% 
% trajectory = [traj1 traj2 traj3];
% time_SF = step_times.Tinit:Ts:step_times.Tend;
sf_traj  = create_trajectory_structure(trajectory, Ts, time_SF);

dsf_traj = sf_traj;
ddsf_traj = sf_traj;

end