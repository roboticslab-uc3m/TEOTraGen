function [cog_traj, dcog_traj, ddcog_traj] = cog_polynomial_trajectory(next_footprint, prev_footprint, Ts,  step_times, alpha, beta, lambda, degree)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

% Get the degree of polynomial interpolation
if nargin < 8,
  degree = 5;
else
  if ischar(degree)
    degree = str2double(degree);
  end
  if isnan(degree)
    error('ErrorTEOTraGen:wrongInput', 'The degree input is not a validate option.');
  end
  if ((degree ~= 3) && (degree ~= 5) && (degree ~= 7))
    error('ErrorTEOTraGen:optNoImplemented', 'This degree is still not implemented.');
  end
end
polynomial_trajectory = str2func(strcat('poly', num2str(degree), '_viapoints_trajectory'));
% polynomial_trajectory = str2func(strcat('poly', num2str(degree), '_segments_trajectory'));

% Extract the total variation
total_delta = (next_footprint - prev_footprint)/2;

% Compute the corresponding poses

% Pose 1
p1_com = prev_footprint - prev_footprint;

% Pose 3
p3_com = zeros(6,1);
p3_com(2) = -(1-alpha)*(next_footprint(2) + prev_footprint(2))/4;
p3_com(1) = p1_com(1) + total_delta(1)/2;
p3_com(4) = p1_com(4) + total_delta(4)/2;
p3_com(5) = p1_com(5) + total_delta(5)/2;
p3_com(6) = p1_com(6) + total_delta(6)/2;

% Pose 5
p5_com = total_delta;

% Pose 2
p2_com = zeros(6,1);
p2_com(1) = p1_com(1) + total_delta(1)*(1-lambda)/2;
p2_com(2) = -(beta)*(next_footprint(2) + prev_footprint(2))/4;
p2_com(3) = p1_com(3) + total_delta(3)*(1-lambda)/2;
p2_com(4) = p1_com(4) + total_delta(4)*(1-lambda)/2;
p2_com(5) = p1_com(5) + total_delta(5)*(1-lambda)/2;
p2_com(6) = p1_com(6) + total_delta(6)*(1-lambda)/2;

% Pose 4
p4_com = zeros(6,1);
p4_com(1) = p1_com(1) + total_delta(1)*(1+lambda)/2;
p4_com(2) = -(beta)*(next_footprint(2) + prev_footprint(2))/4;
p4_com(3) = p1_com(3) + total_delta(3)*(1+lambda)/2;
p4_com(4) = p1_com(4) + total_delta(4)*(1+lambda)/2;
p4_com(5) = p1_com(5) + total_delta(5)*(1+lambda)/2;
p4_com(6) = p1_com(6) + total_delta(6)*(1+lambda)/2;

% Convert to trajectory points structure
P1 = set_trajectory_condition(step_times.Tinit, p1_com, zeros(6,1), zeros(6,1));
P2 = set_trajectory_condition(step_times.TDS1, p2_com, zeros(6,1), zeros(6,1));
P3 = set_trajectory_condition(step_times.TSS, p3_com, zeros(6,1), zeros(6,1));
P4 = set_trajectory_condition(step_times.TDS2, p4_com, zeros(6,1), zeros(6,1));
P5 = set_trajectory_condition(step_times.Tend, p5_com, zeros(6,1), zeros(6,1));
P = [P1 P2 P3 P4 P5];

% Interpolate
[cog_traj, dcog_traj, ddcog_traj] = polynomial_trajectory (P, Ts);

end
