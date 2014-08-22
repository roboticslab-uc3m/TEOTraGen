function [trajectory, d_trajectory, dd_trajectory] = move_arm_simple_support (arm, delta_arm, Ts, T, trajectory, d_trajectory, dd_trajectory, interpolation)
%MOVE_ARM_SIMPLE_SUPPORT Arm motion for the robot TEO considering
%   customvariations (delta)
%   [TRAJECTORY, D_TRAJECTORY, DD_TRAJECTORY] = DS_SS_STEP_TEP(ARM, DELTA_ARM, TS, T, TRAJECTORY, D_TRAJECTORY, DD_TRAJECTORY, ARM_INTERPOLATION)
%   returns operational space trajectory, velocity and acceleration for the
%   selected arm
%
%   INPUT:
%       ARM = 'RH' or 'LH'
%       DELTA_ARM = [XVAR; YVAR; ZVAR]
%       TS = SAMPLE TIME (seconds)
%       T = [INITIAL_TIME END_TIME]
%       TRAJECTORY = OPERATIONAL TRAJECTORY STRUCTURE
%       D_TRAJECTORY = OPERATIONAL VELOCITY TRAJECTORY STRUCTURE
%       DD_TRAJECTORY = OPERATIONAL ACCELERATION TRAJECTORY STRUCTURE
%       INTERPOLATION = ARM INTERPOLATION (e.g. Polynomial5, Spline, etc.)
%
%   See also MOVE_DOUBLE_SUPPORT, MOVE_SIMPLE_SUPPORT.

%   Author: Domingo Esteban
%   RoboticsLab - Universidad Carlos III de Madrid
%   $Revision: 1.0 $  $Date: 2014/01/28 $
% *************************************************************************


ZERO_CON = zeros(6,1);
humanoid_fields = humanoid_operational_fields ();

% Calculate Hand poses for simple support phase
% L = round(T(1)/Ts)+1;
% p0_arm = trajectory.(arm)(:,L);
p0_arm = trajectory.(arm)(:,end);
p1_arm = p0_arm;
p2_arm = p0_arm + delta_arm;
p3_arm = p2_arm;

% Convert to trajectory poses
P0 = set_trajectory_condition(T(1), p0_arm, ZERO_CON, ZERO_CON);
P1 = set_trajectory_condition(T(2), p1_arm, ZERO_CON, ZERO_CON);
P2 = set_trajectory_condition(T(4), p2_arm, ZERO_CON, ZERO_CON);
P3 = set_trajectory_condition(T(5), p3_arm, ZERO_CON, ZERO_CON);
% P4 = set_trajectory_condition(T(5), p4_arm, ZERO_CON, ZERO_CON);


% Select the respective interpolation    
switch interpolation
    
  case 'Linear'
      [pp_arm1, dpp_arm1, ddpp_arm1] = lineartrajectory(P0, P1, Ts);
      [pp_arm2, dpp_arm2, ddpp_arm2] = lineartrajectory(P1, P2, Ts);
      [pp_arm3, dpp_arm3, ddpp_arm3] = lineartrajectory(P2, P3, Ts);
%       [pp_arm4, dpp_arm4, ddpp_arm4] = lineartrajectory(P3, P4, Ts);

  case 'Spline'
      [pp_arm1, dpp_arm1, ddpp_arm1] = spline_interpolation (P0, P1, Ts);
      [pp_arm2, dpp_arm2, ddpp_arm2] = spline_interpolation (P1, P2, Ts);
      [pp_arm3, dpp_arm3, ddpp_arm3] = spline_interpolation (P2, P3, Ts);
%       [pp_arm4, dpp_arm4, ddpp_arm4] = spline_interpolation (P3, P4, Ts);

  case 'Cubic'
      [pp_arm1, dpp_arm1, ddpp_arm1] = cubic_spline_trajectory (P0, P1, Ts);
      [pp_arm2, dpp_arm2, ddpp_arm2] = cubic_spline_trajectory (P1, P2, Ts);
      [pp_arm3, dpp_arm3, ddpp_arm3] = cubic_spline_trajectory (P2, P3, Ts);
%       [pp_arm4, dpp_arm4, ddpp_arm4] = cubic_spline_trajectory (P3, P4, Ts);

  case 'Polynomial3'
      [pp_arm1, dpp_arm1, ddpp_arm1] = poly3_trajectory (P0, P1, Ts);
      [pp_arm2, dpp_arm2, ddpp_arm2] = poly3_trajectory (P1, P2, Ts);
      [pp_arm3, dpp_arm3, ddpp_arm3] = poly3_trajectory (P2, P3, Ts);
%       [pp_arm4, dpp_arm4, ddpp_arm4] = poly3_trajectory (P3, P4, Ts);

  case 'Polynomial5'
      [pp_arm1, dpp_arm1, ddpp_arm1] = poly5_trajectory (P0, P1, Ts);
      [pp_arm2, dpp_arm2, ddpp_arm2] = poly5_trajectory (P1, P2, Ts);
      [pp_arm3, dpp_arm3, ddpp_arm3] = poly5_trajectory (P2, P3, Ts);
%       [pp_arm4, dpp_arm4, ddpp_arm4] = poly5_trajectory (P3, P4, Ts);

  case 'Polynomial7'
      [pp_arm1, dpp_arm1, ddpp_arm1] = poly7_trajectory (P0, P1, Ts);
      [pp_arm2, dpp_arm2, ddpp_arm2] = poly7_trajectory (P1, P2, Ts);
      [pp_arm3, dpp_arm3, ddpp_arm3] = poly7_trajectory (P2, P3, Ts);
%       [pp_arm4, dpp_arm4, ddpp_arm4] = poly7_trajectory (P3, P4, Ts);
        
	otherwise
      error('ErrorTEOTraGen:optNoImplemented', strcat(interpolation,' arm interpolation is still not implemented.'));
end

% Combine two trajectories (climbing and landing phases)
pp_temporal = combine_trajectories(pp_arm1, pp_arm2);
dpp_temporal = combine_trajectories(dpp_arm1, dpp_arm2);
ddpp_temporal = combine_trajectories(ddpp_arm1, ddpp_arm2);
pp_temporal = combine_trajectories(pp_temporal, pp_arm3);
dpp_temporal = combine_trajectories(dpp_temporal, dpp_arm3);
ddpp_temporal = combine_trajectories(ddpp_temporal, ddpp_arm3);
% pp_temporal = combine_trajectories(pp_temporal, pp_arm4);
% dpp_temporal = combine_trajectories(dpp_temporal, dpp_arm4);
% ddpp_temporal = combine_trajectories(ddpp_temporal, ddpp_arm4);


% Final Floating Foot trajectory
pp_arm  = create_trajectory_structure(pp_temporal.data,  Ts, pp_temporal.time);
dpp_arm = create_trajectory_structure(dpp_temporal.data, Ts, dpp_temporal.time);
ddpp_arm = create_trajectory_structure(ddpp_temporal.data, Ts, ddpp_temporal.time);

% Insert Final ARM Trajectory inside full body trajectories structures
trajectory   = insert_trajectory(trajectory,   humanoid_fields, pp_arm, (arm));
d_trajectory = insert_trajectory(d_trajectory, humanoid_fields, dpp_arm, (arm));
dd_trajectory = insert_trajectory(dd_trajectory, humanoid_fields, ddpp_arm, (arm));

end

