function [trajectory, d_trajectory, dd_trajectory] = move_simple_support (delta_com, delta_FF, Ts, T, trajectory, d_trajectory, dd_trajectory, support_leg,interpolationCoM,interpolaFF)
humanoid_fields = humanoid_operational_fields ();

% Setting values
ZERO_CON = zeros(6,1);

%%%%%%%%%%%%%%%%%%%%%%%%%%
% Trajectory for the CoM %
%%%%%%%%%%%%%%%%%%%%%%%%%%
%Debug (define these as global variables)
global P0_com_s P1_com_s P2_com_s P0_FF_s P1_FF_s P2_FF_s
global pp1_com pp2_com
global pp_com dpp_com ddpp_com

% Calculate CoM poses for climbing and landing phases
% p0_com = trajectory.CoM(:,end);
p0_com = trajectory.CoM(:,end);
p1_com = p0_com + delta_com(:,1);
p2_com = p0_com + delta_com(:,2);

% Convert to trajectory poses
P0_com_s = set_trajectory_condition(T(1), p0_com, ZERO_CON, ZERO_CON);
P1_com_s = set_trajectory_condition(T(2), p1_com, evaluate_mean_velocity (delta_com, T), ZERO_CON);
P2_com_s = set_trajectory_condition(T(3), p2_com, ZERO_CON, ZERO_CON);


switch interpolationCoM
  case 'Linear'
    % Linear
    [pp1_com, dpp1_com, ddpp1_com] = lineartrajectory(P0_com_s, P1_com_s, Ts);
    [pp2_com, dpp2_com, ddpp2_com] = lineartrajectory(P1_com_s, P2_com_s, Ts);
  case 'Polynomial3'
    % Polinomic 3 
    [pp1_com, dpp1_com, ddpp1_com] = poly3_trajectory (P0_com_s, P1_com_s, Ts);
    [pp2_com, dpp2_com, ddpp2_com] = poly3_trajectory (P1_com_s, P2_com_s, Ts);
  case 'Spline'
    % Spline
    [pp1_com, dpp1_com, ddpp1_com] = spline_interpolation (P0_com_s, P1_com_s, Ts);
    [pp2_com, dpp2_com, ddpp2_com] = spline_interpolation (P1_com_s, P2_com_s, Ts);
  case 'Cubic'
    % Cubic Spline
    [pp1_com, dpp1_com, ddpp1_com] = cubic_spline_trajectory (P0_com_s, P1_com_s, Ts);
    [pp2_com, dpp2_com, ddpp2_com] = cubic_spline_trajectory (P1_com_s, P2_com_s, Ts);
  case 'Polynomial5'
    % Polinomic 5 
    [pp1_com, dpp1_com, ddpp1_com] = poly5_trajectory (P0_com_s, P1_com_s, Ts);
    [pp2_com, dpp2_com, ddpp2_com] = poly5_trajectory (P1_com_s, P2_com_s, Ts);
  case 'Polynomial7'
    % Polinomic 7 
    [pp1_com, dpp1_com, ddpp1_com] = poly7_trajectory (P0_com_s, P1_com_s, Ts);
    [pp2_com, dpp2_com, ddpp2_com] = poly7_trajectory (P1_com_s, P2_com_s, Ts);
end

% Combine two trajectories (climbing and landing phases)
pp_com1  = combine_trajectories(pp1_com,  pp2_com);
dpp_com1 = combine_trajectories(dpp1_com, dpp2_com);
ddpp_com1 = combine_trajectories(ddpp1_com, ddpp2_com);

% Final CoM trajectory
pp_com  = create_trajectory_structure(pp_com1.data,  Ts, pp_com1.time);
dpp_com = create_trajectory_structure(dpp_com1.data, Ts, dpp_com1.time);
ddpp_com = create_trajectory_structure(ddpp_com1.data, Ts, ddpp_com1.time);

% Insert Final CoM Trajectory inside full body trajectories structures
trajectory   = insert_trajectory(trajectory,   humanoid_fields, pp_com,  'CoM');
d_trajectory = insert_trajectory(d_trajectory, humanoid_fields, dpp_com, 'CoM');
dd_trajectory = insert_trajectory(dd_trajectory, humanoid_fields, ddpp_com, 'CoM');



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Trajectory for the Floating Foot %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Debug (define these as global variables)
global pp_FF dpp_FF ddpp_FF


if strcmp(support_leg,'LF')        % Support on Left Foot
  FF = 'RF';
  support_foot = 1;
elseif strcmp(support_leg,'RF')    % Support on Right Foot
  FF = 'LF';
  support_foot = -1;
end

% Calculate Floating Foot poses for climbing and landing phases
p0_FF = trajectory.(FF)(:, end);
p1_FF = p0_FF + delta_FF(:,1);
p2_FF = p0_FF + delta_FF(:,2);

% Convert to trajectory poses
P0_FF_s = set_trajectory_condition(T(1), p0_FF, ZERO_CON, ZERO_CON);
P1_FF_s = set_trajectory_condition(T(2), p1_FF, evaluate_mean_velocity (delta_FF, T), ZERO_CON);
P2_FF_s = set_trajectory_condition(T(3), p2_FF, ZERO_CON, ZERO_CON);

switch interpolaFF
  case 'Linear'
      % Linear
      [pp1_FF, dpp1_FF, ddpp1_FF] = lineartrajectory(P0_FF_s, P1_FF_s, Ts);
      [pp2_FF, dpp2_FF, ddpp2_FF] = lineartrajectory(P1_FF_s, P2_FF_s, Ts);
  case 'Polynomial3'
      % Polinomic 3 
      [pp1_FF, dpp1_FF, ddpp1_FF] = poly3_trajectory (P0_FF_s, P1_FF_s, Ts);
      [pp2_FF, dpp2_FF, ddpp2_FF] = poly3_trajectory (P1_FF_s, P2_FF_s, Ts);
  case 'Spline'
      % Spline
      [pp1_FF, dpp1_FF, ddpp1_FF] = spline_interpolation (P0_FF_s, P1_FF_s, Ts);
      [pp2_FF, dpp2_FF, ddpp2_FF] = spline_interpolation (P1_FF_s, P2_FF_s, Ts);
  case 'Cubic'
      % Cubic Spline
      [pp1_FF, dpp1_FF, ddpp1_FF] = cubic_spline_trajectory (P0_FF_s, P1_FF_s, Ts);
      [pp2_FF, dpp2_FF, ddpp2_FF] = cubic_spline_trajectory (P1_FF_s, P2_FF_s, Ts);
  case 'Polynomial5'
      % Polinomic 5 
      [pp1_FF, dpp1_FF, ddpp1_FF] = poly5_trajectory (P0_FF_s, P1_FF_s, Ts);
      [pp2_FF, dpp2_FF, ddpp2_FF] = poly5_trajectory (P1_FF_s, P2_FF_s, Ts);

  case 'Polynomial7'
      % Polinomic 7 
      [pp1_FF, dpp1_FF, ddpp1_FF] = poly7_trajectory (P0_FF_s, P1_FF_s, Ts);
      [pp2_FF, dpp2_FF, ddpp2_FF] = poly7_trajectory (P1_FF_s, P2_FF_s, Ts);
end

% Combine two trajectories (climbing and landing phases)
pp_FF1 = combine_trajectories(pp1_FF, pp2_FF);
dpp_FF1 = combine_trajectories(dpp1_FF, dpp2_FF);
ddpp_FF1 = combine_trajectories(ddpp1_FF, ddpp2_FF);

% Final Floating Foot trajectory
pp_FF  = create_trajectory_structure(pp_FF1.data,  Ts, pp_FF1.time);
dpp_FF = create_trajectory_structure(dpp_FF1.data, Ts, dpp_FF1.time);
ddpp_FF = create_trajectory_structure(ddpp_FF1.data, Ts, ddpp_FF1.time);

% Insert Final Floating Foot Trajectory inside full body trajectories structures
trajectory   = insert_trajectory(trajectory,   humanoid_fields, pp_FF,  FF);
d_trajectory = insert_trajectory(d_trajectory, humanoid_fields, dpp_FF, FF);
dd_trajectory = insert_trajectory(dd_trajectory, humanoid_fields, ddpp_FF, FF);

% Update Support Foot field (-1 or 1) of full body trajectories structures
pp_SF  = create_trajectory_structure(support_foot*ones(size(pp_FF1.time)), Ts, pp_FF1.time);
trajectory   = insert_trajectory(trajectory,   humanoid_fields, pp_SF,  'SF');
d_trajectory = insert_trajectory(d_trajectory, humanoid_fields, pp_SF,  'SF');
dd_trajectory = insert_trajectory(dd_trajectory, humanoid_fields, pp_SF,  'SF');


%%Replicate last value
% *CHECK IF THERE IS A BUG
if strcmp(interpolaFF,'Cubic')
  if strcmp(interpolationCoM,'Cubic')
  trajectory.CoM = [trajectory.CoM trajectory.CoM(:,end)];
  d_trajectory.CoM = [d_trajectory.CoM d_trajectory.CoM(:,end)];
  dd_trajectory.CoM = [dd_trajectory.CoM dd_trajectory.CoM(:,end)];
  end
  trajectory.SF = [trajectory.SF trajectory.SF(:,end)];
  d_trajectory.SF = [d_trajectory.SF d_trajectory.SF(:,end)];
  dd_trajectory.SF = [dd_trajectory.SF dd_trajectory.SF(:,end)];
  temporal=trajectory.(FF);
  trajectory.(FF) = [temporal temporal(:,end)];
  temporal=d_trajectory.(FF);
  d_trajectory.(FF) = [temporal temporal(:,end)];
  temporal=dd_trajectory.(FF);
  dd_trajectory.(FF) = [temporal temporal(:,end)];
end

end

function vel_m = evaluate_mean_velocity (delta, T)
vel1 = delta(:,1)/(T(2)-T(1));
vel2 = (delta(:,2)-delta(:,1))/(T(3)-T(2));
vel_m = 0.5*vel1+0.5*vel2;
end