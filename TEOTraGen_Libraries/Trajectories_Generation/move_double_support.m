function [trajectory, d_trajectory, dd_trajectory] = move_double_support (delta_p, Ts, T, trajectory, d_trajectory, dd_trajectory,interpolation)

ZERO_CON = zeros(6,1);

humanoid_fields = humanoid_operational_fields ();


% Calculate poses
% p0_com = trajectory.CoM(:, end);
p0_com = trajectory.CoM(:, end);
p1_com = p0_com + delta_p;

% Convert to trajectory poses
P0 = set_trajectory_condition(T(1), p0_com, ZERO_CON, ZERO_CON); 
P1 = set_trajectory_condition(T(2), p1_com, ZERO_CON, ZERO_CON);


% Select the respective interpolation    
switch interpolation
    
  case 'Linear'
      [pp_com, dpp_com, ddpp_com] = lineartrajectory(P0, P1, Ts);

  case 'Spline'
      [pp_com, dpp_com, ddpp_com] = spline_interpolation (P0, P1, Ts);

  case 'Cubic'
      [pp_com, dpp_com, ddpp_com] = cubic_spline_trajectory (P0, P1, Ts);

  case 'Polynomial3'
      [pp_com, dpp_com, ddpp_com] = poly3_trajectory (P0, P1, Ts);

  case 'Polynomial5'
      [pp_com, dpp_com, ddpp_com] = poly5_trajectory (P0, P1, Ts);

  case 'Polynomial7'
      [pp_com, dpp_com, ddpp_com] = poly7_trajectory (P0, P1, Ts);
        
	otherwise
      error('ErrorTEOTraGen:optNoImplemented', strcat(interpolation,' CoM interpolation is still not implemented.'));
end

% Aqu� cambio el tiempo que le metemos a la funci�n create_trajectory_structure
% ya que es tiempo inicial y tiempo final

trajectory   = insert_trajectory(trajectory,   humanoid_fields, pp_com,  'CoM');
d_trajectory = insert_trajectory(d_trajectory, humanoid_fields, dpp_com, 'CoM');
dd_trajectory = insert_trajectory(dd_trajectory, humanoid_fields, ddpp_com, 'CoM');

% Update Support Foot field (value: 0)
pp_SF  = create_trajectory_structure(0*ones(size(pp_com.time)), Ts, pp_com.time);
trajectory   = insert_trajectory(trajectory,   humanoid_fields, pp_SF,  'SF');
d_trajectory = insert_trajectory(d_trajectory, humanoid_fields, pp_SF,  'SF');
dd_trajectory = insert_trajectory(dd_trajectory, humanoid_fields, pp_SF,  'SF');

end