function [trajectory, d_trajectory, dd_trajectory] = move_double_supportX (delta_p, Ts, T, trajectory, d_trajectory, dd_trajectory, interpola)
ZERO_CON = zeros(6,1);
humanoid_fields = humanoid_operational_fields ();
% L = round(T(1)/Ts)+1;

% Final Positions
p0_com = trajectory.CoM(:, end);
p1_com = p0_com + delta_p;

% Generating foot trajectory
% P = SET_TRAJECTORY_CONDITION(T, X, DX, DDX)
% creates the structure for a point to be used in some interpolating function. 
% Depending on the number of input arguments, it sets the conditions for time ‘t’ also for the first and second derivative.
P0 = set_trajectory_condition(T(1), p0_com, ZERO_CON, ZERO_CON); 
P1 = set_trajectory_condition(T(2), p1_com, ZERO_CON, ZERO_CON); 

    
switch interpola
    
    case 'Linear'
        % Linear
        [pp_com, dpp_com, ddpp_com] = lineartrajectory(P0, P1, Ts);
        
    case 'Spline'
        % Spline
        [pp_com, dpp_com, ddpp_com] = spline_interpolation (P0, P1, Ts);
        
    case 'Cubic'
        [pp_com, dpp_com, ddpp_com] = cubic_spline_trajectory (P0, P1, Ts);

    case 'Polynomial3'
        % Polynomial 3
        [pp_com, dpp_com, ddpp_com] = poly3_trajectory (P0, P1, Ts);
        
    case 'Polynomial5'
        % Polynomial 5
        [pp_com, dpp_com, ddpp_com] = poly5_trajectory (P0, P1, Ts);
        
    case 'Polynomial7'
        % Polynomial 7
        [pp_com, dpp_com, ddpp_com] = poly7_trajectory (P0, P1, Ts);
end

previous_support_foot = trajectory.SF(end);

trajectory   = insert_trajectory(trajectory,   humanoid_fields, pp_com,  'CoM');
d_trajectory = insert_trajectory(d_trajectory, humanoid_fields, dpp_com, 'CoM');
dd_trajectory = insert_trajectory(dd_trajectory, humanoid_fields, ddpp_com, 'CoM');

pp_SF  = create_trajectory_structure(zeros(size(pp_com.time)), Ts, pp_com.time);

trajectory   = insert_trajectory(trajectory,   humanoid_fields, pp_SF,  'SF');
d_trajectory = insert_trajectory(d_trajectory, humanoid_fields, pp_SF,  'SF');
dd_trajectory = insert_trajectory(dd_trajectory, humanoid_fields, pp_SF,  'SF');

end