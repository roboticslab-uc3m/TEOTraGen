function [ trajectory d_trajectory dd_trajectory ] = create_TEO_structures( Ts )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

% Fields corresponding to the operational space of TEO
TEO_fields = humanoid_operational_fields (); 

% Creates the trajectories structures
trajectory = create_trajectory_template (TEO_fields, Ts);
d_trajectory = create_trajectory_template (TEO_fields, Ts);
dd_trajectory = create_trajectory_template (TEO_fields, Ts);

end

