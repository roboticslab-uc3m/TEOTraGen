% This script generates kinematics for the whole TEO robot
clear all
close all
addpath (genpath('.'))
TEO = TEO_structure('numeric', 'rad', 'm');
% TEO = TEO_structure('symbolic');

legs_kinematics  = generate_humanoid_legs_kinematics  ([TEO.legs.link_lengths]);
torso_kinematics = generate_humanoid_torso_kinematics ([TEO.waist.link_lengths], [TEO.torso.link_lengths]);
arms_kinematics  = generate_humanoid_arms_kinematics  ([TEO.chest.link_lengths], [TEO.arms.link_lengths]);

% Adapt to Hoap-3 conventions
TEO_kinematics_functions = convert_to_TEO_conventions (TEO, legs_kinematics, torso_kinematics, arms_kinematics);

% Create library file for the hoap-3
create_nested_functions(TEO_kinematics_functions, 'TEO_kinematics_libraryPRUEBAS');