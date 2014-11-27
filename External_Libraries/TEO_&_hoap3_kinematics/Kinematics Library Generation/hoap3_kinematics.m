% This script generates kinematics for the whole Hoap-3 robot
clear all
close all
addpath (genpath('.'))
hoap3 = hoap3_structure('numeric', 'rad', 'm');
% hoap3 = hoap3_structure('symbolic');

legs_kinematics  = generate_humanoid_legs_kinematics  ([hoap3.legs.link_lengths]);
torso_kinematics = generate_humanoid_torso_kinematics ([hoap3.waist.link_lengths], [hoap3.torso.link_lengths]);
arms_kinematics  = generate_humanoid_arms_kinematics  ([hoap3.chest.link_lengths], [hoap3.arms.link_lengths]);

% Adapt to Hoap-3 conventions
hoap3_kinematics_functions = convert_to_hoap_conventions (hoap3, legs_kinematics, torso_kinematics, arms_kinematics);

% Create library file for the hoap-3
create_nested_functions(hoap3_kinematics_functions, 'hoap3_kinematics_library');