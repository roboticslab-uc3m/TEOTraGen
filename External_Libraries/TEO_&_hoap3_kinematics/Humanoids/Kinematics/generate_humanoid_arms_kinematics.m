function arms_kinematics = generate_humanoid_arms_kinematics (chest_lengths, arm_lengths)
%   GENERATE_HUMANOID_ARMS_KINEMATICS generate kinematics for humanoid arms
%   arms_kinematics = generate_humanoid_arms_kinematics (chest_lengths,
%   arm_lengths) generates transformation matrices and jacobian matrices
%   for both arms. They relate the pose of right/left arm wrt the Center of
%   Mass. The lengths of the links are defined by the input chest_lengths
%   and arm_lengths:
%       chest_lengths(1) = CHEST_LINK1
%       chest_lengths(2) = CHEST_LINK2
%       arm_lengths(1)   = ARM_LINK1
%       arm_lengths(2)   = ARM_LINK2
%       arm_lengths(3)   = ARM_LINK3
%
%   See also HUMANOID_ARM_DH_PARAMETERS, GENERATE_HUMANOID_LEGS_KINEMATICS,
%   GENERATE_HUMANOID_TORSO_KINEMATICS.

%   Author: Paolo Pierro
%   $Revision: 1.3 $  $Date: 2011/08/05 $

% Generate generic generalized variables for both right and left arm
q = generate_symbolic_vector('theta', 6);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                         %
%  CREATE FORWARD KINEMATICS FOR ONE ARM  %
%                                         %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Create the model for one arm
[humanoid_arm.joint, humanoid_arm.n_joints] = humanoid_arm_DH_parameters(arm_lengths(2:3), q);

% Calculate Forward Kinematics
[~, F0_T_F6, F0_P_F6] = forward_kinematics (humanoid_arm);

% Create Transformation Matrix from CoM to Chest
CoM_R_c = sym(eye(3));
CoM_p_c = sym([chest_lengths(1); 0; chest_lengths(2)]);
CoM_T_c = create_homogeneous_matrix(CoM_R_c, CoM_p_c);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                           %
%  CREATE FORWARD KINEMATICS FOR RIGHT ARM  %
%                                           %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Create Transformation Matrix from Chest to Right Arm
c_R_F0 = [ 1,  0,  0;
           0,  0,  1;
           0, -1,  0];
c_p_F0r = [0; -arm_lengths(1); 0];
c_T_F0r = create_homogeneous_matrix(c_R_F0, c_p_F0r);

% Calculate Transformation Matrix from CoM to Right Arm
CoM_T_RH = multiply_homogeneous_matrix({CoM_T_c, c_T_F0r, F0_T_F6});

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                          %
%  CREATE FORWARD KINEMATICS FOR LEFT ARM  %
%                                          %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Create Transformation Matrix from Chest to Left Arm
c_p_F0l = [0; arm_lengths(1); 0];
c_T_F0l = create_homogeneous_matrix(c_R_F0, c_p_F0l);

% Calculate Transformation Matrix from CoM to Left Arm
CoM_T_LH = multiply_homogeneous_matrix({CoM_T_c, c_T_F0l, F0_T_F6});

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                  %
%  CREATE DIFFERENTIAL KINEMATICS  %
%                                  %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Calculate Jacobian (it is valid for both arms)
F0_J_F6 = evaluate_geometric_jacobian (F0_P_F6, humanoid_arm);
CoM_J_H = rotate_jacobian_matrix(F0_J_F6, c_R_F0);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                   %
%  CREATE KINEMATICS STRUCT FIELDS  %
%                                   %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
arms_kinematics.right.T = simplify_symbolic_matrix(CoM_T_RH);
arms_kinematics.right.J = simplify_symbolic_matrix(CoM_J_H);
arms_kinematics.right.q = q;

arms_kinematics.left.T  = simplify_symbolic_matrix(CoM_T_LH);
arms_kinematics.left.J  = arms_kinematics.right.J;
arms_kinematics.left.q  = q;
end