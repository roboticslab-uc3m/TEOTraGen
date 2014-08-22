function torso_kinematics = generate_humanoid_torso_kinematics (waist_lengths, torso_lengths, order)
%   GENERATE_HUMANOID_TORSO_KINEMATICS generate kinematics for humanoid
%   torso
%   torso_kinematics = generate_humanoid_torso_kinematics (waist_lengths,
%   torso_lengths) generates the transformation matrix and the jacobian
%   matrix for the Center of Mass with respect to the robot waist (in the
%   center). The lengths of the links are defined by the inputs
%   waist_lengths and torso_lengths:
%       waist_lengths(1) = WAIST_LINK1
%       waist_lengths(2) = WAIST_LINK2
%       torso_lengths(1) = TORSO_LINK1
%       torso_lengths(2) = TORSO_LINK2
%       torso_lengths(3) = TORSO_LINK3 (NOT USED)
%
%   See also HUMANOID_TORSO_DH_PARAMETERS,
%   GENERATE_HUMANOID_LEGS_KINEMATICS, GENERATE_HUMANOID_ARMS_KINEMATICS.

%   Author: Paolo Pierro
%   $Revision: 1.3 $  $Date: 2011/08/05 $

% Generate generic generalized variables for torso joints
q = generate_symbolic_vector('theta', 2);

output_level = 1;
if nargin > 2
    if strcmpi(order, 'second')
        output_level = 2;
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                %
%  CREATE HUMANOID MODEL: TORSO  %
%                                %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Create the model for the Torso
[humanoid_torso.joint, humanoid_torso.n_joints] = humanoid_torso_DH_parameters(torso_lengths(1), q);

% Calculate forward kinematics for the Torso
[F0_A_F2, F0_T_F2, F0_P_F2] = forward_kinematics (humanoid_torso);

% Create Trasformation Matrix from Waist to Torso origin
w_R_F0 = sym(eye(3));
w_p_F0 = [waist_lengths(1); 0; waist_lengths(2)];
w_T_F0 = create_homogeneous_matrix(w_R_F0, w_p_F0);

% Create Trasformation Matrix from Torso end effector to Center of Mass
F2_R_IMU = [1, 0,  0;
            0, 0, -1;
            0, 1,  0];
F2_p_IMU = [0; -torso_lengths(2); 0];
F2_T_IMU = create_homogeneous_matrix(F2_R_IMU, F2_p_IMU);

% Create Trasformation Matrix from Waist to Center of Mass
w_T_IMU = multiply_homogeneous_matrix({w_T_F0, F0_T_F2, F2_T_IMU});

% Calculate Differential Kinematics
w_J_IMU = evaluate_geometric_jacobian (F0_P_F2, humanoid_torso);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                    %
%  CREATE HUMANOID TORSO MODEL: CREATE STRUCT FIELDS %
%                                                    %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
torso_kinematics.T = simplify_symbolic_matrix(w_T_IMU);
torso_kinematics.J = simplify_symbolic_matrix(w_J_IMU);
torso_kinematics.q = q;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                           %
%  SECOND ONDER KINEMATICS  %
%                           %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if output_level == 2
    w_H_IMU = evaluate_geometric_hessian (F0_P_F2, humanoid_torso);
    torso_kinematics.H = simplify_symbolic_matrix(w_H_IMU);
end
end