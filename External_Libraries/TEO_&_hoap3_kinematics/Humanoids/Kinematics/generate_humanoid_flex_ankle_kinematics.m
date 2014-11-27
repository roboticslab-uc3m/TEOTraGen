function flex_ankle_kinematics = generate_humanoid_flex_ankle_kinematics (order)
%   GENERATE_HUMANOID_FLEX_ANKLE_KINEMATICS generate the kinematics for
%   humanoid passive joints in the ankle
%   [w_T_c, w_J_c, q] = generate_humanoid_flex_ankle_kinematics () generates the
%   transformation matrix and the jacobian matrix for the passive joints in
%   the ankle. They relate the pose of right/left arm with respect to the foot.
%
%   See also HUMANOID_FLEX_ANKLE_DH_PARAMETERS,
%   GENERATE_HUMANOID_ARMS_KINEMATICS, GENERATE_HUMANOID_LEGS_KINEMATICS,
%   GENERATE_HUMANOID_TORSO_KINEMATICS.

%   Author: Paolo Pierro
%   $Revision: 1.1 $  $Date: 2011/08/05 $

% Define generic joints (roll and pitch)
syms thetafr thetafp real
q = [thetafr; thetafp];

output_level = 1;
if nargin > 0
    if strcmpi(order, 'second')
        output_level = 2;
    end
end
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                         %
%  CREATE HUMANOID MODEL: FLEXIBLE ANKLE  %
%                                         %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Create the model for the passive joints
[humanoid_flex_ankle.joint, humanoid_flex_ankle.n_joints] = humanoid_flex_ankle_DH_parameters(q);

% Calculate forward kinematics
[F0_A_F2, F0_T_F2, F0_P_F2] = forward_kinematics (humanoid_flex_ankle);

w_R_F0 = sym([0,  0, -1;
              0,  1,  0;
              1,  0,  0]);
ZERO = sym(zeros(3,1));
w_T_F0 = create_homogeneous_matrix(w_R_F0, ZERO);

w_T_f = multiply_homogeneous_matrix({w_T_F0, F0_T_F2});

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                  %
%  CREATE DIFFERENTIAL KINEMATICS  %
%                                  %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
F0_J_f = evaluate_geometric_jacobian (F0_P_F2, humanoid_flex_ankle);
w_J_f = rotate_jacobian_matrix(F0_J_f, w_R_F0);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                   %
%  CREATE KINEMATICS STRUCT FIELDS  %
%                                   %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
flex_ankle_kinematics.T = simplify_symbolic_matrix(w_T_f);
flex_ankle_kinematics.J = simplify_symbolic_matrix(w_J_f);
flex_ankle_kinematics.q = q;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                           %
%  SECOND ONDER KINEMATICS  %
%                           %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if output_level == 2
    F0_H_f = evaluate_geometric_hessian (F0_P_F2, humanoid_flex_ankle);
    w_H_f = rotate_hessian_matrix(F0_H_f, w_R_F0);
    flex_ankle_kinematics.H = simplify_symbolic_matrix(w_H_f);
end
end