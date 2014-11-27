function legs_kinematics = generate_humanoid_legs_kinematics (link_lengths, order)
%   GENERATE_HUMANOID_LEGS_KINEMATICS generate the kinematics for humanoid
%   legs
%   legs_kinematics = generate_humanoid_legs_kinematics (legs)
%   generates transformation matrices and jacobian matrices for both legs
%   when they are standing and floating. The lengths of the links are
%   defined by the input leg_lenghts:
%       link_lengths(1) = LEG_LINK1
%       link_lengths(2) = LEG_LINK2
%       link_lengths(3) = LEG_LINK3
%       link_lengths(4) = LEG_LINK4
%       link_lengths(5) = LEG_LINK5
%
%   See also HUMANOID_LEG_FLOATING_DH_PARAMETERS,
%   HUMANOID_LEG_STANDING_DH_PARAMETERS, GENERATE_HUMANOID_ARMS_KINEMATICS,
%   GENERATE_HUMANOID_TORSO_KINEMATICS.

%   Author: Paolo Pierro
%   $Revision: 1.3 $  $Date: 2011/08/05 $

output_level = 1;
if nargin > 1
    if strcmpi(order, 'second')
        output_level = 2;
    end
end

% Generate generic generalized variables for both right and left leg
q = generate_symbolic_vector('theta', 6);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                            %
%  CREATE HUMANOID LEG MODEL: FLOATING LEGS  %
%                                            %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Create the model for Floating Legs
[right_leg_floating.joint, right_leg_floating.n_joints] = humanoid_leg_floating_DH_parameters(link_lengths(2:5), q);
[left_leg_floating.joint,  left_leg_floating.n_joints]  = humanoid_leg_floating_DH_parameters([link_lengths(2:3); -link_lengths(4); link_lengths(5)], q);

% Calculate Forward Kinematics of Floating Legs
%F0_A_F6=(:, :, jj) represents the transformation matrix from the reference system jj-1 to the system jj
%F0_T_F6 T which is the transformation matrix from the reference origin to the end-effector reference.
%The matrix P(:,:,jj) represents the cumulative product
%F0_P_F6 of the A matrix until jj
[F0_A_F6_FR, F0_T_F6_FR, F0_P_F6_FR] = forward_kinematics (right_leg_floating); 
[F0_A_F6_FL, F0_T_F6_FL, F0_P_F6_FL] = forward_kinematics (left_leg_floating);

% Create Trasformation Matrix from Waist to each Leg origin
w_R_F0_F  = sym(eye(3));
w_p_F0_FR = [0; -link_lengths(1); 0];
w_p_F0_FL = [0;  link_lengths(1); 0];
w_T_F0_FR = create_homogeneous_matrix(w_R_F0_F, w_p_F0_FR);
w_T_F0_FL = create_homogeneous_matrix(w_R_F0_F, w_p_F0_FL);

% Create Trasformation Matrix from each Leg end effector to Foot
F6_R_f_F = [ 0,  0, -1;
             0,  1,  0;
             1,  0,  0];
F6_T_f_F = create_homogeneous_matrix(F6_R_f_F, sym(zeros(3,1)));

% Calculate Trasformation Matrix from Waist to Right and Left Foot
w_T_RF = multiply_homogeneous_matrix({w_T_F0_FR, F0_T_F6_FR, F6_T_f_F});
w_T_LF = multiply_homogeneous_matrix({w_T_F0_FL, F0_T_F6_FL, F6_T_f_F});

% Calculate Differential Kinematics
w_J_RF = evaluate_geometric_jacobian (F0_P_F6_FR, right_leg_floating);
w_J_LF = evaluate_geometric_jacobian (F0_P_F6_FL, left_leg_floating);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                            %
%  CREATE HUMANOID LEG MODEL: STANDING LEGS  %
%                                            %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Create the model for Standing Legs
[right_leg_standing.joint, right_leg_standing.n_joints] = humanoid_leg_standing_DH_parameters(flipud(link_lengths(1:4)), flipud(q));
[left_leg_standing.joint,  left_leg_standing.n_joints]  = humanoid_leg_standing_DH_parameters([-link_lengths(4); link_lengths(3); link_lengths(2); -link_lengths(1)], flipud(q));

% Calculate Forward Kinematics of Standing Legs
[F0_A_F6_SR, F0_T_F6_SR, F0_P_F6_SR] = forward_kinematics (right_leg_standing);
[F0_A_F6_SL, F0_T_F6_SL, F0_P_F6_SL] = forward_kinematics (left_leg_standing);

% Create Trasformation Matrix from each Foot to respective Leg origin
F_R_F0_S = [ 0,  0, -1;
             0,  1,  0;
             1,  0,  0];
F_p_F0_S = [0; 0; link_lengths(5)];
F_T_F0_S = create_homogeneous_matrix(F_R_F0_S, F_p_F0_S);

% Create Trasformation Matrix from each Leg end effector to Waist
F6_R_w_S = [ 0,  1,  0;
             1,  0,  0;
             0,  0, -1];
F6_T_w_S = create_homogeneous_matrix(F6_R_w_S, sym(zeros(3,1)));

% Calculate Trasformation Matrix from Right and Left Foot to Waist
RF_T_w = multiply_homogeneous_matrix({F_T_F0_S, F0_T_F6_SR, F6_T_w_S});
LF_T_w = multiply_homogeneous_matrix({F_T_F0_S, F0_T_F6_SL, F6_T_w_S});

% Calculate Differential Kinematics
F0_J_F6_SR = evaluate_geometric_jacobian (F0_P_F6_SR, right_leg_standing);
RF_J_w = fliplr(rotate_jacobian_matrix(F0_J_F6_SR, F_R_F0_S));

F0_J_F6_SL = evaluate_geometric_jacobian (F0_P_F6_SL, left_leg_standing);
LF_J_w = fliplr(rotate_jacobian_matrix(F0_J_F6_SL, F_R_F0_S));


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                  %
%  CREATE HUMANOID LEG MODEL: CREATE STRUCT FIELDS %
%                                                  %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

legs_kinematics.right.floating.T = simplify_symbolic_matrix(w_T_RF);
legs_kinematics.right.floating.J = simplify_symbolic_matrix(w_J_RF);
legs_kinematics.right.standing.T = simplify_symbolic_matrix(RF_T_w);
legs_kinematics.right.standing.J = simplify_symbolic_matrix(RF_J_w);
legs_kinematics.right.q = q;

legs_kinematics.left.floating.T = simplify_symbolic_matrix(w_T_LF);
legs_kinematics.left.floating.J = simplify_symbolic_matrix(w_J_LF);
legs_kinematics.left.standing.T = simplify_symbolic_matrix(LF_T_w);
legs_kinematics.left.standing.J = simplify_symbolic_matrix(LF_J_w);
legs_kinematics.left.q = q;

legs_kinematics.link_lengths = link_lengths;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                           %
%  SECOND ONDER KINEMATICS  %
%                           %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if output_level == 2
    w_H_RF = evaluate_geometric_hessian (F0_P_F6_FR, right_leg_floating);
    w_H_LF = evaluate_geometric_hessian (F0_P_F6_FL, left_leg_floating);
    
    F0_H_F6_SR = evaluate_geometric_hessian (F0_P_F6_SR, right_leg_standing);
    RF_H_w = flip_hessian_joints(rotate_hessian_matrix(F0_H_F6_SR, F_R_F0_S));

    F0_H_F6_SL = evaluate_geometric_hessian (F0_P_F6_SL, left_leg_standing);
    LF_H_w = flip_hessian_joints(rotate_hessian_matrix(F0_H_F6_SL, F_R_F0_S));

    legs_kinematics.right.floating.H = simplify_symbolic_matrix(w_H_RF);
    legs_kinematics.right.standing.H = simplify_symbolic_matrix(RF_H_w);
    legs_kinematics.left.floating.H  = simplify_symbolic_matrix(w_H_LF);
    legs_kinematics.left.standing.H  = simplify_symbolic_matrix(LF_H_w);
end
end

function H_p = flip_hessian_joints(H)
H_p = flipdim(flipdim(H, 2), 3);
end