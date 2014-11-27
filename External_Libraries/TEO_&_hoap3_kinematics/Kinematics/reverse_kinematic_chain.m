function [T_rev, J_rev] = reverse_kinematic_chain(T, J, q_var)
%REVERSE_KINEMATIC_CHAIN reverses the homogeneous transformation matrix of
%a kinematic chain and its Jacobian.
%
%   [T_rev, J_rev] = reverse_kinematic_chain(T, J, q_var) determines the
%   transformation matrix of the reversed kinematic chain defined by the
%   transformation matrix T and geometric Jacobian J and joints variables
%   q_var.
%
%   See also CREATE_T_MATRIX, INVERT_HOMOGENEOUS_MATRIX.

%   Author: Paolo Pierro
%   $Revision: 1.0 $  $Date: 2011/01/15 $
T_inv = invert_homogeneous_matrix(T);
T_rev = invert_input_sign(T_inv, q_var);
J_inv = invert_input_sign(J, q_var);
%J_rev = simplify([skew_symmetric_matrix(-T_rev(1:3,4))*J_inv(4:6,:) + T_rev(1:3,1:3)*J_inv(1:3,:);
%                    J_inv(4:6,:)]);
J_rev = [skew_symmetric_matrix(-T_rev(1:3,4))*J_inv(4:6,:) + T_rev(1:3,1:3)*J_inv(1:3,:);
                    J_inv(4:6,:)];
% I have to check the minus sign in J_rev
end

function B = invert_input_sign(A, q)
B = simplify(subs(A, q, -q));
end