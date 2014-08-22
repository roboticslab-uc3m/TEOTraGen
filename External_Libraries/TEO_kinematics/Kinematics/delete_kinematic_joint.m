function reduced_kinematics = delete_kinematic_joint (full_kinematics, n_joint)
%DELETE_KINEMATIC_JOINT deletes one joint from a kinematics.

%   Author: Paolo Pierro
%   $Revision: 1.0 $  $Date: 2011/04/04 $

reduced_kinematics = full_kinematics;

%Deletes joint n_joint from the model
reduced_kinematics.T = delete_from_symbolic_matrix(reduced_kinematics.T, reduced_kinematics.q(n_joint));
reduced_kinematics.J(:,n_joint) = [];
reduced_kinematics.J = delete_from_symbolic_matrix(reduced_kinematics.J, reduced_kinematics.q(n_joint));
reduced_kinematics.q(n_joint) = [];
end

function h_new = delete_from_symbolic_matrix(h, joint)
if any(symvar(h) == joint)
    h_new = subs(h, joint, 0);
else
    h_new = h;
end
end