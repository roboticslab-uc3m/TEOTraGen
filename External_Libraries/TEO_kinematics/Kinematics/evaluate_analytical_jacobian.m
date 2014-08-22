function J = evaluate_analytical_jacobian (x, n, q_var)
%EVALUATE_ANALYTICAL_JACOBIAN computes the differential kinematics for the
%manipulator defined by the position/orientation vector x, by aa number of
%joints and and by the joints variables q_var.
%
%   J = evaluate_geometric_jacobian (A, T, joint_type) generates one matrix
%   6 * n, where n is the number of degrees of freedom of the manipulator
%   defined as input. The J matrix represents the analytical Jacobian of
%   the robotic manipulator defined in input.
%
%   See also FORWARD_KINEMATICS.

%   Author: Paolo Pierro
%   $Revision: 1.0 $  $Date: 2011/01/16 $

J = sym(zeros(6,n));
for jj=1:n
    J(:,jj) = diff(x,q_var(jj));
end

end