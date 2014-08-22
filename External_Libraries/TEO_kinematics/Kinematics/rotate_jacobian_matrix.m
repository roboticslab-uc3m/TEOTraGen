function Jr = rotate_jacobian_matrix (J, R)
%ROTATE_JACOBIAN_MATRIX calculates the Jacobian matrix represented in a
%different origin frame, defined by a rotation matrix.
%
%   Jr = rotate_jacobian_matrix (J, R) generates one matrix which
%   represents the Jacobian in a different Frame, which is defined by the
%   rotation matrix R.
%
%   See also EVALUATE_GEOMETRIC_JACOBIAN, EVALUATE_ANALYTICAL_JACOBIAN,
%   FORWARD_KINEMATICS.
 
%   Author: Paolo Pierro
%   $Revision: 1.0 $  $Date: 2011/01/16 $
Jr = [ R,        zeros(3);
       zeros(3), R       ]*J;
end