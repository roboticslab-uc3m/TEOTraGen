function Hr = rotate_hessian_matrix (H, R)
%ROTATE_HESSIAN_MATRIX calculates the Hessian matrix represented in a
%different origin frame, defined by a rotation matrix.
%
%   Hr = rotate_jacobian_matrix (H, R) generates one matrix which
%   represents the Hessian in a different Frame, which is defined by the
%   rotation matrix R.
%
%   See also EVALUATE_GEOMETRIC_HESSIAN, ROTATE_JACOBIAN_MATRIX,
%   FORWARD_KINEMATICS.
 
%   Author: Paolo Pierro
%   $Revision: 0.1 $  $Date: 2011/06/10 $

n = size(H, 3);
Hr = sym(zeros(size(H)));
for jj=1:n
    Hr(:, :, jj) = [ R,        zeros(3);
                     zeros(3),       R] * H(:, :, jj);
end
end