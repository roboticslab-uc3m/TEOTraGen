function I_r = rotate_inertia_matrix(I, R)
%ROTATE_INERTIA_MATRIX calculates the Jacobian matrix represented in a
%different origin frame, defined by a rotation matrix.
%
%   I_r = rotate_inertia_matrix(I, R) generates one matrix which represents
%   the Inertia Matrix in a different Frame, which is defined by the
%   rotation matrix R.
%
%   See also ROTATE_JACOBIAN_MATRIX.
 
%   Author: Paolo Pierro
%   $Revision: 1.0 $  $Date: 2011/08/05 $
I_r = R * I * transpose(R);
end