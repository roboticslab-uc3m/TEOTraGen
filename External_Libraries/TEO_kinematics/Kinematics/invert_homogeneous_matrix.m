function T_inv = invert_homogeneous_matrix(T)
%INVERT_HOMOGENEOUS_MATRIX inverts a homogeneous transformation matrix.
%
%   See also CREATE_T_MATRIX.

%   Author: Paolo Pierro
%   $Revision: 1.0 $  $Date: 2011/04/04 $

if ~is_homogeneous_matrix(T)
    error('MULTIPLY_HOMOGENEOUS_MATRIX:argChk', 'Wrong type of input arguments: the inputs must be homogeneous transformation matrices')
end

R = transpose(T(1:3,1:3));
p = -R * T(1:3,4);
T_inv = create_homogeneous_matrix(R, p);
end