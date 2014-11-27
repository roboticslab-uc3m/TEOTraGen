function T = create_homogeneous_matrix(R, p)
%CREATE_HOMOGENEOUS_MATRIX defines a new homogeneous transformation matrix.
%
%   T = CREATE_HOMOGENEOUS_MATRIX(R, p) defines a new transformation matrix
%   characterized by the rotation matrix R and the position vector p.

%   Author: Paolo Pierro
%   $Revision: 1.1 $  $Date: 2011/04/04 $
if any(size(R)~=[3,3])
    error('CREATE_HOMOGENEOUS_MATRIX:argChk', 'Wrong type of input arguments: the input R must be a matrix')
end

if any(size(p)~=[3,1])
    error('CREATE_HOMOGENEOUS_MATRIX:argChk', 'Wrong type of input arguments: the input p must be a vector')
end

T = [simplify_symbolic_matrix(R), simplify_symbolic_matrix(p);
                      zeros(1,3),                          1];
end