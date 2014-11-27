function res = is_homogeneous_matrix(A)
%IS_HOMOGENEOUS_MATRIX returns 1 if the input is a homogeneous
%transformation matrix, 0 otherwise.
%
%   See also CREATE_HOMOGENEOUS_MATRIX.

%   Author: Paolo Pierro
%   $Revision: 1.0 $  $Date: 2011/04/04 $
if A(4,1) == 0 && A(4,2) == 0 && A(4,3) == 0 && A(4,4) == 1
    res = 1;
else
    res = 0;
end
end