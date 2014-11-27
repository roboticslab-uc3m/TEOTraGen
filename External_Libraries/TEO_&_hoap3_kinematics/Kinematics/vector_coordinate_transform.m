function b = vector_coordinate_transform(T, a)
%VECTOR_COORDINATE_TRANSFORM converts the input vector in a new coordinate
%system 
%
%   b = vector_coordinate_transform(T, a) multiplies the input vector a by
%   the homogeneous transformation matrix T.
%
%   See also CREATE_HOMOGENEOUS_MATRIX.

%   Author: Paolo Pierro
%   $Revision: 1.1 $  $Date: 2011/08/06 $
c = T * [a; 1];
b = c(1:3);
end