function S = skew_symmetric_matrix (x)
%SKEW_SYMMETRIC_MATRIX calculates the skew symmetric matrix related to the
%vector x.
%
%   S = skew_symmetric_matrix (x) generates the skew symmetric matrix of x.
%   The matrix S is so that its symmetric elements with respect to the main
%   diagonal represent the components of the vector x = [x1 x2 x3]^T.

%   Author: Paolo Pierro
%   $Revision: 1.0 $  $Date: 2011/01/17 $

S = [    0,     -x(3),       x(2);
      x(3),         0,      -x(1);
     -x(2),      x(1),         0];
end