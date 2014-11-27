function Y = MV_sum (A, b)
%MV_SUM sums a matrix with a vector
%   Y = MV_SUM (A, b) sums the matrix A with the vector b. One dimension of A
%   must be the same of the vector's.
%
%   Author: Paolo Pierro
%   $Revision: 1.0 $  $Date: 2010/04/20 $

[n1,n2] = size(A);
[m1,m2] = size(b);

if min(m1,m2) ~=1
    error('CREATE_PLOT_STRUCTURE:argChk', 'Wrong type of input arguments: the input b must be a vector')
end

m = max (m1,m2);
if m == n1
    Y = zeros(n1,n2);
    for jj=1:m1
        Y(jj,:) = A(jj,:) + b(jj);
    end
    
elseif m==n2
    Y = zeros(n1,n2);
    for jj=1:m1
        Y(:,jj) = A(:,jj) + b(jj);
    end
else
    error('CREATE_PLOT_STRUCTURE:argChk', 'Wrong type of input arguments: One dimension of A must be the same of the vector''s')
end
