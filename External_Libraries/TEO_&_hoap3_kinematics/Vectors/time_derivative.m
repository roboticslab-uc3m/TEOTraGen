function dx_dt = time_derivative (x, Ts)
%   TIME_DERIVATIVE derivates a vector with respect to time.

%   Author: Paolo Pierro
%   $Revision: 1.0 $  $Date: 2011/06/16 $

n = size(x,1);
z = zeros(n,1);
dx_dt = [z,diff(x,1,2)/Ts];