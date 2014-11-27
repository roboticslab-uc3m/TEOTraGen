function P = set_trajectory_condition(t, x, dx, ddx)
%SET_TRAJECTORY_CONDITION defines a condition for an interpolant
%polynomial.
%   P = SET_TRAJECTORY_CONDITION(T, X, DX, DDX) creates the structure for
%   a point to be used in some interpolating function. Depending on the
%   number of input arguments, it sets the conditions for time ‘t’ also for
%   the first and second derivative.
%
%   See also SPLINE_INTERPOLATION.

%   Author: Paolo Pierro
%   $Revision: 1.0 $  $Date: 2011/02/04 $
if nargin < 2
    error('SET_TRAJECTORY_CONDITION:argChk', 'Wrong number of input arguments')
end
P.t = t;
P.x = x;
if nargin > 2
    P.dx = dx;
end
if nargin > 3
    P.ddx = ddx;
end
end