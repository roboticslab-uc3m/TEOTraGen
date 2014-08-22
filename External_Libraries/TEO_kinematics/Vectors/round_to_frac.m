function x_r = round_to_frac(x, p)
%ROUND_TO_FRAC  Round time instants to the closest multiple for a precision d.
%   round_to_Ts(x, p) rounds the elements of x to the nearest multiple of 10^(-p).
%
%   See also ROUND.

%   Author: Paolo Pierro
%   $Revision: 0.2 $  $Date: 2011/04/07 $
x_r = round(x*10^p)./10^p;
end