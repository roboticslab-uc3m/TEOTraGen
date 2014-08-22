function t_r = round_to_Ts(t, Ts)
%ROUND_TO_TS  Round time instants to the closest multiple of the sampling time Ts.
%   round_to_Ts(t, Ts) rounds the elements of t to the nearest multiple of Ts.
%
%   See also ROUND.

%   Author: Paolo Pierro
%   Update: Domingo Esteban
%   $Revision: 0.3 $  $Date: 2014/01/10 $
t_r = round(t./Ts).*Ts;



% NOTE: WE ROUND THE FLOAT TO -14 DECIMALS, TO HOMOGENIZE MATLAB DATA
% (SOLVE A BUG OF THIS ALGORITHM) 
t_r =  roundn(t_r,-6); 

end