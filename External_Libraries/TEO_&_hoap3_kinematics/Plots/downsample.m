function data_new = downsample (data_old, Ts_old, Ts_new)
%DOWNSAMPLE outputs a vector with reduced sampling rate of a signal.
% It calculates the downsampling factor M which multiplies the sampling time.
% data_new = downsample (data_old, Ts_old, Ts_new) creates a vector
% data_new with sampling time Ts_new where every M samples have been
% considered.
%
%   Author: Paolo Pierro
%   $Revision: 1.0 $  $Date: 2010/03/31 $

if nargin < 3
    error('DOWNSAMPLE:argChk', 'Wrong number of input arguments')
end
if Ts_new < Ts_old
    error('DOWNSAMPLE:argChk', 'Wrong type of input arguments: Ts_new should be greater than Ts_old')
else
    L = length(data_old);
    n = round(Ts_new/Ts_old);
    data_new = data_old(1:n:L);
end