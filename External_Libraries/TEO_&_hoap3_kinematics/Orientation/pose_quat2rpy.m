function p_rpy = pose_quat2rpy (p_quat, order, type)
%   POSE_QUAT2RPY Convert a pose orientation from quaternion to roll, pitch and yaw angles.
%   P_RPY = POSE_QUAT2RPY(P_QUAT, ORDER, TYPE) calculates the pose with orientation in
%   RPY angles for the input pose with orientation expressed in quaternion.
%   The input should be a 7-by-M vector and the result will be a 6-by-M
%   vector, representing the corresponding position and orientation in RPY.
% TODO: Manage gimbal lock (yaw = pi/2)

%   See also ROT_MAT2RPY, RPY2QUAT, QUAT2RPY, ROT_MAT2QUAT, QUAT2ROT_MAT.

%   Author: Domingo Esteban
%   $Revision: 1.0 $  $Date: 2014/03/03 $

if nargin < 3
  type = 'local';
end
if nargin <2
  order = 'xyz';
end

% Check values of arguments
if ~(strcmp(order,'zyx') || strcmp(order,'xyz'))
  error ('Wrong order option. Use "xyz" or "yzx"');
end

if ~(strcmp(type,'local') || strcmp(type,'global'))
  error ('Wrong type option. Use "local" or "global"');
end

p_rpy(1:3,:) = p_quat(1:3,:);
p_rpy(4:6,:) = Quat2RPY(real(p_quat(4:7,:)));
end