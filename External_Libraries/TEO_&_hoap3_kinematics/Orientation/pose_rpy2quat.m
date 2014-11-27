function p_quat = pose_rpy2quat (p_rpy,order,type)
%   POSE_QUAT2RPY Convert a pose orientation from roll, pitch and yaw angles to quaternion.
%   P_QUAT = POSE_RPY2QUAT(P_RPY) calculates the pose with orientation in
%   quaternion for the input pose whose orientation is expressed in  RPY
%   angles. The input should be a 6-by-M vector and the result will be a
%   7-by-M vector, representing the corresponding position and orientation
%   in RPYs.
%   It has two possible options: order and type
%   - xyz order --> roll is rotation about x, pitch about y, and yaw about
%   z.
%   - zyx order --> roll is rotation about z, pitch about y, and yaw about
%   x.
%   - 'global' type --> rotations is about the global frame
%   - 'local' type --> rotations is about the local frame
%
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

p_quat(1:3,:) = p_rpy(1:3,:);
p_quat(4:7,:) = RPY2Quat(p_rpy(4:6,:),order,type);
end