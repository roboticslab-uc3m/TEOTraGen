function q = pose_tr2rpy (R, order, type)
%   POSE_TR2RPY Convert rotation matrix to roll, pitch and yaw angles.
%   Q = ROT_MAT2RPY(R, ORDER, TYPE) calculates the set of RPY angles for a given
%   rotation matrix. The input could be a 3-by-3-by-M vector resulting in a
%   3-by-M vector representing the corresponding RPY angles.
%   It has two possible options: order and type
%   - xyz order --> roll is rotation about x, pitch about y, and yaw about
%   z.
%   - zyx order --> roll is rotation about z, pitch about y, and yaw about
%   x.
%   - 'global' type --> rotations is about the global frame
%   - 'local' type --> rotations is about the local frame
%
%   If no options are given, default configuration: ORDER='xyz' TYPE='local'
%
%   Example:
%       POSE_RPY = [1; 2; 3; 0; 0; pi/2];
%       POSE_ROT = pose_tr2rpy(POSE_RPY,'xyz','local');
%       POSE_RPY_ = pose_tr2rpy(POSE_ROT,'xyz','local');
%
%   See also RPY2ROT_MAT, RPY2QUAT, QUAT2RPY, ROT_MAT2QUAT, QUAT2ROT_MAT.

%   Author: Domingo Esteban
%   $Revision: 1.0 $  $Date: 2014/03/03 $

% Check number of arguments

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

% Calculate roll, pitch and yaw
if strcmp(type,'global')
  if strcmp(order, 'zyx')
    roll = atan2(R(2,1,:),R(1,1,:));
    pitch = atan2(-R(3,1,:),sqrt(R(3,2,:).^2+R(3,3,:).^2));
    yaw = atan2(R(3,2,:),R(3,3,:));

  elseif strcmp(order, 'xyz')
    roll = atan2(-R(2,3,:),R(3,3,:));
    pitch = atan2(R(1,3,:),sqrt(R(2,3,:).^2+R(3,3,:).^2));
    yaw = atan2(-R(1,2,:),R(1,1,:)); 
  end
  
elseif strcmp(type,'local')
  if strcmp(order, 'zyx')
    roll = atan2(-R(1,2,:),R(1,1,:));
    pitch = atan2(R(1,3,:),sqrt(R(2,3,:).^2+R(3,3,:).^2));
    yaw = atan2(-R(2,3,:),R(3,3,:)); 
        
  elseif strcmp(order, 'xyz')
    roll = atan2(R(3,2,:),R(3,3,:));
    pitch = atan2(-R(3,1,:),sqrt(R(3,2,:).^2+R(3,3,:).^2));
    yaw = atan2(R(2,1,:),R(1,1,:));
  end
end

q = [ R(1,4,:) ;
      R(2,4,:) ;
      R(3,4,:) ;
      roll ;
      pitch ;
      yaw] ;
            
end