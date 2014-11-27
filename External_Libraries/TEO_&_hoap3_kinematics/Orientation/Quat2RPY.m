function RPY = Quat2RPY(Q,order,type)
%   QUAT2RPY Convert quaternion to roll, pitch and yaw angles.
%   RPY = QUAT2RPY(Q) calculates the RPY angles for a given quaternion. The
%   input should be a 4-by-M vector and the result will be a 3-by-M vector,
%   representing the corresponding RPYs.
%
%   Example:
%       RPY = [0; 0; pi/2];
%       Q = RPY2Quat(RPY, 'xyz', 'local');
%       RPY_ = Quat2RPY(Q);
%
%   See also RPY2QUAT, ROT_MAT2QUAT, QUAT2ROT_MAT, RPY2ROT_MAT,
%   ROT_MAT2RPY.
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


% Obtain equivalent Rotation Matrix
R(1,1,:) = 2*(Q(1,:).^2 + Q(2,:).^2) - 1;
R(2,1,:) = 2*(Q(2,:).*Q(3,:) + Q(1,:).*Q(4,:));
R(3,1,:) = 2*(Q(2,:).*Q(4,:) - Q(1,:).*Q(3,:));
R(1,2,:) = 2*(Q(2,:).*Q(3,:) - Q(1,:).*Q(4,:));
R(2,2,:) = 2*(Q(1,:).^2 + Q(3,:).^2) - 1;
R(3,2,:) = 2*(Q(3,:).*Q(4,:) + Q(1,:).*Q(2,:));
R(1,3,:) = 2*(Q(2,:).*Q(4,:) + Q(1,:).*Q(3,:));
R(2,3,:) = 2*(Q(3,:).*Q(4,:) - Q(1,:).*Q(2,:));
R(3,3,:) = 2*(Q(1,:).^2 + Q(4,:).^2) - 1;


% Calculate rpy angles
if strcmp(type,'global')
  if strcmp(order, 'zyx')
    RPY = [ atan2(R(2,1,:),R(1,1,:));
          atan2(-R(3,1,:),sqrt(R(3,2,:).^2+R(3,3,:).^2));
          atan2(R(3,2,:),R(3,3,:))];

  elseif strcmp(order, 'xyz')
    RPY = [ atan2(-R(2,3,:),R(3,3,:));
          atan2(R(1,3,:),sqrt(R(2,3,:).^2+R(3,3,:).^2));
          atan2(-R(1,2,:),R(1,1,:))]; 
  end
  
elseif strcmp(type,'local')
  if strcmp(order, 'zyx')
    RPY = [ atan2(-R(1,2,:),R(1,1,:));
          atan2(R(1,3,:),sqrt(R(2,3,:).^2+R(3,3,:).^2));
          atan2(-R(2,3,:),R(3,3,:))]; 
        
  elseif strcmp(order, 'xyz')
    RPY = [ atan2(R(3,2,:),R(3,3,:));
          atan2(-R(3,1,:),sqrt(R(3,2,:).^2+R(3,3,:).^2));
          atan2(R(2,1,:),R(1,1,:))];
  end
end

end