function p_tr = pose_rpy2tr (p_rpy)
%   POSE_RPY2TR Convert a pose in roll, pitch and yaw angles to a pose expressed
%   in homogeneous transformation matrix.
%   POSE_TR = POSE_RPY2TR(POS_RPY,ORDER,TYPE) calculates the homogeneous transformation matrix for a given set of
%   poses in RPY angles. The input should be a 4-by-M vector and the result will be
%   a 4-by-4-by-M vector, representing the corresponding rotation matrices.
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
%       POSE_TR = pose_rpy2tr(POSE_RPY,'xyz','local');
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


% Calculate the position vector
pos(:,1,:) = p_rpy(1:3,:);

% Calculate the rotational matrix

c1 = cos(p_rpy(4,:));
s1 = sin(p_rpy(4,:));
c2 = cos(p_rpy(5,:));
s2 = sin(p_rpy(5,:));
c3 = cos(p_rpy(6,:));
s3 = sin(p_rpy(6,:));

% Calculate rotation matrix
if strcmp(type,'global')
  if strcmp(order, 'zyx')
    R1 = [c1.*c2; c2.*s1; -s2];
    R2 = [c1.*s2.*s3 - c3.*s1; c1.*c3+s1.*s2.*s3; c2.*s3];
    R3 = [s1.*s3 + c1.*c3.*s2; c3.*s1.*s2 - c1.*s3; c2.*c3];

  elseif strcmp(order, 'xyz')
    R1 = [c2.*c3; c1.*s3 + c3.*s1.*s2; s1.*s3 - c1.*c3.*s2];
    R2 = [-c2.*s3; c1.*c3 - s1.*s2.*s3; c3.*s1 + c1.*s2.*s3];
    R3 = [s2; -c2.*s1; c1.*c2];
  end
  
elseif strcmp(type,'local')
  if strcmp(order, 'zyx')
    R1 = [c1.*c2; c3.*s1 + c1.*s2.*s3; s1.*s3 - c1.*c3.*s2];
    R2 = [-c2.*s1; c1.*c3 - s1.*s2.*s3; c1.*s3 + c3.*s1.*s2];
    R3 = [s2; -c2.*s3; c2.*c3];
        
  elseif strcmp(order, 'xyz')
    R1 = [c2.*c3; c2.*s3; -s2];
    R2 = [c3.*s1.*s2 - c1.*s3; c1.*c3 + s1.*s2.*s3; c2.*s1];
    R3 = [s1.*s3 + c1.*c3.*s2; c1.*s2.*s3 - c3.*s1; c1.*c2];
  end
end

R(1:3,1,:) = R1;
R(1:3,2,:) = R2;
R(1:3,3,:) = R3;

B = zeros(1,4,size(p_rpy,2));
B(1,4,:) = 1;

% Combine matrixes
p_tr = [R pos;B];

end