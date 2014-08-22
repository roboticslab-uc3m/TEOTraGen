function Q = RPY2Quat(RPY, order, type)
%   RPY2QUAT Convert roll, pitch and yaw angles to quaternion.
%   Q = RPY2ROT_MAT(RPY, ORDER, TYPE) calculates the quaternion for a given set of RPY
%   angles. The input should be a 3-by-M vector and the result will be a
%   4-by-M vector, representing the corresponding quaternions.
%   It has two possible options: order and type
%   - xyz order --> roll is rotation about x, pitch about y, and yaw about
%   z.
%   - zyx order --> roll is rotation about z, pitch about y, and yaw about
%   x.
%   - 'global' type --> rotations is about the global frame
%   - 'local' type --> rotations is about the local frame
%
%   If no options are given, default configuration: ORDER='xyz' TYPE='local'
%   Example:
%       RPY = [0; 0; pi/2];
%       Q = RPY2Quat(RPY,'xyz','local');
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


c1 = cos(RPY(1,:));
s1 = sin(RPY(1,:));
c2 = cos(RPY(2,:));
s2 = sin(RPY(2,:));
c3 = cos(RPY(3,:));
s3 = sin(RPY(3,:));

% Calculate rotation matrix
if strcmp(type,'global')
  if strcmp(order, 'zyx')
    r11 = c1.*c2;
    r21 = c2.*s1;
    r31 = -s2;
    r12 = c1.*s2.*s3 - c3.*s1;
    r22 = c1.*c3+s1.*s2.*s3;
    r32 = c2.*s3;
    r13 = s1.*s3 + c1.*c3.*s2;
    r23 = c3.*s1.*s2 - c1.*s3;
    r33 = c2.*c3;

  elseif strcmp(order, 'xyz')
    r11 = c2.*c3;
    r21 = c1.*s3 + c3.*s1.*s2;
    r31 = s1.*s3 - c1.*c3.*s2;
    r12 = -c2.*s3;
    r22 = c1.*c3 - s1.*s2.*s3;
    r32 = c3.*s1 + c1.*s2.*s3;
    r13 = s2;
    r23 = -c2.*s1;
    r33 = c1.*c2;
  end
  
elseif strcmp(type,'local')
  if strcmp(order, 'zyx')
    r11 = c1.*c2;
    r21 = c3.*s1 + c1.*s2.*s3;
    r31 = s1.*s3 - c1.*c3.*s2;
    r12 = -c2.*s1;
    r22 = c1.*c3 - s1.*s2.*s3;
    r32 = c1.*s3 + c3.*s1.*s2;
    r13 = s2;
    r23 = -c2.*s3;
    r33 = c2.*c3;
        
  elseif strcmp(order, 'xyz')
    r11 = c2.*c3;
    r21 = c2.*s3;
    r31 = -s2;
    r12 = c3.*s1.*s2 - c1.*s3;
    r22 = c1.*c3 + s1.*s2.*s3;
    r32 = c2.*s1;
    r13 = s1.*s3 + c1.*c3.*s2;
    r23 = c1.*s2.*s3 - c3.*s1;
    r33 = c1.*c2;
  end
end

Q(1,:) = 1/2.*sqrt(r11 + r22 + r33 +1);
Q(2,:) = 1/2.*sign(r32 - r23).*sqrt(r11 - r22 - r33 + 1);
Q(3,:) = 1/2.*sign(r13 - r31).*sqrt(r22 - r33 - r11 + 1);
Q(4,:) = 1/2.*sign(r21 - r12).*sqrt(r33 - r11 - r22 + 1);

end