function R = RPY2Rot_Mat (q, order, type)
%   RPY2ROT_MAT Convert roll, pitch and yaw angles to rotation matrix.
%   R = RPY2ROT_MAT(Q,ORDER,TYPE) calculates the rotation matrix for a given set of
%   RPY angles. The input should be a 3-by-M vector and the result will be
%   a 3-by-3-by-M vector, representing the corresponding rotation matrices.
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
%       RPY = [0; 0; pi/2];
%       R = RPY2Rot_Mat(RPY,'xyz','local');
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


c1 = cos(q(1,:));
s1 = sin(q(1,:));
c2 = cos(q(2,:));
s2 = sin(q(2,:));
c3 = cos(q(3,:));
s3 = sin(q(3,:));

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
end