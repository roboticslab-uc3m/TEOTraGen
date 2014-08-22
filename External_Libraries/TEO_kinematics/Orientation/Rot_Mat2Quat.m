function Q = Rot_Mat2Quat (R)
%   ROT_MAT2QUAT Convert rotation matrix to quaternion.
%   Q = ROT_MAT2QUAT(R) calculates the quaternion Q, as a column vector,
%   for given, rotation matrix R. R can be also a vector of matrices. In
%   such case, Q will be a matrix where every column corresponds to the
%   quaternion of the corresponding rotation matrix. Q returns a 4-by-M
%   matrix containing M quaternions. Q has its scalar number as the first
%   row. 
%   Example:
%       RPY = [0; 0; pi/2];
%       R = RPY2Rot_Mat(RPY);
%       Q = Rot_Mat2Quat(R);
%
%   See also QUAT2ROT_MAT, RPY2ROT_MAT, ROT_MAT2RPY, RPY2QUAT, QUAT2RPY.

%   Author: Paolo Pierro
%   $Revision: 1.0 $  $Date: 2011/03/03 $

Q(1,:)   = 1/2.*sqrt(R(1,1,:)+R(2,2,:)+R(3,3,:)+1);
Q(2:4,:) = 1/2.*[ sign(R(3,2,:)-R(2,3,:)).*sqrt(R(1,1,:)-R(2,2,:)-R(3,3,:)+1);
          sign(R(1,3,:)-R(3,1,:)).*sqrt(R(2,2,:)-R(1,1,:)-R(3,3,:)+1);
          sign(R(2,1,:)-R(1,2,:)).*sqrt(R(3,3,:)-R(2,2,:)-R(1,1,:)+1)];
end