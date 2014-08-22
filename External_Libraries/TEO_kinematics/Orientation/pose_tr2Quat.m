function Q = pose_tr2Quat (R)
%   TR2QUAT Convert rotation matrix to quaternion.
%   P_QUAT = TR2QUAT(R) calculates the pose orientation from quaternion Q, as a column vector,
%   for given, homogeneus transformation matrix. R can be also a vector of matrices.
%
%   See also QUAT2ROT_MAT, RPY2ROT_MAT, ROT_MAT2RPY, RPY2QUAT, QUAT2RPY.

%   Author: Domingo Esteban
%   $Revision: 1.0 $  $Date: 2013/09/01 $

Q(1,:) = R(1,4,:);
Q(2,:) = R(2,4,:);
Q(3,:) = R(3,4,:);
Q(4,:)   = 1/2.*sqrt(R(1,1,:)+R(2,2,:)+R(3,3,:)+1);
Q(5:7,:) = 1/2.*[ sign(R(3,2,:)-R(2,3,:)).*sqrt(R(1,1,:)-R(2,2,:)-R(3,3,:)+1);
          sign(R(1,3,:)-R(3,1,:)).*sqrt(R(2,2,:)-R(1,1,:)-R(3,3,:)+1);
          sign(R(2,1,:)-R(1,2,:)).*sqrt(R(3,3,:)-R(2,2,:)-R(1,1,:)+1)];


end