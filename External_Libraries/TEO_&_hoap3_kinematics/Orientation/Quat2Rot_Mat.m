function R = Quat2Rot_Mat(Q)
%   QUAT2ROT_MAT Convert quaternion to rotation matrix.
%   R = QUAT2ROT_MAT(Q) calculates the rotation matrix R for a given
%   quaternion Q. Q  should be a 4-by-M vector and the result will be
%   provided as a 3-by-3-by-M vector, representing the corresponding
%   rotation matrices.
%   Example:
%       RPY = [0; 0; pi/2];
%       Q = RPY2Quat(RPY);
%       R = Quat2Rot_Mat(Q);
%
%   See also ROT_MAT2QUAT, RPY2ROT_MAT, ROT_MAT2RPY, RPY2QUAT, QUAT2RPY.

%   Author: Paolo Pierro
%   $Revision: 1.0 $  $Date: 2011/03/03 $

R1 = [2*(Q(1,:).^2+Q(2,:).^2)-1; 2*(Q(2,:).*Q(3,:)-Q(1,:).*Q(4,:)); 2*(Q(2,:).*Q(4,:)+Q(1,:).*Q(3,:))];
R2 = [2*(Q(2,:).*Q(3,:)+Q(1,:).*Q(4,:)); 2*(Q(1,:).^2+Q(3,:).^2)-1; 2*(Q(3,:).*Q(4,:)-Q(1,:).*Q(2,:))];
R3 = [2*(Q(2,:).*Q(4,:)-Q(1,:).*Q(3,:)); 2*(Q(3,:).*Q(4,:)+Q(1,:).*Q(2,:)); 2*(Q(1,:).^2+Q(4,:).^2)-1];

R(1,1:3,:)=R1;
R(2,1:3,:)=R2;
R(3,1:3,:)=R3;
end