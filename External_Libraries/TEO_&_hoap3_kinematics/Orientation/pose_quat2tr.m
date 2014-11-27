function p_tr = pose_quat2tr (p_quat)
%   POSE_QUAT2TR Convert a pose orientation from quaternion to a homogeneous transform matrix.
%   P_TR = POSE_QUAT2TR(P_QUAT) calculates the pose in homogeneous transform 
%   matrix for the input pose with orientation expressed in quaternion.
%   The input should be a 7-by-1 vector and the result will be a 4-by-4
%   matrix, representing the corresponding position and orientation in a homogeneous transform matrix.
%
%   See also POSE_QUAT2RPY, QUAT2RPY, RPY2QUAT.

%   Author: Domingo Esteban
%   $Revision: 1.0 $  $Date: 2013/09/01 $

p_tr = rt2tr(Quat2Rot_Mat(p_quat(4:7,:)), p_quat(1:3,:));

end