function mat_R_T_M = set_manual_goal(goal)
%--------------------------------------------------------------------------
% Computes a homogeneous transform based on xyz position and zyz euler
% representation encoded as [x,y,z, z,y,z]
%
% Input
% goal [1x6] - [x,y,z, z,y,z]
%
% Output
% mat_R_T_M [4x4] - goal pose as homogeneous trans wrt base_link 
%--------------------------------------------------------------------------

    % Set translation and orientation
    gripperTranslation = [goal(1) goal(2) goal(3)];
    gripperRotation    = [goal(4) goal(5) goal(6)]; %  [Z Y Z] radians
    
    % Compute the homogeneous transformation via an euler (ZYZ) representation
    mat_R_T_M = eul2tform(gripperRotation);
    mat_R_T_M(1:3,4) = gripperTranslation';
end
