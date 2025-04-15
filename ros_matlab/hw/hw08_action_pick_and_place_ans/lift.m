function mat_current = lift(mat_current,z_offset)
%--------------------------------------------------------------------------
% Increases the z-position of mat_current by z_offset
%
% Inputs: 
% mat_current [4x4] - a homogeneous transform
% z_offset (double) - some vertical offset
%
% Output
% mat_current [4x4]
%--------------------------------------------------------------------------

    % Check Inputs
    [r,c] = size(mat_current);
    assert(r == 4 && c == 4,'lift:mat_current is not a 4x4');

    % Add offset to z-position
    mat_current(3,4) = mat_current(3,4) + z_offset;
end