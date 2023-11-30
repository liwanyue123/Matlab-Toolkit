function [axis, angle] = rotMat2AxisAngle(R)
    % Convert rotation matrix to axis-angle representation
    % Input: R - 3x3 rotation matrix
    % Output: axis - 3x1 unit axis vector
    %         angle - rotation angle (in radians)
    
    % Check if R is a 3x3 matrix
    [rows, cols] = size(R);
    if rows ~= 3 || cols ~= 3
        error('Input matrix must be a 3x3 matrix');
    end
    
    % Calculate rotation angle
    angle = acos((trace(R) - 1) / 2);
    
    % Calculate rotation axis
    if angle ~= 0
        axis = (1 / (2 * sin(angle))) * [R(3, 2) - R(2, 3); R(1, 3) - R(3, 1); R(2, 1) - R(1, 2)];
    else
        axis = [0; 0; 1]; % When angle is 0, the axis can be any vector; here, default axis vector is [0, 0, 1]
    end
end
