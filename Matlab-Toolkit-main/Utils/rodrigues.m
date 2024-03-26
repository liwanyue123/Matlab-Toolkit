function R = rodrigues(axis, angle)
    % Normalize the axis
    axis = axis / norm(axis);

    % Rodrigues formula
    K = [0, -axis(3), axis(2); axis(3), 0, -axis(1); -axis(2), axis(1), 0];
    R = eye(3) + sin(angle) * K + (1 - cos(angle)) * (K * K);
end