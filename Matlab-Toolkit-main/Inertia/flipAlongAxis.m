function P = flipAlongAxis(PseudoInertia, axis)
    P = PseudoInertia;
    X = eye(4);
    if axis == 'X'
        X(1, 1) = -1;
    elseif axis == 'Y'
        X(2, 2) = -1;
    elseif axis == 'Z'
        X(3, 3) = -1;
    end
    P = X * P * X;
end
