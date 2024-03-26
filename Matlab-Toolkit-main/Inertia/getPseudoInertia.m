function P = getPseudoInertia(inertia)
    h = unHat(inertia(1:3, 4:6));
    Ibar = inertia(1:3, 1:3);
    m = inertia(4, 4);
    
    P = zeros(4);
    P(1:3, 1:3) = 0.5 * trace(Ibar) * eye(3) - Ibar;
    P(1:3, 4) = h;
    P(4, 1:3) = h';
    P(4, 4) = m;
end

 