function [R1] = calRotMatrix(axis,theta)

if isempty(theta)
    error("theta is empty!!!!!!!")
end
c = cos(theta);
s = sin(theta);

Rx = [1,0,0;
    0,c,-s;
    0,s,c;];

Ry = [c,0,s;
    0,1,0;
    -s,0,c;];

Rz = [c,-s,0;
    s,c,0;
    0,0,1;];

R1 = eye(3);

if axis == 'X'
    R1 = Rx;
elseif axis == 'Y'
    R1 = Ry;
elseif axis == 'Z'
    R1 = Rz;
end


end

