function [R1] = calRotMatrix(axis,theta)
global isSym
if(isSym)
    R1=calRotMatrix_syms(axis,theta);
else
    R1= calRotMatrix_num(axis,theta);
end
end

function [R1] = calRotMatrix_num(axis,theta)

if isempty(theta)
    error="theta is empty!!!!!!!"
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

R1 = eye(4);

if axis == 'X'
    R1 = Rx;
elseif axis == 'Y'
    R1 = Ry;
elseif axis == 'Z'
    R1 = Rz;
end


end


function [R1] = calRotMatrix_syms(axis,theta)

if theta(1)=='-'
    t = sym(theta(2:end))*(-1);
else
    t = sym(theta);
end

c = sym(cos(t));
s = sym(sin(t));

Rx = [1,0,0;
    0,c,-s;
    0,s,c;];

Ry = [c,0,s;
    0,1,0;
    -s,0,c;];

Rz = [c,-s,0;
    s,c,0;
    0,0,1;];

R1 = eye(4);

if axis == 'X'
    R1 = Rx;
elseif axis == 'Y'
    R1 = Ry;
elseif axis == 'Z'
    R1 = Rz;
end


end

