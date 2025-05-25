
function coordinates = arc(C,R,normal,t)
% ARC function generates coordinates to draw a circular arc in 3D
% arc(C,R,inplane,normal,t)
% R       - Radius
% C       - Centre
% inplane - A vector in plane of the circle
% normal  - A vector normal to the plane of the circle
% t       - vector of theta. E.g. for a full circle this is 0:0.01:2*pi
% By Chandrakanth R.Terupally
inplane= findOrthogonalVector(normal);
v = cross(normal,inplane);
coordinates = [C(1) + R*cos(t)*inplane(1) + R*sin(t)*v(1);
    C(2) + R*cos(t)*inplane(2) + R*sin(t)*v(2);
    C(3) + R*cos(t)*inplane(3) + R*sin(t)*v(3)];
end

function orthogonal_vector = findOrthogonalVector(normal)
% 生成一个随机向量
% random_vector = rand(1, 3);
random_vector=[1,2,3];
if(normal==random_vector)
    random_vector=[1,3,4];
end
random_vector=random_vector/norm(random_vector);
% 使用叉乘找到与给定向量垂直的向量
orthogonal_vector = cross(normal, random_vector);
end