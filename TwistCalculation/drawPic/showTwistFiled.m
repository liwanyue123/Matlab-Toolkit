function [] = showTwistFiled(p0,V0,distance,showType,rate)
%一个速度旋量V0,和它在世界坐标系下所在的位置p0，以及场范围n，rate是缩放
% 它所代表的速度场
n=4*distance;
for x=-n:distance:n
    for y=-n:distance:n
        for z=-n:distance:n
            r_01=[x y z]';
            calTwistFromOtherTwist(p0,V0,r_01,showType,rate,"green");
        end
    end
end
end

