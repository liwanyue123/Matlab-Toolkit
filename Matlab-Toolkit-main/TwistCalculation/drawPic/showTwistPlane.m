function [] = showTwistPlane(p0,V0,num,distance,showType,rate)
%һ���ٶ�����V0,��������������ϵ�����ڵ�λ��p0���Լ�����Χn��rate������
% ����������ٶȳ�
n=num*distance;
for x=-n:distance:n
    for y=-n:distance:n
        z=p0(3)
        r_01=[x y z]';
        calTwistFromOtherTwist(p0,V0,r_01,showType,rate,"green");
    end
end
end

