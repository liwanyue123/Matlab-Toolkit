function [] = showTwistFiled(p0,V0,distance,showType,rate)
%һ���ٶ�����V0,��������������ϵ�����ڵ�λ��p0���Լ�����Χn��rate������
% ����������ٶȳ�
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

