function [J] = Jocobian(P,theta)

P=subTime(P) ;
theta=subTime(theta)  ;
%�������һ��
col=simplify(dif(P,theta(1)));
J=[col];
num =max(size(theta));
%�ſɱ�һ��һ����
for i=2:num  
    col=simplify(dif(P,theta(i)));
    J=[J col];
end
end

