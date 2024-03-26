function [J] = Jocobian(P,theta)

P=subTime(P) ;
theta=subTime(theta)  ;
%先算出第一列
col=simplify(dif(P,theta(1)));
J=[col];
num =max(size(theta));
%雅可比一列一列算
for i=2:num  
    col=simplify(dif(P,theta(i)));
    J=[J col];
end
end

